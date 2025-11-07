#!/usr/bin/env python3
"""
Multi-Vessel Encounter Scenario Test
=====================================

다중 선박 조우 시나리오에서:
1. 각 선박의 위치 및 heading 확인
2. 선박 간 상대 거리(range) 및 방위각(bearing) 계산
3. 좌표계 변환 검증 (ir-sim math → NED/maritime)
4. CPA (Closest Point of Approach) 계산
5. colregs-core 패키지 기능 활용

Author: Navigation Officer & DRL Developer
Date: 2025-11-07
"""

import numpy as np
from irsim.env import EnvBase
from irsim.util.util import (
    math_to_maritime_heading,
    math_to_ned_heading
)

# colregs-core 패키지 import
from colregs_core import (
    calculate_relative_bearing,
    calculate_distance,
    calculate_cpa_tcpa,
    heading_to_velocity
)


def convert_irsim_to_ned(vessel):
    """
    ir-sim vessel을 NED 좌표계로 변환
    
    Args:
        vessel: ir-sim robot or obstacle 객체
    
    Returns:
        dict: NED 좌표계 정보
            - position: (x_north, y_east)
            - heading: degrees, [0, 360), 0=North, CW
            - velocity: (vx_ned, vy_ned)
            - speed: m/s
    """
    state = vessel.state
    
    # Math coordinates에서 위치 추출
    x_math = state[0, 0]  # East
    y_math = state[1, 0]  # North
    psi_math = state[2, 0] if state.shape[0] >= 3 else 0.0
    
    # Math → NED 좌표계 변환
    # Math: x=East, y=North
    # NED: x=North, y=East
    x_ned = y_math  # North
    y_ned = x_math  # East
    
    # Heading 변환: Math → NED (degrees)
    psi_ned = math_to_ned_heading(psi_math)
    if psi_ned < 0:
        psi_ned += 2 * np.pi
    psi_ned_deg = np.degrees(psi_ned)
    
    # Velocity 추출 및 변환
    if hasattr(vessel, 'velocity_xy'):
        vel_world = vessel.velocity_xy  # [vx, vy] in math coordinates
        vx_math = vel_world[0, 0]  # East 방향 속도
        vy_math = vel_world[1, 0]  # North 방향 속도
        
        # Math → NED velocity 변환
        vx_ned = vy_math  # North 방향 속도
        vy_ned = vx_math  # East 방향 속도
        
        speed = np.sqrt(vx_ned**2 + vy_ned**2)
    else:
        vx_ned, vy_ned = 0.0, 0.0
        speed = 0.0
    
    return {
        'position': (x_ned, y_ned),
        'heading': psi_ned_deg,
        'velocity': (vx_ned, vy_ned),
        'speed': speed
    }


def print_vessel_info(vessel, idx, is_robot=True):
    """
    선박 정보 출력 (Math와 NED 좌표계 모두 표시)
    
    Args:
        vessel: robot or obstacle 객체
        idx: 선박 인덱스
        is_robot: robot 여부
    """
    vessel_type = "ROBOT" if is_robot else "OBSTACLE"
    state = vessel.state
    
    # Math coordinates
    x_math, y_math = state[0,0], state[1,0]
    psi_math = state[2,0] if state.shape[0] >= 3 else 0.0
    psi_math_deg = np.degrees(psi_math)
    
    # NED coordinates
    ned_data = convert_irsim_to_ned(vessel)
    x_ned, y_ned = ned_data['position']
    psi_ned_deg = ned_data['heading']
    speed = ned_data['speed']
    
    print(f"\n  [{vessel_type} {idx}]")
    print(f"    Position (Math):    ({x_math:7.2f}E, {y_math:7.2f}N) m")
    print(f"    Position (NED):     ({x_ned:7.2f}N, {y_ned:7.2f}E) m")
    print(f"    Heading (Math):     {psi_math:7.4f} rad = {psi_math_deg:7.2f}°")
    print(f"    Heading (NED):      {psi_ned_deg:7.2f}° (0°=North, CW)")
    print(f"    Speed:              {speed:7.2f} m/s")
    
    if speed > 0.01:
        vx_ned, vy_ned = ned_data['velocity']
        print(f"    Velocity (NED):     ({vx_ned:7.2f}N, {vy_ned:7.2f}E) m/s")


def analyze_encounter(own_vessel, target_vessel, own_idx, target_idx):
    """
    두 선박 간 조우 상황 분석 (colregs-core 함수 활용)
    
    Args:
        own_vessel: 자선
        target_vessel: 타선
        own_idx: 자선 인덱스
        target_idx: 타선 인덱스
    """
    # NED 좌표계로 변환
    own_ned = convert_irsim_to_ned(own_vessel)
    target_ned = convert_irsim_to_ned(target_vessel)
    
    # colregs-core 함수로 거리 및 방위각 계산
    distance = calculate_distance(own_ned['position'], target_ned['position'])
    relative_bearing = calculate_relative_bearing(
        own_ned['position'],
        own_ned['heading'],
        target_ned['position']
    )
    
    print(f"\n{'='*70}")
    print(f"ENCOUNTER ANALYSIS: {own_idx} → {target_idx}")
    print(f"{'='*70}")
    print(f"  Distance:                {distance:8.2f} m")
    print(f"  Relative Bearing:        {relative_bearing:8.2f}° (NED)")
    
    # 방위 설명
    if 337.5 <= relative_bearing or relative_bearing < 22.5:
        bearing_desc = "Dead Ahead (정선수)"
    elif 22.5 <= relative_bearing < 67.5:
        bearing_desc = "Starboard Bow (우현선수)"
    elif 67.5 <= relative_bearing < 112.5:
        bearing_desc = "Starboard Beam (정횡우현)"
    elif 112.5 <= relative_bearing < 157.5:
        bearing_desc = "Starboard Quarter (우현선미)"
    elif 157.5 <= relative_bearing < 202.5:
        bearing_desc = "Dead Astern (정선미)"
    elif 202.5 <= relative_bearing < 247.5:
        bearing_desc = "Port Quarter (좌현선미)"
    elif 247.5 <= relative_bearing < 292.5:
        bearing_desc = "Port Beam (정횡좌현)"
    else:
        bearing_desc = "Port Bow (좌현선수)"
    
    print(f"  Position:                {bearing_desc}")
    
    # colregs-core 함수로 CPA/TCPA 계산
    dcpa, tcpa = calculate_cpa_tcpa(
        own_ned['position'],
        own_ned['velocity'],
        target_ned['position'],
        target_ned['velocity']
    )
    
    print(f"  TCPA (Time to CPA):      {tcpa:8.2f} sec")
    print(f"  DCPA (Distance at CPA):  {dcpa:8.2f} m")
    
    if dcpa < 50:
        print(f"  ⚠️  WARNING: Collision risk! DCPA < 50m")
    elif dcpa < 100:
        print(f"  ⚠️  CAUTION: Close encounter, DCPA < 100m")


def main():
    """
    메인 테스트 함수
    """
    print("\n" + "="*80)
    print("MULTI-VESSEL ENCOUNTER SCENARIO TEST")
    print("="*80)
    print("\nLoading scenario: imazu_case_01.yaml")
    print("Testing: Multi-vessel position, bearing, distance, CPA analysis\n")
    
    # Environment 초기화
    env = EnvBase(
        world_name='imazu_case_01.yaml',
        display=False,
        save_ani=False
    )
    
    num_robots = len(env.robots)
    num_obstacles = len(env.obstacles)
    
    print(f"\n[Environment Info]")
    print(f"  Number of Robots:    {num_robots}")
    print(f"  Number of Obstacles: {num_obstacles}")
    print(f"  Total Vessels:       {num_robots + num_obstacles}")
    
    if num_robots + num_obstacles < 2:
        print("\n❌ ERROR: Need at least 2 vessels for encounter test!")
        return
    
    # 시뮬레이션 5 steps 실행하여 선박들이 움직이게 함
    print(f"\n{'='*80}")
    print("Running 5 simulation steps...")
    print(f"{'='*80}")
    
    for step in range(5):
        env.step()
        print(f"  Step {step} completed")
    
    # 모든 선박 정보 출력
    print(f"\n{'='*80}")
    print("VESSEL STATES AFTER 5 STEPS")
    print(f"{'='*80}")
    
    all_vessels = []
    
    # Robots
    for idx, robot in enumerate(env.robots):
        print_vessel_info(robot, idx, is_robot=True)
        all_vessels.append(('robot', idx, robot))
    
    # Obstacles
    for idx, obstacle in enumerate(env.obstacles):
        print_vessel_info(obstacle, idx, is_robot=False)
        all_vessels.append(('obstacle', idx, obstacle))
    
    # 모든 vessel pair에 대해 encounter 분석
    print(f"\n{'='*80}")
    print("PAIRWISE ENCOUNTER ANALYSIS")
    print(f"{'='*80}")
    
    for i in range(len(all_vessels)):
        for j in range(i+1, len(all_vessels)):
            type1, idx1, vessel1 = all_vessels[i]
            type2, idx2, vessel2 = all_vessels[j]
            
            print(f"\n{'-'*70}")
            analyze_encounter(vessel1, vessel2, 
                            f"{type1.upper()}_{idx1}", 
                            f"{type2.upper()}_{idx2}")
    
    print(f"\n{'='*80}")
    print("TEST COMPLETED")
    print(f"{'='*80}")
    print("\n✅ Multi-vessel encounter test finished!")
    print("   - Verify distances match visualization")
    print("   - Check bearing angles (0°=선수, 90°=우현, 180°=선미, 270°=좌현)")
    print("   - Confirm CPA calculations are reasonable\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
