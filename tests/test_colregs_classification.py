#!/usr/bin/env python3
"""
COLREGs Classification Test
============================

COLREGs Rules 13-15 조우 상황 분류:
1. ir-sim의 vessel states를 NED 좌표계로 변환
2. colregs-core를 사용하여 encounter classification
3. Head-on / Crossing / Overtaking 판정
4. Give-way / Stand-on vessel 판정
5. 충돌 위험도 평가

Author: Navigation Officer & DRL Developer
Date: 2025-11-06
"""

import numpy as np
from irsim.env import EnvBase
from irsim.util.util import (
    math_to_maritime_state,
    math_to_ned_heading,
    WrapToPi
)

# colregs-core 패키지 import
try:
    from colregs_core import (
        EncounterClassifier,
        CollisionRiskAssessor,
        VesselState
    )
    COLREGS_AVAILABLE = True
except ImportError as e:
    COLREGS_AVAILABLE = False
    print(f"WARNING: colregs-core not available: {e}")
    print("Make sure colregs-core is installed: cd /home/hyo/DRL-otter-navigation/colregs-core && poetry install")


def convert_irsim_to_vessel_state(vessel, vessel_id="vessel"):
    """
    ir-sim vessel을 colregs-core VesselState로 변환
    
    Args:
        vessel: ir-sim robot or obstacle 객체
        vessel_id: vessel 식별자
    
    Returns:
        VesselState: colregs-core의 VesselState 객체
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
    
    # Heading 변환: Math → NED
    # Math: 0=East, CCW, [-π, π]
    # NED: 0=North, CW, [0, 2π)
    psi_ned = math_to_ned_heading(psi_math)
    if psi_ned < 0:
        psi_ned += 2 * np.pi
    
    # Velocity 추출
    if hasattr(vessel, 'velocity_xy'):
        vel_world = vessel.velocity_xy  # [vx, vy] in math coordinates
        vx_math = vel_world[0, 0]  # East 방향 속도
        vy_math = vel_world[1, 0]  # North 방향 속도
        
        # Speed 계산
        speed = np.sqrt(vx_math**2 + vy_math**2)
    else:
        speed = 0.0
    
    # VesselState 생성 (NED coordinates)
    vessel_state = VesselState(
        vessel_id=vessel_id,
        position=np.array([x_ned, y_ned]),  # [North, East]
        heading=psi_ned,  # [0, 2π), 0=North, CW
        speed=speed,  # m/s
        length=vessel.shape_params.get('length', 2.0) if hasattr(vessel, 'shape_params') else 2.0
    )
    
    return vessel_state


def print_vessel_state_info(vessel_state, label):
    """
    VesselState 정보 출력
    
    Args:
        vessel_state: colregs-core VesselState 객체
        label: 선박 라벨
    """
    print(f"\n  [{label}]")
    print(f"    ID:       {vessel_state.vessel_id}")
    print(f"    Position: ({vessel_state.position[0]:7.2f}, {vessel_state.position[1]:7.2f}) m (NED)")
    print(f"    Heading:  {vessel_state.heading:7.4f} rad = {np.degrees(vessel_state.heading):7.2f}° (NED)")
    print(f"    Speed:    {vessel_state.speed:7.2f} m/s")
    print(f"    Length:   {vessel_state.length:7.2f} m")


def print_encounter_classification(result, own_id, target_id):
    """
    COLREGs encounter classification 결과 출력
    
    Args:
        result: EncounterClassifier 결과
        own_id: 자선 ID
        target_id: 타선 ID
    """
    print(f"\n{'='*70}")
    print(f"COLREGS CLASSIFICATION: {own_id} ↔ {target_id}")
    print(f"{'='*70}")
    
    print(f"\n[Geometric Data]")
    print(f"  Range:                    {result.range:8.2f} m")
    print(f"  Relative Bearing:         {result.relative_bearing:8.2f}° (NED)")
    print(f"  Bearing to Target:        {result.bearing_to_target:8.2f}° (NED)")
    print(f"  Aspect Angle:             {result.aspect_angle:8.2f}°")
    
    print(f"\n[Encounter Type]")
    print(f"  Classification:           {result.encounter_type.upper()}")
    
    if result.encounter_type == 'head-on':
        print(f"  Description:              두 선박이 정면으로 마주보며 접근")
        print(f"  COLREGs Rule:             Rule 14 - Head-on Situation")
        print(f"  Action:                   Both vessels alter course to starboard")
        
    elif result.encounter_type == 'crossing':
        print(f"  Description:              두 선박이 교차 항로로 접근")
        print(f"  COLREGs Rule:             Rule 15 - Crossing Situation")
        if result.is_give_way:
            print(f"  {own_id}:                  GIVE-WAY vessel (타선을 우현에 두고 있음)")
            print(f"  Action:                   Alter course/speed to avoid collision")
        else:
            print(f"  {own_id}:                  STAND-ON vessel (타선을 좌현에 두고 있음)")
            print(f"  Action:                   Maintain course and speed")
            
    elif result.encounter_type == 'overtaking':
        print(f"  Description:              한 선박이 다른 선박을 추월")
        print(f"  COLREGs Rule:             Rule 13 - Overtaking")
        if result.is_give_way:
            print(f"  {own_id}:                  GIVE-WAY vessel (추월하는 선박)")
            print(f"  Action:                   Keep clear of the vessel being overtaken")
        else:
            print(f"  {own_id}:                  STAND-ON vessel (추월당하는 선박)")
            print(f"  Action:                   Maintain course and speed")
            
    elif result.encounter_type == 'none':
        print(f"  Description:              조우 상황 아님 (거리가 멀거나 위험 없음)")
    
    print(f"\n[Vessel Roles]")
    print(f"  Give-way:                 {'YES (피항선)' if result.is_give_way else 'NO (유지선)'}")
    print(f"  Stand-on:                 {'YES (유지선)' if not result.is_give_way else 'NO (피항선)'}")


def print_collision_risk(risk_result, own_id, target_id):
    """
    충돌 위험도 평가 결과 출력
    
    Args:
        risk_result: CollisionRiskAssessor 결과
        own_id: 자선 ID
        target_id: 타선 ID
    """
    print(f"\n{'='*70}")
    print(f"COLLISION RISK ASSESSMENT: {own_id} ↔ {target_id}")
    print(f"{'='*70}")
    
    print(f"\n[CPA Data]")
    print(f"  TCPA (Time to CPA):       {risk_result.tcpa:8.2f} sec")
    print(f"  DCPA (Distance at CPA):   {risk_result.dcpa:8.2f} m")
    
    print(f"\n[Risk Assessment]")
    print(f"  Collision Risk:           {risk_result.collision_risk.upper()}")
    print(f"  Risk Score:               {risk_result.risk_score:8.4f}")
    
    if risk_result.collision_risk == 'high':
        print(f"  ⚠️  HIGH RISK - Immediate action required!")
    elif risk_result.collision_risk == 'medium':
        print(f"  ⚠️  MEDIUM RISK - Monitor closely and prepare action")
    elif risk_result.collision_risk == 'low':
        print(f"  ✓  LOW RISK - Safe passing expected")
    else:
        print(f"  ✓  NO RISK - No collision danger")


def main():
    """
    메인 테스트 함수
    """
    print("\n" + "="*80)
    print("COLREGS CLASSIFICATION TEST")
    print("="*80)
    
    if not COLREGS_AVAILABLE:
        print("\n❌ ERROR: colregs-core package not available!")
        print("Install it with: cd /home/hyo/DRL-otter-navigation/colregs-core && poetry install")
        return
    
    print("\nLoading scenario: imazu_case_01.yaml")
    print("Testing: COLREGs Rules 13-15 encounter classification\n")
    
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
        print("\n❌ ERROR: Need at least 2 vessels for COLREGs test!")
        return
    
    # 시뮬레이션 10 steps 실행 (선박들이 움직이면서 다양한 조우 상황 발생)
    print(f"\n{'='*80}")
    print("Running 10 simulation steps...")
    print(f"{'='*80}")
    
    for step in range(10):
        env.step()
        print(f"  Step {step} completed")
    
    # ir-sim vessels → colregs-core VesselStates 변환
    print(f"\n{'='*80}")
    print("CONVERTING VESSEL STATES (ir-sim → colregs-core)")
    print(f"{'='*80}")
    
    vessel_states = []
    
    for idx, robot in enumerate(env.robots):
        vessel_id = f"ROBOT_{idx}"
        vessel_state = convert_irsim_to_vessel_state(robot, vessel_id)
        vessel_states.append(vessel_state)
        print_vessel_state_info(vessel_state, vessel_id)
    
    for idx, obstacle in enumerate(env.obstacles):
        vessel_id = f"OBSTACLE_{idx}"
        vessel_state = convert_irsim_to_vessel_state(obstacle, vessel_id)
        vessel_states.append(vessel_state)
        print_vessel_state_info(vessel_state, vessel_id)
    
    # COLREGs Classifier 초기화
    classifier = EncounterClassifier()
    risk_assessor = CollisionRiskAssessor()
    
    # 모든 vessel pair에 대해 COLREGs classification
    print(f"\n{'='*80}")
    print("COLREGS ENCOUNTER CLASSIFICATION")
    print(f"{'='*80}")
    
    for i in range(len(vessel_states)):
        for j in range(i+1, len(vessel_states)):
            own_vessel = vessel_states[i]
            target_vessel = vessel_states[j]
            
            print(f"\n{'-'*70}")
            
            # Encounter classification
            try:
                result = classifier.classify_encounter(own_vessel, target_vessel)
                print_encounter_classification(result, own_vessel.vessel_id, target_vessel.vessel_id)
                
                # Collision risk assessment
                risk_result = risk_assessor.assess_risk(own_vessel, target_vessel)
                print_collision_risk(risk_result, own_vessel.vessel_id, target_vessel.vessel_id)
                
            except Exception as e:
                print(f"ERROR classifying encounter: {e}")
                import traceback
                traceback.print_exc()
    
    print(f"\n{'='*80}")
    print("TEST COMPLETED")
    print(f"{'='*80}")
    print("\n✅ COLREGs classification test finished!")
    print("   - Verify encounter types match expected scenarios")
    print("   - Check give-way/stand-on assignments follow COLREGs")
    print("   - Confirm collision risk assessments are reasonable")
    print("\n📖 COLREGs Rules Reference:")
    print("   - Rule 13: Overtaking")
    print("   - Rule 14: Head-on Situation")
    print("   - Rule 15: Crossing Situation\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
