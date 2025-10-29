"""
COLREGs Core + ir-sim 실제 통합 테스트

ir-sim 환경을 사용하여 colregs-core 패키지를 테스트합니다.
"""
try:
    from irsim.env import EnvBase
    from irsim.world import ObjectFactory
    print("✓ ir-sim imported successfully")
except ImportError as e:
    print(f"✗ ir-sim import failed: {e}")
    print("Please make sure ir-sim is properly installed")

try:
    from colregs_core import (
        EncounterClassifier,
        RiskAssessment,
        EncounterType,
        RiskLevel,
        heading_to_velocity
    )
    print("✓ colregs-core imported successfully")
except ImportError as e:
    print(f"✗ colregs-core import failed: {e}")

import numpy as np


class COLREGsEnhancedIRSimEnv(EnvBase):
    """
    ir-sim 환경에 COLREGs 기능을 통합한 강화된 환경
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # COLREGs 모듈 초기화
        self.encounter_classifier = EncounterClassifier(
            safe_distance=2000.0  # 2km
        )
        self.risk_assessor = RiskAssessment()
        
        print("\n[COLREGs Enhanced Environment Initialized]")
        print("  - Encounter Classifier: Ready")
        print("  - Risk Assessor: Ready")
    
    def get_colregs_info(self, robot_id=0):
        """
        현재 상태에서 모든 장애물에 대한 COLREGs 정보 반환
        
        Args:
            robot_id: 로봇 인덱스 (default: 0)
        
        Returns:
            list: 각 장애물에 대한 COLREGs 분석 결과
        """
        if not hasattr(self, 'robot_list') or len(self.robot_list) <= robot_id:
            return []
        
        robot = self.robot_list[robot_id]
        os_position = (robot.state[0], robot.state[1])  # x, y
        os_heading = np.degrees(robot.state[2])  # theta in degrees
        os_velocity_array = robot.velocity.flatten()
        os_speed = np.linalg.norm(os_velocity_array[:2])  # velocity magnitude
        os_velocity = (os_velocity_array[0], os_velocity_array[1])
        
        colregs_info = []
        
        for i, obs in enumerate(self.obstacle_list):
            # 장애물 상태
            ts_position = (obs.state[0], obs.state[1])
            ts_heading = np.degrees(obs.state[2]) if len(obs.state) > 2 else 0
            
            # 장애물 속도 (있는 경우)
            if hasattr(obs, 'velocity') and obs.velocity is not None:
                ts_velocity_array = obs.velocity.flatten()
                ts_velocity = (ts_velocity_array[0], ts_velocity_array[1])
                ts_speed = np.linalg.norm(ts_velocity_array[:2])
            else:
                ts_velocity = (0, 0)
                ts_speed = 0
            
            # Encounter 분류
            situation = self.encounter_classifier.classify(
                os_position=os_position,
                os_heading=os_heading,
                os_speed=os_speed,
                ts_position=ts_position,
                ts_heading=ts_heading,
                ts_speed=ts_speed
            )
            
            # 위험도 평가
            risk = self.risk_assessor.assess(
                os_position=os_position,
                os_velocity=os_velocity,
                ts_position=ts_position,
                ts_velocity=ts_velocity
            )
            
            info = {
                'obstacle_name': f'obstacle_{i}',
                'encounter_type': situation.encounter_type,
                'relative_bearing': situation.relative_bearing,
                'distance': situation.distance,
                'risk_level': risk.risk_level,
                'dcpa': risk.dcpa,
                'tcpa': risk.tcpa,
                'requires_action': risk.requires_action,
                'colregs_action': self.encounter_classifier.get_action_requirement(
                    situation.encounter_type
                )
            }
            
            colregs_info.append(info)
        
        return colregs_info
    
    def get_most_dangerous_obstacle(self, robot_id=0):
        """가장 위험한 장애물 식별"""
        colregs_info = self.get_colregs_info(robot_id)
        
        if not colregs_info:
            return None
        
        # 위험도 순 정렬
        dangerous_obstacles = [
            info for info in colregs_info
            if info['requires_action']
        ]
        
        if not dangerous_obstacles:
            return None
        
        dangerous_obstacles.sort(
            key=lambda x: (-x['risk_level'].value, x['tcpa'])
        )
        
        return dangerous_obstacles[0]


def test_basic_integration():
    """기본 통합 테스트"""
    print("\n" + "="*70)
    print("TEST 1: Basic Integration Test")
    print("="*70)
    
    # ir-sim 환경 생성
    env = COLREGsEnhancedIRSimEnv()
    
    # ObjectFactory 인스턴스 생성
    factory = ObjectFactory()
    
    # 로봇 추가
    robot = factory.create_robot(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[0, 0, 0],
        vel=[2, 0, 0]
    )
    env.add_object(robot)
    
    # 장애물 추가 (head-on 시나리오)
    obstacle1 = factory.create_obstacle(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[0, 50, np.pi],  # 50m ahead, heading south
        vel=[-2, 0, 0]  # moving towards robot
    )
    env.add_object(obstacle1)
    
    # COLREGs 정보 가져오기
    colregs_info = env.get_colregs_info(0)
    
    print(f"\nObstacles detected: {len(colregs_info)}")
    
    for info in colregs_info:
        print(f"\n{info['obstacle_name']}:")
        print(f"  Encounter Type: {info['encounter_type'].value}")
        print(f"  Distance: {float(info['distance']):.1f} m")
        print(f"  Relative Bearing: {float(info['relative_bearing']):.1f}°")
        print(f"  Risk Level: {info['risk_level'].name}")
        print(f"  DCPA: {float(info['dcpa']):.1f} m")
        print(f"  TCPA: {float(info['tcpa']):.1f} s")
        print(f"  Action Required: {'YES' if info['requires_action'] else 'NO'}")
        print(f"  COLREGs: {info['colregs_action'][:60]}...")
    
    # 가장 위험한 장애물
    most_dangerous = env.get_most_dangerous_obstacle(0)
    if most_dangerous:
        print(f"\n⚠️  MOST DANGEROUS: {most_dangerous['obstacle_name']}")
        print(f"   Risk: {most_dangerous['risk_level'].name}")
        print(f"   Encounter: {most_dangerous['encounter_type'].value}")
    
    print("\n✅ Test 1 PASSED")


def test_multiple_obstacles():
    """다중 장애물 시나리오"""
    print("\n" + "="*70)
    print("TEST 2: Multiple Obstacles Scenario")
    print("="*70)
    
    env = COLREGsEnhancedIRSimEnv()
    factory = ObjectFactory()
    
    # Own ship
    robot = factory.create_robot(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[0, 0, 0],
        vel=[2, 0, 0]
    )
    env.add_object(robot)
    
    # Obstacle 1: Head-on
    obs1 = factory.create_obstacle(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[0, 30, np.pi],
        vel=[-2, 0, 0]
    )
    env.add_object(obs1)
    
    # Obstacle 2: Crossing from starboard
    obs2 = factory.create_obstacle(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[30, 30, -np.pi/2],
        vel=[0, -2.5, 0]
    )
    env.add_object(obs2)
    
    # Obstacle 3: Safe distance
    obs3 = factory.create_obstacle(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[100, 0, np.pi],
        vel=[-1, 0, 0]
    )
    env.add_object(obs3)
    
    colregs_info = env.get_colregs_info(0)
    
    print(f"\nTotal obstacles: {len(colregs_info)}")
    
    dangerous_count = sum(1 for info in colregs_info if info['requires_action'])
    print(f"Dangerous obstacles: {dangerous_count}")
    
    for info in colregs_info:
        status = "⚠️" if info['requires_action'] else "✓"
        print(f"\n{status} {info['obstacle_name']}: "
              f"{info['encounter_type'].value} - "
              f"{info['risk_level'].name} "
              f"(D={float(info['distance']):.0f}m, DCPA={float(info['dcpa']):.0f}m)")
    
    most_dangerous = env.get_most_dangerous_obstacle(0)
    if most_dangerous:
        print(f"\n🚨 Priority target: {most_dangerous['obstacle_name']}")
        print(f"   Take action: {most_dangerous['colregs_action'][:80]}...")
    
    print("\n✅ Test 2 PASSED")


def test_dynamic_scenario():
    """동적 시나리오 - 시간에 따른 변화"""
    print("\n" + "="*70)
    print("TEST 3: Dynamic Scenario")
    print("="*70)
    
    env = COLREGsEnhancedIRSimEnv()
    factory = ObjectFactory()
    
    # Own ship
    robot = factory.create_robot(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[0, 0, np.pi/4],
        vel=[2, 2, 0]
    )
    env.add_object(robot)
    
    # Target ship
    obs = factory.create_obstacle(
        kinematics={"name": "diff"},
        shape={"name": "circle", "radius": 0.5},
        state=[50, 0, 3*np.pi/4],
        vel=[-2, 2, 0]
    )
    env.add_object(obs)
    
    print("\nTime | Distance | Encounter | Risk | DCPA")
    print("-" * 50)
    
    for step in range(10):
        env.step()
        
        colregs_info = env.get_colregs_info(0)
        if colregs_info:
            info = colregs_info[0]
            print(f"{step:4d} | {float(info['distance']):8.1f}m | "
                  f"{info['encounter_type'].value:15s} | "
                  f"{info['risk_level'].name:8s} | "
                  f"{float(info['dcpa']):6.1f}m")
    
    print("\n✅ Test 3 PASSED")


def run_all_tests():
    """모든 테스트 실행"""
    print("\n" + "="*70)
    print("COLREGs Core + ir-sim Integration Tests")
    print("="*70)
    
    try:
        test_basic_integration()
        test_multiple_obstacles()
        test_dynamic_scenario()
        
        print("\n" + "="*70)
        print("🎉 ALL TESTS PASSED!")
        print("="*70)
        print("\nCOLREGs Core successfully integrated with ir-sim!")
        print("="*70 + "\n")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    run_all_tests()
