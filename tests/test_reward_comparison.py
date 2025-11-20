#!/usr/bin/env python3
"""
Jeon vs Chun Reward Calculator 비교 테스트

전도현 (2024)와 Chun et al. (2021, 2024) reward function을 비교합니다.
"""
import numpy as np
from colregs_core.reward import JeonRewardCalculator, ChunRewardCalculator
from colregs_core.risk import ShipDomainParams
from colregs_core.encounter.types import EncounterType


def test_reward_calculators():
    """두 reward calculator 비교 테스트"""
    
    print("=" * 80)
    print("Jeon vs Chun Reward Calculator 비교 테스트")
    print("=" * 80)
    
    # Common parameters
    d_max = 10.0  # 10m cross-track error limit
    v_ref = 2.0   # 2 m/s target speed
    cr_allowable = 0.3
    dt = 0.1
    phi_max = 4.0
    d_obs = 200.0
    
    ship_domain = ShipDomainParams(
        r_bow=6.0,
        r_stern=2.0,
        r_starboard=6.0,
        r_port=2.0
    )
    
    # Initialize calculators
    print("\n[1] Initializing Reward Calculators...")
    jeon_calc = JeonRewardCalculator(
        d_max=d_max,
        v_ref=v_ref,
        cr_allowable=cr_allowable,
        dt=dt,
        ship_domain=ship_domain,
        d_obs=d_obs,
        phi_max=phi_max,
        cr_method='jeon',
        os_speed_for_cr=2.0,
        ts_speed_for_cr=2.0
    )
    print("  ✓ JeonRewardCalculator initialized")
    
    chun_calc = ChunRewardCalculator(
        d_cl=d_max,
        v_ref=v_ref,
        cr_allowable=cr_allowable,
        dt=dt,
        ship_domain=ship_domain,
        d_obs=d_obs,
        phi_max=phi_max,
        cr_method='chun',
        os_speed_for_cr=2.0,
        ts_speed_for_cr=2.0,
        check_point_reward_enabled=False,
        maintain_reward_enabled=False
    )
    print("  ✓ ChunRewardCalculator initialized")
    
    # Test scenario
    print("\n[2] Test Scenario Setup")
    print("  - OS: (0, 0), heading 0°, speed 2.0 m/s")
    print("  - TS: (50, 50), heading 180°, speed 2.0 m/s")
    print("  - Start: (0, 0), Goal: (100, 0)")
    print("  - Previous distance: 100m, Current distance: 99.8m, Next distance: 99.6m")
    print("  - Cross-track error: 5m")
    
    # Scenario parameters
    start_position = (0.0, 0.0)
    goal_position = (100.0, 0.0)
    
    os_position = (0.2, 5.0)  # slightly moved with cross-track error
    os_velocity = (2.0, 0.0)  # 2 m/s in x-direction (East)
    os_heading = 0.0  # North (NED)
    os_speed = 2.0
    previous_heading = 0.0
    
    ts_position = (50.0, 50.0)
    ts_velocity = (-2.0, 0.0)  # 2 m/s in -x direction (West)
    ts_heading = 180.0  # South (NED)
    ts_speed = 2.0
    
    previous_distance = 100.0
    current_distance = 99.8
    next_distance = 99.6
    cross_track_error = 5.0
    
    # Calculate CR for both methods
    print("\n[3] Collision Risk Calculation")
    print("-" * 80)
    
    jeon_cr_result = jeon_calc.cr_calculator.calculate_collision_risk(
        os_speed, os_position, os_velocity, os_heading,
        ts_speed, ts_position, ts_velocity, ts_heading
    )
    
    chun_cr_result = chun_calc.cr_calculator.calculate_collision_risk(
        os_speed, os_position, os_velocity, os_heading,
        ts_speed, ts_position, ts_velocity, ts_heading
    )
    
    print(f"\n  Jeon CR Calculation:")
    print(f"    CR: {jeon_cr_result['cr']:.4f}")
    print(f"    DCPA: {jeon_cr_result['dcpa']:.2f} m")
    print(f"    TCPA: {jeon_cr_result['tcpa']:.2f} s")
    print(f"    Encounter Type: {jeon_cr_result['encounter_type']}")
    
    print(f"\n  Chun CR Calculation:")
    print(f"    CR: {chun_cr_result['cr']:.4f}")
    print(f"    DCPA: {chun_cr_result['dcpa']:.2f} m")
    print(f"    TCPA: {chun_cr_result['tcpa']:.2f} s")
    print(f"    Encounter Type: {chun_cr_result['encounter_type']}")
    
    # Use higher CR for conservative testing
    cr_max = max(jeon_cr_result['cr'], chun_cr_result['cr'])
    tcpa = jeon_cr_result['tcpa']  # Use Jeon's TCPA
    
    # Test individual reward components
    print("\n[4] Individual Reward Components Comparison")
    print("-" * 80)
    
    # Goal reward
    print("\n  [Goal Reward]")
    jeon_goal = jeon_calc.calculate_goal_reward(current_distance, previous_distance, os_speed)
    chun_goal = chun_calc.calculate_goal_reward(current_distance, next_distance, os_speed)
    print(f"    Jeon: {jeon_goal:+.4f}")
    print(f"    Chun: {chun_goal:+.4f}")
    print(f"    Difference: {abs(jeon_goal - chun_goal):.6f}")
    
    # Cross reward
    print("\n  [Cross Reward]")
    jeon_cross = jeon_calc.calculate_cross_reward(cross_track_error)
    chun_cross = chun_calc.calculate_cross_reward(cross_track_error)
    print(f"    Jeon: {jeon_cross:+.4f}")
    print(f"    Chun: {chun_cross:+.4f}")
    print(f"    Difference: {abs(jeon_cross - chun_cross):.6f}")
    
    # Speed reward
    print("\n  [Speed Reward]")
    jeon_speed = jeon_calc.calculate_speed_reward(os_speed)
    chun_speed = chun_calc.calculate_speed_reward(os_speed)
    print(f"    Jeon: {jeon_speed:+.4f}")
    print(f"    Chun: {chun_speed:+.4f}")
    print(f"    Difference: {abs(jeon_speed - chun_speed):.6f}")
    
    # Risk/Collision reward
    print("\n  [Risk/Collision Reward]")
    jeon_risk = jeon_calc.calculate_risk_reward(cr_max)
    chun_collision = chun_calc.calculate_collision_reward(cr_max)
    print(f"    Jeon (Risk): {jeon_risk:+.4f}")
    print(f"    Chun (Collision): {chun_collision:+.4f}")
    print(f"    Difference: {abs(jeon_risk - chun_collision):.6f}")
    
    # COLREGs reward (use HEAD_ON as example encounter type)
    print("\n  [COLREGs Reward]")
    encounter_type = EncounterType.HEAD_ON
    
    jeon_colregs = jeon_calc.calculate_colregs_reward(
        os_heading, cr_max, tcpa,
        start_position, goal_position,
        encounter_type, is_static_obstacle=False
    )
    chun_colregs = chun_calc.calculate_colregs_reward(
        os_heading, cr_max, tcpa,
        start_position, goal_position,
        encounter_type, is_static_obstacle=False
    )
    print(f"    Jeon: {jeon_colregs:+.4f}")
    print(f"    Chun: {chun_colregs:+.4f}")
    print(f"    Difference: {abs(jeon_colregs - chun_colregs):.6f}")
    
    # Heading reward
    print("\n  [Heading Reward]")
    jeon_heading = jeon_calc.calculate_heading_reward(os_heading, previous_heading)
    chun_heading = chun_calc.calculate_heading_reward(os_heading, previous_heading)
    print(f"    Jeon: {jeon_heading:+.4f}")
    print(f"    Chun: {chun_heading:+.4f}")
    print(f"    Difference: {abs(jeon_heading - chun_heading):.6f}")
    
    # Total reward
    print("\n[5] Total Reward Comparison")
    print("-" * 80)
    
    jeon_result = jeon_calc.calculate_total_reward(
        current_distance=current_distance,
        previous_distance=previous_distance,
        cross_track_error=cross_track_error,
        os_speed=os_speed,
        os_position=os_position,
        os_velocity=os_velocity,
        os_heading=os_heading,
        previous_heading=previous_heading,
        ts_speed=ts_speed,
        ts_position=ts_position,
        ts_velocity=ts_velocity,
        ts_heading=ts_heading,
        start_position=start_position,
        goal_position=goal_position,
        CR_max=cr_max,
        encounter_type=encounter_type,
        is_static_obstacle=False,
        w_efficiency=1.0,
        w_safety=1.0
    )
    
    chun_result = chun_calc.calculate_total_reward(
        current_distance=current_distance,
        next_distance=next_distance,
        cross_track_error=cross_track_error,
        os_speed=os_speed,
        os_position=os_position,
        os_velocity=os_velocity,
        os_heading=os_heading,
        previous_heading=previous_heading,
        ts_speed=ts_speed,
        ts_position=ts_position,
        ts_velocity=ts_velocity,
        ts_heading=ts_heading,
        start_position=start_position,
        goal_position=goal_position,
        CR_max=cr_max,
        encounter_type=encounter_type,
        is_static_obstacle=False,
        check_point_position=None,
        next_os_position=None,
        sailing_distance_between_checkpoints=0.0,
        w_efficiency=1.0,
        w_safety=1.0
    )
    
    print("\n  Jeon Total Reward Components:")
    for key, value in jeon_result.items():
        print(f"    {key:20s}: {value:+.4f}")
    
    print("\n  Chun Total Reward Components:")
    for key, value in chun_result.items():
        print(f"    {key:20s}: {value:+.4f}")
    
    print("\n[6] Summary")
    print("-" * 80)
    print(f"  Jeon Total Reward: {jeon_result['r_total']:+.4f}")
    print(f"  Chun Total Reward: {chun_result['r_total']:+.4f}")
    print(f"  Difference: {abs(jeon_result['r_total'] - chun_result['r_total']):.6f}")
    
    # Key differences
    print("\n[7] Key Differences")
    print("-" * 80)
    print("  1. Goal Reward Formula:")
    print("     - Jeon: Uses previous_distance (distance at t-1)")
    print("     - Chun: Uses next_distance (distance at t+1)")
    print("\n  2. Cross Reward Formula:")
    print("     - Jeon: R_cross = 1 - 2*|y_e|/d_max")
    print("     - Chun: R_cross = (d_cl/2 - |y_e|) / (d_cl/2)")
    print("\n  3. Speed Reward Formula:")
    print("     - Jeon: R_speed = 1 - 2*|v_e|/v_ref")
    print("     - Chun: Different formula for v_o <= v_ref vs v_o > v_ref")
    print("\n  4. Risk/Collision Reward:")
    print("     - Same formula, but Jeon uses JeonCR and Chun uses ChunCR")
    print("     - Jeon CR: Based on improved Mou formula")
    print("     - Chun CR: Based on exponential decay")
    print("\n  5. COLREGs Compliance Check:")
    print("     - Both use ColregsCompliant class")
    print("     - Threshold-based checking (heading change, TCPA, etc.)")
    print("\n  6. Additional Chun Features (disabled in this test):")
    print("     - Check Point Reward (Chun 2021)")
    print("     - COLREGs-Maintain Reward (Chun 2021)")
    
    print("\n" + "=" * 80)
    print("✓ Test completed successfully!")
    print("=" * 80)


if __name__ == '__main__':
    test_reward_calculators()
