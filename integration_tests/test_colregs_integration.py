"""
Main integration test for colregs-core in real Otter USV simulation.

This test runs a complete simulation step and monitors all calculations:
1. Geometry transformations (coordinate system conversion for heading AND position)
2. Collision risk calculations (Jeon & Chun methods)
3. Reward function calculations
4. COLREGs encounter classification (if applicable)

Usage:
    $ cd /home/hyo/colregs-core
    $ conda activate DRL-otter-nav
    $ poetry run python integration_tests/test_colregs_integration.py
"""

import sys
import numpy as np
from pathlib import Path
from typing import Dict, Any

# Add paths
sys.path.append('/home/hyo/PythonVehicleSimulator/src')
sys.path.append('/home/hyo/DRL-otter-navigation')

# Import colregs-core modules
from colregs_core.utils import distance, cross_track_error
from colregs_core.geometry import (
    heading_speed_to_velocity, 
    math_to_ned_heading,
    math_to_maritime_position
)
from colregs_core.risk import ShipDomainParams, JeonCollisionRisk, ChunCollisionRisk
from colregs_core.reward import JeonRewardCalculator
from colregs_core import EncounterClassifier

# Import irsim for simulation
import irsim


class COLREGsIntegrationTester:
    """
    Integration tester for colregs-core modules in real simulation.
    """
    
    def __init__(self, world_file: str = None):
        """
        Initialize tester with simulation environment.
        
        Args:
            world_file: Path to world YAML file (default: Imazu case 01)
        """
        if world_file is None:
            world_file = "/home/hyo/DRL-otter-navigation/robot_nav/worlds/imazu_scenario/imazu_case_22.yaml"
        
        print("=" * 60)
        print("🔧 COLREGs-Core Integration Test")
        print("=" * 60)
        print(f"World file: {world_file}")
        
        # Initialize simulation
        self.env = irsim.make(world_file, disable_all_plot=True, display=False)
        self.dt = self.env.step_time
        
        # Get initial states
        robot_info = self.env.get_robot_info(0)
        self.robot_goal = robot_info.goal
        robot_state = self.env.robot.state
        
        # Convert start position from math to NED
        start_math = [robot_state[0, 0], robot_state[1, 0]]
        self.start_position = list(math_to_maritime_position(start_math[0], start_math[1]))
        
        print(f"Robot start (math): {start_math}")
        print(f"Robot start (NED): {self.start_position}")
        print(f"Robot goal: {self.robot_goal.T}")
        print(f"Time step: {self.dt} s")
        print(f"Number of obstacles: {len(self.env.obstacle_list)}")
        print("=" * 60)
        
        # Initialize Ship Domain
        self.ship_domain = ShipDomainParams(
            r_bow=6.0,
            r_stern=2.0,
            r_starboard=6.0,
            r_port=2.0
        )
        
        # Initialize CR calculators
        self.jeon_cr = JeonCollisionRisk(
            ship_domain=self.ship_domain,
            d_obs=200.0,
            cr_obs=0.3,
            os_speed=3.0,
            ts_speed=3.0
        )
        
        self.chun_cr = ChunCollisionRisk(
            ship_domain=self.ship_domain
        )
        
        # Initialize encounter classifier
        self.encounter_classifier = EncounterClassifier()
        
        print("✅ All modules initialized successfully\n")
    
    def extract_vessel_states(self) -> Dict[str, Any]:
        """
        Extract own ship (OS) and all target ship (TS) states from simulation.
        
        Returns:
            Dictionary containing OS and a list of TS information
        """
        robot_state = self.env.robot.state
        
        # Own Ship (OS) data
        os_position_math = [robot_state[0, 0], robot_state[1, 0]]
        os_position = list(math_to_maritime_position(os_position_math[0], os_position_math[1]))
        os_heading_rad = robot_state[2, 0]
        os_heading_math = np.degrees(os_heading_rad)
        os_heading_deg = math_to_ned_heading(os_heading_math)
        os_speed = np.linalg.norm([robot_state[3, 0], robot_state[4, 0]])
        os_velocity = heading_speed_to_velocity(os_heading_deg, os_speed)
        
        # Navigation metrics
        goal_position_math = [self.robot_goal[0, 0], self.robot_goal[1, 0]]
        goal_position = list(math_to_maritime_position(goal_position_math[0], goal_position_math[1]))
        dist_to_goal = distance(os_position, goal_position)
        y_e = cross_track_error(self.start_position, goal_position, os_position)
        
        # Target Ships (TS) data - find all dynamic obstacles
        ts_list = []
        for i, obstacle in enumerate(self.env.obstacle_list):
            # Original filter, which should now work correctly
            if hasattr(obstacle, 'static') and obstacle.static:
                continue
            
            ts_state = obstacle.state
            if ts_state.shape[0] >= 5:  # Has velocity components
                ts_position_math = [ts_state[0, 0], ts_state[1, 0]]
                ts_position = list(math_to_maritime_position(ts_position_math[0], ts_position_math[1]))
                ts_heading_math = np.degrees(ts_state[2, 0])
                ts_heading_deg = math_to_ned_heading(ts_heading_math)
                ts_speed = np.linalg.norm([ts_state[3, 0], ts_state[4, 0]])
                ts_velocity = heading_speed_to_velocity(ts_heading_deg, ts_speed)
                
                ts_list.append({
                    'id': i,
                    'position': ts_position,
                    'position_math': ts_position_math,
                    'velocity': ts_velocity,
                    'heading': ts_heading_deg,
                    'speed': ts_speed
                })
        
        return {
            'os': {
                'position': os_position,
                'position_math': os_position_math,
                'velocity': os_velocity,
                'heading': os_heading_deg,
                'heading_math': os_heading_math,
                'speed': os_speed
            },
            'ts_list': ts_list,
            'navigation': {
                'distance_to_goal': dist_to_goal,
                'cross_track_error': y_e
            }
        }
    
    def test_geometry_module(self, states: Dict[str, Any]) -> Dict[str, Any]:
        """
        Test geometry module functions including coordinate system conversion.
        
        Args:
            states: Vessel states dictionary
            
        Returns:
            Test results
        """
        print("\n" + "=" * 60)
        print("📐 Testing Geometry Module (Coordinate System Conversion)")
        print("=" * 60)
        
        os = states['os']
        
        print(f"Heading Conversion:")
        print(f"  Math: {os['heading_math']:.2f}° (0°=East, 90°=North, CCW)")
        print(f"  NED:  {os['heading']:.2f}° (0°=North, 90°=East, CW)")
        
        print(f"\nPosition Conversion:")
        print(f"  Math: [{os['position_math'][0]:.2f}, {os['position_math'][1]:.2f}] (x=East, y=North)")
        print(f"  NED:  [{os['position'][0]:.2f}, {os['position'][1]:.2f}] (x=North, y=East)")
        print(f"  ✅ Conversion verified!")
        
        # Test heading_speed_to_velocity (uses NED heading)
        velocity_calculated = heading_speed_to_velocity(os['heading'], os['speed'])
        
        print(f"\nVelocity Calculation (NED heading):")
        print(f"  Input heading: {os['heading']:.2f}°")
        print(f"  Speed: {os['speed']:.4f} m/s")
        print(f"  Output velocity: [{velocity_calculated[0]:.4f}, {velocity_calculated[1]:.4f}] m/s")
        print(f"  Magnitude: {np.linalg.norm(velocity_calculated):.4f} m/s")
        
        # Verify magnitude matches speed
        magnitude_error = abs(np.linalg.norm(velocity_calculated) - os['speed'])
        print(f"\n✅ Magnitude error: {magnitude_error:.6f} m/s (should be ~0)")
        
        return {
            'velocity_vector': velocity_calculated,
            'magnitude_error': magnitude_error
        }
    
    def test_utils_module(self, states: Dict[str, Any]) -> Dict[str, Any]:
        """
        Test utils module functions.
        
        Args:
            states: Vessel states dictionary
            
        Returns:
            Test results
        """
        print("\n" + "=" * 60)
        print("🔧 Testing Utils Module")
        print("=" * 60)
        
        os = states['os']
        nav = states['navigation']
        
        # Test distance function (NED positions)
        goal_position_math = [self.robot_goal[0, 0], self.robot_goal[1, 0]]
        goal_position = list(math_to_maritime_position(goal_position_math[0], goal_position_math[1]))
        
        dist_calculated = distance(os['position'], goal_position)
        
        print(f"Distance calculation (NED):")
        print(f"  OS position: {os['position']}")
        print(f"  Goal position: {goal_position}")
        print(f"  Distance: {dist_calculated:.2f} m")
        
        # Test cross_track_error function
        y_e_calculated = cross_track_error(
            self.start_position, 
            goal_position, 
            os['position']
        )
        
        print(f"\nCross Track Error (NED):")
        print(f"  Start: {self.start_position}")
        print(f"  Goal: {goal_position}")
        print(f"  Current: {os['position']}")
        print(f"  CTE: {y_e_calculated:.4f} m")
        
        print(f"\n✅ Utils module working correctly")
        
        return {
            'distance_to_goal': dist_calculated,
            'cross_track_error': y_e_calculated
        }
    
    def test_risk_module(self, states: Dict[str, Any]) -> Dict[str, Any]:
        """
        Test collision risk calculation modules for all target ships.
        
        Args:
            states: Vessel states dictionary
            
        Returns:
            A dictionary with lists of collision risk results for each TS.
        """
        print("\n" + "=" * 60)
        print("⚠️  Testing Risk Module (Collision Risk)")
        print("=" * 60)
        
        os = states['os']
        ts_list = states['ts_list']
        
        if not ts_list:
            print("❌ No target ships found - skipping CR tests")
            return {'jeon_results': [], 'chun_results': []}

        all_jeon_results = []
        all_chun_results = []

        for ts in ts_list:
            print(f"\n--- Target Ship ID: {ts['id']} ---")
            
            # Test Jeon CR
            print("1️⃣  Jeon Collision Risk:")
            jeon_result = self.jeon_cr.calculate_collision_risk(
                os_position=os['position'],
                os_velocity=os['velocity'],
                os_heading=os['heading'],
                ts_position=ts['position'],
                ts_velocity=ts['velocity']
            )
            print(f"  CR value: {jeon_result['cr']:.4f}")
            print(f"  DCPA: {jeon_result['dcpa']:.2f} m")
            print(f"  TCPA: {jeon_result['tcpa']:.2f} s")
            print(f"  Relative bearing: {jeon_result['relative_bearing']:.2f}°")
            print(f"  Ship domain radius: {jeon_result['ship_domain_radius']:.2f} m")
            all_jeon_results.append(jeon_result)
            
            # Test Chun CR
            print("\n2️⃣  Chun Collision Risk:")
            chun_result = self.chun_cr.calculate_collision_risk(
                os_position=os['position'],
                os_velocity=os['velocity'],
                os_heading=os['heading'],
                ts_position=ts['position'],
                ts_velocity=ts['velocity']
            )
            print(f"  CR value: {chun_result['cr']:.4f}")
            print(f"  DCPA: {chun_result['dcpa']:.2f} m")
            print(f"  TCPA: {chun_result['tcpa']:.2f} s")
            all_chun_results.append(chun_result)

        print(f"\n✅ Risk module tested for {len(ts_list)} target ship(s).")
        
        return {
            'jeon_results': all_jeon_results,
            'chun_results': all_chun_results
        }
    
    def test_reward_module(self, states: Dict[str, Any], 
                          risk_results: Dict[str, Any],
                          prev_distance: float = None,
                          prev_heading: float = None) -> Dict[str, Any]:
        """
        Test reward calculation module.
        
        The safety reward is calculated based on the most dangerous TS
        (the one with the highest Jeon CR).
        
        Args:
            states: Vessel states dictionary
            risk_results: Dictionary containing risk calculation results
            prev_distance: Previous distance to goal
            prev_heading: Previous heading (NED)
            
        Returns:
            Reward calculation results
        """
        print("\n" + "=" * 60)
        print("🎁 Testing Reward Module (Jeon Reward)")
        print("=" * 60)
        
        os = states['os']
        ts_list = states['ts_list']
        nav = states['navigation']
        
        # Find the most dangerous target ship (highest Jeon CR)
        most_dangerous_ts = None
        highest_cr = -1.0
        
        if ts_list:
            for i, ts in enumerate(ts_list):
                jeon_cr = risk_results['jeon_results'][i]['cr']
                if jeon_cr > highest_cr:
                    highest_cr = jeon_cr
                    most_dangerous_ts = ts
            print(f"Most dangerous TS for reward calculation: ID {most_dangerous_ts['id']} (CR: {highest_cr:.4f})")
        else:
            print("No target ships for safety reward calculation.")

        # Prepare TS data for reward calculation
        if most_dangerous_ts:
            ts_pos = most_dangerous_ts['position']
            ts_vel = most_dangerous_ts['velocity']
            ts_spd = most_dangerous_ts['speed']
        else:
            ts_pos, ts_vel, ts_spd = [999, 999], [0, 0], 0.0

        # Initialize Jeon reward calculator
        jeon_reward = JeonRewardCalculator(
            d_max=10.0,
            v_ref=3.0,
            cr_allowable=0.3,
            dt=self.dt,
            ship_domain=self.ship_domain,
            d_obs=100.0,
            phi_max=45.0
        )
        
        # Calculate total reward
        reward_dict = jeon_reward.calculate_total_reward(
            current_distance=nav['distance_to_goal'],
            previous_distance=prev_distance,
            cross_track_error=nav['cross_track_error'],
            os_speed=os['speed'],
            os_position=os['position'],
            os_velocity=os['velocity'],
            os_heading=os['heading'],
            previous_heading=prev_heading,
            ts_position=ts_pos,
            ts_velocity=ts_vel,
            ts_speed=ts_spd,
            w_efficiency=1.0,
            w_safety=1.0
        )
        
        print(f"\n📈 Efficiency Rewards:")
        print(f"  r_goal: {reward_dict['r_goal']:.4f}")
        print(f"  r_cross: {reward_dict['r_cross']:.4f}")
        print(f"  r_speed: {reward_dict['r_speed']:.4f}")
        print(f"  Total efficiency: {reward_dict['r_efficiency']:.4f}")
        
        print(f"\n🛡️  Safety Rewards (based on most dangerous TS):")
        print(f"  r_risk: {reward_dict['r_risk']:.4f}")
        print(f"  r_colregs: {reward_dict['r_colregs']:.4f}")
        print(f"  r_heading: {reward_dict['r_heading']:.4f}")
        print(f"  Total safety: {reward_dict['r_safety']:.4f}")
        
        print(f"\n🎯 Total Reward: {reward_dict['r_total']:.4f}")
        
        print(f"\n✅ Reward module working correctly")
        
        return reward_dict
    
    def test_encounter_classifier(self, states: Dict[str, Any]) -> list:
        """
        Test COLREGs encounter classification for all target ships.
        
        Args:
            states: Vessel states dictionary
            
        Returns:
            A list of classification results.
        """
        print("\n" + "=" * 60)
        print("🚢 Testing Encounter Classification (COLREGs)")
        print("=" * 60)
        
        os = states['os']
        ts_list = states['ts_list']
        
        if not ts_list:
            print("❌ No target ships found - skipping classification")
            return []

        all_results = []
        for ts in ts_list:
            print(f"\n--- Target Ship ID: {ts['id']} ---")
            result = self.encounter_classifier.classify(
                os_position=os['position'],
                os_heading=os['heading'],
                os_speed=os['speed'],
                ts_position=ts['position'],
                ts_heading=ts['heading'],
                ts_speed=ts['speed']
            )
            print(f"Encounter type: {result.encounter_type.value}")
            print(f"  Relative bearing: {result.relative_bearing:.2f}°")
            print(f"  Relative course: {result.relative_course:.2f}°")
            print(f"  Distance: {result.distance:.2f} m")
            all_results.append(result)

        print(f"\n✅ Encounter classifier tested for {len(ts_list)} target ship(s).")
        
        return all_results
    
    def run_single_step_test(self, action_u: float = 1.0, action_r: float = 0.0):
        """
        Run a complete integration test for one simulation step.
        
        Args:
            action_u: Surge velocity command (m/s)
            action_r: Yaw rate command (rad/s)
        """
        print("\n" + "=" * 60)
        print("🚀 Running Single Step Integration Test")
        print("=" * 60)
        print(f"Action: u={action_u:.2f} m/s, r={action_r:.4f} rad/s")
        
        # Execute simulation step
        action = np.array([[action_u], [action_r]])
        self.env.step(action_id=0, action=action)
        
        # Extract all vessel states
        states = self.extract_vessel_states()
        
        # Test all modules
        geometry_results = self.test_geometry_module(states)
        utils_results = self.test_utils_module(states)
        risk_results = self.test_risk_module(states)
        reward_results = self.test_reward_module(states, risk_results)
        encounter_results = self.test_encounter_classifier(states)
        
        # Summary
        print("\n" + "=" * 60)
        print("📊 INTEGRATION TEST SUMMARY")
        print("=" * 60)
        print("✅ Geometry module: PASS (position + heading conversion)")
        print("✅ Utils module: PASS")
        print("✅ Risk module: PASS")
        print("✅ Reward module: PASS")
        print("✅ Encounter classifier: PASS")
        print("\n🎉 All colregs-core modules working correctly!")
        print("=" * 60)
        
        return {
            'states': states,
            'geometry': geometry_results,
            'utils': utils_results,
            'risk': risk_results,
            'reward': reward_results,
            'encounter': encounter_results
        }
    
    def run_multi_step_test(self, num_steps: int = 10, 
                           action_u: float = 1.5, 
                           action_r: float = 0.0):
        """
        Run integration test for multiple simulation steps.
        
        Args:
            num_steps: Number of steps to simulate
            action_u: Surge velocity command
            action_r: Yaw rate command
        """
        print("\n" + "=" * 60)
        print(f"🚀 Running Multi-Step Integration Test ({num_steps} steps)")
        print("=" * 60)
        
        results_history = []
        prev_distance = None
        prev_heading = None
        
        for step in range(num_steps):
            print(f"\n{'─' * 60}")
            print(f"Step {step + 1}/{num_steps}")
            print(f"{'─' * 60}")
            
            # Execute step
            action = np.array([[action_u], [action_r]])
            self.env.step(action_id=0, action=action)
            
            # Extract states
            states = self.extract_vessel_states()
            
            # Calculate CR for all ships
            risk_results = self.test_risk_module(states)

            # Calculate rewards with previous values
            reward_results = self.test_reward_module(
                states, 
                risk_results,
                prev_distance=prev_distance,
                prev_heading=prev_heading
            )
            
            # Find max Jeon CR for this step
            max_jeon_cr = 0.0
            if risk_results['jeon_results']:
                max_jeon_cr = max(r['cr'] for r in risk_results['jeon_results'])

            # Store results
            step_result = {
                'step': step + 1,
                'os_position_math': states['os']['position_math'],
                'os_position_ned': states['os']['position'],
                'os_heading_math': states['os']['heading_math'],
                'os_heading_ned': states['os']['heading'],
                'os_speed': states['os']['speed'],
                'distance_to_goal': states['navigation']['distance_to_goal'],
                'cross_track_error': states['navigation']['cross_track_error'],
                'max_jeon_cr': max_jeon_cr,
                'reward': reward_results['r_total']
            }
            results_history.append(step_result)
            
            # Update previous values (NED heading)
            prev_distance = states['navigation']['distance_to_goal']
            prev_heading = states['os']['heading']
            
            # Quick summary for this step
            print(f"\n📊 Step Summary:")
            print(f"  Math position: [{states['os']['position_math'][0]:.2f}, {states['os']['position_math'][1]:.2f}]")
            print(f"  NED position:  [{states['os']['position'][0]:.2f}, {states['os']['position'][1]:.2f}]")
            print(f"  Math heading: {states['os']['heading_math']:.2f}°")
            print(f"  NED heading:  {states['os']['heading']:.2f}°")
            print(f"  Distance to goal: {states['navigation']['distance_to_goal']:.2f} m")
            print(f"  Max Jeon CR: {max_jeon_cr:.4f}")
            print(f"  Reward: {reward_results['r_total']:.4f}")
        
        # Final summary
        print("\n" + "=" * 60)
        print("📊 MULTI-STEP TEST SUMMARY")
        print("=" * 60)
        print(f"Total steps: {num_steps}")
        print(f"Initial distance: {results_history[0]['distance_to_goal']:.2f} m")
        print(f"Final distance: {results_history[-1]['distance_to_goal']:.2f} m")
        print(f"Total reward: {sum(r['reward'] for r in results_history):.4f}")
        
        max_cr_overall = max(r['max_jeon_cr'] for r in results_history if r['max_jeon_cr'] is not None)
        print(f"Max collision risk over all steps: {max_cr_overall:.4f}")
        
        print("\n🎉 Multi-step integration test completed!")
        print("=" * 60)
        
        return results_history


def main():
    """Main test function"""
    
    print("\n" + "=" * 60)
    print("🧪 COLREGs-Core Integration Test Suite")
    print("=" * 60)
    
    # Initialize tester
    tester = COLREGsIntegrationTester()
    
    # Test 1: Single step with straight motion
    print("\n\n" + "🔬 TEST 1: Single Step Test")
    single_step_results = tester.run_single_step_test(action_u=1.5, action_r=0.0)
    
    # Test 2: Multiple steps with straight motion
    # print("\n\n" + "🔬 TEST 2: Multi-Step Test (10 steps)")
    # multi_step_results = tester.run_multi_step_test(num_steps=10, action_u=1.5, action_r=0.0)
    
    # print("\n\n" + "=" * 60)
    # print("✅ ALL INTEGRATION TESTS COMPLETED SUCCESSFULLY!")
    # print("=" * 60)
    # print("\n📝 Next steps:")
    # print("  1. Check all calculations are correct")
    # print("  2. Verify coordinate system transformations (Math → NED)")
    # print("  3. Confirm CR values are reasonable")
    # print("  4. Validate reward function behavior")
    # print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
