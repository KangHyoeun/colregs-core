import numpy as np
import time
import matplotlib.pyplot as plt

try:
    import irsim
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


class MaritimeSimulation(EnvBase):
    """
    Enhanced maritime simulation with COLREGs integration
    """
    
    def __init__(self, world_name='otter_world.yaml', **kwargs):
        super().__init__(world_name, **kwargs)
        
        # COLREGs modules
        self.encounter_classifier = EncounterClassifier(
            safe_distance=2000.0  # 2km maritime safe distance
        )
        self.risk_assessor = RiskAssessment(
            dcpa_critical=100.0,   # 100m critical distance
            dcpa_high=300.0,       # 300m high risk
            dcpa_medium=500.0,      # 500m medium risk
            dcpa_low=1000.0,        # 1km low risk
            tcpa_critical=300.0,    # 5 minutes critical
            tcpa_high=600.0,        # 10 minutes high
            tcpa_medium=1200.0,     # 20 minutes medium
            tcpa_low=1800.0         # 30 minutes low
        )
        
        # Maritime simulation parameters
        self.simulation_time = 0.0
        self.encounter_history = []
        self.risk_history = []
        
        print("\n🌊 MARITIME SIMULATION INITIALIZED")
        print("=" * 50)
        print("🚢 Otter USV with full 6-DOF dynamics")
        print("📊 COLREGs encounter classification")
        print("⚠️  Real-time collision risk assessment")
        print("=" * 50)
    
    def get_maritime_state(self, robot_id=0):
        """
        Get current maritime state for COLREGs analysis
        
        Args:
            robot_id: Index of the own ship robot
            
        Returns:
            dict: Maritime state information
        """
        if not hasattr(self, 'robot_list') or len(self.robot_list) <= robot_id:
            return None
        
        robot = self.robot_list[robot_id]
        
        # Get position and heading
        os_position = (float(robot.state[0, 0]), float(robot.state[1, 0]))
        os_heading = float(np.degrees(robot.state[2, 0]))  # Convert to degrees
        
        # Get velocity (body-fixed frame)
        if hasattr(robot, 'get_velocities'):
            velocities = robot.get_velocities()
            os_velocity = (velocities['u'], velocities['v'])
            os_speed = np.sqrt(velocities['u']**2 + velocities['v']**2)
        else:
            # Fallback to state-based velocity
            os_velocity_array = robot.velocity.flatten()
            os_velocity = (float(os_velocity_array[0]), float(os_velocity_array[1]))
            os_speed = np.linalg.norm(os_velocity_array[:2])
        
        return {
            'position': os_position,
            'heading': os_heading,
            'velocity': os_velocity,
            'speed': os_speed,
            'robot': robot
        }
    
    def analyze_encounters(self, robot_id=0):
        """
        Analyze all encounters using COLREGs
        
        Args:
            robot_id: Index of the own ship robot
            
        Returns:
            list: Encounter analysis results
        """
        own_ship = self.get_maritime_state(robot_id)
        if own_ship is None:
            return []
        
        encounters = []
        
        for i, obstacle in enumerate(self.obstacle_list):
            # Get target ship state
            ts_position = (float(obstacle.state[0, 0]), float(obstacle.state[1, 0]))
            ts_heading = float(np.degrees(obstacle.state[2, 0]))
            
            # Get target ship velocity
            if hasattr(obstacle, 'get_velocities'):
                ts_velocities = obstacle.get_velocities()
                ts_velocity = (ts_velocities['u'], ts_velocities['v'])
                ts_speed = np.sqrt(ts_velocities['u']**2 + ts_velocities['v']**2)
            else:
                ts_velocity_array = obstacle.velocity.flatten()
                ts_velocity = (float(ts_velocity_array[0]), float(ts_velocity_array[1]))
                ts_speed = np.linalg.norm(ts_velocity_array[:2])
            
            # Classify encounter
            situation = self.encounter_classifier.classify(
                os_position=own_ship['position'],
                os_heading=own_ship['heading'],
                os_speed=own_ship['speed'],
                ts_position=ts_position,
                ts_heading=ts_heading,
                ts_speed=ts_speed
            )
            
            # Assess collision risk
            risk = self.risk_assessor.assess(
                os_position=own_ship['position'],
                os_velocity=own_ship['velocity'],
                ts_position=ts_position,
                ts_velocity=ts_velocity
            )
            
            encounter_info = {
                'target_id': i,
                'encounter_type': situation.encounter_type,
                'relative_bearing': float(situation.relative_bearing),
                'distance': float(situation.distance),
                'risk_level': risk.risk_level,
                'dcpa': float(risk.dcpa),
                'tcpa': float(risk.tcpa),
                'requires_action': risk.requires_action,
                'colregs_action': self.encounter_classifier.get_action_requirement(
                    situation.encounter_type
                ),
                'tactical_action': self.risk_assessor.get_recommended_action(risk)
            }
            
            encounters.append(encounter_info)
        
        return encounters
    
    def get_most_dangerous_encounter(self, robot_id=0):
        """
        Identify the most dangerous encounter
        
        Returns:
            dict or None: Most dangerous encounter info
        """
        encounters = self.analyze_encounters(robot_id)
        
        if not encounters:
            return None
        
        # Filter encounters requiring action
        dangerous_encounters = [
            enc for enc in encounters 
            if enc['requires_action']
        ]
        
        if not dangerous_encounters:
            return None
        
        # Sort by risk level and TCPA
        dangerous_encounters.sort(
            key=lambda x: (-x['risk_level'].value, x['tcpa'])
        )
        
        return dangerous_encounters[0]
    
    def print_maritime_status(self, encounters):
        """
        Print current maritime status
        """
        print(f"\n🌊 MARITIME STATUS - Time: {self.simulation_time:.1f}s")
        print("-" * 60)
        
        if not encounters:
            print("✓ No vessels detected")
            return
        
        for enc in encounters:
            status_icon = "⚠️" if enc['requires_action'] else "✓"
            print(f"{status_icon} Vessel {enc['target_id']}: "
                  f"{enc['encounter_type'].value.upper()} - "
                  f"{enc['risk_level'].name} "
                  f"(D={enc['distance']:.0f}m, DCPA={enc['dcpa']:.0f}m, TCPA={enc['tcpa']:.0f}s)")
        
        # Show most dangerous encounter
        most_dangerous = self.get_most_dangerous_encounter()
        if most_dangerous:
            print(f"\n🚨 MOST DANGEROUS: Vessel {most_dangerous['target_id']}")
            print(f"   Risk: {most_dangerous['risk_level'].name}")
            print(f"   Encounter: {most_dangerous['encounter_type'].value}")
            print(f"   Action: {most_dangerous['colregs_action'][:80]}...")
    
    def step(self, action=None):
        """
        Enhanced step method with COLREGs analysis
        """
        # Update simulation time
        self.simulation_time += self.step_time
        
        # Call parent step
        super().step(action)
        
        # Analyze encounters every 10 steps (1 second)
        if int(self.simulation_time * 10) % 10 == 0:
            encounters = self.analyze_encounters()
            self.encounter_history.append({
                'time': self.simulation_time,
                'encounters': encounters
            })
            
            # Print status every 5 seconds
            if int(self.simulation_time) % 5 == 0:
                self.print_maritime_status(encounters)


def run_maritime_simulation():
    """
    Run the complete maritime simulation
    """
    print("🚢 STARTING MARITIME SIMULATION")
    print("=" * 60)
    
    # Create maritime environment
    env = MaritimeSimulation(
        world_name='otter_world.yaml',
        display=True,
        save_ani=False
    )
    
    print(f"\n📊 Simulation Parameters:")
    print(f"   World Size: {env._world.width}m x {env._world.height}m")
    print(f"   Step Time: {env.step_time}s")
    print(f"   Own Ship: Otter USV with full dynamics")
    print(f"   Target Ships: {len(env.obstacle_list)} vessels")
    
    # Run simulation
    max_steps = 2000  # 200 seconds
    step_count = 0
    
    print(f"\n🌊 Starting simulation for {max_steps} steps...")
    
    try:
        for step in range(max_steps):
            env.step()
            env.render(0.01)  # 100 FPS rendering
            
            step_count += 1
            
            # Check if simulation should end
            if env.done():
                print(f"\n✅ Simulation completed at step {step_count}")
                break
            
            # Emergency stop if too many steps
            if step_count >= max_steps:
                print(f"\n⏰ Simulation reached maximum steps ({max_steps})")
                break
                
    except KeyboardInterrupt:
        print(f"\n⏹️  Simulation stopped by user at step {step_count}")
    
    # Final analysis
    print(f"\n📈 SIMULATION SUMMARY")
    print("=" * 40)
    print(f"Total Steps: {step_count}")
    print(f"Simulation Time: {env.simulation_time:.1f}s")
    print(f"Encounter Analyses: {len(env.encounter_history)}")
    
    # Show final encounters
    final_encounters = env.analyze_encounters()
    if final_encounters:
        print(f"\n🌊 Final Maritime Status:")
        env.print_maritime_status(final_encounters)
    
    # Cleanup
    env.end()
    print(f"\n🏁 Maritime simulation completed!")


if __name__ == "__main__":
    run_maritime_simulation()
