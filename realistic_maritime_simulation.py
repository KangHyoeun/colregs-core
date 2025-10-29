
import numpy as np
import time
from irsim.env import Env
from irsim.world import ObjectFactory
from colregs_core.encounter.classifier import EncounterClassifier
from colregs_core.risk.risk_matrix import RiskAssessment

class RealisticMaritimeSimulation(Env):
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
        
        # Sensor-based detection storage
        self.detected_objects = []
        self.sensor_range = 70.0  # LiDAR range in meters
        self.sensor_fov = np.pi  # 180 degrees field of view
        
        print("\n🌊 REALISTIC MARITIME SIMULATION INITIALIZED")
        print("=" * 60)
        print("🚢 Otter USV with full 6-DOF dynamics")
        print("📡 LiDAR 2D sensor-based detection (NO perfect knowledge!)")
        print("📊 COLREGs encounter classification from sensor data")
        print("⚠️  Real-time collision risk assessment")
        print("=" * 60)
    
    def get_sensor_data(self, robot_id=0):
        """
        Get LiDAR sensor data for obstacle detection
        
        Args:
            robot_id: Index of the own ship robot
            
        Returns:
            dict: Sensor data including detected objects
        """
        if not hasattr(self, 'robot_list') or len(self.robot_list) <= robot_id:
            return None
        
        robot = self.robot_list[robot_id]
        
        # Get robot's LiDAR sensor
        if not hasattr(robot, 'sensors') or not robot.sensors:
            return None
        
        lidar_sensor = None
        for sensor in robot.sensors:
            if hasattr(sensor, 'sensor_type') and sensor.sensor_type == 'lidar2d':
                lidar_sensor = sensor
                break
        
        if lidar_sensor is None:
            return None
        
        # Get sensor scan data
        scan_data = lidar_sensor.get_scan()
        point_cloud = lidar_sensor.get_points()
        
        # Get robot's own state
        os_position = (float(robot.state[0, 0]), float(robot.state[1, 0]))
        os_heading = float(np.degrees(robot.state[2, 0]))
        
        # Get robot's velocity
        if hasattr(robot, 'get_velocities'):
            velocities = robot.get_velocities()
            os_velocity = (velocities['u'], velocities['v'])
            os_speed = np.sqrt(velocities['u']**2 + velocities['v']**2)
        else:
            os_velocity_array = robot.velocity.flatten()
            os_velocity = (float(os_velocity_array[0]), float(os_velocity_array[1]))
            os_speed = np.linalg.norm(os_velocity_array[:2])
        
        return {
            'robot_position': os_position,
            'robot_heading': os_heading,
            'robot_velocity': os_velocity,
            'robot_speed': os_speed,
            'scan_data': scan_data,
            'point_cloud': point_cloud,
            'sensor_range': self.sensor_range,
            'sensor_fov': self.sensor_fov
        }
    
    def detect_objects_from_sensor(self, sensor_data):
        """
        Detect and classify objects from LiDAR sensor data
        
        Args:
            sensor_data: Sensor data from get_sensor_data()
            
        Returns:
            list: Detected objects with positions and properties
        """
        if sensor_data is None:
            return []
        
        detected_objects = []
        scan_data = sensor_data['scan_data']
        ranges = scan_data['ranges']
        angles = np.linspace(scan_data['angle_min'], scan_data['angle_max'], len(ranges))
        
        # Find detections (ranges less than max range)
        detections = ranges < scan_data['range_max'] - 0.1
        
        if np.sum(detections) == 0:
            return []
        
        # Get detection ranges and angles
        detection_ranges = ranges[detections]
        detection_angles = angles[detections]
        
        # Convert to world coordinates
        robot_x, robot_y = sensor_data['robot_position']
        robot_heading = sensor_data['robot_heading']
        
        # Transform sensor points to world coordinates
        cos_h = np.cos(np.radians(robot_heading))
        sin_h = np.sin(np.radians(robot_heading))
        
        world_points = []
        for i in range(len(detection_ranges)):
            range_val = detection_ranges[i]
            angle = detection_angles[i]
            
            # Convert to local coordinates
            x_local = range_val * np.cos(angle)
            y_local = range_val * np.sin(angle)
            
            # Transform to world coordinates
            x_world = robot_x + x_local * cos_h - y_local * sin_h
            y_world = robot_y + x_local * sin_h + y_local * cos_h
            
            world_points.append([x_world, y_world])
        
        if not world_points:
            return []
        
        world_points = np.array(world_points).T
        
        # Cluster nearby points to identify objects
        clusters = self._cluster_points(world_points)
        
        # Create detected objects
        for i, cluster in enumerate(clusters):
            if cluster.shape[1] < 2:  # Need at least 2 points to form a meaningful object
                continue
            
            # Calculate object properties
            center_x = np.mean(cluster[0, :])
            center_y = np.mean(cluster[1, :])
            center = np.array([center_x, center_y])
            
            # Calculate size as maximum distance from center
            distances = []
            for j in range(cluster.shape[1]):
                point = cluster[:, j]
                dist = np.linalg.norm(point - center)
                distances.append(dist)
            size = np.max(distances)
            
            # Estimate object velocity (simplified - would need tracking in real implementation)
            velocity = (0.0, 0.0)  # Placeholder - would need temporal tracking
            
            detected_object = {
                'id': i,
                'position': (center_x, center_y),
                'size': size,
                'velocity': velocity,
                'heading': 0.0,  # Placeholder - would need orientation estimation
                'speed': 0.0,    # Placeholder - would need velocity estimation
                'detection_confidence': min(1.0, len(cluster) / 5.0),  # Based on point count
                'distance': np.sqrt((center_x - robot_x)**2 + (center_y - robot_y)**2)
            }
            
            detected_objects.append(detected_object)
        
        return detected_objects
    
    def _cluster_points(self, points, max_distance=2.0):
        """
        Simple clustering algorithm for point cloud data
        
        Args:
            points: Point cloud data (2xN array)
            max_distance: Maximum distance for points to be in same cluster
            
        Returns:
            list: List of clusters, each containing points
        """
        if points.shape[1] == 0:
            return []
        
        clusters = []
        used_points = set()
        
        for i in range(points.shape[1]):
            if i in used_points:
                continue
            
            cluster = [points[:, i]]
            used_points.add(i)
            
            # Find nearby points
            for j in range(i + 1, points.shape[1]):
                if j in used_points:
                    continue
                
                distance = np.linalg.norm(points[:, i] - points[:, j])
                if distance < max_distance:
                    cluster.append(points[:, j])
                    used_points.add(j)
            
            if len(cluster) > 0:
                clusters.append(np.array(cluster).T)
        
        return clusters
    
    def analyze_encounters_from_sensor(self, robot_id=0):
        """
        Analyze encounters using only sensor-detected objects
        
        Args:
            robot_id: Index of the own ship robot
            
        Returns:
            list: Encounter analysis results
        """
        sensor_data = self.get_sensor_data(robot_id)
        if sensor_data is None:
            return []
        
        detected_objects = self.detect_objects_from_sensor(sensor_data)
        if not detected_objects:
            return []
        
        encounters = []
        
        for obj in detected_objects:
            # Get own ship state
            os_position = sensor_data['robot_position']
            os_heading = sensor_data['robot_heading']
            os_speed = sensor_data['robot_speed']
            
            # Get target ship state (from sensor detection)
            ts_position = obj['position']
            ts_heading = obj['heading']
            ts_speed = obj['speed']
            ts_velocity = obj['velocity']
            
            # Classify encounter
            situation = self.encounter_classifier.classify(
                os_position=os_position,
                os_heading=os_heading,
                os_speed=os_speed,
                ts_position=ts_position,
                ts_heading=ts_heading,
                ts_speed=ts_speed
            )
            
            # Assess collision risk
            risk = self.risk_assessor.assess(
                os_position=os_position,
                os_velocity=sensor_data['robot_velocity'],
                ts_position=ts_position,
                ts_velocity=ts_velocity
            )
            
            encounter_info = {
                'target_id': obj['id'],
                'detection_confidence': obj['detection_confidence'],
                'distance': obj['distance'],
                'encounter_type': situation.encounter_type,
                'relative_bearing': float(situation.relative_bearing),
                'sensor_distance': float(situation.distance),
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
        Identify the most dangerous encounter from sensor data
        
        Returns:
            dict or None: Most dangerous encounter info
        """
        encounters = self.analyze_encounters_from_sensor(robot_id)
        
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
        Print current maritime status based on sensor data
        """
        print(f"\n🌊 MARITIME STATUS - Time: {self.simulation_time:.1f}s")
        print("-" * 60)
        
        if not encounters:
            print("📡 No vessels detected by LiDAR sensor")
            return
        
        print(f"📡 LiDAR Detection: {len(encounters)} vessels detected")
        
        for enc in encounters:
            confidence = enc['detection_confidence']
            status_icon = "⚠️" if enc['requires_action'] else "✓"
            print(f"{status_icon} Vessel {enc['target_id']}: "
                  f"{enc['encounter_type'].value.upper()} - "
                  f"{enc['risk_level'].name} "
                  f"(D={enc['sensor_distance']:.0f}m, DCPA={enc['dcpa']:.0f}m, TCPA={enc['tcpa']:.0f}s, Conf={confidence:.2f})")
        
        # Show most dangerous encounter
        most_dangerous = self.get_most_dangerous_encounter()
        if most_dangerous:
            print(f"\n🚨 MOST DANGEROUS: Vessel {most_dangerous['target_id']}")
            print(f"   Risk: {most_dangerous['risk_level'].name}")
            print(f"   Encounter: {most_dangerous['encounter_type'].value}")
            print(f"   Detection Confidence: {most_dangerous['detection_confidence']:.2f}")
            print(f"   Action: {most_dangerous['colregs_action'][:80]}...")
    
    def step(self, action=None):
        """
        Enhanced step method with sensor-based COLREGs analysis
        """
        # Update simulation time
        self.simulation_time += self.step_time
        
        # Call parent step
        super().step(action)
        
        # Analyze encounters every 10 steps (0.5 seconds at 20Hz)
        if int(self.simulation_time * 20) % 10 == 0:
            encounters = self.analyze_encounters_from_sensor()
            self.encounter_history.append({
                'time': self.simulation_time,
                'encounters': encounters
            })
            
            # Print status every 5 seconds
            if int(self.simulation_time) % 5 == 0:
                self.print_maritime_status(encounters)


def run_realistic_maritime_simulation():
    """
    Run the realistic maritime simulation with sensor-based detection
    """
    print("🚢 STARTING REALISTIC MARITIME SIMULATION")
    print("=" * 60)
    print("📡 SENSOR-BASED DETECTION - NO PERFECT KNOWLEDGE!")
    print("=" * 60)
    
    # Create maritime environment
    env = RealisticMaritimeSimulation(
        world_name='otter_world.yaml',
        display=True,
        save_ani=False
    )
    
    print(f"\n📊 Simulation Parameters:")
    print(f"   World Size: {env._world.width}m x {env._world.height}m")
    print(f"   Step Time: {env.step_time}s")
    print(f"   Own Ship: Otter USV with full dynamics")
    print(f"   LiDAR Range: {env.sensor_range}m")
    print(f"   LiDAR FOV: {np.degrees(env.sensor_fov):.0f}°")
    print(f"   Target Ships: {len(env.obstacle_list)} vessels")
    
    # Run simulation
    max_steps = 2000  # 100 seconds at 20Hz
    step_count = 0
    
    print(f"\n🌊 Starting realistic simulation for {max_steps} steps...")
    
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
    print(f"\n📈 REALISTIC SIMULATION SUMMARY")
    print("=" * 50)
    print(f"Total Steps: {step_count}")
    print(f"Simulation Time: {env.simulation_time:.1f}s")
    print(f"Encounter Analyses: {len(env.encounter_history)}")
    print(f"Sensor-Based Detection: LiDAR 2D with {env.sensor_range}m range")
    
    # Show final encounters
    final_encounters = env.analyze_encounters_from_sensor()
    if final_encounters:
        print(f"\n🌊 Final Maritime Status:")
        env.print_maritime_status(final_encounters)
    
    # Cleanup
    env.end()
    print(f"\n🏁 Realistic maritime simulation completed!")


if __name__ == "__main__":
    run_realistic_maritime_simulation()
