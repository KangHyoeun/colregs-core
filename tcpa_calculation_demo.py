#!/usr/bin/env python3
"""
TCPA Calculation Demo for Head-on Collision Scenario

This script calculates the initial distance required for a TCPA of 30 seconds
in a head-on situation between two Otter USVs.

Scenario:
- Own ship: Course 0° (North), Speed 3.0 m/s
- Target ship: Course 180° (South), Speed 3.0 m/s  
- Both ships moving at maximum speed (max_u = 3.0 m/s)
- Target TCPA: 30 seconds

Calculation Method:
TCPA = Distance / Relative_Speed
Where Relative_Speed = |V_own + V_target| = |3.0 + 3.0| = 6.0 m/s

Therefore: Distance = TCPA × Relative_Speed = 30 × 6.0 = 180 meters
"""

import numpy as np
from colregs_core import (
    EncounterClassifier,
    RiskAssessment,
    heading_to_velocity,
    calculate_cpa_tcpa
)

def calculate_head_on_tcpa_distance():
    """
    Calculate the initial distance required for a specific TCPA in a head-on scenario.
    """
    print("=" * 70)
    print("TCPA CALCULATION FOR HEAD-ON COLLISION SCENARIO")
    print("=" * 70)
    
    # Scenario parameters
    target_tcpa = 30.0  # seconds
    own_speed = 3.0     # m/s (Otter USV max_u)
    target_speed = 3.0  # m/s (Otter USV max_u)
    
    # Head-on scenario: Own ship North (0°), Target ship South (180°)
    own_heading = 0.0   # North
    target_heading = 180.0  # South
    
    print(f"\n[SCENARIO PARAMETERS]")
    print(f"Own Ship:   Heading = {own_heading}° (North), Speed = {own_speed} m/s")
    print(f"Target Ship: Heading = {target_heading}° (South), Speed = {target_speed} m/s")
    print(f"Target TCPA: {target_tcpa} seconds")
    
    # Calculate relative speed (head-on collision)
    # In head-on: relative speed = |V_own + V_target|
    # Since they're moving towards each other: 3.0 + 3.0 = 6.0 m/s
    relative_speed = own_speed + target_speed
    print(f"\nRelative Speed: {own_speed} + {target_speed} = {relative_speed} m/s")
    
    # Calculate required initial distance
    # TCPA = Distance / Relative_Speed
    # Therefore: Distance = TCPA × Relative_Speed
    required_distance = target_tcpa * relative_speed
    
    print(f"\n[CALCULATION]")
    print(f"TCPA = Distance / Relative_Speed")
    print(f"{target_tcpa} = Distance / {relative_speed}")
    print(f"Distance = {target_tcpa} × {relative_speed} = {required_distance} meters")
    
    return required_distance, own_heading, target_heading, own_speed, target_speed

def verify_with_colregs_core(required_distance, own_heading, target_heading, own_speed, target_speed):
    """
    Verify the calculation using COLREGs Core functions.
    """
    print(f"\n[VERIFICATION USING COLREGs CORE]")
    print("-" * 50)
    
    # Set up positions for head-on scenario
    # Own ship at origin, target ship at calculated distance
    own_position = (0.0, 0.0)
    target_position = (0.0, required_distance)  # Target ship North of own ship
    
    print(f"Own Ship Position: {own_position}")
    print(f"Target Ship Position: {target_position}")
    print(f"Initial Distance: {required_distance:.1f} m")
    
    # Convert headings to velocities
    own_velocity = heading_to_velocity(own_heading, own_speed)
    target_velocity = heading_to_velocity(target_heading, target_speed)
    
    print(f"\nOwn Ship Velocity: {own_velocity} m/s")
    print(f"Target Ship Velocity: {target_velocity} m/s")
    
    # Use COLREGs Core to calculate CPA/TCPA
    dcpa, tcpa = calculate_cpa_tcpa(
        os_position=own_position,
        os_velocity=own_velocity,
        ts_position=target_position,
        ts_velocity=target_velocity
    )
    
    print(f"\n[COLREGs CORE RESULTS]")
    print(f"DCPA: {dcpa:.2f} m")
    print(f"TCPA: {tcpa:.2f} s")
    print(f"Expected TCPA: 30.0 s")
    print(f"Difference: {abs(tcpa - 30.0):.2f} s")
    
    # Verify encounter classification
    classifier = EncounterClassifier()
    situation = classifier.classify(
        os_position=own_position,
        os_heading=own_heading,
        os_speed=own_speed,
        ts_position=target_position,
        ts_heading=target_heading,
        ts_speed=target_speed
    )
    
    print(f"\n[ENCOUNTER CLASSIFICATION]")
    print(f"Encounter Type: {situation.encounter_type.value}")
    print(f"Relative Bearing: {situation.relative_bearing:.1f}°")
    print(f"Distance: {situation.distance:.1f} m")
    
    return tcpa

def demonstrate_varying_distances():
    """
    Demonstrate how TCPA changes with different initial distances.
    """
    print(f"\n[TCPA vs INITIAL DISTANCE ANALYSIS]")
    print("-" * 50)
    
    own_heading = 0.0
    target_heading = 180.0
    own_speed = 3.0
    target_speed = 3.0
    relative_speed = own_speed + target_speed
    
    print(f"Distance (m) | TCPA (s) | Time to Impact")
    print("-" * 40)
    
    distances = [60, 90, 120, 150, 180, 210, 240, 300]
    
    for distance in distances:
        tcpa = distance / relative_speed
        print(f"{distance:10.0f} | {tcpa:8.1f} | {tcpa/60:8.1f} min")
    
    print(f"\nKey Insight: TCPA = Distance / {relative_speed}")
    print(f"For 30-second TCPA: Distance = 30 × {relative_speed} = {30 * relative_speed} m")

def main():
    """
    Main function to run the TCPA calculation demonstration.
    """
    # Calculate required distance
    required_distance, own_heading, target_heading, own_speed, target_speed = calculate_head_on_tcpa_distance()
    
    # Verify with COLREGs Core
    calculated_tcpa = verify_with_colregs_core(
        required_distance, own_heading, target_heading, own_speed, target_speed
    )
    
    # Demonstrate varying distances
    demonstrate_varying_distances()
    
    # Final answer
    print(f"\n" + "=" * 70)
    print(f"🎯 FINAL ANSWER")
    print(f"=" * 70)
    print(f"For a TCPA of 30 seconds in a head-on collision scenario:")
    print(f"• Own Ship: 0° (North), 3.0 m/s")
    print(f"• Target Ship: 180° (South), 3.0 m/s")
    print(f"• Relative Speed: 6.0 m/s")
    print(f"• Required Initial Distance: {required_distance:.0f} meters")
    print(f"")
    print(f"Verification: COLREGs Core calculated TCPA = {calculated_tcpa:.1f} seconds")
    print(f"Error: {abs(calculated_tcpa - 30.0):.2f} seconds")
    print(f"=" * 70)

if __name__ == "__main__":
    main()
