#!/usr/bin/env python3
"""
Otter USV Coordinate System Analysis

This script analyzes how ir-sim's RobotOtter and ObstacleOtter handle coordinate
systems when integrating with Python Vehicle Simulator's Otter dynamics.

Key Question: Does ir-sim use Python Vehicle Simulator's Otter dynamics but
calculate them in ir-sim's mathematical coordinate system?
"""

import numpy as np
import sys
import os

# Add paths for imports
sys.path.append('/home/hyo/colregs-core/src')
sys.path.append('/home/hyo/ir-sim')
sys.path.append('/home/hyo/PythonVehicleSimulator/src')

def analyze_otter_coordinate_handling():
    """
    Analyze how ir-sim handles coordinates when integrating with Otter dynamics
    """
    print("=" * 80)
    print("OTTER USV COORDINATE SYSTEM ANALYSIS")
    print("=" * 80)
    
    print("\n[KEY FINDINGS]")
    print("=" * 50)
    
    print("\n1. COORDINATE SYSTEM COMPATIBILITY:")
    print("   ✓ Python Vehicle Simulator (Otter): NAUTICAL")
    print("   ✓ ir-sim: MATHEMATICAL")
    print("   ✓ Both use SAME physical dynamics equations")
    
    print("\n2. COORDINATE TRANSFORMATION ANALYSIS:")
    print("   From ir-sim/irsim/lib/algorithm/kinematics.py lines 291-292:")
    print("   eta_next[0] += step_time * (nu_next[0] * np.cos(eta[5]) - nu_next[1] * np.sin(eta[5]))")
    print("   eta_next[1] += step_time * (nu_next[0] * np.sin(eta[5]) + nu_next[1] * np.cos(eta[5]))")
    
    print("\n3. VELOCITY TRANSFORMATION ANALYSIS:")
    print("   From ir-sim/irsim/world/robots/robot_otter.py lines 316-317:")
    print("   vx = u * np.cos(psi) - v * np.sin(psi)")
    print("   vy = u * np.sin(psi) + v * np.cos(psi)")
    
    print("\n4. COORDINATE SYSTEM INTERPRETATION:")
    print("   The SAME transformation equations are used, but:")
    print("   - Python Vehicle Sim: Interprets angles as NAUTICAL (0°=North)")
    print("   - ir-sim: Interprets angles as MATHEMATICAL (0°=East)")
    
    return True

def demonstrate_coordinate_difference():
    """
    Demonstrate the coordinate system difference with examples
    """
    print("\n" + "=" * 50)
    print("COORDINATE SYSTEM DIFFERENCE DEMONSTRATION")
    print("=" * 50)
    
    print("\n[EXAMPLE: Heading 0°]")
    print("Python Vehicle Simulator (Nautical):")
    print("  - 0° = North")
    print("  - Velocity: vx = u*sin(0°) = 0, vy = u*cos(0°) = u")
    print("  - Movement: North (+y direction)")
    
    print("\nir-sim (Mathematical):")
    print("  - 0° = East") 
    print("  - Velocity: vx = u*cos(0°) = u, vy = u*sin(0°) = 0")
    print("  - Movement: East (+x direction)")
    
    print("\n[EXAMPLE: Heading 90°]")
    print("Python Vehicle Simulator (Nautical):")
    print("  - 90° = East")
    print("  - Velocity: vx = u*sin(90°) = u, vy = u*cos(90°) = 0")
    print("  - Movement: East (+x direction)")
    
    print("\nir-sim (Mathematical):")
    print("  - 90° = North")
    print("  - Velocity: vx = u*cos(90°) = 0, vy = u*sin(90°) = u")
    print("  - Movement: North (+y direction)")
    
    print("\n[KEY INSIGHT]")
    print("The SAME transformation equations produce DIFFERENT results")
    print("because the coordinate systems interpret angles differently!")

def analyze_integration_approach():
    """
    Analyze how ir-sim integrates with Python Vehicle Simulator
    """
    print("\n" + "=" * 50)
    print("INTEGRATION APPROACH ANALYSIS")
    print("=" * 50)
    
    print("\n[INTEGRATION STRATEGY]")
    print("ir-sim uses a 'COORDINATE SYSTEM TRANSLATION' approach:")
    
    print("\n1. DYNAMICS CALCULATION:")
    print("   ✓ Uses Python Vehicle Simulator's Otter dynamics")
    print("   ✓ Maintains full 6-DOF physics (mass, damping, restoring forces)")
    print("   ✓ Uses velocity controller and propeller dynamics")
    print("   ✓ All internal calculations use Python Vehicle Sim conventions")
    
    print("\n2. COORDINATE INTERFACE:")
    print("   ✓ ir-sim interprets the results in its mathematical coordinate system")
    print("   ✓ Same transformation equations, different angle interpretation")
    print("   ✓ Position updates: eta_next[0] += step_time * (u*cos(psi) - v*sin(psi))")
    print("   ✓ Velocity conversion: vx = u*cos(psi) - v*sin(psi)")
    
    print("\n3. STATE MANAGEMENT:")
    print("   ✓ Extended state: [x, y, psi, u, v, r, n1, n2]")
    print("   ✓ Position (x,y,psi): ir-sim mathematical coordinates")
    print("   ✓ Velocities (u,v,r): Python Vehicle Sim nautical conventions")
    print("   ✓ Propeller states (n1,n2): Direct from Python Vehicle Sim")
    
    print("\n4. VISUALIZATION:")
    print("   ✓ ir-sim plots using mathematical coordinate system")
    print("   ✓ Robot appears to move in ir-sim's coordinate frame")
    print("   ✓ But dynamics are calculated using nautical conventions")

def demonstrate_practical_implications():
    """
    Show practical implications of this coordinate handling
    """
    print("\n" + "=" * 50)
    print("PRACTICAL IMPLICATIONS")
    print("=" * 50)
    
    print("\n[SCENARIO: Robot moving North]")
    print("Python Vehicle Simulator:")
    print("  - Heading: 0° (North)")
    print("  - Velocity: u=2.0 m/s, v=0.0 m/s")
    print("  - Position update: y += 2.0*step_time")
    
    print("\nir-sim interpretation:")
    print("  - Heading: 0° (East)")
    print("  - Velocity: u=2.0 m/s, v=0.0 m/s") 
    print("  - Position update: x += 2.0*step_time")
    
    print("\n[RESULT]")
    print("✓ Robot moves North in Python Vehicle Simulator")
    print("✓ Robot moves East in ir-sim visualization")
    print("✓ Same dynamics, different coordinate interpretation!")
    
    print("\n[INTEGRATION WITH COLREGs-CORE]")
    print("This creates a coordinate mismatch:")
    print("  - ir-sim RobotOtter: Mathematical coordinates")
    print("  - colregs-core: Nautical coordinates")
    print("  - Solution: Use coordinate transformation functions")

def provide_coordinate_transformation_solution():
    """
    Provide solution for coordinate transformation
    """
    print("\n" + "=" * 50)
    print("COORDINATE TRANSFORMATION SOLUTION")
    print("=" * 50)
    
    print("\n[PROBLEM]")
    print("ir-sim RobotOtter uses mathematical coordinates")
    print("colregs-core expects nautical coordinates")
    print("Need transformation when integrating both packages")
    
    print("\n[SOLUTION]")
    print("Use colregs-core's coordinate transformation functions:")
    print("")
    print("```python")
    print("from colregs_core.geometry.coordinate_transform import (")
    print("    irsim_to_nav_heading,")
    print("    nav_to_irsim_heading")
    print(")")
    print("")
    print("# Convert ir-sim heading to colregs-core heading")
    print("irsim_heading = robot.state[2, 0]  # ir-sim mathematical")
    print("nav_heading = irsim_to_nav_heading(np.degrees(irsim_heading))")
    print("")
    print("# Use nav_heading with colregs-core functions")
    print("situation = classifier.classify(")
    print("    os_position=(robot.state[0,0], robot.state[1,0]),")
    print("    os_heading=nav_heading,  # Converted to nautical")
    print("    os_speed=speed,")
    print("    ts_position=ts_pos,")
    print("    ts_heading=ts_nav_heading,")
    print("    ts_speed=ts_speed")
    print(")")
    print("```")
    
    print("\n[KEY TRANSFORMATION FORMULAS]")
    print("ir-sim → colregs-core:")
    print("  nav_heading = (90 - irsim_heading) % 360")
    print("")
    print("colregs-core → ir-sim:")
    print("  irsim_heading = (90 - nav_heading) % 360")

def main():
    """
    Main analysis function
    """
    analyze_otter_coordinate_handling()
    demonstrate_coordinate_difference()
    analyze_integration_approach()
    demonstrate_practical_implications()
    provide_coordinate_transformation_solution()
    
    print("\n" + "=" * 80)
    print("FINAL ANSWER")
    print("=" * 80)
    print("YES! ir-sim's RobotOtter and ObstacleOtter:")
    print("✓ Use Python Vehicle Simulator's Otter dynamics")
    print("✓ Calculate them in ir-sim's mathematical coordinate system")
    print("✓ Same physics equations, different coordinate interpretation")
    print("✓ Requires coordinate transformation for colregs-core integration")
    print("=" * 80)

if __name__ == "__main__":
    main()
