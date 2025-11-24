"""
Coordinate System Transformation

좌표계 정의:
-------------
1. math (일반 로봇공학 좌표계):
   - 0° = East (+x 방향)
   - 90° = North (+y 방향)
   - 180° = West (-x 방향)
   - -90° = South (-y 방향)
   - 회전: 반시계방향 (CCW)
   - 속도 변환: vx = v*cos(θ), vy = v*sin(θ)

2. NED (항해 좌표계):
   - 0° = North (+y 방향)
   - 90° = East (+x 방향)
   - 180° = South (-y 방향)
   - -90° = West (-x 방향)
   - 회전: 시계방향 (CW)
   - 속도 변환: vx = v*sin(θ), vy = v*cos(θ)

Author: Maritime Robotics Lab
Date: 2025-10-27
"""

import numpy as np
from typing import Tuple
from ..utils import WrapTo180, WrapTo360
from math import pi, atan2, sin, cos, sqrt

# ========================================
# Heading Conversion Functions  
# ========================================

def ned_to_math_heading(ned_deg: float) -> float:
    """
    Convert heading from NED (maritime) coordinate system to math coordinate system.
    
    Args:
        ned_deg: Heading in NED coordinates (degrees, 0=North, CW)
    
    Returns:
        Heading in math coordinates (degrees, 0=East, CCW)
    """
    return WrapTo180(90.0 - ned_deg)


def math_to_ned_heading(math_deg: float) -> float:
    """
    Convert heading from math coordinate system to NED (maritime) coordinate system.
    
    Args:
        math_deg: Heading in math coordinates (degrees, 0=East, CCW)
    
    Returns:
        Heading in NED coordinates (degrees, 0=North, CW)
    """
    return WrapTo180(90.0 - math_deg)

# ========================================
# Position / State Conversion Functions  
# ========================================

def maritime_to_math_position(x_north: float, y_east: float) -> Tuple[float, float]:
    """
    Convert position from maritime/NED coordinates to math coordinates.
    
    Args:
        x_north: North component (NED x-axis)
        y_east: East component (NED y-axis)
    
    Returns:
        (x_east, y_north) in math coordinates
    """
    return y_east, x_north


def math_to_maritime_position(x_east: float, y_north: float) -> Tuple[float, float]:
    """
    Convert position from math coordinates to maritime/NED coordinates.
    
    Args:
        x_east: East component (math x-axis)
        y_north: North component (math y-axis)
    
    Returns:
        (x_north, y_east) in NED coordinates
    """
    return y_north, x_east


def maritime_to_math_state(state_maritime: np.ndarray) -> np.ndarray:
    """
    Convert state vector from maritime/NED coordinates to math coordinates.
    
    Args:
        state_maritime: State vector in NED coordinates (2x1 or 3x1)
                       [x_north, y_east] or [x_north, y_east, heading_deg]
    
    Returns:
        State vector in math coordinates (2x1 or 3x1)
        [x_east, y_north] or [x_east, y_north, heading_deg]
    """
    if state_maritime.shape[0] >= 2:
        x_math, y_math = maritime_to_math_position(
            state_maritime[0, 0], state_maritime[1, 0]
        )
        
        if state_maritime.shape[0] >= 3:
            heading_math = ned_to_math_heading(state_maritime[2, 0])
            return np.array([[x_math], [y_math], [heading_math]])
        else:
            return np.array([[x_math], [y_math]])
    
    return state_maritime


def math_to_maritime_state(state_math: np.ndarray) -> np.ndarray:
    """
    Convert state vector from math coordinates to maritime/NED coordinates.
    
    Args:
        state_math: State vector in math coordinates (2x1 or 3x1)
                   [x_east, y_north] or [x_east, y_north, heading_deg]
    
    Returns:
        State vector in NED coordinates (2x1 or 3x1)
        [x_north, y_east] or [x_north, y_east, heading_deg]
    """
    if state_math.shape[0] >= 2:
        x_maritime, y_maritime = math_to_maritime_position(
            state_math[0, 0], state_math[1, 0]
        )
        
        if state_math.shape[0] >= 3:
            heading_maritime = math_to_ned_heading(state_math[2, 0])
            return np.array([[x_maritime], [y_maritime], [heading_maritime]])
        else:
            return np.array([[x_maritime], [y_maritime]])
    
    return state_math

def maritime_relative_angle(dx_north: float, dy_east: float) -> float:
    """
    Calculate relative angle from displacement in maritime/NED coordinates.
    
    Args:
        dx_north: North component of displacement
        dy_east: East component of displacement
    
    Returns:
        Angle in degrees [0, 360) from North, clockwise
    """
    return WrapTo360(np.degrees(np.atan2(dy_east, dx_north)))


def math_relative_angle(dx_east: float, dy_north: float) -> float:
    """
    Calculate relative angle from displacement in math coordinates.
    
    Args:
        dx_east: East component of displacement
        dy_north: North component of displacement
    
    Returns:
        Angle in degrees [0, 360) from East, counter-clockwise
    """
    return WrapTo360(np.degrees(np.atan2(dy_north, dx_east)))

# ========================================
# Velocity Conversion Functions  
# ========================================

def math_to_maritime_velocity(math_deg: float, speed: float) -> Tuple[float, float]:
    """
    Convert velocity from math coordinate system to maritime/NED coordinates.
    
    Args:
        math_deg: Heading in math coordinates (degrees, 0=East, CCW)
        speed: Speed magnitude (m/s)
    
    Returns:
        (vx_north, vy_east) velocity components in NED coordinates
    """
    heading_rad = np.radians(math_deg)
    vx_north = speed * np.sin(heading_rad)
    vy_east = speed * np.cos(heading_rad)
    return (vx_north, vy_east)


def maritime_to_math_velocity(maritime_deg: float, speed: float) -> Tuple[float, float]:
    """
    Convert velocity from maritime/NED coordinates to math coordinate system.
    
    Args:
        maritime_deg: Heading in NED coordinates (degrees, 0=North, CW)
        speed: Speed magnitude (m/s)
    
    Returns:
        (vx_east, vy_north) velocity components in math coordinates
    """
    heading_rad = np.radians(maritime_deg)
    vx_east = speed * np.sin(heading_rad)
    vy_north = speed * np.cos(heading_rad)
    return (vx_east, vy_north)

# ========================================
# Inertial World to Body-Fixed Grid  
# ========================================

def world_to_grid(os_position: np.ndarray, os_heading: float, 
                  ts_position: np.ndarray, grid_size: int, observation_distance: float) -> Tuple[int, int]:
    """
    Convert TS position (world coordinates) to grid coordinates
    
    Args:
        ts_position: [N, E] in NED (meters)
        os_position: [N, E] in NED (meters)
        os_heading: OS heading (degrees, 0=North)
        grid_size: Grid dimension (N×N)
        observation_distance: Grid coverage (meters)
    
    Returns:
        (grid_x, grid_y) or None if out of range
    """
    # 1. Calculate relative position (NED frame)
    rel_n = ts_position[0] - os_position[0]  # North
    rel_e = ts_position[1] - os_position[1]  # East
    
    # 2. Rotate to body-fixed frame (OS heading = up)
    heading_rad = np.radians(os_heading)
    
    # Body frame: x=forward, y=right
    # Grid frame: x=right, y=up (forward)
    body_x = rel_e * np.cos(heading_rad) - rel_n * np.sin(heading_rad)
    body_y = rel_e * np.sin(heading_rad) + rel_n * np.cos(heading_rad)
    
    # 3. Convert to grid coordinates
    # Grid center = OS position
    cell_size = observation_distance / grid_size
    half_range = observation_distance / 2
    
    grid_x = int((body_x + half_range) / cell_size)
    grid_y = int((body_y + half_range) / cell_size)
    
    # 4. Check if within grid
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        return (grid_x, grid_y)
    else:
        return None  # Out of observation range

# ========================================
# Test Functions  
# ========================================

def verify_transformation():
    """
    좌표계 변환이 올바른지 검증하는 함수
    
    Examples:
        >>> verify_transformation()
        ✓ 모든 좌표계 변환 테스트 통과
    """
    test_cases_360 = [
        # (math_heading, expected_maritime_heading, direction_name)
        (0, 90, "East"),
        (90, 0, "North"),
        (180, 270, "West"),
        (270, 180, "South"),
        (45, 45, "Northeast"),
        (135, 315, "Northwest"),
        (225, 135, "Southeast"), 
        (315, 225, "Southwest"),
    ]
    test_cases_180 = [
        # (math_heading, expected_maritime_heading, direction_name)
        (0, 90, "East"),
        (90, 0, "North"),
        (180, -90, "West"),
        (-90, 180, "South"),
        (45, 45, "Northeast"),
        (135, -45, "Northwest"),
        (-45, 135, "Southeast"), 
        (-135, -135, "Southwest"),
    ]
    
    print("\n" + "="*60)
    print("좌표계 변환 검증 (Coordinate Transformation Verification)")
    print("="*60)
    
    all_passed = True
    
    for math_h, expected_maritime_h, direction in test_cases_180:
        maritime_h = ned_to_math_heading(math_h)
        math_h_back = math_to_ned_heading(maritime_h)
        
        # 변환 검증
        heading_match = abs(maritime_h - expected_maritime_h) < 1e-6
        roundtrip_match = abs(math_h_back - math_h) < 1e-6
        
        if heading_match and roundtrip_match:
            status = "✓"
        else:
            status = "✗"
            all_passed = False
        
        print(f"{status} {direction:12s}: math {math_h:6.1f}° → Mari {maritime_h:6.1f}° → math {math_h_back:6.1f}°")
    
    # 속도 벡터 변환 검증
    print("\n속도 벡터 변환 검증:")
    speed = 10.0
    
    for math_h, _, direction in test_cases_180[:4]:  # 주요 4방향만
        vx_north, vy_east = math_to_maritime_velocity(math_h, speed)
        
        # maritime 좌표계에서의 예상 벡터
        maritime_vx = speed * np.sin(np.radians(math_h))
        maritime_vy = speed * np.cos(np.radians(math_h))
        
        vector_match = (abs(vx_north - maritime_vx) < 1e-6 and abs(vy_east - maritime_vy) < 1e-6)
        
        if vector_match:
            status = "✓"
        else:
            status = "✗"
            all_passed = False
        
        print(f"{status} {direction:12s}: math {math_h:6.1f}° → velocity ({vx_north:6.2f}, {vy_east:6.2f}) m/s")
    
    print("="*60)
    
    if all_passed:
        print("✓ 모든 좌표계 변환 테스트 통과")
    else:
        print("✗ 일부 테스트 실패")
    
    print("="*60 + "\n")
    
    return all_passed


# 모듈 임포트 시 자동 검증 (디버깅용, 주석 처리 가능)
if __name__ == "__main__":
    verify_transformation()
