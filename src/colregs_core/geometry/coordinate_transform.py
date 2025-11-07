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
from colregs_core.utils.utils import WrapTo180, WrapTo360
from math import pi, atan2, sin, cos, sqrt
import numpy as np
import math

# ========================================
# Heading Conversion Functions  
# ========================================

def ned_to_math_heading(ned_deg: float) -> float:

    return WrapTo180(90.0 - ned_deg)


def math_to_ned_heading(math_deg: float) -> float:

    return WrapTo180(90.0 - math_deg)

# ========================================
# Position / State Conversion Functions  
# ========================================

def maritime_to_math_position(x_north: float, y_east: float) -> Tuple[float, float]:

    return y_east, x_north


def math_to_maritime_position(x_east: float, y_north: float) -> Tuple[float, float]:

    return y_north, x_east


def maritime_to_math_state(state_maritime: np.array) -> np.array:

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


def math_to_maritime_state(state_math: np.array) -> np.array:

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

    angle_deg = atan2(dy_east, dx_north)
    return WrapTo180(angle_deg)


def math_relative_angle(dx_east: float, dy_north: float) -> float:
    
    angle_deg = atan2(dy_north, dx_east)
    return WrapTo180(angle_deg)

# ========================================
# Velocity Conversion Functions  
# ========================================

def math_to_maritime_velocity(math_deg: float, speed: float) -> Tuple[float, float]:

    heading_rad = np.radians(math_deg)
    vx = speed * np.cos(heading_rad)
    vy = speed * np.sin(heading_rad)
    return (vx, vy)


def maritime_to_math_velocity(maritime_deg: float, speed: float) -> Tuple[float, float]:

    heading_rad = np.radians(maritime_deg)
    vx = speed * np.sin(heading_rad)
    vy = speed * np.cos(heading_rad)
    return (vx, vy)

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
        vx, vy = math_to_maritime_velocity(math_h, speed)
        
        # math 좌표계에서의 예상 벡터
        math_vx = speed * np.cos(np.radians(math_h))
        math_vy = speed * np.sin(np.radians(math_h))
        
        vector_match = (abs(vx - math_vx) < 1e-6 and abs(vy - math_vy) < 1e-6)
        
        if vector_match:
            status = "✓"
        else:
            status = "✗"
            all_passed = False
        
        print(f"{status} {direction:12s}: math {math_h:6.1f}° → velocity ({vx:6.2f}, {vy:6.2f}) m/s")
    
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
