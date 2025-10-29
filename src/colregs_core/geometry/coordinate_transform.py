"""
좌표계 변환 유틸리티 (Coordinate System Transformation)

ir-sim과 항해 좌표계(Navigation Frame) 간의 변환을 제공합니다.

좌표계 정의:
-------------
1. ir-sim (일반 로봇공학 좌표계):
   - 0° = East (+x 방향)
   - 90° = North (+y 방향)
   - 180° = West (-x 방향)
   - 270° = South (-y 방향)
   - 회전: 반시계방향 (CCW)
   - 속도 변환: vx = v*cos(θ), vy = v*sin(θ)

2. Navigation Frame (항해 좌표계):
   - 0° = North (+y 방향)
   - 90° = East (+x 방향)
   - 180° = South (-y 방향)
   - 270° = West (-x 방향)
   - 회전: 시계방향 (CW)
   - 속도 변환: vx = v*sin(θ), vy = v*cos(θ)

Author: Maritime Robotics Lab
Date: 2025-10-27
"""

import numpy as np
from typing import Tuple


def irsim_to_nav_heading(irsim_heading: float) -> float:
    """
    ir-sim heading을 항해 좌표계 heading으로 변환
    
    변환 규칙:
    - ir-sim 0° (East) → Nav 90°
    - ir-sim 90° (North) → Nav 0°
    - ir-sim 180° (West) → Nav 270°
    - ir-sim 270° (South) → Nav 180°
    
    Args:
        irsim_heading: ir-sim heading (degrees, 0=East, CCW)
    
    Returns:
        nav_heading: Navigation heading (degrees, 0=North, CW)
    
    Examples:
        >>> irsim_to_nav_heading(0)    # East
        90.0
        >>> irsim_to_nav_heading(90)   # North
        0.0
        >>> irsim_to_nav_heading(180)  # West
        270.0
        >>> irsim_to_nav_heading(270)  # South
        180.0
    """
    nav_heading = (90.0 - irsim_heading) % 360.0
    return nav_heading


def nav_to_irsim_heading(nav_heading: float) -> float:
    """
    항해 좌표계 heading을 ir-sim heading으로 변환
    
    변환 규칙:
    - Nav 0° (North) → ir-sim 90°
    - Nav 90° (East) → ir-sim 0°
    - Nav 180° (South) → ir-sim 270°
    - Nav 270° (West) → ir-sim 180°
    
    Args:
        nav_heading: Navigation heading (degrees, 0=North, CW)
    
    Returns:
        irsim_heading: ir-sim heading (degrees, 0=East, CCW)
    
    Examples:
        >>> nav_to_irsim_heading(0)    # North
        90.0
        >>> nav_to_irsim_heading(90)   # East
        0.0
        >>> nav_to_irsim_heading(180)  # South
        270.0
        >>> nav_to_irsim_heading(270)  # West
        180.0
    """
    irsim_heading = (90.0 - nav_heading) % 360.0
    return irsim_heading


def irsim_velocity_to_nav(irsim_heading: float, speed: float) -> Tuple[float, float]:
    """
    ir-sim의 heading과 속도를 항해 좌표계의 속도 벡터로 변환
    
    ir-sim의 속도 벡터 변환:
        vx = speed * cos(θ_irsim)
        vy = speed * sin(θ_irsim)
    
    이것을 항해 좌표계로 변환하면 동일한 벡터가 됩니다
    (단지 해석이 다를 뿐)
    
    Args:
        irsim_heading: ir-sim heading (degrees, 0=East, CCW)
        speed: 속도 크기 (m/s)
    
    Returns:
        (vx, vy): 속도 벡터 (m/s) - 항해 좌표계 기준
    
    Examples:
        >>> # ir-sim에서 동쪽(0°)으로 10m/s
        >>> vx, vy = irsim_velocity_to_nav(0, 10)
        >>> print(f"vx={vx:.1f}, vy={vy:.1f}")
        vx=10.0, vy=0.0
        
        >>> # ir-sim에서 북쪽(90°)으로 10m/s
        >>> vx, vy = irsim_velocity_to_nav(90, 10)
        >>> print(f"vx={vx:.1f}, vy={vy:.1f}")
        vx=0.0, vy=10.0
    """
    heading_rad = np.radians(irsim_heading)
    vx = speed * np.cos(heading_rad)
    vy = speed * np.sin(heading_rad)
    return (vx, vy)


def nav_velocity_to_irsim(nav_heading: float, speed: float) -> Tuple[float, float]:
    """
    항해 좌표계의 heading과 속도를 ir-sim의 속도 벡터로 변환
    
    항해 좌표계의 속도 벡터 변환:
        vx = speed * sin(θ_nav)
        vy = speed * cos(θ_nav)
    
    이것을 ir-sim 좌표계로 변환하면 동일한 벡터가 됩니다
    
    Args:
        nav_heading: Navigation heading (degrees, 0=North, CW)
        speed: 속도 크기 (m/s)
    
    Returns:
        (vx, vy): 속도 벡터 (m/s) - ir-sim 좌표계 기준
    
    Examples:
        >>> # 항해 좌표계에서 북쪽(0°)으로 10m/s
        >>> vx, vy = nav_velocity_to_irsim(0, 10)
        >>> print(f"vx={vx:.1f}, vy={vy:.1f}")
        vx=0.0, vy=10.0
        
        >>> # 항해 좌표계에서 동쪽(90°)으로 10m/s
        >>> vx, vy = nav_velocity_to_irsim(90, 10)
        >>> print(f"vx={vx:.1f}, vy={vy:.1f}")
        vx=10.0, vy=0.0
    """
    heading_rad = np.radians(nav_heading)
    vx = speed * np.sin(heading_rad)
    vy = speed * np.cos(heading_rad)
    return (vx, vy)


def verify_transformation():
    """
    좌표계 변환이 올바른지 검증하는 함수
    
    Examples:
        >>> verify_transformation()
        ✓ 모든 좌표계 변환 테스트 통과
    """
    test_cases = [
        # (irsim_heading, expected_nav_heading, direction_name)
        (0, 90, "East"),
        (90, 0, "North"),
        (180, 270, "West"),
        (270, 180, "South"),
        (45, 45, "Northeast"),
        (135, 315, "Northwest"),
    ]
    
    print("\n" + "="*60)
    print("좌표계 변환 검증 (Coordinate Transformation Verification)")
    print("="*60)
    
    all_passed = True
    
    for irsim_h, expected_nav_h, direction in test_cases:
        nav_h = irsim_to_nav_heading(irsim_h)
        irsim_h_back = nav_to_irsim_heading(nav_h)
        
        # 변환 검증
        heading_match = abs(nav_h - expected_nav_h) < 1e-6
        roundtrip_match = abs(irsim_h_back - irsim_h) < 1e-6
        
        if heading_match and roundtrip_match:
            status = "✓"
        else:
            status = "✗"
            all_passed = False
        
        print(f"{status} {direction:12s}: ir-sim {irsim_h:6.1f}° → Nav {nav_h:6.1f}° → ir-sim {irsim_h_back:6.1f}°")
    
    # 속도 벡터 변환 검증
    print("\n속도 벡터 변환 검증:")
    speed = 10.0
    
    for irsim_h, _, direction in test_cases[:4]:  # 주요 4방향만
        vx, vy = irsim_velocity_to_nav(irsim_h, speed)
        
        # ir-sim 좌표계에서의 예상 벡터
        irsim_vx = speed * np.cos(np.radians(irsim_h))
        irsim_vy = speed * np.sin(np.radians(irsim_h))
        
        vector_match = (abs(vx - irsim_vx) < 1e-6 and abs(vy - irsim_vy) < 1e-6)
        
        if vector_match:
            status = "✓"
        else:
            status = "✗"
            all_passed = False
        
        print(f"{status} {direction:12s}: ir-sim {irsim_h:6.1f}° → velocity ({vx:6.2f}, {vy:6.2f}) m/s")
    
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
