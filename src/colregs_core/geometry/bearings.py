"""
항해 기하학 계산 유틸리티 (Maritime/NED 좌표계 사용)
"""
import numpy as np
from typing import Tuple
from ..utils import WrapTo180, WrapTo360

def calculate_relative_bearing(
    os_position: Tuple[float, float],
    os_heading: float,
    ts_position: Tuple[float, float]
) -> float:
    """
    Own Ship에서 Target Ship의 상대 방위각 계산
    
    Args:
        os_position: OS 위치 (x, y) in meters (tuple, list, or numpy array)
        os_heading: OS heading (degrees, 0=North, clockwise)
        ts_position: TS 위치 (x, y) in meters (tuple, list, or numpy array)
    
    Returns:
        상대 방위각 (degrees, [0, 360), 0=dead ahead)
    """
    # Extract scalar values from numpy arrays or tuples
    if isinstance(os_position, np.ndarray):
        if os_position.ndim == 1:
            os_x, os_y = float(os_position[0]), float(os_position[1])
        else:
            os_x, os_y = float(os_position[0, 0]), float(os_position[1, 0])
    else:
        os_x, os_y = float(os_position[0]), float(os_position[1])
    
    if isinstance(ts_position, np.ndarray):
        if ts_position.ndim == 1:
            ts_x, ts_y = float(ts_position[0]), float(ts_position[1])
        else:
            ts_x, ts_y = float(ts_position[0, 0]), float(ts_position[1, 0])
    else:
        ts_x, ts_y = float(ts_position[0]), float(ts_position[1])
    
    dx = ts_x - os_x  # North direction displacement
    dy = ts_y - os_y  # East direction displacement
    
    # 절대 방위각 계산 (North=0, clockwise)
    # Maritime/NED 좌표계: atan2(dy_east, dx_north) 사용
    # atan2(East, North) gives angle from North, clockwise
    absolute_bearing = np.degrees(np.arctan2(dy, dx))
    
    # 상대 방위각 = 절대 방위각 - OS heading
    relative_bearing = WrapTo360(absolute_bearing - os_heading)
    
    return relative_bearing

def calculate_relative_velocity(
    os_velocity: Tuple[float, float],
    ts_velocity: Tuple[float, float]
) -> Tuple[float, float]:
    """
    상대 속도 벡터 계산 (TS relative to OS)
    
    Args:
        os_velocity: OS 속도 벡터 (vx, vy) in m/s
        ts_velocity: TS 속도 벡터 (vx, vy) in m/s
    
    Returns:
        상대 속도 벡터 (vx, vy)
    """
    return (
        ts_velocity[0] - os_velocity[0],
        ts_velocity[1] - os_velocity[1]
    )


def heading_speed_to_velocity(heading: float, speed: float) -> Tuple[float, float]:
    """
    Heading과 속도를 속도 벡터로 변환
    
    Args:
        heading: Heading (degrees, 0=North, clockwise)
        speed: 속도 (m/s or knots)
    
    Returns:
        속도 벡터 (vx, vy)
    """
    # Maritime/NED 좌표계: vx=North, vy=East
    # heading=0°(North) → vx=speed, vy=0
    # heading=90°(East) → vx=0, vy=speed
    vx = speed * np.cos(np.radians(heading))  # North component
    vy = speed * np.sin(np.radians(heading))  # East component
    return (vx, vy)


def velocity_to_heading_speed(velocity: Tuple[float, float]) -> Tuple[float, float]:
    """
    속도 벡터를 heading과 속도로 변환
    
    Args:
        velocity: 속도 벡터 (vx, vy)
    
    Returns:
        (heading, speed)
        heading: degrees, (-180, 180], 0=North, clockwise
        speed: magnitude of velocity
    """
    vx, vy = velocity  # vx=North, vy=East
    speed = np.sqrt(vx**2 + vy**2)
    # Maritime/NED 좌표계: atan2(vy_east, vx_north) 사용
    heading = WrapTo180(np.degrees(np.arctan2(vy, vx)))
    return heading, speed

def calculate_aspect_angle(
    ts_heading: float,
    os_position: Tuple[float, float],
    ts_position: Tuple[float, float]
) -> float:
    """
    Target Ship의 aspect angle 계산
    (TS에서 OS를 바라보는 상대 방위각)
    
    Args:
        ts_heading: TS heading (degrees)
        os_position: OS 위치 (x, y) (tuple, list, or numpy array)
        ts_position: TS 위치 (x, y) (tuple, list, or numpy array)
    
    Returns:
        Aspect angle (degrees, [0, 360))
        0 = OS가 TS의 정선수 방향
        90 = OS가 TS의 우현
        160 = OS가 TS의 정선미
        270 = OS가 TS의 좌현
    """
    # Extract scalar values from numpy arrays or tuples
    if isinstance(os_position, np.ndarray):
        if os_position.ndim == 1:
            os_x, os_y = float(os_position[0]), float(os_position[1])
        else:
            os_x, os_y = float(os_position[0, 0]), float(os_position[1, 0])
    else:
        os_x, os_y = float(os_position[0]), float(os_position[1])
    
    if isinstance(ts_position, np.ndarray):
        if ts_position.ndim == 1:
            ts_x, ts_y = float(ts_position[0]), float(ts_position[1])
        else:
            ts_x, ts_y = float(ts_position[0, 0]), float(ts_position[1, 0])
    else:
        ts_x, ts_y = float(ts_position[0]), float(ts_position[1])
    
    dx = os_x - ts_x  # North direction displacement
    dy = os_y - ts_y  # East direction displacement
    
    # Maritime/NED 좌표계: atan2(dy_east, dx_north) 사용
    # atan2(East, North) gives angle from North, clockwise
    absolute_bearing = np.degrees(np.arctan2(dy, dx))
    aspect = WrapTo360(absolute_bearing - ts_heading)
    
    return aspect
