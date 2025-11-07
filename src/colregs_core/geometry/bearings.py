"""
항해 기하학 계산 유틸리티
"""
import numpy as np
from typing import Tuple
from colregs_core.utils.utils import WrapTo180, WrapTo360, distance, dist_hypot


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
    # Maritime/NED 좌표계: atan2(dy, dx) 사용
    absolute_bearing = np.degrees(np.arctan2(dy, dx))
    
    # 상대 방위각 = 절대 방위각 - OS heading
    relative_bearing = absolute_bearing - os_heading
    
    return float(WrapTo360(relative_bearing))

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
    heading_rad = np.radians(heading)
    # Maritime/NED 좌표계: vx=North, vy=East
    # heading=0°(North) → vx=speed, vy=0
    # heading=90°(East) → vx=0, vy=speed
    vx = speed * np.cos(heading_rad)  # North component
    vy = speed * np.sin(heading_rad)  # East component
    return (vx, vy)


def velocity_to_heading_speed(velocity: Tuple[float, float]) -> Tuple[float, float]:
    """
    속도 벡터를 heading과 속도로 변환
    
    Args:
        velocity: 속도 벡터 (vx, vy)
    
    Returns:
        (heading, speed)
        heading: degrees, [0, 360)
        speed: magnitude of velocity
    """
    vx, vy = velocity  # vx=North, vy=East
    speed = np.sqrt(vx**2 + vy**2)
    # Maritime/NED 좌표계: atan2(vy, vx) 사용
    heading = np.degrees(np.arctan2(vy, vx))
    return (WrapTo360(heading), speed)


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
        180 = OS가 TS의 정선미
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
    
    # Maritime/NED 좌표계: atan2(dy, dx) 사용
    absolute_bearing = np.degrees(np.arctan2(dy, dx))
    aspect = absolute_bearing - ts_heading
    
    return float(WrapTo360(aspect))


def calculate_bearing_rate(
    os_position: Tuple[float, float],
    os_velocity: Tuple[float, float],
    ts_position: Tuple[float, float],
    ts_velocity: Tuple[float, float]
) -> float:
    """
    방위각 변화율 계산 (deg/s)
    일정한 경우 충돌 위험 존재 (constant bearing, decreasing range)
    
    Args:
        os_position: OS 위치 (x, y) (tuple, list, or numpy array)
        os_velocity: OS 속도 벡터 (vx, vy) (tuple, list, or numpy array)
        ts_position: TS 위치 (x, y) (tuple, list, or numpy array)
        ts_velocity: TS 속도 벡터 (vx, vy) (tuple, list, or numpy array)
    
    Returns:
        방위각 변화율 (deg/s)
        0에 가까우면 충돌 위험
    """
    # Extract scalar values from numpy arrays or tuples
    def extract_coords(pos):
        if isinstance(pos, np.ndarray):
            if pos.ndim == 1:
                return float(pos[0]), float(pos[1])
            else:
                return float(pos[0, 0]), float(pos[1, 0])
        else:
            return float(pos[0]), float(pos[1])
    
    os_x, os_y = extract_coords(os_position)
    ts_x, ts_y = extract_coords(ts_position)
    os_vx, os_vy = extract_coords(os_velocity)
    ts_vx, ts_vy = extract_coords(ts_velocity)
    
    # 상대 위치 벡터
    dx = ts_x - os_x
    dy = ts_y - os_y
    range_sq = dx**2 + dy**2
    
    if range_sq < 1e-6:  # 거의 같은 위치
        return 0.0
    
    # 상대 속도
    dvx = ts_vx - os_vx
    dvy = ts_vy - os_vy
    
    # 방위각 변화율 = (r × v_rel) / |r|^2
    # r × v_rel (2D cross product)
    cross_product = dx * dvy - dy * dvx
    
    # rad/s to deg/s
    bearing_rate_rad = cross_product / range_sq
    return float(np.degrees(bearing_rate_rad))
