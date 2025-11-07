"""
Closest Point of Approach (CPA) 및 Time to CPA (TCPA) 계산
"""
import numpy as np
from typing import Tuple, Optional


def calculate_cpa_tcpa(
    os_position: Tuple[float, float],
    os_velocity: Tuple[float, float],
    ts_position: Tuple[float, float],
    ts_velocity: Tuple[float, float]
) -> Tuple[float, float]:
    """
    CPA와 TCPA 계산
    
    두 선박이 현재 속도와 침로를 유지할 때 최접근 거리와 시간 계산
    
    Args:
        os_position: Own Ship 위치 (x, y) in meters (tuple, list, or numpy array)
        os_velocity: Own Ship 속도 벡터 (vx, vy) in m/s (tuple, list, or numpy array)
        ts_position: Target Ship 위치 (x, y) in meters (tuple, list, or numpy array)
        ts_velocity: Target Ship 속도 벡터 (vx, vy) in m/s (tuple, list, or numpy array)
    
    Returns:
        (dcpa, tcpa)
        dcpa: Distance at CPA (meters) as Python float
        tcpa: Time to CPA (seconds) as Python float, 음수는 이미 CPA 통과
    
    Notes:
        상대 속도가 0이면 (평행하게 같은 속도로 이동) DCPA는 현재 거리
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
    
    # 상대 속도 벡터 (TS relative to OS)
    dvx = ts_vx - os_vx
    dvy = ts_vy - os_vy
    
    # 상대 속도의 크기
    rel_speed_sq = dvx**2 + dvy**2
    
    # 상대 속도가 0이면 (평행하게 같은 속도)
    if rel_speed_sq < 1e-6:
        current_distance = float(np.sqrt(dx**2 + dy**2))
        return (current_distance, float('inf'))
    
    # TCPA 계산: t = -(r · v_rel) / |v_rel|^2
    # 음수면 이미 CPA 통과
    tcpa = -(dx * dvx + dy * dvy) / rel_speed_sq
    
    # DCPA 계산: CPA 시점의 거리
    # r_cpa = r + v_rel * tcpa
    cpa_dx = dx + dvx * tcpa
    cpa_dy = dy + dvy * tcpa
    dcpa = float(np.sqrt(cpa_dx**2 + cpa_dy**2))
    tcpa = float(tcpa)
    
    return (dcpa, tcpa)


def calculate_cpa_position(
    os_position: Tuple[float, float],
    os_velocity: Tuple[float, float],
    ts_position: Tuple[float, float],
    ts_velocity: Tuple[float, float]
) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    """
    CPA 시점의 두 선박 위치 계산
    
    Args:
        os_position: Own Ship 위치 (x, y)
        os_velocity: Own Ship 속도 벡터 (vx, vy)
        ts_position: Target Ship 위치 (x, y)
        ts_velocity: Target Ship 속도 벡터 (vx, vy)
    
    Returns:
        (os_cpa_position, ts_cpa_position, tcpa)
        os_cpa_position: CPA 시점 OS 위치
        ts_cpa_position: CPA 시점 TS 위치
        tcpa: Time to CPA (seconds)
    """
    _, tcpa = calculate_cpa_tcpa(os_position, os_velocity, ts_position, ts_velocity)
    
    if np.isinf(tcpa):
        # 상대 속도가 0인 경우
        return (os_position, ts_position, tcpa)
    
    # CPA 시점 위치 계산
    os_cpa = (
        os_position[0] + os_velocity[0] * tcpa,
        os_position[1] + os_velocity[1] * tcpa
    )
    ts_cpa = (
        ts_position[0] + ts_velocity[0] * tcpa,
        ts_position[1] + ts_velocity[1] * tcpa
    )
    
    return (os_cpa, ts_cpa, tcpa)


def is_collision_course(
    dcpa: float,
    tcpa: float,
    dcpa_threshold: float = 500.0,  # meters
    tcpa_threshold: float = 1200.0   # seconds (20 minutes)
) -> bool:
    """
    충돌 침로 여부 판단
    
    Args:
        dcpa: Distance at CPA (meters)
        tcpa: Time to CPA (seconds)
        dcpa_threshold: DCPA 임계값 (기본 500m, 약 0.27 NM)
        tcpa_threshold: TCPA 임계값 (기본 1200s = 20분)
    
    Returns:
        충돌 위험이 있으면 True
    
    Notes:
        IMO Performance Standards:
        - DCPA: 일반적으로 0.5 NM (약 926m) 이하
        - TCPA: 20분 이하
        실전에서는 선박 크기, 조종성능, 해역 특성에 따라 조정
    """
    if tcpa < 0:
        # 이미 CPA 통과
        return False
    
    return dcpa < dcpa_threshold and tcpa < tcpa_threshold


def calculate_miss_distance(
    os_position: Tuple[float, float],
    os_velocity: Tuple[float, float],
    ts_position: Tuple[float, float],
    ts_velocity: Tuple[float, float],
    time_horizon: float = 1800.0  # seconds (30 minutes)
) -> float:
    """
    특정 시간 이후 최소 접근 거리 계산
    
    Args:
        os_position: OS 위치
        os_velocity: OS 속도 벡터
        ts_position: TS 위치
        ts_velocity: TS 속도 벡터
        time_horizon: 예측 시간 범위 (seconds)
    
    Returns:
        예측 시간 내 최소 거리 (meters)
    """
    dcpa, tcpa = calculate_cpa_tcpa(os_position, os_velocity, ts_position, ts_velocity)
    
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
    
    if tcpa < 0:
        # 이미 CPA 통과, 현재 거리 반환
        dx = ts_x - os_x
        dy = ts_y - os_y
        return float(np.sqrt(dx**2 + dy**2))
    
    if tcpa > time_horizon:
        # CPA가 예측 범위 밖, time_horizon 시점 거리 반환
        dx = (ts_x + ts_vx * time_horizon) - (os_x + os_vx * time_horizon)
        dy = (ts_y + ts_vy * time_horizon) - (os_y + os_vy * time_horizon)
        return float(np.sqrt(dx**2 + dy**2))
    
    # CPA가 예측 범위 내
    return float(dcpa)


def calculate_bow_crossing_range(
    os_position: Tuple[float, float],
    os_velocity: Tuple[float, float],
    ts_position: Tuple[float, float],
    ts_velocity: Tuple[float, float]
) -> Optional[float]:
    """
    Bow Crossing Range (BCR) 계산
    TS가 OS의 선수를 가로지르는 시점의 거리
    
    Args:
        os_position: OS 위치
        os_velocity: OS 속도 벡터
        ts_position: TS 위치
        ts_velocity: TS 속도 벡터
    
    Returns:
        BCR (meters), TS가 선수를 가로지르지 않으면 None
    
    Notes:
        Crossing 상황에서 중요한 지표
        BCR이 작을수록 위험
    """
    # 상대 위치 및 속도
    dx = ts_position[0] - os_position[0]
    dy = ts_position[1] - os_position[1]
    dvx = ts_velocity[0] - os_velocity[0]
    dvy = ts_velocity[1] - os_velocity[1]
    
    # OS의 진행 방향 벡터
    os_speed = np.sqrt(os_velocity[0]**2 + os_velocity[1]**2)
    if os_speed < 1e-6:
        return None
    
    os_dir = (os_velocity[0] / os_speed, os_velocity[1] / os_speed)
    
    # TS가 OS의 진행선(bow line)을 가로지르는 시간
    # (r + v_rel * t) · os_dir = 0
    denominator = -(dvx * os_dir[0] + dvy * os_dir[1])
    
    if abs(denominator) < 1e-6:
        # TS가 OS의 진행선과 평행
        return None
    
    t_crossing = (dx * os_dir[0] + dy * os_dir[1]) / denominator
    
    if t_crossing < 0:
        # 이미 가로지름
        return None
    
    # Crossing 시점의 거리
    crossing_dx = dx + dvx * t_crossing
    crossing_dy = dy + dvy * t_crossing
    bcr = np.sqrt(crossing_dx**2 + crossing_dy**2)
    
    return bcr
