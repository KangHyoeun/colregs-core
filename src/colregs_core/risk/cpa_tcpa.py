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
    
    Paper Formulas (Eq. 1 and 2):
    - Eq. (1): TCPA = -((P_O - P_T) · (V_O - V_T)) / ||V_O - V_T||²
               TCPA = 0 if ||V_O - V_T|| = 0
    - Eq. (2): DCPA = || (P_O + V_O · TCPA) - (P_T + V_T · TCPA) ||
    
    where:
    - P_O, P_T: Position vectors of OS and TS
    - V_O, V_T: Velocity vectors of OS and TS
    
    두 선박이 현재 속도와 침로를 유지할 때 최접근 거리와 시간 계산
    
    Args:
        os_position: Own Ship 위치 P_O (x, y) in meters (tuple, list, or numpy array)
        os_velocity: Own Ship 속도 벡터 V_O (vx, vy) in m/s (tuple, list, or numpy array)
        ts_position: Target Ship 위치 P_T (x, y) in meters (tuple, list, or numpy array)
        ts_velocity: Target Ship 속도 벡터 V_T (vx, vy) in m/s (tuple, list, or numpy array)
    
    Returns:
        (dcpa, tcpa)
        dcpa: Distance at CPA (meters) as Python float
        tcpa: Time to CPA (seconds) as Python float, 음수는 이미 CPA 통과
    
    Notes:
        - 상대 속도가 0이면 (||V_O - V_T|| = 0): TCPA = 0 (Paper Eq. 1), DCPA = 현재 거리
        - TCPA < 0: 이미 CPA 통과 (선박들이 멀어지는 중)
        - TCPA > 0: CPA가 미래에 발생 (선박들이 가까워지는 중)
        - TCPA = 0: 현재가 CPA 시점 (평행 이동, 거리 일정)
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
    
    # 상대 위치 벡터: P_O - P_T
    dx = os_x - ts_x
    dy = os_y - ts_y
    
    # 상대 속도 벡터: V_O - V_T
    dvx = os_vx - ts_vx
    dvy = os_vy - ts_vy
    
    # 상대 속도의 크기 제곱: ||V_O - V_T||²
    rel_speed_sq = dvx**2 + dvy**2
    
    # Paper Eq. (1): If ||V_O - V_T|| = 0, TCPA = 0
    if rel_speed_sq < 1e-6:
        # 상대 속도가 0이면 평행 이동, 현재 거리가 DCPA
        current_distance = float(np.sqrt(dx**2 + dy**2))
        return (current_distance, 0.0)  
    
    # Paper Eq. (1): TCPA = -((P_O - P_T) · (V_O - V_T)) / ||V_O - V_T||²
    # 음수 부호 필수! 내적이 음수면 가까워지는 중 → TCPA > 0
    tcpa = -(dx * dvx + dy * dvy) / rel_speed_sq
    
    # Paper Eq. (2): DCPA = || (P_O + V_O · TCPA) - (P_T + V_T · TCPA) ||
    cpa_dx = dx + dvx * tcpa
    cpa_dy = dy + dvy * tcpa
    dcpa = float(np.sqrt(cpa_dx**2 + cpa_dy**2))
    tcpa = float(tcpa)
    
    return (dcpa, tcpa)
