"""
Ship Domain 기반 Collision Risk Assessment

두 논문의 방법론 구현:
1. Chun et al. 2024: 비대칭 타원형 Ship Domain + 지수함수 기반 CR
2. 전도현 논문: 비대칭 타원형 Ship Domain + 개선된 Mou 공식

References:
- Chun et al. (2024). "Method for collision avoidance based on deep reinforcement learning with path-speed control for an autonomous ship."
- 전도현 (2024). "A Method for Collision Avoidance of a Ship Based on Reinforcement Learning in Complex Maritime Situations(복잡한 해상 상황에서의 강화 학습 기반 선박 충돌 회피 방법)"
"""
import numpy as np
from typing import Tuple, Optional, Dict
from dataclasses import dataclass
from .cpa_tcpa import calculate_cpa_tcpa
from ..geometry import calculate_relative_bearing, velocity_to_heading_speed
from ..utils import WrapTo360
from ..encounter.classifier import EncounterClassifier


@dataclass
class ShipDomainParams:
    """
    Ship Domain 파라미터
    
    비대칭 타원형 Ship Domain 정의
    COLREGs에 따라 전방 및 우현이 더 긴 형태
    
    Attributes:
        r_bow: 선수(전방) 방향 반경 (meters)
        r_stern: 선미(후방) 방향 반경 (meters)
        r_starboard: 우현(오른쪽) 방향 반경 (meters)
        r_port: 좌현(왼쪽) 방향 반경 (meters)
    """
    r_bow: float  # 전방
    r_stern: float  # 후방
    r_starboard: float  # 우현 (오른쪽)
    r_port: float  # 좌현 (왼쪽)


def calculate_ship_domain_distance(
    relative_bearing: float,
    ship_domain: ShipDomainParams
) -> float:
    """
    특정 방위각에서 Ship Domain 경계까지의 거리 계산
    
    Paper (Figure 2-4)의 정확한 방정식 사용:
    - Top-right (0-90°): x² + y² = r_bow² (circle)
    - Top-left (90-180°): (x/L)² + (y/r_bow)² = 1 (ellipse)
    - Bottom-left (180-270°): x² + y² = L² (circle)
    - Bottom-right (270-360°): (x/r_bow)² + (y/L)² = 1 (ellipse)
    
    where:
    - x: horizontal (port-starboard), positive to starboard
    - y: vertical (stern-bow), positive to bow
    - L: shorter length = r_stern = r_port
    
    Note: Ship Domain is defined relative to OS's heading (bow direction = 0°)
    
    Args:
        relative_bearing: OS에서 TS로의 상대 방위각 (degrees, 0=bow, clockwise)
                         This is the bearing relative to OS's heading, not absolute bearing
        ship_domain: Ship Domain 파라미터
    
    Returns:
        해당 방위각에서 Ship Domain 경계까지의 거리 (meters)
    """
    # 방위각을 0-360 범위로 정규화 (중요!)
    bearing = WrapTo360(relative_bearing)
    bearing_rad = np.radians(bearing)
    
    # L = shorter length (r_stern = r_port)
    L = min(ship_domain.r_stern, ship_domain.r_port)
    r_bow = ship_domain.r_bow
    
    # For a point on the domain boundary at bearing θ:
    # x = r * sin(θ)  (East component)
    # y = r * cos(θ)  (North component)
    
    if 0 <= bearing < 90:
        # Top-right quadrant: x² + y² = r_bow² (circle)
        # r² = r_bow² → r = r_bow
        return r_bow
    
    elif 90 <= bearing < 180:
        # Top-left quadrant: (x/L)² + (y/r_bow)² = 1
        # (r*sin(θ)/L)² + (r*cos(θ)/r_bow)² = 1
        # r² * [sin²(θ)/L² + cos²(θ)/r_bow²] = 1
        # r = 1 / sqrt(sin²(θ)/L² + cos²(θ)/r_bow²)
        sin_theta = np.sin(bearing_rad)
        cos_theta = np.cos(bearing_rad)
        r = 1.0 / np.sqrt((sin_theta / L)**2 + (cos_theta / r_bow)**2)
        return r
    
    elif 180 <= bearing < 270:
        # Bottom-left quadrant: x² + y² = L² (circle)
        # r² = L² → r = L
        return L
    
    else:  # 270 <= bearing < 360
        # Bottom-right quadrant: (x/r_bow)² + (y/L)² = 1
        # (r*sin(θ)/r_bow)² + (r*cos(θ)/L)² = 1
        # r² * [sin²(θ)/r_bow² + cos²(θ)/L²] = 1
        # r = 1 / sqrt(sin²(θ)/r_bow² + cos²(θ)/L²)
        sin_theta = np.sin(bearing_rad)
        cos_theta = np.cos(bearing_rad)
        r = 1.0 / np.sqrt((sin_theta / r_bow)**2 + (cos_theta / L)**2)
        return r


class ChunCollisionRisk:
    """
    Chun et al. 2024 방법론의 Collision Risk Assessment
    
    비대칭 타원형 Ship Domain + 지수함수 기반 CR 계산
    
    Paper Eq. (3) / Eq. (6):
    CR = f_angle · exp(-DCPA/a) · exp(-TCPA/b)
    
    Paper Eq. (4) - DCPA coefficient:
    a = - dr / ln(CRal)
    
    Paper Eq. (5) - TCPA coefficient:
    b = - dr / (V · ln(CRal))
    
    where:
    - dr = recognition distance (d_obs)
    - CRal = allowable CR (cr_obs)
    - V = relative speed (typically v_os + v_ts for head-on)
    
    Attributes:
        ship_domain: Ship Domain 파라미터
            - LL (Long Length): front and right lengths
            - LS (Short Length): rear and left lengths
        d_obs: Recognition distance (meters) - OS가 TS를 인식하고 모니터링 시작하는 거리
        cr_obs: Allowable CR value - 회피 개시 기준이 되는 CR 값
        a_coeff: DCPA 지수 계수 (meters)
        b_coeff: TCPA 지수 계수 (seconds)
    
    Notes:
        - Ship domain: Asymmetric ellipse with LL (front/right) and LS (rear/left)
        - Coefficients a and b are calculated using Eq. (4) and (5)
        - CR = 1: Ship Domain 경계선 상
        - CR > 1: 충돌 발생 (TS inside ship domain)
        - CR → 0: 위험 소멸 (TS moves away)
        - f_angle = 1.0 (normal case)
        - f_angle = 0.25 (when TCPA < 0, ships moving away) to prevent over-assessment
    """
    
    def __init__(
        self,
        ship_domain: ShipDomainParams,
        d_obs: float = 200.0,
        cr_obs: float = 0.3,
        os_speed: Optional[float] = None,
        ts_speed: Optional[float] = None,
        a_coeff: Optional[float] = None,
        b_coeff: Optional[float] = None
    ):
        """
        Args:
            ship_domain: Ship Domain 파라미터
            d_obs: Recognition distance (meters), default 200m
            cr_obs: Allowable CR value (기본값 0.3)
            os_speed: Own Ship 속도 (m/s), b_coeff 자동 계산용
            ts_speed: Target Ship 속도 (m/s), b_coeff 자동 계산용
            a_coeff: DCPA 지수 계수. None이면 Eq. (4)로 자동 계산
            b_coeff: TCPA 지수 계수. None이면 Eq. (5)로 자동 계산
        """
        self.ship_domain = ship_domain
        self.d_obs = d_obs
        self.cr_obs = cr_obs
        
        # Paper Eq. (4): a = - dr / ln(CRal)
        # where dr = recognition distance, CRal = allowable CR
        if a_coeff is None:
            if cr_obs <= 0 or cr_obs >= 1:
                raise ValueError(f"cr_obs must be in (0, 1), got {cr_obs}")
            self.a_coeff = -d_obs / np.log(cr_obs)
        else:
            self.a_coeff = a_coeff
        
        # Paper Eq. (5): b = - dr / (V · ln(CRal))
        # where V = relative speed (v_os + v_ts for head-on scenario)
        if b_coeff is None:
            if os_speed is not None and ts_speed is not None:
                # Use provided speeds (head-on: v_os + v_ts)
                V = os_speed + ts_speed
                if V < 1e-6:
                    raise ValueError("Combined speed (os_speed + ts_speed) too small")
                self.b_coeff = -d_obs / (V * np.log(cr_obs))
            else:
                # Default: assume typical speeds (e.g., 2 m/s each -> 4 m/s combined)
                default_V = 4.0  # m/s (combined speed for head-on)
                self.b_coeff = -d_obs / (default_V * np.log(cr_obs))
        else:
            self.b_coeff = b_coeff
        self.encounter_classifier = EncounterClassifier()
    
    def calculate_collision_risk(
        self,
        os_speed: float,
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        os_heading: float,
        ts_speed: float,
        ts_position: Tuple[float, float],
        ts_velocity: Tuple[float, float],
        ts_heading: float
    ) -> Dict[str, float]:
        """
        Collision Risk 계산
        
        Args:
            os_position: Own Ship 위치 (x, y) in meters
            os_velocity: Own Ship 속도 벡터 (vx, vy) in m/s
            os_heading: Own Ship 선수방위 (degrees, 0=북, 시계방향)
            ts_position: Target Ship 위치 (x, y) in meters
            ts_velocity: Target Ship 속도 벡터 (vx, vy) in m/s
        
        Returns:
            Dictionary containing:
                - cr: Collision Risk 값
                - dcpa: Distance at CPA (meters)
                - tcpa: Time to CPA (seconds)
                - relative_bearing: OS에서 TS로의 상대 방위각 (degrees)
                - ship_domain_radius: 해당 방위각에서 Ship Domain 반경
                - f_angle: 각도 보정 계수
        """
        # DCPA, TCPA 계산
        dcpa, tcpa = calculate_cpa_tcpa(
            os_position, os_velocity,
            ts_position, ts_velocity
        )
        
        # 상대 방위각 (Ship Domain은 OS 중심으로 회전)
        relative_bearing = calculate_relative_bearing(os_position, os_heading, ts_position)
        
        # 해당 방위각에서 Ship Domain 반경
        domain_radius = calculate_ship_domain_distance(
            relative_bearing, 
            self.ship_domain
        )    

        result = self.encounter_classifier.classify(
            os_position,
            os_heading,
            os_speed,
            ts_position,
            ts_heading,
            ts_speed
        )
        enc_type = result.encounter_type.value
        
        # f_angle 계산 (Paper: normally 1.0, but 0.25 when TCPA < 0 to prevent over-assessment)
        if tcpa < 0:
            # Ships are moving away from each other (already passed CPA)
            f_angle = 0.25
        else:
            f_angle = 1.0
        
        # Paper Eq. (6): CR = f_angle · exp(-DCPA/a) · exp(-TCPA/b)
        # Note: Use abs(tcpa) for TCPA term since when TCPA < 0, risk should decrease as time passes
        cr_dcpa = np.exp(-dcpa / self.a_coeff)
        cr_tcpa = np.exp(-abs(tcpa) / self.b_coeff)
        cr = f_angle * cr_dcpa * cr_tcpa
        
        return {
            'cr': cr,
            'dcpa': dcpa,
            'tcpa': tcpa,
            'encounter_type': enc_type,
            'relative_bearing': relative_bearing,
            'ship_domain_radius': domain_radius,
            'f_angle': f_angle,
            'cr_dcpa_component': cr_dcpa,
            'cr_tcpa_component': cr_tcpa
        }


class JeonCollisionRisk:
    """
    전도현 논문 (2024) 방법론의 Collision Risk Assessment
    
    비대칭 타원형 Ship Domain + 개선된 Mou 공식
    
    Paper Eq. (3):
    CR = cos( (π/2) * (1 - exp( - ( (TCPA / c_TCPA) + (DCPA / c_DCPA) ) ) ) )
    
    Attributes:
        ship_domain: Ship Domain 파라미터
        c_dcpa: DCPA 지수 계수 (meters)
        c_tcpa: TCPA 지수 계수 (seconds)
        d_obs: Recognition distance (meters) - OS가 TS를 인식하고 모니터링 시작하는 거리
        cr_obs: Allowable CR value - 회피 개시 기준이 되는 CR 값
    
    Notes:
        - CR = 1: Ship Domain 경계선
        - CR → 0: 위험 최소값 (DCPA or TCPA → ∞)
        - CR > 1: TS가 Ship Domain 내부에 침입
        
    계수 계산 (논문 식 4, 5):
        Eq. (4): c_DCPA = - d_obs / ln(1 - (2/π) * cos⁻¹(CR_obs))
        Eq. (5): c_TCPA = - d_obs / (ln(1 - (2/π) * cos⁻¹(CR_obs)) * (v_os + v_ts))
        
    Figure 2-5: TCPA = 0, DCPA = d_obs → CR = CR_obs
    Figure 2-6: DCPA = 0, TCPA = d_obs/(v_os + v_ts) → CR = CR_obs
    """
    
    def __init__(
        self,
        ship_domain: ShipDomainParams,
        d_obs: float = 200.0,
        cr_obs: float = 0.3,
        os_speed: Optional[float] = None,
        ts_speed: Optional[float] = None
    ):
        """
        Args:
            ship_domain: Ship Domain 파라미터
            d_obs: Recognition distance (meters)
            cr_obs: Allowable CR value (기본값 0.3)
            os_speed: Own Ship 속도 (m/s), c_tcpa 자동 계산용
            ts_speed: Target Ship 속도 (m/s), c_tcpa 자동 계산용
        """
        self.ship_domain = ship_domain
        self.d_obs = d_obs
        self.cr_obs = cr_obs
        
        # Paper Eq. (4): c_DCPA = - d_obs / ln(1 - (2/π) * cos⁻¹(CR_obs))
        # Figure 2-5: TCPA = 0, DCPA = d_obs → CR = CR_obs
        # Substituting into Eq. (3): CR_obs = cos((π/2) * (1 - exp(-d_obs/c_DCPA)))
        # Solving: c_DCPA = - d_obs / ln(1 - (2/π) * cos⁻¹(CR_obs))
        acos_cr_obs = np.arccos(cr_obs)  # cos⁻¹(CR_obs)
        denominator = np.log(1.0 - (2.0 / np.pi) * acos_cr_obs)
        if abs(denominator) < 1e-10:
            raise ValueError(f"Invalid cr_obs={cr_obs}: denominator is too close to zero")
        self.c_dcpa = -d_obs / denominator
        
        # Paper Eq. (5): c_TCPA = - d_obs / (ln(1 - (2/π) * cos⁻¹(CR_obs)) * (v_os + v_ts))
        # Figure 2-6: DCPA = 0, TCPA = d_obs/(v_os + v_ts) → CR = CR_obs
        if os_speed is not None and ts_speed is not None:
            self.c_tcpa = -d_obs / (denominator * (os_speed + ts_speed))
        else:
            # Default: assume typical speeds (e.g., 5 m/s each)
            default_speed = 2.0  # m/s
            self.c_tcpa = -d_obs / (denominator * (default_speed + default_speed))

        self.encounter_classifier = EncounterClassifier()
    
    def calculate_collision_risk(
        self,
        os_speed: float,
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        os_heading: float,
        ts_speed: float,
        ts_position: Tuple[float, float],
        ts_velocity: Tuple[float, float],
        ts_heading: float
    ) -> Dict[str, float]:
        """
        Collision Risk 계산
        
        Args:
            os_position: Own Ship 위치 (x, y) in meters
            os_velocity: Own Ship 속도 벡터 (vx, vy) in m/s
            os_heading: Own Ship 선수방위 (degrees, 0=북, 시계방향)
            ts_position: Target Ship 위치 (x, y) in meters
            ts_velocity: Target Ship 속도 벡터 (vx, vy) in m/s
        
        Returns:
            Dictionary containing collision risk metrics
        """
        # DCPA, TCPA 계산
        dcpa, tcpa = calculate_cpa_tcpa(
            os_position, os_velocity,
            ts_position, ts_velocity
        )
        
        # 상대 방위각
        relative_bearing = calculate_relative_bearing(os_position, os_heading, ts_position)
        
        # 해당 방위각에서 Ship Domain 반경
        domain_radius = calculate_ship_domain_distance(
            relative_bearing,
            self.ship_domain
        )     

        result = self.encounter_classifier.classify(
            os_position,
            os_heading,
            os_speed,
            ts_position,
            ts_heading,
            ts_speed
        )
        enc_type = result.encounter_type.value
        
        # Paper Eq. (3): CR = cos( (π/2) * (1 - exp( - ( (TCPA / c_TCPA) + (DCPA / c_DCPA) ) ) ) )
        # Note: TCPA can be negative (already passed CPA), use absolute value
        tcpa_norm = abs(tcpa) / self.c_tcpa if self.c_tcpa > 0 else 0.0
        dcpa_norm = dcpa / self.c_dcpa if self.c_dcpa > 0 else 0.0
        
        # Inner exponential: exp(-(TCPA/c_TCPA + DCPA/c_DCPA))
        inner_exp = np.exp(-(tcpa_norm + dcpa_norm))
        
        # CR = cos((π/2) * (1 - inner_exp))
        cr = np.cos((np.pi / 2.0) * (1.0 - inner_exp))
        
        return {
            'cr': cr,
            'dcpa': dcpa,
            'tcpa': tcpa,
            'encounter_type': enc_type,
            'relative_bearing': relative_bearing,
            'ship_domain_radius': domain_radius,
            'tcpa_norm': tcpa_norm,
            'dcpa_norm': dcpa_norm,
            'inner_exp': inner_exp
        }
