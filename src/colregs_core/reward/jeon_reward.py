"""
CR 기반의 보상 함수

References:
- Chun et al. (2024). "Method for collision avoidance based on deep reinforcement learning 
with path-speed control for an autonomous ship."
- 전도현 (2024). "A Method for Collision Avoidance of a Ship Based on Reinforcement Learning
in Complex Maritime Situations(복잡한 해상 상황에서의 강화 학습 기반 선박 충돌 회피 방법)"
"""
import numpy as np
from typing import Tuple, Optional, Dict
from ..risk import JeonCollisionRisk, ChunCollisionRisk, ShipDomainParams
from ..encounter.types import EncounterType
from ..encounter.classifier import EncounterClassifier
from ..utils import WrapTo180
from .colregs_compliant import ColregsCompliant


class JeonRewardCalculator:
    """
    전도현 논문 (2024) 보상함수 계산기
    
    Parameters:
        d_max: Cross-track error 최대 허용 거리 (m)
        v_ref: 목표 속도 (m/s)
        cr_allowable: 회피 시작 기준 CR 값 (0~1)
        dt: 시뮬레이션 timestep (sec)
        ship_domain: Ship Domain 파라미터 (optional, 기본값 사용)
        d_obs: Recognition distance (m) - OS가 TS를 인식하고 모니터링 시작하는 거리
        phi_max: 최대 heading 차이 (degrees)
    """
    
    def __init__(
        self,
        d_max: float = 50.0,             # 50m
        v_ref: float = 2.0,              # 3 m/s
        cr_allowable: float = 0.3,       # CR 0.3 이하는 안전
        dt: float = 0.1,                 # 1 second
        ship_domain: Optional[ShipDomainParams] = None,
        d_obs: float = 200.0,            # 200m
        phi_max: float = 4.0,            # 4 degrees 
        cr_method: str = 'jeon',         # 'jeon' or 'chun'
        os_speed_for_cr: float = 2.0,    # OS speed for CR calculation (Jeon)
        ts_speed_for_cr: float = 2.0     # TS speed for CR calculation (Jeon)
    ):
        self.d_max = d_max
        self.v_ref = v_ref
        self.cr_allowable = cr_allowable
        self.dt = dt
        self.d_obs = d_obs
        self.phi_max = phi_max
        self.cr_method = cr_method.lower()

        # Default ship domain if not provided
        if ship_domain is None:
            ship_domain = ShipDomainParams(
                r_bow=6.0,
                r_stern=2.0,
                r_starboard=6.0,
                r_port=2.0
            )
        self.ship_domain = ship_domain
        
        # Initialize CR calculator based on method
        if self.cr_method == 'jeon':
            self.cr_calculator = JeonCollisionRisk(
                ship_domain=self.ship_domain,
                d_obs=self.d_obs,
                cr_obs=self.cr_allowable,
                os_speed=os_speed_for_cr,
                ts_speed=ts_speed_for_cr
            )
        elif self.cr_method == 'chun':
            self.cr_calculator = ChunCollisionRisk(
                ship_domain=self.ship_domain,
                a_coeff=None, # Auto-calculate
                b_coeff=None  # Auto-calculate
            )
        else:
            raise ValueError(f"Unknown CR method: {cr_method}. Use 'jeon' or 'chun'.")
        
        # Encounter classifier for automatic encounter type detection
        self.encounter_classifier = EncounterClassifier()
        self.colregs_checker = ColregsCompliant()
    
    def calculate_goal_reward(
        self,
        current_distance: float,
        previous_distance: Optional[float],
        os_speed: float
    ) -> float:
        """
        Paper Formula (Eq. 20):
        R_goal = (L_(t-1) - L_t) / (v_O * dt)
        
        where:
        - L_(t-1) = previous_distance (distance from previous position to goal)
        - L_t = current_distance (distance from current position to goal)
        - v_O = os_speed (speed of OS)
        - dt = self.dt (time step)
        
        Returns:
            Goal reward [-1, +1]
            - 목표에 가장 가까워지는 경우: +1
            - 목표에서 가장 멀어지는 경우: -1
        """
        # Handle first step (previous_distance is None)
        if previous_distance is None:
            return 0.0
        
        # 거리 변화량: L_(t-1) - L_t
        # 양수면 접근 (좋음), 음수면 이탈 (나쁨)
        d_cross = previous_distance - current_distance
        
        # 이상적인 거리 변화량: v_O * dt
        ideal_change = os_speed * self.dt
        
        if abs(ideal_change) < 1e-6:
            return 0.0
        
        # Paper formula (Eq. 20): R_goal = (L_(t-1) - L_t) / (v_O * dt)
        r_goal = d_cross / ideal_change
        
        return np.clip(r_goal, -1.0, 1.0)
    
    def calculate_cross_reward(
        self,
        cross_track_error: float
    ) -> float:
        """
        Paper Formula (Eq. 21):
        R_cross = (d_cl / 2 - y_e) / (d_cl / 2) = 1 - 2 * y_e / d_cl
        
        where:
        - y_e = cross_track_error (distance from OS to reference path)
        - d_cl = self.d_max (maximum allowable cross-track error)
        
        When y_e = 0 (on path): R_cross = 1.0
        When y_e = d_cl (max deviation): R_cross = -1.0
        
        Returns:
            Cross reward [-1, +1]
            - 경로 상 (y_e = 0): +1
            - 최대 이탈 (y_e = d_cl): -1
        """
        if self.d_max < 1e-6:
            return 1.0
        
        # y_e: cross-track error 
        y_e = cross_track_error
        
        # Paper formula (Eq. 21): R_cross = 1 - 2 * abs(y_e) / d_cl
        # Use abs(y_e) because y_e is now signed
        r_cross = 1.0 - 2.0 * abs(y_e) / self.d_max
        
        return np.clip(r_cross, -1.0, 1.0)
    
    def calculate_speed_reward(
        self,
        os_speed: float
    ) -> float:
        """
        Paper Formula (Eq. 22):
        R_speed = 1 - 2 * (|v_e| / v_desired)
        R_speed = -1 if |v_e| > 2 * v_desired
        
        where:
        - v_e = v_desired - v_current (speed error)
        - v_desired = self.v_ref (desired/reference speed)
        
        When |v_e| = 0 (current = desired): R_speed = 1.0
        When |v_e| = v_desired (current = 0 or 2*v_desired): R_speed = -1.0
        When |v_e| > 2*v_desired: R_speed = -1.0 (clipped)

        Returns:
            Speed reward [-1, +1]
            - 목표 속도 유지 (|v_e| = 0): +1
            - 속도 0 또는 2배 초과 (|v_e| = v_desired): -1
        """
        if self.v_ref < 1e-6:
            return 0.0
        
        # Speed error: v_e = v_desired - v_current
        v_e = self.v_ref - os_speed
        
        # Paper formula (Eq. 22): R_speed = 1 - 2 * (|v_e| / v_desired)
        r_speed = 1.0 - 2.0 * abs(v_e) / self.v_ref
        
        # Clip to [-1, 1] (explicit -1 for |v_e| > 2*v_desired)
        return np.clip(r_speed, -1.0, 1.0)
    
    
    def calculate_risk_reward(
        self,
        cr: float
    ) -> float:
        """
        Paper Formula (Eq. 23):
        R_cr = 1.0                    if CR < CR_obs
        R_cr = ((1+CR_obs)/(1-CR_obs)) - (2/(1-CR_obs)) * CR    if CR >= CR_obs
        
        where:
        - CR: Collision Risk (calculated using Eq. 3 via JeonCollisionRisk)
        - CR_obs: Allowable collision risk threshold (self.cr_allowable)
        
        According to the paper:
        - When CR = 0 (safe, no hazards): R_cr = 1.0
        - When CR < CR_obs: R_cr = 1.0 (fixed, safe margin)
        - When CR >= CR_obs: R_cr decreases linearly
        - When CR = 1 (collision): R_cr = -1.0
        - CR_obs is the criterion for OS to initiate collision avoidance
        
        Args:
            os_heading: Own Ship heading (degrees)
        
        Returns:
            Risk reward [-1, +1]
            - CR = 0 or CR < CR_obs (안전): +1
            - CR = 1 (충돌): -1
        """
        # Paper formula (Eq. 23):
        # if CR < CR_obs: R_cr = 1.0
        if cr < self.cr_allowable:
            return 1.0
        
        # else: R_cr = ((1+CR_obs)/(1-CR_obs)) - (2/(1-CR_obs)) * CR
        denominator = 1.0 - self.cr_allowable
        if abs(denominator) < 1e-6:
            return -1.0
        
        r_risk = ((1.0 + self.cr_allowable) / denominator) - (2.0 / denominator) * cr
        
        return np.clip(r_risk, -1.0, 1.0)
    
    def calculate_colregs_reward(
        self,
        os_heading: float,
        cr: float,
        tcpa: float,
        start_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        encounter_type: Optional[EncounterType] = None,
        is_static_obstacle: bool = False
    ) -> float:
        """
        Paper Formula (Eq. 24):
        R_COLREGs = 1    if CR <= CR_obs or satisfies rules
        R_COLREGs = -1   otherwise
        
        According to the paper:
        - R_COLREGs = 1 if:
          * TS's collision risk is lower than CR_obs (safe situation), OR
          * OS undertook a rule-compliant move in a collision avoidance situation
            where CR > CR_obs
        - R_COLREGs = -1 if:
          * In a collision avoidance situation (CR > CR_obs), OS did not travel
            according to COLREGs rules 12-17
        
        Args:
            encounter_type: Optional encounter type. If None, automatically classified
                          using EncounterClassifier
            os_heading: Own Ship heading (degrees)
        
        Returns:
            COLREGs reward {-1, +1}
            - CR <= CR_obs (안전) or is_compliant (규칙 준수): +1
            - CR > CR_obs and not is_compliant (규칙 위반): -1
        """

        # Check for COLREGs compliance
        is_compliant = self.colregs_checker.is_compliant(
            start_position=start_position,
            goal_position=goal_position,
            os_heading=os_heading,
            tcpa=tcpa,
            encounter_type=encounter_type,
            is_static_obstacle=is_static_obstacle
        )

        # Paper formula (Eq. 24):
        # R_COLREGs = 1 if CR <= CR_obs or satisfies rules (is_compliant)
        if cr <= self.cr_allowable or is_compliant:
            return 1.0
        else:
            # R_COLREGs = -1 otherwise (CR > CR_obs and not compliant)
            return -1.0

    def calculate_heading_reward(
        self,
        os_heading: float,
        previous_heading: Optional[float],
    ) -> float:
        """
        Chun et al. (2024)의 heading reward 만 차용
        Paper Formula (Eq. 14):
        R_heading = 1 - 2 * (phi_diff / self.phi_max)

        where:
        - phi_diff = |phi_bf - phi_af|
        
        Args:
            os_heading: Current OS heading (degrees)
            previous_heading: Previous OS heading (degrees)
        
        Note: Both headings must be in the same units as phi_max (degrees).
        """
        # Handle first step (previous_heading is None)
        if previous_heading is None:
            return 0.0
        
        # Use WrapTo180 to handle 0/360 degree boundary crossing
        # e.g., 359 -> 1 should be diff of 2, not 358
        phi_diff = abs(WrapTo180(os_heading - previous_heading))
        
        if phi_diff < 1e-6:
            return 1.0
        elif phi_diff > self.phi_max:
            return -1.0
        else:
            r_heading = 1.0 - 2.0 * phi_diff / self.phi_max
            return np.clip(r_heading, -1.0, 1.0)
    
    def calculate_yawrate_penalty(
        self,
        current_yaw_rate: float,
        cr: float,
        max_yaw_rate: float = 0.0873  # 5 deg/s in rad/s
    ) -> float:
        """
        직진 구간에서 불필요한 yaw rate 억제
        
        Args:
            current_yaw_rate: Current yaw rate (rad/s)
            cr: Collision Risk [0, 1]
            max_yaw_rate: Maximum expected yaw rate (rad/s)
        
        Returns:
            Penalty [-1, 0]
            - CR이 낮은 직진 구간에서 yaw rate가 클수록 큰 패널티
        """
        # CR이 높으면 (회피 중) 패널티 완화
        if cr > self.cr_allowable:
            return 0.0
        
        # CR이 낮은 직진 구간에서는 yaw rate 억제
        yaw_rate_ratio = abs(current_yaw_rate) / max_yaw_rate
        penalty = -yaw_rate_ratio  # [0, -1]
        
        return np.clip(penalty, -1.0, 0.0)
    
    def calculate_total_reward(
        self,
        # Efficiency reward parameters
        current_distance: float,
        previous_distance: Optional[float],
        cross_track_error: float,
        os_speed: float,
        # Safety reward parameters
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        os_heading: float,
        previous_heading: Optional[float],

        ts_speed: float,
        ts_position: Tuple[float, float],
        ts_velocity: Tuple[float, float],
        ts_heading: float,
        # COLREGs parameters
        start_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        CR_max: float,
        encounter_type: Optional[EncounterType] = None,
        is_static_obstacle: bool = False,
        # Weights
        w_efficiency: float = 1.0,
        w_safety: float = 1.0
    ) -> Dict[str, float]:
        """
        전체 보상 계산
        Total Reward = w_efficiency * Re + w_safety * Rs
        
        Returns:
            Dictionary with all reward components:
            - r_goal, r_cross, r_speed
            - r_risk, r_colregs, r_heading
            - r_efficiency, r_safety
            - r_total
        """
        # Use self.cr_calculator to calculate CR and TCPA
        result = self.cr_calculator.calculate_collision_risk(
            os_speed, os_position, os_velocity, os_heading,
            ts_speed, ts_position, ts_velocity, ts_heading
        )
        cr = CR_max
        tcpa = result['tcpa']

        # Efficiency rewards
        r_goal = self.calculate_goal_reward(
            current_distance, previous_distance,
            os_speed
        )
        r_cross = self.calculate_cross_reward(cross_track_error)
        r_speed = self.calculate_speed_reward(os_speed)
        r_efficiency = r_goal + r_cross + r_speed
        
        # Safety rewards
        r_risk = self.calculate_risk_reward(cr)
        r_colregs = self.calculate_colregs_reward(
            os_heading, cr, tcpa,
            start_position,
            goal_position,
            encounter_type,
            is_static_obstacle
        )
        r_heading = self.calculate_heading_reward(
            os_heading, previous_heading
        )

        # Yaw rate penalty (optional - only use if needed)
        # Get current yaw rate from os_velocity if available
        # For now, we'll skip this in total calculation
        # To enable: pass os_yaw_rate as parameter and uncomment below
        # r_yawrate = self.calculate_yawrate_penalty(os_yaw_rate, cr)
        
        r_safety = r_risk + r_colregs + r_heading
        
        # Total reward
        r_total = w_efficiency * r_efficiency + w_safety * r_safety
        
        return {
            'r_goal': r_goal,
            'r_cross': r_cross,
            'r_speed': r_speed,
            'r_risk': r_risk,
            'r_colregs': r_colregs,
            'r_heading': r_heading,
            'r_efficiency': r_efficiency,
            'r_safety': r_safety,
            'r_total': r_total
        }
