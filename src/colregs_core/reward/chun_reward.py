"""
Chun et al. Reward Function

References:
- Chun et al. (2021). "Collision avoidance algorithm based on deep reinforcement learning
for an autonomous ship." Ocean Engineering 234 (2021) 109216
- Chun et al. (2024). "Method for collision avoidance based on deep reinforcement learning 
with path-speed control for an autonomous ship." IJNAOE 16 (2024) 100579
"""
import numpy as np
from typing import Tuple, Optional, Dict, List
from ..risk import JeonCollisionRisk, ChunCollisionRisk, ShipDomainParams
from ..encounter.types import EncounterType
from ..encounter.classifier import EncounterClassifier
from ..geometry import velocity_to_heading_speed
from .colregs_compliant import ColregsCompliant


class ChunRewardCalculator:
    """
    Chun et al. (2021, 2024) 보상함수 계산기
    
    Parameters:
        d_cl: Maximum cross-track error 허용 거리 (m)
        v_ref: 목표 속도 (m/s)
        cr_allowable: 회피 시작 기준 CR 값 (0~1)
        dt: 시뮬레이션 timestep (sec)
        ship_domain: Ship Domain 파라미터 (optional, 기본값 사용)
        d_obs: Recognition distance (m) - OS가 TS를 인식하고 모니터링 시작하는 거리
        phi_max: 최대 heading 차이 (degrees)
        cr_method: 'jeon' or 'chun'
        os_speed_for_cr: OS speed for CR calculation (Jeon only)
        ts_speed_for_cr: TS speed for CR calculation (Jeon only)
        check_point_reward_enabled: Enable check point reward (Chun 2021 feature)
        maintain_reward_enabled: Enable COLREGs-maintain reward (Chun 2021 feature)
    """
    
    def __init__(
        self,
        d_cl: float = 10.0,              # 10m
        v_ref: float = 2.0,               # 3 m/s
        cr_allowable: float = 0.3,        # CR 0.3 이하는 안전
        dt: float = 0.1,                  # 0.1 second
        ship_domain: Optional[ShipDomainParams] = None,
        d_obs: float = 200.0,             # 100m
        phi_max: float = 4.0,            # 45 degrees
        cr_method: str = 'chun',          # 'jeon' or 'chun'
        os_speed_for_cr: float = 2.0,     # OS speed for CR calculation (Jeon)
        ts_speed_for_cr: float = 2.0,     # TS speed for CR calculation (Jeon)
        check_point_reward_enabled: bool = False,  # Chun 2021 feature
        maintain_reward_enabled: bool = False      # Chun 2021 feature
    ):
        self.d_cl = d_cl
        self.v_ref = v_ref
        self.cr_allowable = cr_allowable
        self.dt = dt
        self.d_obs = d_obs
        self.phi_max = phi_max
        self.cr_method = cr_method.lower()
        self.check_point_reward_enabled = check_point_reward_enabled
        self.maintain_reward_enabled = maintain_reward_enabled

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
                d_obs=self.d_obs,
                cr_obs=self.cr_allowable,
                os_speed=os_speed_for_cr,
                ts_speed=ts_speed_for_cr
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
        Chun et al. (2021) Formula (Eq. 16):
        R_Goal = (L_t - L_(t+1)) / (V_O * dt)
        
        where:
        - L_t = previous_distance (distance from previous position to goal)
        - L_(t+1) = current_distance (distance from current position to goal)
        - V_O = os_speed (speed of OS)
        - dt = self.dt (time step)
        
        Returns:
            Goal reward [-1, +1]
            - 목표에 가장 가까워지는 경우: +1
            - 목표에서 가장 멀어지는 경우: -1
        """
        # Handle first step (previous_distance is None)
        if previous_distance is None:
            return 0.0
        
        # 거리 변화량: L_t - L_(t+1)
        # 양수면 접근 (좋음), 음수면 이탈 (나쁨)
        d_change = previous_distance - current_distance
        
        # 이상적인 거리 변화량: V_O * dt
        ideal_change = os_speed * self.dt
        
        if abs(ideal_change) < 1e-6:
            return 0.0
        
        # Chun formula (Eq. 16): R_Goal = (L_t - L_(t+1)) / (V_O * dt)
        r_goal = d_change / ideal_change
        
        return np.clip(r_goal, -1.0, 1.0)
    
    def calculate_cross_reward(
        self,
        cross_track_error: float
    ) -> float:
        """
        Chun et al. (2021) Formula (Eq. 17):
        R_Cross = (d_cl/2 - y_e) / (d_cl/2)
        
        where:
        - y_e = cross_track_error (distance from OS to reference path)
        - d_cl = self.d_cl (maximum allowable cross-track error)
        
        When y_e = 0 (on path): R_Cross = 1.0
        When y_e = d_cl/2 (half max deviation): R_Cross = 0.0
        When y_e = d_cl (max deviation): R_Cross = -1.0
        
        Returns:
            Cross reward [-1, +1]
            - 경로 상 (y_e = 0): +1
            - 최대 이탈 (y_e = d_cl): -1
        """
        if self.d_cl < 1e-6:
            return 1.0
        
        # y_e: cross-track error (use absolute value since formula assumes unsigned)
        y_e = abs(cross_track_error)
        
        # Chun formula (Eq. 17): R_Cross = (d_cl/2 - y_e) / (d_cl/2)
        r_cross = (self.d_cl / 2.0 - y_e) / (self.d_cl / 2.0)
        
        return np.clip(r_cross, -1.0, 1.0)
    
    def calculate_speed_reward(
        self,
        os_speed: float
    ) -> float:
        """
        Chun et al. (2024) Formula (Eq. 11):
        R_speed = (2*v_O - v_ref) / v_ref          if v_O <= v_ref
        R_speed = (3*v_ref - 2*v_O) / v_ref        if v_O > v_ref
        
        where:
        - v_O = os_speed (current speed of OS)
        - v_ref = self.v_ref (target/reference speed)
        
        When v_O = 0: R_speed = -1.0
        When v_O = v_ref: R_speed = 1.0
        When v_O = 1.5*v_ref: R_speed = 0.0
        When v_O >= 1.5*v_ref: R_speed < 0 (decreasing)
        
        Returns:
            Speed reward (typically [-1, +1] but can exceed these bounds)
            - 목표 속도 유지 (v_O = v_ref): +1
            - 속도 0 (v_O = 0): -1
        """
        if self.v_ref < 1e-6:
            return 0.0
        
        # Chun 2024 formula (Eq. 11):
        if os_speed <= self.v_ref:
            r_speed = (2.0 * os_speed - self.v_ref) / self.v_ref
        else:
            r_speed = (3.0 * self.v_ref - 2.0 * os_speed) / self.v_ref
        
        # Clip to reasonable range
        return np.clip(r_speed, -1.0, 1.0)
    
    def calculate_check_point_reward(
        self,
        check_point_position: Optional[Tuple[float, float]],
        next_os_position: Tuple[float, float],
        sailing_distance_between_checkpoints: float,
        os_speed: float,
        epsilon_check: float = 50.0
    ) -> float:
        """
        Chun et al. (2021) Formula (Eq. 18):
        R_Check = D_Check / (V_O * dt)     if ||P_Check - P_O,t+1|| < ε_Check
        R_Check = -D_Check / (V_O * dt)    otherwise
        
        where:
        - D_Check = sailing_distance_between_checkpoints
        - V_O = os_speed
        - dt = self.dt
        - P_Check = check_point_position
        - P_O,t+1 = next_os_position
        - ε_Check = epsilon_check (margin for recognizing passing)
        
        Args:
            check_point_position: (x, y) position of the check point, None if no check point
            next_os_position: (x, y) next position of the OS
            sailing_distance_between_checkpoints: Distance between check points
            os_speed: Speed of OS
            epsilon_check: Margin for recognizing check point passage (default 50m)
        
        Returns:
            Check point reward (positive if passing, negative otherwise, 0 if no check point)
        """
        if not self.check_point_reward_enabled or check_point_position is None:
            return 0.0
        
        if abs(os_speed * self.dt) < 1e-6:
            return 0.0
        
        # Calculate distance from next OS position to check point
        dx = next_os_position[0] - check_point_position[0]
        dy = next_os_position[1] - check_point_position[1]
        distance_to_checkpoint = np.sqrt(dx**2 + dy**2)
        
        # Base reward magnitude
        reward_magnitude = sailing_distance_between_checkpoints / (os_speed * self.dt)
        
        # Check if OS passed the check point
        if distance_to_checkpoint < epsilon_check:
            return reward_magnitude  # Positive reward
        else:
            return -reward_magnitude  # Negative penalty
    
    def calculate_collision_reward(
        self,
        cr: float
    ) -> float:
        """
        Chun et al. (2024) Formula (Eq. 12) - same formula, different naming:
        R_risk = 1                                                     if CR <= CR_allowable
        R_risk = -2/(1-CR_allowable) * CR + (1+CR_allowable)/(1-CR_allowable)  if CR > CR_allowable
        
        where:
        - CR: Collision Risk (calculated using ChunCollisionRisk or JeonCollisionRisk)
        - CR_al / CR_allowable: Allowable collision risk threshold (self.cr_allowable)
        
        Args:
            cr: Collision risk
        
        Returns:
            Collision/Risk reward [-1, +1]
            - CR < CR_allowable (안전): +1
            - CR = 1 (충돌): -1
        """
        # Paper formula (Eq. 12):
        # if CR <= CR_obs: R_cr = 1.0
        if cr <= self.cr_allowable:
            return 1.0
        
        # else: R_cr = ((1+CR_obs)/(1-CR_obs)) - (2/(1-CR_obs)) * CR
        denominator = 1.0 - self.cr_allowable
        if abs(denominator) < 1e-6:
            return -1.0
        
        r_collision = ((1.0 + self.cr_allowable) / denominator) - (2.0 / denominator) * cr
        
        return np.clip(r_collision, -1.0, 1.0)
    
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
        Chun et al. (2021) Formula (Eq. 20):
        R_COLREGs = r_COLREGs      if satisfies COLREGs 13-17
        R_COLREGs = -r_COLREGs     otherwise
        where r_COLREGs = 1
        
        Chun et al. (2024) Formula (Eq. 13):
        R_colregs = 1    if CR <= CR_allowable or satisfies rules
        R_colregs = -1   otherwise
        
        Args:
            encounter_type: Optional encounter type. If None, automatically classified
            os_heading: Own Ship heading (degrees)
        
        Returns:
            COLREGs reward {-1, +1}
            - CR <= CR_allowable (안전) or is_compliant (규칙 준수): +1
            - CR > CR_allowable and not is_compliant (규칙 위반): -1
        """
        # Chun 2024 formula (Eq. 13)
        # R_colregs = 1 if CR <= CR_allowable or satisfies rules (is_compliant)
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
        Chun et al. (2024) Formula (Eq. 14):
        R_heading = 1                          if φ_diff = 0
        R_heading = 1 - 2 * (φ_diff / φ_max)   if 0 < φ_diff <= φ_max
        R_heading = -1                         if φ_diff > φ_max
        
        where:
        - φ_diff = |φ_before - φ_after|
        - φ_max = self.phi_max (maximum shift in heading angle)
        
        Args:
            os_heading: Current OS heading (degrees)
            previous_heading: Previous OS heading (degrees)
        
        Note: Both headings must be in the same units as phi_max (degrees).
        
        Returns:
            Heading reward [-1, +1]
            - 방향 유지 (φ_diff = 0): +1
            - 최대 변화 (φ_diff = φ_max): -1
        """
        # Handle first step (previous_heading is None)
        if previous_heading is None:
            return 0.0
        
        # Calculate heading difference
        phi_diff = abs(os_heading - previous_heading)
        
        # Chun 2024 formula (Eq. 14):
        if phi_diff < 1e-6:
            return 1.0
        elif phi_diff > self.phi_max:
            return -1.0
        else:
            r_heading = 1.0 - 2.0 * phi_diff / self.phi_max
            return np.clip(r_heading, -1.0, 1.0)
    
    def calculate_maintain_reward(
        self,
        os_heading: float,
        previous_heading: Optional[float],
        cr: float
    ) -> float:
        """
        Chun et al. (2021) Formula (Eq. 21):
        R_Maintain = r_Maintain     if satisfies COLREGs 8b
        R_Maintain = -r_Maintain    otherwise
        where r_Maintain = 1
        
        COLREGs Rule 8b: "Any alteration of course and/or speed to avoid collision
        shall, if the circumstances of the case admit, be large enough to be readily
        apparent to another vessel observing visually or by radar; a succession of
        small alterations of course and/or speed should be avoided."
        
        This reward encourages OS to maintain its direction to avoid ambiguity.
        In practice, we check if OS maintains heading when not in collision risk situation.
        
        Args:
            os_heading: Current OS heading (degrees)
            previous_heading: Previous OS heading (degrees)
            cr: Current collision risk (to determine if avoidance is needed)
        
        Returns:
            Maintain reward {-1, +1}
            - Maintains direction when safe: +1
            - Frequent small alterations: -1
        """
        if not self.maintain_reward_enabled:
            return 0.0
        
        # Handle first step
        if previous_heading is None:
            return 0.0
        
        # Calculate heading difference
        phi_diff = abs(os_heading - previous_heading)
        
        # If CR is low (safe situation), reward maintaining direction
        # If CR is high (collision risk), allow heading changes
        if cr < self.cr_allowable:
            # Safe situation - maintain direction is good
            if phi_diff < 5.0:  # Small threshold for "maintaining"
                return 1.0
            else:
                return -1.0
        else:
            # Collision risk situation - heading changes are acceptable
            return 0.0
    
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

        # Check point parameters (optional, Chun 2021)
        check_point_position: Optional[Tuple[float, float]] = None,
        next_os_position: Optional[Tuple[float, float]] = None,
        sailing_distance_between_checkpoints: float = 0.0,

        # Weights
        w_efficiency: float = 1.0,
        w_safety: float = 1.0
    ) -> Dict[str, float]:
        """
        전체 보상 계산
        Total Reward = w_efficiency * Re + w_safety * Rs
        
        Efficiency Rewards (Re):
        - Goal, Cross, Speed, (Check Point)
        
        Safety Rewards (Rs):
        - Collision/Risk, COLREGs, Heading, (Maintain)
        
        Returns:
            Dictionary with all reward components:
            - r_goal, r_cross, r_speed, r_check (if enabled)
            - r_collision, r_colregs, r_heading, r_maintain (if enabled)
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
            current_distance, previous_distance, os_speed
        )
        r_cross = self.calculate_cross_reward(cross_track_error)
        r_speed = self.calculate_speed_reward(os_speed)
        r_efficiency = r_goal + r_cross + r_speed
        
        # Check point reward (Chun 2021 feature)
        r_check = 0.0
        if self.check_point_reward_enabled and next_os_position is not None:
            r_check = self.calculate_check_point_reward(
                check_point_position, next_os_position,
                sailing_distance_between_checkpoints, os_speed
            )
            r_efficiency += r_check
        
        # Safety rewards
        r_collision = self.calculate_collision_reward(cr)
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
        r_safety = r_collision + r_colregs + r_heading
        
        # Maintain reward (Chun 2021 feature)
        r_maintain = 0.0
        if self.maintain_reward_enabled:
            r_maintain = self.calculate_maintain_reward(
                os_heading, previous_heading, cr
            )
            r_safety += r_maintain
        
        # Total reward
        r_total = w_efficiency * r_efficiency + w_safety * r_safety
        
        return {
            'r_goal': r_goal,
            'r_cross': r_cross,
            'r_speed': r_speed,
            'r_check': r_check,
            'r_collision': r_collision,
            'r_colregs': r_colregs,
            'r_heading': r_heading,
            'r_maintain': r_maintain,
            'r_efficiency': r_efficiency,
            'r_safety': r_safety,
            'r_total': r_total
        }
