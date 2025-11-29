import numpy as np
from typing import Tuple, Optional, Dict
from ..risk import JeonCollisionRisk, ShipDomainParams
from ..encounter.types import EncounterType
from ..encounter.classifier import EncounterClassifier
from ..utils import WrapTo180
from .colregs_compliant import ColregsCompliant

class MyReward:
    """
    Custom Reward Calculator based on Jeon et al. (2024) structure.
    Modify the formulas in each method to implement your own reward logic.
    """
    
    def __init__(
        self,
        d_max: float = 30.0,             # Max cross-track error (m)
        v_ref: float = 3.0,              # Reference speed (m/s)
        cr_allowable: float = 0.3,       # Collision Risk threshold
        dt: float = 0.1,                 # Time step (s)
        ship_domain: Optional[ShipDomainParams] = None,
        d_obs: float = 200.0,            # Observation distance (m)
        phi_max: float = 4.0,            # Max heading deviation (deg)
        os_speed_for_cr: float = 3.0,    # OS speed for CR calculation
        ts_speed_for_cr: float = 3.0,    # TS speed for CR calculation
        fuel_weight: float = 0.1,        # Weight for fuel reward
        smoothness_weight: float = 0.1,  # Weight for smoothness reward
        r_weight: float = 1.0,           # Relative weight for yaw rate smoothness
        n_max: float = 1092.0            # Max RPM for normalization (992 RPM)
    ):
        self.d_max = d_max
        self.v_ref = v_ref
        self.cr_allowable = cr_allowable
        self.dt = dt
        self.d_obs = d_obs
        self.phi_max = phi_max
        
        self.fuel_weight = fuel_weight
        self.smoothness_weight = smoothness_weight
        self.r_weight = r_weight
        self.n_max = n_max

        if ship_domain is None:
            ship_domain = ShipDomainParams(
                r_bow=6.0,
                r_stern=2.0,
                r_starboard=6.0,
                r_port=2.0
            )
        self.ship_domain = ship_domain
        
        self.cr_calculator = JeonCollisionRisk(
            ship_domain=self.ship_domain,
            d_obs=self.d_obs,
            cr_obs=self.cr_allowable,
            os_speed=os_speed_for_cr,
            ts_speed=ts_speed_for_cr
        )
        
        # Encounter classifier for automatic encounter type detection
        self.encounter_classifier = EncounterClassifier()
        self.colregs_checker = ColregsCompliant()
    
    def calculate_goal_reward(
        self,
        current_distance: float,
        previous_distance: Optional[float],
        os_speed: float
    ) -> float:
        if previous_distance is None:
            return 0.0
        
        d_cross = previous_distance - current_distance
        ideal_change = os_speed * self.dt
        
        if abs(ideal_change) < 1e-6:
            return 0.0
        
        r_goal = d_cross / ideal_change
        return np.clip(r_goal, -1.0, 1.0)

    def calculate_cross_reward(
        self, 
        cross_track_error: float,
        CR_max: float
    ) -> float:
        """
        Dynamic Weighting Strategy:
        Soft-Switching: Reduce CTE penalty as Risk increases to allow avoidance maneuvers.
        """
        if self.d_max < 1e-6: return 1.0
        
        raw_cross = 1.0 - 2.0 * abs(cross_track_error) / self.d_max
        
        # Risk Factor: 0.0 (Safe) -> 1.0 (Critical)
        # Smooth transition instead of hard if/else
        risk_factor = np.clip((CR_max - 0.1) / (self.cr_allowable + 0.2), 0.0, 1.0)
        
        # If risk is high, dynamic_weight becomes 0 (Ignore Path)
        # If safe, dynamic_weight becomes 1 (Follow Path)
        dynamic_weight = 1.0 - risk_factor 
        
        return dynamic_weight * np.clip(raw_cross, -1.0, 1.0)
    
    def calculate_speed_reward(
        self,
        os_speed: float
    ) -> float:
        if self.v_ref < 1e-6:
            return 0.0
        
        v_e = self.v_ref - os_speed
        r_speed = 1.0 - 2.0 * abs(v_e) / self.v_ref
        return np.clip(r_speed, -1.0, 1.0)
    
    def calculate_risk_reward(
        self,
        cr: float
    ) -> float:
        if cr < self.cr_allowable:
            return 1.0
        
        denominator = 1.0 - self.cr_allowable
        if abs(denominator) < 1e-6:
            return -1.0
        
        r_risk = ((1.0 + self.cr_allowable) / denominator) - (2.0 / denominator) * cr
        return np.clip(r_risk, -1.0, 1.0)
    
    def calculate_colregs_reward(
        self,
        cr: float,
        tcpa: float,
        encounter_type: Optional[EncounterType] = None,
        is_static_obstacle: bool = False,
        relative_course: Optional[float] = None,
    ) -> float:
        compliance_reward = self.colregs_checker.is_compliant(
            relative_course=relative_course,
            tcpa=tcpa,
            encounter_type=encounter_type,
            is_static_obstacle=is_static_obstacle
        )

        if cr <= self.cr_allowable:
            return 1.0
        
        return compliance_reward

    def calculate_heading_reward(
        self,
        os_heading: float,
        previous_heading: Optional[float],
    ) -> float:
        if previous_heading is None:
            return 0.0
        
        phi_diff = abs(WrapTo180(os_heading - previous_heading))
        
        if phi_diff < 1e-6:
            return 1.0
        elif phi_diff > self.phi_max:
            return -1.0
        else:
            r_heading = 1.0 - 2.0 * phi_diff / self.phi_max
            return np.clip(r_heading, -1.0, 1.0)

    def calculate_fuel_reward(
        self,
        n_left: float,
        n_right: float
    ) -> float:
        if abs(n_left) < 1e-6 and abs(n_right) < 1e-6:
            return 0.0
        
        nl_norm = n_left / self.n_max
        nr_norm = n_right / self.n_max

        power_proxy = abs(nl_norm)**3 + abs(nr_norm)**3
        r_fuel = -self.fuel_weight * power_proxy
        return np.clip(r_fuel, -1.0, 1.0)

    def calculate_smooth_reward(
        self,
        action: Tuple[float, float],
        previous_action: Optional[Tuple[float, float]]
    ) -> float:
        """
        Action Smoothness (Jerk) Penalty.
        """
        if previous_action is None: return 0.0
        
        u_curr, r_curr = action[0], action[1]
        u_prev, r_prev = previous_action[0], previous_action[1]
        
        u_diff = abs(u_curr - u_prev)
        r_diff = abs(r_curr - r_prev)
        
        # Penalize Yaw rate change more heavily
        jerk_cost = u_diff + self.r_weight * r_diff
        
        return np.clip(-self.smoothness_weight * jerk_cost, -1.0, 0.0)
    
    def calculate_total_reward(
        self,
        current_distance: float,
        previous_distance: Optional[float],
        cross_track_error: float,
        os_speed: float,
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        os_heading: float,
        previous_heading: Optional[float],
        ts_speed: float,
        ts_position: Tuple[float, float],
        ts_velocity: Tuple[float, float],
        ts_heading: float,
        CR_max: float,
        action: Tuple[float, float],
        previous_action: Optional[Tuple[float, float]],
        encounter_type: Optional[EncounterType] = None,
        is_static_obstacle: bool = False,
        relative_course: Optional[float] = None,
        n_left: float = 0.0,
        n_right: float = 0.0,
        previous_velocity: Optional[Tuple[float, float]] = None,
        w_efficiency: float = 1.0,
        w_safety: float = 1.0
    ) -> Dict[str, float]:
        result = self.cr_calculator.calculate_collision_risk(
            os_speed, os_position, os_velocity, os_heading,
            ts_speed, ts_position, ts_velocity, ts_heading
        )
        cr = CR_max
        tcpa = result['tcpa']

        r_goal = self.calculate_goal_reward(current_distance, previous_distance, os_speed)
        r_cross = self.calculate_cross_reward(cross_track_error, CR_max)
        r_speed = self.calculate_speed_reward(os_speed)
        
        r_fuel = self.calculate_fuel_reward(n_left, n_right)
        r_smooth = self.calculate_smooth_reward(action, previous_action)
        
        r_efficiency = r_goal + r_cross + r_speed + r_fuel + r_smooth
        
        r_risk = self.calculate_risk_reward(cr)
        r_colregs = self.calculate_colregs_reward(
            cr=cr, tcpa=tcpa, encounter_type=encounter_type,
            is_static_obstacle=is_static_obstacle, relative_course=relative_course
        )
        r_heading = self.calculate_heading_reward(os_heading, previous_heading)
        
        r_safety = r_risk + r_colregs + r_heading
        
        r_total = w_efficiency * r_efficiency + w_safety * r_safety
        
        return {
            'r_goal': r_goal,
            'r_cross': r_cross,
            'r_speed': r_speed,
            'r_fuel': r_fuel,
            'r_smooth': r_smooth,
            'r_risk': r_risk,
            'r_colregs': r_colregs,
            'r_heading': r_heading,
            'r_efficiency': r_efficiency,
            'r_safety': r_safety,
            'r_total': r_total
        }