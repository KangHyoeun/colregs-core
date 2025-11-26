"""
COLREGs 규칙에 따른 올바른 회피 기동을 수행하는지 평가하는 함수
"""
import numpy as np
from typing import Tuple, Optional, Dict
from ..encounter.types import EncounterType
from ..utils import calculate_ref_path, WrapTo180


class ColregsCompliant:
    """
    Parameters:
        target_heading_giveway: Give-way 상황 목표 회피각 (degrees)
        target_heading_standon: Stand-on 상황 목표 회피각 (degrees)
        target_heading_static: Static obstacle 목표 회피각 (degrees)
    """
    
    def __init__(
        self,
        headon_threshold: float = 30.0,         # Head-on 임계값 (degrees)
        standon_threshold: float = 75.0,        # Stand-on 임계값 (degrees)
        overtaking_threshold: float = 120.0,           # TCPA 임계값 (seconds)
        giveway_threshold: float = 40.0,           # TCPA 임계값 (seconds)
        static_threshold: float = 45.0,          # Static 임계값 (degrees)
    ):
        self.headon_threshold = headon_threshold
        self.standon_threshold = standon_threshold
        self.overtaking_threshold = overtaking_threshold
        self.giveway_threshold = giveway_threshold
        self.static_threshold = static_threshold

    def is_compliant(
        self,
        relative_course: float,
        tcpa: float,
        encounter_type: Optional[EncounterType] = None,
        is_static_obstacle: bool = False
    ) -> float:

        if encounter_type == EncounterType.SAFE:
            return 0.0
        elif encounter_type == EncounterType.HEAD_ON:
            return self.is_compliant_headon(relative_course)
        elif encounter_type == EncounterType.CROSSING_STAND_ON:
            return self.is_compliant_standon(relative_course)
        elif encounter_type == EncounterType.CROSSING_GIVE_WAY:
            return self.is_compliant_giveway(tcpa)
        elif encounter_type == EncounterType.OVERTAKING:
            return self.is_compliant_overtaking(tcpa)
        elif is_static_obstacle:
            return self.is_compliant_static(relative_course)
        else:
            # For UNDEFINED or other cases, assume not compliant
            return 0.0

    def is_compliant_headon(
        self,
        relative_course: float,
    ) -> float:
        """
        Check if head-on avoidance maneuver is COLREGs-compliant.
        
        According to COLREGs Rule 14 and Woo & Kim (2020):
        - Must alter course to starboard (right) by at least 30°
        
        Args:
            relative_course: Relative course angle (degrees)
        
        Returns:
            float: reward value [-1.0, 1.0]
        """
        required_course_threshold = self.headon_threshold # 30도
        
        # 1. 좌현 변침 (Bad): relative_course < 0
        if relative_course < -0.01:   # deadzone 고려
            return -1.0 # 강력한 페널티
        
        # 2. 불충분한 우현 변침 (Weak Bad): 0 <= relative_course < required_course_threshold
        elif relative_course < required_course_threshold:   # deadzone 고려
            return -1.0 + (relative_course / required_course_threshold) * 1.0 # [-1.0, 0.0]
        
        # 3. 충분한 우현 변침 (Good): relative_course >= required_course_threshold
        else:
            reward_exp = np.exp((relative_course - required_course_threshold) * 0.02) - 1.0
            return np.clip(reward_exp, 0.0, 1.0) # [0.0, 1.0]

    def is_compliant_standon(
        self,
        relative_course: float,
    ) -> float:
        """
        Check if stand-on vessel's collision avoidance is COLREGs-compliant.
        
        According to COLREGs Rule 17(b) and Woo & Kim (2020):
        - When stand-on vessel must act, large course change required (75°)
        
        Args:
            relative_course: Relative course angle (degrees)
        
        Returns:
            float: reward value [-1.0, 1.0]
        """
        required_course_threshold = self.standon_threshold # 75도

        # 1. 좌현 변침 (Bad): relative_course < 0
        if relative_course < -0.01:   # deadzone 고려
            return -1.0 # 강력한 페널티
        
        # 2. 불충분한 우현 변침 (Weak Bad): 0 <= relative_course < required_course_threshold
        elif relative_course < required_course_threshold:   # deadzone 고려
            return -1.0 + (relative_course / required_course_threshold) * 1.0 # [-1.0, 0.0]
        
        # 3. 충분한 우현 변침 (Good): relative_course >= required_course_threshold
        else:
            reward_exp = np.exp((relative_course - required_course_threshold) * 0.02) - 1.0
            return np.clip(reward_exp, 0.0, 1.0) # [0.0, 1.0]

    def is_compliant_giveway(
        self,
        tcpa: float,
    ) -> bool:
        """
        Check if give-way vessel's avoidance is COLREGs-compliant.
        """
        tcpa_threshold = self.giveway_threshold # 40초 (기본값)

        # 1. TCPA가 임계값보다 작으면 (위험) -> 페널티
        if tcpa < tcpa_threshold:
            return -1.0 + (tcpa / tcpa_threshold) * 1.0 # [-1.0, 0.0)
        
        # 2. TCPA가 임계값 이상 (안전) -> 보상
        else:
            diff = tcpa - tcpa_threshold
            if diff > 200.0: return 1.0 # Overflow 방지
            
            reward_exp = np.exp(diff * 0.02) - 1.0
            return np.clip(reward_exp, 0.0, 1.0)

    def is_compliant_overtaking(
        self,
        tcpa: float,
    ) -> bool:
        """
        Check if overtaking vessel's avoidance is COLREGs-compliant.
        """
        tcpa_threshold = self.overtaking_threshold # 120초 (기본값)

        # 1. TCPA가 임계값보다 작으면 (위험) -> 페널티
        if tcpa < tcpa_threshold:
            return -1.0 + (tcpa / tcpa_threshold) * 1.0 # [-1.0, 0.0)
        
        # 2. TCPA가 임계값 이상 (안전) -> 보상
        else:
            diff = tcpa - tcpa_threshold
            if diff > 200.0: return 1.0 # Overflow 방지
            
            reward_exp = np.exp(diff * 0.02) - 1.0
            return np.clip(reward_exp, 0.0, 1.0)

    def is_compliant_static(
        self,
        relative_course: float,
    ) -> float:
        """
        Check if static obstacle avoidance is compliant.
        
        According to Woo & Kim (2020):
        - Either port or starboard turn acceptable

        Args:
            relative_course: Relative course angle (degrees)
        
        Returns:
            float: reward value [-1.0, 1.0]
        """
        required_course_threshold = self.static_threshold # 45도
        
        # 2. 불충분한 변침 (Weak Bad): abs(relative_course) < required_course_threshold
        if abs(relative_course) < required_course_threshold:   # deadzone 고려 
            return -1.0 + (abs(relative_course) / required_course_threshold) * 1.0 # [-1.0, 0.0]
        
        # 3. 충분한 변침 (Good): abs(relative_course) >= required_course_threshold
        else:
            reward_exp = np.exp((abs(relative_course) - required_course_threshold) * 0.02) - 1.0
            return np.clip(reward_exp, 0.0, 1.0) # [0.0, 1.0]
    
        # For UNDEFINED or other cases, assume not compliant
        return 0.0