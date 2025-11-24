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
        tcpa_threshold: float = 30.0,           # TCPA 임계값 (seconds)
        static_threshold: float = 45.0,          # Static 임계값 (degrees)
    ):
        self.headon_threshold = headon_threshold
        self.standon_threshold = standon_threshold
        self.tcpa_threshold = tcpa_threshold
        self.static_threshold = static_threshold

    def is_compliant(
        self,
        start_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        os_heading: float,
        tcpa: float,
        encounter_type: Optional[EncounterType] = None,
        is_static_obstacle: bool = False
    ) -> bool:

        if encounter_type == EncounterType.SAFE:
            return True
        elif encounter_type == EncounterType.HEAD_ON:
            return self.is_compliant_headon(start_position, goal_position, os_heading)
        elif encounter_type == EncounterType.CROSSING_STAND_ON:
            return self.is_compliant_standon(start_position, goal_position, os_heading)
        elif encounter_type == EncounterType.CROSSING_GIVE_WAY:
            return self.is_compliant_giveway(tcpa)
        elif encounter_type == EncounterType.OVERTAKING:
            return self.is_compliant_overtaking(tcpa)
        elif is_static_obstacle:
            return self.is_compliant_static(start_position, goal_position, os_heading)
        else:
            # For UNDEFINED or other cases, assume not compliant
            return False

    def is_compliant_headon(
        self,
        start_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        os_heading: float,
    ) -> bool:
        """
        Check if head-on avoidance maneuver is COLREGs-compliant.
        
        According to COLREGs Rule 14 and Woo & Kim (2020):
        - Must alter course to starboard (right) by at least 30°
        - χ^avoid = os_heading - χ_path ≥ 30°
        
        Args:
            start_position: Start position (origin)
            goal_position: Goal position (destination)
            os_heading: Current OS heading (degrees, NED)
        
        Returns:
            True if course angle change ≥ 30° to starboard
        """
        ref_course = calculate_ref_path(start_position, goal_position)  # Return degrees
        # χ^avoid: course angle change from reference path
        # Use WrapTo180 to distinguish starboard (+) from port (-)
        course_angle = WrapTo180(os_heading - ref_course)

        # COLREGs Rule 14: Starboard alteration (positive angle) ≥ 30°
        return course_angle >= self.headon_threshold

    def is_compliant_standon(
        self,
        start_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        os_heading: float,
    ) -> bool:
        """
        Check if stand-on vessel's collision avoidance is COLREGs-compliant.
        
        According to COLREGs Rule 17(b) and Woo & Kim (2020):
        - When stand-on vessel must act, large course change required (75°)
        - χ^avoid = os_heading - χ_path ≥ 75°
        
        Args:
            start_position: Start position (origin)
            goal_position: Goal position (destination)
            os_heading: Current OS heading (degrees, NED)
        
        Returns:
            True if course angle change ≥ 75° to starboard
        """
        ref_course = calculate_ref_path(start_position, goal_position)
        # χ^avoid: course angle change from reference path
        # Use WrapTo180 to distinguish starboard (+) from port (-)
        course_angle = WrapTo180(os_heading - ref_course)

        # COLREGs Rule 17(b): Large alteration (positive angle) ≥ 75°
        return course_angle >= self.standon_threshold

    def is_compliant_giveway(
        self,
        tcpa: float,
    ) -> bool:
        """
        Check if give-way vessel's avoidance is COLREGs-compliant.
        
        According to Woo & Kim (2020):
        - Immediate avoidance required (TCPA-based criterion)
        - TCPA > 30 seconds indicates successful avoidance
        
        Args:
            tcpa: Time to Closest Point of Approach (seconds)
        
        Returns:
            True if TCPA > threshold (successful avoidance)
        """
        return tcpa > self.tcpa_threshold

    def is_compliant_overtaking(
        self,
        tcpa: float,
    ) -> bool:
        """
        Check if overtaking vessel's avoidance is COLREGs-compliant.
        
        According to COLREGs Rule 13 and Woo & Kim (2020):
        - Immediate avoidance required (TCPA-based criterion)
        - TCPA > 30 seconds indicates successful avoidance
        
        Args:
            tcpa: Time to Closest Point of Approach (seconds)
        
        Returns:
            True if TCPA > threshold (successful avoidance)
        """
        return tcpa > self.tcpa_threshold

    def is_compliant_static(
        self,
        start_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        os_heading: float,
    ) -> bool:
        """
        Check if static obstacle avoidance is compliant.
        
        According to Woo & Kim (2020):
        - Either port or starboard turn acceptable
        - |χ^avoid| = |os_heading - χ_path| ≥ 45°
        
        Args:
            start_position: Start position (origin)
            goal_position: Goal position (destination)
            os_heading: Current OS heading (degrees, NED)
        
        Returns:
            True if |course angle change| ≥ 45° (either direction)
        """
        ref_course = calculate_ref_path(start_position, goal_position)
        # χ^avoid: course angle change from reference path
        # Use WrapTo180 to get proper angle difference [-180, 180]
        course_angle = WrapTo180(os_heading - ref_course)

        # Static obstacle: Either direction OK, check absolute value
        return abs(course_angle) >= self.static_threshold
    
    def set_avoidance_parameters(
        self,
        headon_threshold: Optional[float] = None,
        standon_threshold: Optional[float] = None,
        tcpa_threshold: Optional[float] = None,
        static_threshold: Optional[float] = None,
    ):
        """
        회피 목표 각도 파라미터 변경
        
        Args:
            headon_threshold: Head-on 임계값 (degrees)
            standon_threshold: Stand-on 임계값 (degrees)
            tcpa_threshold: TCPA 임계값 (seconds)
            static_threshold: Static 임계값 (degrees)
        """
        if headon_threshold is not None:
            self.headon_threshold = headon_threshold
        if standon_threshold is not None:
            self.standon_threshold = standon_threshold
        if tcpa_threshold is not None:
            self.tcpa_threshold = tcpa_threshold
        if static_threshold is not None:
            self.static_threshold = static_threshold