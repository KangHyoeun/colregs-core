"""
COLREGs Core - Maritime Encounter Classification and Collision Risk Assessment

A comprehensive package for COLREGs-based encounter situation classification
and collision risk assessment for maritime navigation.
"""

from .encounter.classifier import EncounterClassifier
from .encounter.types import EncounterType, EncounterSituation, RiskLevel, CollisionRisk
from .risk.risk_matrix import RiskAssessment
from .risk.cpa_tcpa import calculate_cpa_tcpa, is_collision_course
from .geometry.bearings import (
    calculate_relative_bearing,
    calculate_aspect_angle,
    heading_speed_to_velocity,
    velocity_to_heading_speed,
    calculate_bearing_rate
)
from .geometry.coordinate_transform import (
    ned_to_math_heading,
    math_to_ned_heading,
    maritime_to_math_position,
    math_to_maritime_position,
    maritime_to_math_state,
    math_to_maritime_state,
    maritime_relative_angle,
    math_relative_angle,
    math_to_maritime_velocity,
    maritime_to_math_velocity
)
from .utils.utils import WrapTo180, WrapTo360, distance, dist_hypot

__version__ = "0.1.0"
__author__ = "Maritime Robotics Lab"

__all__ = [
    # Main classes
    "EncounterClassifier",
    "RiskAssessment",
    
    # Types and enums
    "EncounterType",
    "EncounterSituation", 
    "RiskLevel",
    "CollisionRisk",
    
    # Utility functions
    "calculate_cpa_tcpa",
    "is_collision_course",
    "calculate_relative_bearing",
    "calculate_aspect_angle",
    "heading_speed_to_velocity",
    "velocity_to_heading_speed",
    "calculate_bearing_rate",
    "ned_to_math_heading",
    "math_to_ned_heading",
    "maritime_to_math_position",
    "math_to_maritime_position",
    "maritime_to_math_state",
    "math_to_maritime_state",
    "maritime_relative_angle",
    "math_relative_angle",
    "math_to_maritime_velocity",
    "maritime_to_math_velocity",
    "WrapTo180",
    "WrapTo360",
    "distance",
    "dist_hypot",
]
