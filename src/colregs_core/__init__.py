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
    calculate_distance,
    calculate_aspect_angle,
    normalize_angle,
    normalize_angle_360,
    heading_to_velocity
)
from .geometry.coordinate_transform import (
    irsim_to_nav_heading,
    nav_to_irsim_heading,
    irsim_velocity_to_nav,
    nav_velocity_to_irsim,
    verify_transformation
)

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
    "calculate_distance",
    "calculate_aspect_angle",
    "normalize_angle",
    "normalize_angle_360",
    "heading_to_velocity",
    
    # Coordinate transformation
    "irsim_to_nav_heading",
    "nav_to_irsim_heading",
    "irsim_velocity_to_nav",
    "nav_velocity_to_irsim",
    "verify_transformation"
]
