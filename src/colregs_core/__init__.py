"""
COLREGs Core - Maritime Encounter Classification and Collision Risk Assessment

A comprehensive package for COLREGs-based encounter situation classification
and collision risk assessment for maritime navigation.
"""

from .encounter.classifier import EncounterClassifier
from .encounter.types import EncounterType, EncounterSituation, RiskLevel, CollisionRisk


__version__ = "0.1.0"
__author__ = "Maritime Robotics Lab"

__all__ = [
    # Main classes
    "EncounterClassifier",
    
    # Types and enums
    "EncounterType",
    "EncounterSituation", 
    "RiskLevel",
    "CollisionRisk"
]
