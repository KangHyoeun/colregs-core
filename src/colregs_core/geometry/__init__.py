"""
Geometry utilities for maritime navigation
"""

from .bearings import (
    normalize_angle,
    normalize_angle_360,
    calculate_relative_bearing,
    calculate_distance,
    calculate_relative_velocity,
    heading_to_velocity,
    velocity_to_heading_speed,
    calculate_aspect_angle,
    calculate_bearing_rate,
)

from .coordinate_transform import (
    irsim_to_nav_heading,
    nav_to_irsim_heading,
    irsim_velocity_to_nav,
    nav_velocity_to_irsim,
    verify_transformation,
)

__all__ = [
    # bearings
    'normalize_angle',
    'normalize_angle_360',
    'calculate_relative_bearing',
    'calculate_distance',
    'calculate_relative_velocity',
    'heading_to_velocity',
    'velocity_to_heading_speed',
    'calculate_aspect_angle',
    'calculate_bearing_rate',
    # coordinate_transform
    'irsim_to_nav_heading',
    'nav_to_irsim_heading',
    'irsim_velocity_to_nav',
    'nav_velocity_to_irsim',
    'verify_transformation',
]
