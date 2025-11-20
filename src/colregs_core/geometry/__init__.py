"""
Geometry utilities for maritime navigation
"""

from .bearings import (
    calculate_relative_bearing,
    calculate_relative_velocity,
    heading_speed_to_velocity,
    velocity_to_heading_speed,
    calculate_aspect_angle,
)

from .coordinate_transform import (
    ned_to_math_heading,
    math_to_ned_heading,
    maritime_to_math_position,
    math_to_maritime_position,
    maritime_to_math_state,
    math_to_maritime_state,
    maritime_relative_angle,
    math_relative_angle,
    math_to_maritime_velocity,
    maritime_to_math_velocity,
)

__all__ = [
    # bearings
    'calculate_relative_bearing',
    'calculate_relative_velocity',
    'heading_speed_to_velocity',
    'velocity_to_heading_speed',
    'calculate_aspect_angle',
    # coordinate_transform
    'ned_to_math_heading',
    'math_to_ned_heading',
    'maritime_to_math_position',
    'math_to_maritime_position',
    'maritime_to_math_state',
    'math_to_maritime_state',
    'maritime_relative_angle',
    'math_relative_angle',
    'math_to_maritime_velocity',
    'maritime_to_math_velocity',
]
