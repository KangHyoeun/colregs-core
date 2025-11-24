import os
import sys
from math import pi, atan2, sin, cos, sqrt
import numpy as np
import time
from typing import Any
import math

def wrap_angle(angle, unit='rad', positive=False):
    """
    Wraps an angle to the range (-limit, limit], where limit is pi for radians or 180 for degrees.

    This uses a trigonometric identity to correctly and efficiently wrap the angle.

    Args:
        angle (float): The angle value.
        unit (str): The unit of the angle, either 'rad' for radians or 'deg' for degrees.
        positive (bool): If True, returns the absolute value of the wrapped angle.

    Returns:
        float: The wrapped angle in the same unit.
    """
    if unit == 'rad':
        # np.arctan2(sin(angle), cos(angle)) wraps angle to (-pi, pi]
        wrapped_angle = np.arctan2(np.sin(angle), np.cos(angle))
    elif unit == 'deg':
        # Convert to radians, wrap, then convert back to degrees
        rad = np.deg2rad(angle)
        wrapped_rad = np.arctan2(np.sin(rad), np.cos(rad))
        wrapped_angle = np.rad2deg(wrapped_rad)
    else:
        raise ValueError("Unit must be 'rad' or 'deg'.")
        
    return abs(wrapped_angle) if positive else wrapped_angle


def angle_difference(angle1, angle2, unit='rad'):
    """
    Calculates the shortest difference between two angles (angle1 - angle2).

    The result is wrapped to the range (-pi, pi] for radians or (-180, 180] for degrees.
    A positive result means angle1 is clockwise from angle2 (in a standard mathematical frame).
    A negative result means angle1 is counter-clockwise from angle2.

    Args:
        angle1 (float): The first angle.
        angle2 (float): The second angle.
        unit (str): The unit of the angles, 'rad' or 'deg'.

    Returns:
        float: The wrapped angle difference.
    """
    diff = angle1 - angle2
    return wrap_angle(diff, unit=unit)


def WrapToPi(rad, positive=False):
    """The function `WrapToPi` transforms an angle in radians to the range (-pi, pi]."""
    return wrap_angle(rad, unit='rad', positive=positive)


def WrapToRegion(rad, range):
    """
    Transform an angle in radians to a defined range [min, max).
    The range must have a length of 2*pi.
    """
    min_val, max_val = range
    if not np.isclose(max_val - min_val, 2 * np.pi):
        raise ValueError("The range for WrapToRegion must have a length of 2*pi.")
        
    return wrap_to_range(rad, min_val, max_val)

def WrapTo180(deg, positive=False):
    """Transform an angle to the range (-180, 180]."""
    return wrap_angle(deg, unit='deg', positive=positive)


def wrap_to_range(angle, min_val, max_val):
    """
    Wraps an angle to a given range [min_val, max_val).

    The length of the range (max_val - min_val) is assumed to be a full circle (2*pi or 360).

    Args:
        angle (float): The angle value to wrap.
        min_val (float): The minimum value of the range (inclusive).
        max_val (float): The maximum value of the range (exclusive).

    Returns:
        float: The wrapped angle.
    """
    span = max_val - min_val
    if span <= 0:
        raise ValueError("max_val must be greater than min_val.")
        
    # Wrap the angle to the range [min_val, max_val)
    wrapped = (angle - min_val) % span + min_val
    
    # Snap to min_val if the result is very close to max_val (due to float inaccuracies)
    if np.isclose(wrapped, max_val):
        return min_val
        
    return wrapped


def WrapTo360(deg):
    """
    Transform an angle in degrees to the range [0, 360).
    """
    return wrap_to_range(deg, 0.0, 360.0)

def calculate_distance(point1, point2):
    """
    Compute the distance between two points.

    Args:
        point1: First point [x, y] (list, tuple, or np.array).
        point2: Second point [x, y] (list, tuple, or np.array).

    Returns:
        float: Distance between points.
    """
    # Normalize inputs to 2-element NumPy arrays
    p1 = np.asarray(point1, dtype=float).flatten()[:2]
    p2 = np.asarray(point2, dtype=float).flatten()[:2]

    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def calculate_cte(start_position, goal_position, os_position):
    """
    Compute the signed cross-track error (CTE) using a vector-based calculation.

    The reference path is the straight line from start_position to goal_position.
    - Positive CTE: OS is to the right of the reference path (in NED coordinates).
    - Negative CTE: OS is to the left of the reference path.

    Notes:
        - Assumes NED coordinates (x=North, y=East).
        - CTE = 0 means OS is exactly on the reference path.
        - This implementation is optimized to use NumPy and avoids heavy dependencies.
    """
    # Normalize inputs to 2-element NumPy arrays
    start = np.asarray(start_position, dtype=float).flatten()[:2]
    goal = np.asarray(goal_position, dtype=float).flatten()[:2]
    os_pos = np.asarray(os_position, dtype=float).flatten()[:2]

    # Vectors for path and OS position relative to the start
    path_vec = goal - start
    os_vec = os_pos - start

    # Length of the path segment
    path_len = np.linalg.norm(path_vec)

    # If the path has negligible length, CTE is the distance from start to OS
    if path_len < 1e-6:
        return np.linalg.norm(os_vec)

    # 2D cross-product gives signed area; dividing by path length gives signed distance (CTE)
    # The sign indicates which side of the line the point is on.
    # In NED (x=N, y=E), a positive cross product (path_vec[0]*os_vec[1] - path_vec[1]*os_vec[0])
    # means the OS is to the East (right) of the path if heading North.
    cross_product = np.cross(path_vec, os_vec)
    
    signed_cte = cross_product / path_len
    
    return signed_cte

def calculate_ref_path(start_position, goal_position):
    """
    Calculate reference course angle from start to goal position in NED coordinates.
    
    This function computes the direction angle (heading) from the start position to the 
    goal position, representing the optimal straight-line path direction (χ_path).
    
    Notes:
        - Uses NED (North-East-Down) coordinate system (x=North, y=East).
        - Heading is measured clockwise from North.
        - Result is in degrees.
    """
    # Normalize inputs to 2-element NumPy arrays
    start = np.asarray(start_position, dtype=float).flatten()[:2]
    goal = np.asarray(goal_position, dtype=float).flatten()[:2]
    
    # Calculate delta in NED coordinates
    dx = goal[0] - start[0]  # North direction change
    dy = goal[1] - start[1]  # East direction change
    
    # atan2(dy, dx) gives the angle in radians from the positive x-axis (North).
    # np.degrees converts it to degrees.
    return np.degrees(np.arctan2(dy, dx))

def calculate_desired_course_angle(os_position,
                                   start_position, goal_position, 
                                   chi_inf=1, k=1):
    
    chi_path = calculate_ref_path(start_position, goal_position)
    
    e_y = calculate_cte(start_position, goal_position, os_position)

    chi_d = chi_inf * math.atan(k * e_y) + chi_path

    return np.degrees(chi_d)
    
def chi_angle(os_position, previous_position):

    # Normalize inputs to 2-element NumPy arrays
    next_pos = np.asarray(os_position, dtype=float).flatten()[:2]
    pos = np.asarray(previous_position, dtype=float).flatten()[:2]

    dx = next_pos[0] - pos[0]  # North direction change
    dy = next_pos[1] - pos[1]  # East direction change

    return np.degrees(np.arctan2(dy, dx))