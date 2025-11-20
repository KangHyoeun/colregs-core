import os
import sys
from math import pi, atan2, sin, cos, sqrt
import numpy as np
import time
from typing import Any
import math


def WrapToPi(rad, positive=False):
    """The function `WrapToPi` transforms an angle in radians to the range [-pi, pi].

    Args:

        rad (float): Angle in radians.
            The `rad` parameter in the `WrapToPi` function represents an angle in radians that you want to
        transform to the range [-π, π]. The function ensures that the angle is within this range by wrapping
        it around if it exceeds the bounds.

        positive (bool): Whether to return the positive value of the angle. Useful for angles difference.

    Returns:
        The function `WrapToPi(rad)` returns the angle `rad` wrapped to the range [-pi, pi].

    """
    while rad > pi:
        rad = rad - 2 * pi
    while rad < -pi:
        rad = rad + 2 * pi

    return rad if not positive else abs(rad)


def WrapToRegion(rad, range):
    """
    Transform an angle to a defined range, with length of 2*pi.

    Args:
        rad (float): Angle in radians.
        range (list): List defining the range [min, max].

    Returns:
        float: Wrapped angle.
    """
    assert len(range) >= 2 and range[1] - range[0] == 2 * pi
    while rad > range[1]:
        rad = rad - 2 * pi
    while rad < range[0]:
        rad = rad + 2 * pi
    return rad

def WrapTo180(deg, positive=False):
    """
    Transform an angle to the range (-180, 180].
    """
    while deg > 180:
        deg = deg - 360
    while deg < -180:
        deg = deg + 360

    return deg if not positive else abs(deg)


def WrapTo360(deg):
    """
    Transform an angle to the range [0, 360).
    
    Notes:
        - 0° ≤ result < 360°
        - 360° is wrapped to 0°
        - Handles floating-point precision issues
    
    Examples:
        >>> WrapTo360(0.0)
        0.0
        >>> WrapTo360(360.0)
        0.0
        >>> WrapTo360(-0.0000001)
        0.0
        >>> WrapTo360(359.9999999)
        359.9999999
    """
    # [0, 360) 범위로 wrapping
    # Use fmod for floating point modulo, which handles negative numbers correctly
    # and ensures result is in (-360, 360)
    deg = math.fmod(deg, 360.0)
    
    # Ensure positive result for [0, 360) range
    if deg < 0:
        deg += 360.0
    
    # 부동소수점 오차 처리: 0° 또는 360° 근처는 0°로 정규화
    eps = 1e-6 # Increased epsilon to handle larger floating-point inaccuracies
    if abs(deg) < eps or abs(deg - 360.0) < eps:
        return 0.0
    
    return deg

def distance(point1, point2):
    """
    Compute the distance between two points.

    Args:
        point1: First point [x, y] (list, tuple, or np.array).
        point2: Second point [x, y] (list, tuple, or np.array).

    Returns:
        float: Distance between points.
    """
    # Handle different input types
    if isinstance(point1, (list, tuple)):
        x1, y1 = float(point1[0]), float(point1[1])
    elif isinstance(point1, np.ndarray):
        if point1.ndim == 1:
            x1, y1 = float(point1[0]), float(point1[1])
        else:
            x1, y1 = float(point1[0, 0]), float(point1[1, 0])
    else:
        x1, y1 = float(point1[0]), float(point1[1])
    
    if isinstance(point2, (list, tuple)):
        x2, y2 = float(point2[0]), float(point2[1])
    elif isinstance(point2, np.ndarray):
        if point2.ndim == 1:
            x2, y2 = float(point2[0]), float(point2[1])
        else:
            x2, y2 = float(point2[0, 0]), float(point2[1, 0])
    else:
        x2, y2 = float(point2[0]), float(point2[1])
    
    return dist_hypot(x1, y1, x2, y2)


def dist_hypot(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def cross_track_error(start_position, goal_position, os_position):
    """
    Compute the signed cross-track error (CTE) - perpendicular distance from OS to the reference path.
    
    The reference path is defined as the straight line from start_position to goal_position.
    - Positive CTE: OS is to the right of the reference path.
    - Negative CTE: OS is to the left of the reference path.
    
    Notes:
        - In NED coordinates: (x=North, y=East)
        - CTE = 0 means OS is exactly on the reference path
        - Uses point-to-line distance formula
    """
    from shapely.geometry import LineString, Point
    
    # Convert inputs to tuples if they are numpy arrays
    if isinstance(start_position, np.ndarray):
        if start_position.shape == (2, 1):
            start = (start_position[0, 0], start_position[1, 0])
        else:
            start = tuple(start_position)
    else:
        start = tuple(start_position)
    
    if isinstance(goal_position, np.ndarray):
        if goal_position.shape == (2, 1):
            goal = (goal_position[0, 0], goal_position[1, 0])
        elif goal_position.shape == (3, 1):
            # Handle [x, y, theta] format - only use x, y
            goal = (goal_position[0, 0], goal_position[1, 0])
        elif goal_position.ndim == 1:
            # 1D array - take first 2 elements
            goal = (float(goal_position[0]), float(goal_position[1]))
        else:
            # 2D array - take first 2 elements from first column
            goal = (float(goal_position[0, 0]), float(goal_position[1, 0]))
    else:
        # List or tuple - take first 2 elements
        if len(goal_position) >= 2:
            goal = (float(goal_position[0]), float(goal_position[1]))
        else:
            goal = tuple(goal_position)
    
    if isinstance(os_position, np.ndarray):
        if os_position.shape == (2, 1):
            os = (os_position[0, 0], os_position[1, 0])
        else:
            os = tuple(os_position)
    else:
        os = tuple(os_position)
    
    # Create reference path (straight line from start to goal)
    ref_path = LineString([start, goal])
    
    # Create point for OS position
    os_point = Point(os)
    
    # Calculate unsigned perpendicular distance
    unsigned_cte = ref_path.distance(os_point)
    
    # If distance is negligible, CTE is 0
    if unsigned_cte < 1e-6:
        return 0.0
        
    # Determine the sign of the CTE using the 2D cross-product method
    # z = (x_g - x_s) * (y_o - y_s) - (y_g - y_s) * (x_o - x_s)
    # In NED: x is North, y is East. A positive z means OS is to the right of the path.
    dx_path = goal[0] - start[0]
    dy_path = goal[1] - start[1]
    dx_os = os[0] - start[0]
    dy_os = os[1] - start[1]
    
    cross_product = dx_path * dy_os - dy_path * dx_os
    
    # Get the sign (-1, 0, or 1)
    sign = np.sign(cross_product)
    
    # If the path has no length, sign cannot be determined, return unsigned cte
    if dx_path**2 + dy_path**2 < 1e-6:
        return unsigned_cte

    # Combine sign and magnitude
    signed_cte = unsigned_cte * sign
    
    return signed_cte


def ref_course_angle(start_position, goal_position):
    """
    Calculate reference course angle from start to goal position in NED coordinates.
    
    This function computes the direction angle (heading) from the start position to the 
    goal position, representing the optimal straight-line path direction (χ_path).
    
    Notes:
        - Uses NED (North-East-Down) coordinate system
        - Heading is measured clockwise from North
        - This angle represents χ_path in Woo & Kim (2020)
    
    Example:
        >>> start = (0.0, 0.0)
        >>> goal = (100.0, 0.0)  # 100m north
        >>> ref_course_angle(start, goal)
        0.0  # North direction (degrees)
        
        >>> goal = (0.0, 100.0)  # 100m east
        >>> ref_course_angle(start, goal)
        90.0  # East direction (degrees)
    """
    # Normalize input format
    if isinstance(start_position, (tuple, list)):
        start_x, start_y = start_position[0], start_position[1]
    else:
        start_x = start_position[0, 0] if start_position.shape == (2, 1) else start_position[0]
        start_y = start_position[1, 0] if start_position.shape == (2, 1) else start_position[1]
    
    if isinstance(goal_position, (tuple, list)):
        goal_x, goal_y = goal_position[0], goal_position[1]
    else:
        goal_x = goal_position[0, 0] if goal_position.shape == (2, 1) else goal_position[0]
        goal_y = goal_position[1, 0] if goal_position.shape == (2, 1) else goal_position[1]
    
    # Calculate delta in NED coordinates
    dx = goal_x - start_x  # North direction change
    dy = goal_y - start_y  # East direction change
    
    return np.degrees(np.arctan2(dy, dx))