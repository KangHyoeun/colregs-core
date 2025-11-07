import os
import sys
from math import pi, atan2, sin, cos, sqrt
import numpy as np
from shapely import ops
import time
from typing import Any
from irsim.global_param import env_param 
import math
from shapely.affinity import affine_transform


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
    Transform an angle to the range [-180, 180].
    """
    while deg > 180:
        deg = deg - 360
    while deg < -180:
        deg = deg + 360

    return deg if not positive else abs(deg)


def WrapTo360(deg):
    """
    Transform an angle to the range [0, 360].
    """
    while deg > 360:
        deg = deg - 360
    while deg < 0:
        deg = deg + 360
    return deg

def distance(point1, point2):
    """
    Compute the distance between two points.

    Args:
        point1 (np.array): First point [x, y] (2x1).
        point2 (np.array): Second point [x, y] (2x1).

    Returns:
        float: Distance between points.
    """
    return dist_hypot(point1[0, 0], point1[1, 0], point2[0, 0], point2[1, 0])


def dist_hypot(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)
