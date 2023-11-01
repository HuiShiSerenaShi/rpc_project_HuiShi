#!/usr/bin/env python2

from geometry_msgs.msg import Pose, PoseStamped
#from math import dist, fabs, cos
from math import fabs, cos
from moveit_commander.conversions import pose_to_list
#from typing import Union
from math import sqrt

def euclidean_distance_3d(point1, point2):
    """
    Calculate the Euclidean distance between two 3D points represented as tuples (x, y, z).
    """
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)


# def all_close(goal: Union["list[float]", Pose, PoseStamped], 
#             actual: Union["list[float]", Pose, PoseStamped], 
#             tolerance: float) -> bool:

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a 
    tolerance of each other. For Pose and PoseStamped inputs, the angle 
    between the two quaternions is compared (the angle between the identical 
    orientations q and -q is calculated correctly).

    Parameters
    ----------
    goal : list[float], Pose, PoseStamped
        A list-like of float values
    actual : list[float], Pose, PoseStamped
        A list-like of float values
    tolerance : float
        The maximum distance between elements of the list-likes
    
    Returns
    -------
    (bool) 
        False if one of the elements of the list-likes is not within the 
    tolerance
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        # d = dist((x1, y1, z1), (x0, y0, z0))
        d = euclidean_distance_3d((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

#def round_list(list: "list[float]", digits: int=3) -> "list[float]":
def round_list(list, digits=3):
    """
    Convenience method for rounding all the elements of a list to a given number
    of digits.

    Parameters
    ----------
    list : list[float]
        A list of floats
    digits : int
        An integer specifying the digits to round to (default 3).
    
    Returns
    -------
    list[float] 
        A list of rounded floats
    """
    return [round(l, digits) for l in list]