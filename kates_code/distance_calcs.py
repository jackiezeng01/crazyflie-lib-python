import numpy as np
import math


def line_from_vector(p,v):
    """
    given a point and vector, find the line that runs through the point in the direction of vector
    point- [x,y]
    vector - [x,y]
    returns list of coefficients a,b,c for the line format ax + by + c = 0
    """
    # print("line from vector")
    # print("p", p, "v", v)
    a = v[1]
    b = -1*v[0]
    c = (v[0]*p[1]) - (v[1]*p[0])
    return [a,b,c]

def dist_point_line(p, l):
    """
    distance between point and line
    p - [x,y]
    l - [a,b,c] coefficients of line equation (ax + by + c = 0)
    returns float distance value
    """
    return (abs(l[0]*p[0] + l[1]*p[1] + l[2]) / np.sqrt(l[0]**2 + l[1]**2))

def dist_state_to_path(state, waypoint):
    """
    find orthoganal distance between current state and linear path passing through waypoint
    state - current x,y,z,yaw of drone
    next_waypoint - x,y,z,yaw of next waypoint
    returns float distance value
    """
    # print("dist to path")
    # print("state", state, "waypoint", waypoint)
    line_coeffs = line_from_vector([state.x, state.y], [math.cos(waypoint[0]), math.sin(waypoint[1])])
    dist = dist_point_line([state.x, state.y], line_coeffs)
    sign = sign_of_dist(state, waypoint)
    return dist * sign

def sign_of_dist(state, waypoint):
    """
    finds the sign of the angle between waypoing yaw and the vector between waypoint and drone
    state - drone state of type State
    waypoint - [x,y,z,yaw] of waypoint
    returns - 1 or -1 based on sign
    """
    wp_d_angle = np.arctan2(waypoint[0] - state.x, waypoint[1] - state.y)
    if waypoint[3] - wp_d_angle < 0:
        return -1
    return 1


