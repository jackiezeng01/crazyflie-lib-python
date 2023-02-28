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
    c = (v[1]*p[0]) - (v[0]*p[1])
    return [a,b,c]

def dist_point_line(p, l):
    """
    distance between point and line
    p - [x,y]
    l - [a,b,c] coefficients of line equation (ax + by + c = 0)
    returns float distance value
    """
    return (abs((l[0]*p[0]) + (l[1]*p[1]) + l[2]) / np.sqrt(l[0]**2 + l[1]**2))

def dist_state_to_path(state, waypoint):
    """
    find orthoganal distance between current state and linear path passing through waypoint
    state - current x,y,z,yaw of drone
    next_waypoint - x,y,z,yaw of next waypoint
    returns float distance value
    """
    # print("dist to path")
    # print("state", state, "waypoint", waypoint)
    line_coeffs = line_from_vector([waypoint[0], waypoint[1]], [math.cos(waypoint[3]), math.sin(waypoint[3])])
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
    cf = np.array([state.x, state.y])
    wp = np.array([waypoint[0], waypoint[1]])
    theta = -1 * waypoint[3]

    translated_cf = np.subtract(cf, wp)
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    rotated_cf = np.dot(rotation, translated_cf)

    if rotated_cf[1] < 0:
        return -1
    else:
        return 1

    
    
    # wp_cf_angle = np.arctan2(waypoint[0] - state.x, waypoint[1] - state.y) - (math.pi/2)
    # angle_before = wp_cf_angle
    # if wp_cf_angle < 0:
    #     wp_cf_angle += 2*math.pi

    # print(wp_cf_angle, " : wpcf_angle, ", waypoint[3], " : waypoint angle ", angle_before, " : angle before" )

    # if waypoint[3] - wp_cf_angle > 0:
    #     return -1
    # return 1


