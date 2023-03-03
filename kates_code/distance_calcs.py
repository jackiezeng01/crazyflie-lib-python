import numpy as np
import math


def dist_state_to_path(state, waypoint):
    """
    finds the orthogonal distance between waypoing along yaw and the crazyflie state
    state - drone state of type State
    waypoint - [x,y,z,yaw] of waypoint
    
    """
    cf = np.array([state.x, state.y])
    wp = np.array([waypoint[0], waypoint[1]])
    theta = -1 * waypoint[3]

    translated_cf = np.subtract(cf, wp)
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    rotated_cf = np.dot(rotation, translated_cf)

    return rotated_cf[1]

    
    
    # wp_cf_angle = np.arctan2(waypoint[0] - state.x, waypoint[1] - state.y) - (math.pi/2)
    # angle_before = wp_cf_angle
    # if wp_cf_angle < 0:
    #     wp_cf_angle += 2*math.pi

    # print(wp_cf_angle, " : wpcf_angle, ", waypoint[3], " : waypoint angle ", angle_before, " : angle before" )

    # if waypoint[3] - wp_cf_angle > 0:
    #     return -1
    # return 1


