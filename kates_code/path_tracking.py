import numpy as np
import time
import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

from distance_calcs import dist_state_to_path
import math

URI = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7E7')

#gain constant
GAIN = .1
MULTIPLIER = 1
FORWARD_VEL = .1
THRESHOLD = .1

position_estimate = [0, 0, 0, 0]

def log_pos_callback(timestamp, data, logconf):
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    position_estimate[3] = data['stabilizer.yaw']

class State():
    def __init__(self, x, y, z, yaw) -> None:
        self.x = x
        self.y = y 
        self.z = z 
        self.yaw = yaw
    def __repr__(self) -> str:
        return f"x: {self.x}, y: {self.y}, z: {self.z}, yaw: {self.yaw}"

def get_data():
    
    x = position_estimate[0]
    y = position_estimate[1]
    z = position_estimate[2]
    yaw = position_estimate[3]
    return State(x,y,z,yaw)

def get_yaw_vel(state, next_waypoint, velocity):
    """
    state = current x,y,z,yaw of the robot
    next_waypoint = x,y,z,yaw list of next waypoint
    velocity = forward velocity float
    """
    # print("get yaw vel")
    # print("state: ", state, "waypoint", next_waypoint, "velocity", velocity)
    theta_error = state.yaw - next_waypoint[3]
    dist_error = dist_state_to_path(state, next_waypoint)
    delta = theta_error + np.arctan(GAIN * dist_error/ velocity)
    return delta


def get_next_waypoint(state, path, current_index):
    """
    get index of the next closest waypoint
    """
    vector_state_current = [path[current_index][0] - state.x, path[current_index][1] - state.y]
    vector_state_next = [path[current_index + 1][0] - state.x, path[current_index + 1][1] - state.y]

    vector_current = [math.cos(path[current_index][3]), math.sin(path[current_index][3])]
    vector_next = [math.cos(path[current_index + 1][3]), math.sin(path[current_index + 1][3])]

    if np.dot(vector_state_current, vector_current) < np.dot(vector_state_next, vector_next):
        print("NEW WAYPOINT")
        return current_index + 1
    return current_index


def check_success(state,goal,offset=.1):
    """
    check if position is close enough to goal to call it good
    """
    return (abs(state.x - goal[0]) < offset 
        and abs(state.y - goal[1]) < offset)
        # and abs(state.z - goal[2]) < offset)
        # and abs(state.yaw - goal[3]) < offset)
        

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    logging.basicConfig(level=logging.ERROR)

    """
    GET PATH HERE
    """
    path = ([0, 0, 1, 0], [0, 0.5, 1, 1.5707963267948966], [0, 1, 1, 0.0], [0.5, 1, 1, 0.0], [1, 1, 0, -1.5707963267948966], [1, 0.5, 1, -1.5707963267948966], [1, 0, 1, -1.5707963267948966])

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('logging setup ')
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stabilizer.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        logconf.start()
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            state = get_data()
            current_waypoint_index = 0
            print("ascend")
            mc.up(.5)
            time.sleep(1)
            print("start loop")
            mc.start_forward(velocity=FORWARD_VEL)

            while not (check_success(state, path[-1])):
                if current_waypoint_index < len(path) -1:
                    current_waypoint_index = get_next_waypoint(state, path, current_waypoint_index)
                yaw = get_yaw_vel(state, path[current_waypoint_index], FORWARD_VEL) * MULTIPLIER
                print("yaw:", yaw)
                if abs(yaw) > THRESHOLD:
                    mc.start_turn_forward(velocity=FORWARD_VEL, rate=yaw)
                else:
                    mc.start_forward(velocity=FORWARD_VEL)
                state = get_data()
                time.sleep(.01)

            print("descend")
            mc.stop()
            time.sleep(1)

            logconf.stop()
