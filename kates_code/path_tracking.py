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
YAW_RATE_RATIO = 100
YAW_RATE_GAIN = 1.25
FORWARD_VEL = .2
MAX_YAW_RATE = 25

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
    theta_error = state.yaw - next_waypoint[3]
    dist_error = dist_state_to_path(state, next_waypoint)
    delta = theta_error + (YAW_RATE_RATIO* dist_error) 
    return delta


def get_next_waypoint(state, path, current_index):
    """
    get index of the next closest waypoint
    """
    closest = current_index
    for index,waypoint in enumerate(path[current_index:-1]):
        if math.sqrt((waypoint[0] - state.x)**2 + (waypoint[1] - state.y)**2) < math.sqrt((path[closest][0] - state.x)**2 + (path[closest][1] - state.y)**2):
            closest = index

    vector_state_closest = [path[closest][0] - state.x, path[closest][1] - state.y]
    vector_closest = [math.cos(path[closest][3]), math.sin(path[closest][3])]

    if np.dot(vector_state_closest, vector_closest) < 0:
        if closest+1 != len(path):
            new_waypoint = closest + 1
        else:
            new_waypoint = len(path) - 1
    else:
        new_waypoint = closest
    if new_waypoint != current_index:
        print("NEW WAYPOINT, ", closest)
    return new_waypoint


def check_success(state,goal,offset=.2):
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
    # path = ([0, 0, 1, 0], [.6, 0, 1, 0], [1.2, 0, 1, 0])
    path = [(1.0, 0.3999999999999999, 1, 0.7853981633974448), (1.0315789473684214, 0.43157894736842106, 1, 0.7853981633974545), (1.063157894736842, 0.46315789473684216, 1, 0.7853981633974492), (1.094736842105263, 0.49473684210526314, 1, 0.7853981633974483), (1.1263157894736842, 0.5263157894736843, 1, 0.7853981633974413), (1.1578947368421058, 0.5578947368421054, 1, 0.7853981633974536), (1.1894736842105265, 0.5894736842105265, 1, 0.78539816339745), (1.2210526315789474, 0.6210526315789475, 1, 0.7853981633974448), (1.2526315789473688, 0.6526315789473687, 1, 0.7853981633974465), (1.28421052631579, 0.6842105263157897, 1, 0.78539816339745), (1.3157894736842108, 0.7157894736842108, 1, 0.7853981633974465), (1.347368421052632, 0.7473684210526318, 1, 0.7853981633974483), (1.378947368421053, 0.7789473684210527, 1, 0.78539816339745), (1.4105263157894739, 0.8105263157894738, 1, 0.78539816339745), (1.4421052631578948, 0.8421052631578948, 1, 0.7853981633974448), (1.4736842105263162, 0.873684210526316, 1, 0.78539816339745), (1.5052631578947369, 0.9052631578947368, 1, 0.78539816339745), (1.5368421052631578, 0.9368421052631578, 1, 0.7853981633974448), (1.568421052631579, 0.9684210526315787, 1, 0.78539816339745), (1.5999999999999999, 0.9999999999999998, 1, 0.7)]


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
        cf = scf.cf
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            state = get_data()
            current_waypoint_index = 0
            print("ascend")
            mc.up(.8)
            time.sleep(1)
            print("start loop")
            while not (check_success(state, path[-1])):
                current_waypoint_index = get_next_waypoint(state, path, current_waypoint_index)
                yaw_rate = get_yaw_vel(state, path[current_waypoint_index], FORWARD_VEL) * YAW_RATE_GAIN
                # print("yaw:", yaw_rate)
                cf.commander.send_hover_setpoint(FORWARD_VEL, 0.0, min(MAX_YAW_RATE, yaw_rate), 1.0)
                time.sleep(.01)
                state = get_data()


            print("descend")
            mc.land()       
            time.sleep(1)
            logconf.stop()
