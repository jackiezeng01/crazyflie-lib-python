import numpy as np
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from async_logging import LoggingAsync
from async_position import PositionAsync

path = [[0,0,1],[1,1,1],[1,2,1],[2,2,1]]

#gain
GAIN = 1

class State():
    def __init__(self, x, y, z, yaw) -> None:
        self.x = x
        self.y = y 
        self.z = z 
        self.yaw = yaw


def get_data(logging, position):
    x = position.data.x
    y = position.data.y
    z = position.data.z
    yaw = logging.data.yaw
    return State(x,y,z,yaw)

def get_yaw_vel(state, next_waypoint, velocity):
    """
    state = current x,y,z,yaw of the robot
    next_waypoint = x,y,z,yaw list of next waypoint
    velocity = forward velocity float
    """
    theta_error = state.yaw - next_waypoint.yaw
    dist_error = np.sqrt((state.x**2 - next_waypoint[0]**2) + (state.y**2 - next_waypoint[1]**2))

    delta = theta_error * np.arctan2( GAIN * dist_error/ velocity)


logging = LoggingAsync()
position = PositionAsync()

state = [0,0,1,0]
next = [0,1,1,0]
vel = 1

print(get_yaw_vel(state, next, vel))