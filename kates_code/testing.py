

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
from cflib.crazyflie.commander import Commander

FORWARD_VEL = .1
THRESHOLD = .1
URI = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7E7')
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

commander = Commander

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions
            # mc.forward(0.8)
            # mc.back(0.8)
            # time.sleep(1)

            mc.up(1)
            print("up")
            time.sleep(1)
            mc.start_linear_motion(0.0, .1, 0.0, 5)
            mc.forward(.5)
            time.sleep(1)
            print("down")
            mc.down(1)