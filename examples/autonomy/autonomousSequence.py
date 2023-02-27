# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E7E7')

# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
    (0.0, 0.0, 0.4, 0),
    (0.0, 0.0, 1.2, 0),
    (0.5, -0.5, 1.2, 0),
    (0.5, 0.5, 1.2, 0),
    (-0.5, 0.5, 1.2, 0),
    (-0.5, -0.5, 1.2, 0),
    (0.0, 0.0, 1.2, 0),
    (0.0, 0.0, 0.4, 0),
]
sequence = [
(2.8988879056271433, 0.3795392065710368, 1, 0.9792422483428889), 
(2.994799330055643, 0.5223051669787783, 1, 1.3914575544125347), 
(3.0186066452366305, 0.6536294338846556, 1, 1.880518469095501), 
(2.983675177669276, 0.7627830509744528, 1, 2.3489562367144794), 
(2.903833624611075, 0.8437888784774883, 1, 2.7075348614542802), 
(2.7933333958378053, 0.8950103189625179, 1, 2.9712773746899326), 
(2.66463900363536, 0.917143361822162, 1, 3.135854284108799), 
(2.524355942939882, 0.9179483666923476, 1, -3.0643864584988036), 
(2.3779012594671394, 0.9066186374737015, 1, -3.0449499445756025), 
(2.2306919989328975, 0.892347478066851, 1, -3.0830969113731634), 
(2.0878337275220202, 0.883981332980591, 1, 3.1178634816348874), 
(1.9516014914853295, 0.8872146180172389, 1, 2.9963492841520036), 
(1.8227606686761455, 0.906060602674872, 1, 2.8331466853767293), 
(1.702062513783966, 0.9445168290873975, 1, 2.6348541448702965), 
(1.5902574760787083, 1.0065780774911144, 1, 2.4433612629021124), 
(1.4871776206365066, 1.093089858882852, 1, 2.3291807418220594), 
(1.3899713666720448, 1.195695083193036, 1, 2.2874640590114415), 
(1.295299555237877, 1.304364685901508, 1, 2.3101269856386004), 
(1.1998230273865582, 1.4090696024881086, 1, 2.4029693835205186), 
(1.1002026241706435, 1.4997807684326794, 1, 0)
]

sequence = [(1.0, 0.3999999999999999, 1, 0.7853981633974448), (1.0315789473684214, 0.43157894736842106, 1, 0.7853981633974545), (1.063157894736842, 0.46315789473684216, 1, 0.7853981633974492), (1.094736842105263, 0.49473684210526314, 1, 0.7853981633974483), (1.1263157894736842, 0.5263157894736843, 1, 0.7853981633974413), (1.1578947368421058, 0.5578947368421054, 1, 0.7853981633974536), (1.1894736842105265, 0.5894736842105265, 1, 0.78539816339745), (1.2210526315789474, 0.6210526315789475, 1, 0.7853981633974448), (1.2526315789473688, 0.6526315789473687, 1, 0.7853981633974465), (1.28421052631579, 0.6842105263157897, 1, 0.78539816339745), (1.3157894736842108, 0.7157894736842108, 1, 0.7853981633974465), (1.347368421052632, 0.7473684210526318, 1, 0.7853981633974483), (1.378947368421053, 0.7789473684210527, 1, 0.78539816339745), (1.4105263157894739, 0.8105263157894738, 1, 0.78539816339745), (1.4421052631578948, 0.8421052631578948, 1, 0.7853981633974448), (1.4736842105263162, 0.873684210526316, 1, 0.78539816339745), (1.5052631578947369, 0.9052631578947368, 1, 0.78539816339745), (1.5368421052631578, 0.9368421052631578, 1, 0.7853981633974448), (1.568421052631579, 0.9684210526315787, 1, 0.78539816339745), (1.5999999999999999, 0.9999999999999998, 1, None)]



def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def run_sequence(scf, sequence):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(50):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        # start_position_printing(scf)
        run_sequence(scf, sequence)
