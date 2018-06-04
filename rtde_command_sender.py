#!/usr/bin/env python

# Copyright 2018 NoMagic Sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import threading

import rospy
import yaml

import rtde_client_35.rtde.rtde as rtde
from script_executor import ScriptExecutor

RTDE_OUTPUTS = ['output_bit_registers0_to_31',
                'output_int_register_1']
RTDE_INPUTS = ['input_bit_registers0_to_31', 'input_int_register_0', 'input_int_register_1',
               'input_double_register_0',
               'input_double_register_1',
               'input_double_register_2',
               'input_double_register_3',
               'input_double_register_4',
               'input_double_register_5',
               'input_double_register_6',
               'input_double_register_7',
               'input_double_register_8',
               'input_double_register_9',
               'input_double_register_10',
               'input_double_register_11',
               'input_double_register_12',
               'input_double_register_13',
               'input_double_register_14',
               'input_double_register_15']


def _test_bit(int_type, offset):
    mask = 1 << offset
    return (int_type & mask) == mask


def _set_bit(int_type, offset):
    mask = 1 << offset
    return int_type | mask


def _clear_bit(int_type, offset):
    mask = ~(1 << offset)
    return int_type & mask


def _is_command_buffer_ready(state):
    return _test_bit(state.output_bit_registers0_to_31, 0)


class RTDECommandSender:

    def __init__(self, robot_ip, robot_port):
        self._robot_ip = robot_ip
        self._robot_port = robot_port
        self._lock = threading.Lock()
        self.con = None
        self.command = None
        self.last_ur_state = None

        self._template_params = {"USE_RG2": rospy.get_param('/galaxy/hardware/use_rg2', True) and \
                                            rospy.get_param('/universe') == 'Reality',
                                 "USE_OPTOFORCE": rospy.get_param('/galaxy/hardware/use_optoforce', False) and \
                                                  rospy.get_param('/universe') == 'Reality',
                                 "OPTOFORCE_IP": rospy.get_param('/galaxy/hardware/optoforce_ip')}
        f = os.path.join(os.path.dirname(__file__), 'follow_joint_trajectory_action_server.yaml')
        with open(f, 'r') as stream:
            for k, v in yaml.load(stream).iteritems():
                self._template_params[k] = v
        self._executor = ScriptExecutor(os.path.join(os.path.dirname(__file__), 'scripts'), True)

        self._initialize()

    def __del__(self):
        self._cleanup()

    def _initialize(self):
        rospy.loginfo("Initializing RTDE with IP={} and PORT={}".format(self._robot_ip, self._robot_port))
        self.con = rtde.RTDE(self._robot_ip, self._robot_port)
        self.con.connect()

        self.con.send_output_setup(RTDE_OUTPUTS)
        self.command = self.con.send_input_setup(RTDE_INPUTS)

        if not self.con.send_start():
            rospy.logerr("unable to start RTDE")
            sys.exit()

        # Before starting URscript we need make sure that registers are cleared.
        self.clear_command()
        self.con.send(self.command)

        # Now we can load the script
        rospy.loginfo("Loading URscript")
        self._executor.call_script('rtde_move.urscript', self._template_params)
        self.last_ur_state = None

    def _cleanup(self):
        rospy.loginfo("Cleaning up RTDE connection")
        if self.con:
            try:
                self.con.send_pause()
            except Exception as e:
                # If we can't pause the connection we don't really care about it.
                rospy.logwarn("Pausing connection failed: {}".format(e))
            self.con.disconnect()
        self.con = None
        self.command = None

    def _reconnect(self):
        rospy.loginfo("Reconnecting to RTDE")
        self._cleanup()
        self._initialize()

    def _set_new_command_flag(self):
        self.command.input_bit_registers0_to_31 = _set_bit(self.command.input_bit_registers0_to_31, 0)

    def _clear_new_command_flag(self):
        self.command.input_bit_registers0_to_31 = _clear_bit(self.command.input_bit_registers0_to_31, 0)

    def clear_command(self):
        self.command.input_bit_registers0_to_31 = 0
        self.command.input_int_register_0 = 0
        self.command.input_int_register_1 = 0
        for i in range(0, 16):
            setattr(self.command, "input_double_register_{}".format(i), 0.0)

    def send(self):
        try:
            if not self.con.send(self.command):
                raise Exception("Cannot send command to the robot")
        except Exception as e:
            # If there was an exception we want to reconnect to the robot but also immediately stop execution of the
            # current command.
            rospy.logwarn("Exception while sending: {}".format(e))
            self._reconnect()
            raise e

    def receive(self):
        try:
            state = self.con.receive()
            if state is None:
                raise Exception("Cannot get UR state")
            return state
        except Exception as e:
            # If there was an exception we want to reconnect to the robot but also immediately stop execution of the
            # current command.
            rospy.logwarn("Exception while sending: {}".format(e))
            self._reconnect()
            raise e

    def wait_until(self, f, expected_value=True):
        rospy.logdebug("Waiting for {} to become {}.".format(f.__name__, expected_value))
        while True:
            self.last_ur_state = self.receive()
            if f(self.last_ur_state) == expected_value:
                return
            # In order to receive something we need to resend our last command.
            self.send()

    def send_command(self, wait_for_completion=True, subcommand_cb=None):
        if not self._lock.acquire(False):
            raise Exception("Two RTDE commands sent in parallel")
        try:
            # 1. Wait until URscript is ready to take another command.
            self.wait_until(_is_command_buffer_ready)
            # 2. Send new command.
            self._set_new_command_flag()
            self.send()
            # 3. Wait until URscript confirms it has received new command.
            self.wait_until(_is_command_buffer_ready, False)
            # 4. New command has been received so we can stop re-sending it.
            self._clear_new_command_flag()
            self.send()
            # 5. If we have a callback for handling subcommands then run it.
            if subcommand_cb:
                subcommand_cb()
            # 6. Wait until URscript confirms it has completed the command.
            if wait_for_completion:
                self.wait_until(_is_command_buffer_ready)
        finally:
            self._lock.release()
