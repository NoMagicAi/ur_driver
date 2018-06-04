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

import actionlib
import rospy
import tf.transformations as trans

from nomagic_ur_driver.msg import MoveURAction, MoveURGoal, MoveURResult


def _joints2command(command, joints):
    command.input_double_register_0 = joints[0]
    command.input_double_register_1 = joints[1]
    command.input_double_register_2 = joints[2]
    command.input_double_register_3 = joints[3]
    command.input_double_register_4 = joints[4]
    command.input_double_register_5 = joints[5]


class MoveURActionServer:

    def __init__(self, command_sender):
        self._command_sender = command_sender

        self._as = actionlib.SimpleActionServer('/move_robot', MoveURAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def _prepare_command(self, goal):
        command = self._command_sender.command
        command.input_int_register_0 = goal.move_type
        if goal.move_type == MoveURGoal.MOVE_JOINT \
            or goal.move_type == MoveURGoal.MOVE_LINEAR_FK:
            _joints2command(command, goal.target_joints)
            command.input_double_register_6 = goal.velocity
            command.input_double_register_7 = goal.acceleration
        elif goal.move_type == MoveURGoal.MOVE_JOINT_IK \
            or goal.move_type == MoveURGoal.MOVE_LINEAR:
            rpy = trans.euler_from_quaternion([goal.target_pose.orientation.x, goal.target_pose.orientation.y,
                                               goal.target_pose.orientation.z, goal.target_pose.orientation.w])
            command.input_double_register_0 = goal.target_pose.position.x
            command.input_double_register_1 = goal.target_pose.position.y
            command.input_double_register_2 = goal.target_pose.position.z
            command.input_double_register_3 = rpy[0]
            command.input_double_register_4 = rpy[1]
            command.input_double_register_5 = rpy[2]
            command.input_double_register_6 = goal.velocity
            command.input_double_register_7 = goal.acceleration
            rospy.loginfo("Target position: {}".format(goal.target_pose.position))
            rospy.loginfo("Target orientation: {}".format(goal.target_pose.orientation))
            rospy.loginfo("Target RPY: {}".format(rpy))
        elif goal.move_type == MoveURGoal.MOVE_GRIPPER or \
            goal.move_type == MoveURGoal.MOVE_GRIPPER_DC:
            command.input_double_register_0 = goal.gripper_width
            command.input_double_register_1 = goal.gripper_force
        elif goal.move_type in [MoveURGoal.FT_SEARCH, MoveURGoal.MOVE_TO_SUCK]:
            rpy = trans.euler_from_quaternion([goal.target_pose.orientation.x, goal.target_pose.orientation.y,
                                               goal.target_pose.orientation.z, goal.target_pose.orientation.w])
            command.input_double_register_0 = goal.target_pose.position.x
            command.input_double_register_1 = goal.target_pose.position.y
            command.input_double_register_2 = goal.target_pose.position.z
            command.input_double_register_3 = rpy[0]
            command.input_double_register_4 = rpy[1]
            command.input_double_register_5 = rpy[2]
            command.input_double_register_6 = goal.velocity
            command.input_double_register_7 = goal.acceleration

            command.input_double_register_8 = goal.ft_threshold.force.x
            command.input_double_register_9 = goal.ft_threshold.force.y
            command.input_double_register_10 = goal.ft_threshold.force.z

            command.input_double_register_11 = goal.ft_threshold.torque.z
            command.input_double_register_12 = goal.ft_threshold.torque.z
            command.input_double_register_13 = goal.ft_threshold.torque.z

            command.input_double_register_14 = goal.ft_threshold.force_mag
            command.input_double_register_15 = goal.ft_threshold.torque_mag

    def execute_cb(self, goal):
        self._prepare_command(goal)
        self._command_sender.send_command(goal.move_type != MoveURGoal.STOP_PROGRAM)
        res = MoveURResult()
        self._as.set_succeeded(res)
