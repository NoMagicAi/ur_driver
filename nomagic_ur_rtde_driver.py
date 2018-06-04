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

import rospy

from follow_joint_trajectory_action_server import FollowJointTrajectoryActionServer
from move_ur_action_server import MoveURActionServer
from rtde_command_sender import RTDECommandSender

ROBOT_PORT = 30004
ROBOT_IP = ''

if __name__ == "__main__":
    rospy.init_node('nomagic_ur_rtde_driver', anonymous=False, log_level=rospy.INFO)
    ROBOT_IP = rospy.get_param('/galaxy/hardware/robot_ip')

    command_sender = RTDECommandSender(ROBOT_IP, ROBOT_PORT)
    move_ur_action_server = MoveURActionServer(command_sender)
    follow_join_trajectory_action_server = FollowJointTrajectoryActionServer(command_sender)

    rospy.spin()