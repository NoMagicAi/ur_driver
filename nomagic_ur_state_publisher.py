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

import sys

import rospy
import tf
import tf.transformations as trans
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

import rtde_client_35.rtde.rtde as rtde
from nomagic_ur_driver.msg import IOState

REQUIRED_JOINT_SEQUENCE = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
RTDE_OUTPUTS = ['actual_q', 'actual_qd', 'target_moment', 'actual_TCP_pose',
                'actual_digital_input_bits',
                'output_double_register_0',
                'output_double_register_1',
                'output_double_register_2',
                'output_double_register_3',
                'output_double_register_4',
                'output_double_register_5']
ROBOT_PORT = 30004


def testBit(int_type, offset):
    mask = 1 << offset
    return (int_type & mask) == mask


if __name__ == "__main__":
    rospy.init_node('nomagic_ur_state_publisher', anonymous=False)
    universe = rospy.get_param('/universe')
    joints_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    ft_pub = rospy.Publisher('/ft', WrenchStamped, queue_size=10)
    io_pub = rospy.Publisher('/io_state', IOState, queue_size=10)
    robot_ip = rospy.get_param('/galaxy/hardware/robot_ip')
    base_frame = rospy.get_param('~base_frame')
    tool_frame = rospy.get_param('~tool_frame')
    br = tf.TransformBroadcaster()

    con = rtde.RTDE(robot_ip, ROBOT_PORT)
    if con.connect():
        rospy.loginfo("unable to open connection to robot")
        sys.exit()

    con.send_output_setup(RTDE_OUTPUTS)

    if not con.send_start():
        rospy.loginfo("unable to start RTDE")
        sys.exit()

    while not rospy.is_shutdown():
        try:
            state = con.receive()
            state_timestamp = rospy.get_rostime()
            joint_states = JointState()
            joint_states.name = REQUIRED_JOINT_SEQUENCE
            joint_states.position = state.actual_q
            joint_states.velocity = state.actual_qd
            joint_states.effort = state.target_moment
            joint_states.header.stamp = rospy.Time.now()
            joints_pub.publish(joint_states)

            wrench_stamped = WrenchStamped()
            wrench_stamped.header.stamp = state_timestamp
            wrench = wrench_stamped.wrench
            wrench.force.x = state.output_double_register_0
            wrench.force.y = state.output_double_register_1
            wrench.force.z = state.output_double_register_2

            wrench.torque.x = state.output_double_register_3
            wrench.torque.y = state.output_double_register_4
            wrench.torque.z = state.output_double_register_5
            ft_pub.publish(wrench_stamped)

            io_state = IOState()
            io_state.header.stamp = state_timestamp

            io_state.standard_0 = testBit(state.actual_digital_input_bits, 0)
            io_state.standard_1 = testBit(state.actual_digital_input_bits, 1)
            io_state.standard_2 = testBit(state.actual_digital_input_bits, 2)
            io_state.standard_3 = testBit(state.actual_digital_input_bits, 3)
            io_state.standard_4 = testBit(state.actual_digital_input_bits, 4)
            io_state.standard_5 = testBit(state.actual_digital_input_bits, 5)
            io_state.standard_6 = testBit(state.actual_digital_input_bits, 6)
            io_state.standard_7 = testBit(state.actual_digital_input_bits, 7)

            io_state.configurable_0 = testBit(state.actual_digital_input_bits, 8)
            io_state.configurable_1 = testBit(state.actual_digital_input_bits, 9)
            io_state.configurable_2 = testBit(state.actual_digital_input_bits, 10)
            io_state.configurable_3 = testBit(state.actual_digital_input_bits, 11)
            io_state.configurable_4 = testBit(state.actual_digital_input_bits, 12)
            io_state.configurable_5 = testBit(state.actual_digital_input_bits, 13)
            io_state.configurable_6 = testBit(state.actual_digital_input_bits, 14)
            io_state.configurable_7 = testBit(state.actual_digital_input_bits, 15)

            io_state.tool_0 = testBit(state.actual_digital_input_bits, 16)
            io_state.tool_1 = testBit(state.actual_digital_input_bits, 17)

            if universe not in ["URsim"]:
                io_pub.publish(io_state)

            tcp_quat = trans.quaternion_about_axis(trans.vector_norm([state.actual_TCP_pose[3],
                                                                      state.actual_TCP_pose[4],
                                                                      state.actual_TCP_pose[5]]),
                                                   trans.unit_vector([state.actual_TCP_pose[3],
                                                                      state.actual_TCP_pose[4],
                                                                      state.actual_TCP_pose[5]]))
            br.sendTransform((state.actual_TCP_pose[0], state.actual_TCP_pose[1], state.actual_TCP_pose[2]),
                             tcp_quat, rospy.Time.now(), tool_frame, base_frame)
        except IOError:
            pass

    con.send_pause()
    con.disconnect()
