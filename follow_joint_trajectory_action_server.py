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

import copy

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryResult, FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, \
    FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from nomagic_ur_driver.msg import MoveURGoal
from nomagic_ur_driver.srv import FollowJointTrajectorySpeedMultiplier, FollowJointTrajectorySpeedMultiplierRequest, \
    FollowJointTrajectorySpeedMultiplierResponse

REQUIRED_JOINT_SEQUENCE = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

NUM_JOINTS = len(REQUIRED_JOINT_SEQUENCE)
EMPTY_VALUES = [0.0] * NUM_JOINTS

POSITIONS_OFFSET = 0
VELOCITIES_OFFSET = NUM_JOINTS
TIME_OFFSET = 2 * NUM_JOINTS


def _get_urscript_index(index):
    # As the default value of int registers is 0, we use 1-based indexing in URscript to avoid misinterpretting
    # default values.
    return index + 1


def _is_ready_for_waypoint(ur_state, index):
    return ur_state.output_int_register_1 == _get_urscript_index(index)


class FollowJointTrajectoryActionServer:

    def __init__(self, command_sender):
        self.command_sender = command_sender
        self.success = False

        self.positions = []
        self.velocities = []
        self.times = []
        self.name = "robot_controller/follow_joint_trajectory"
        self.default_speed_multiplier = 1.0
        self.next_speed_multiplier = self.default_speed_multiplier
        self._as = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction,
                                                execute_cb=self.follow_joint_trajectory_received, auto_start=False)
        self._as.start()
        rospy.loginfo("Started FollowJointTrajectoryAction server: {}".format(self.name))
        self.set_speed_multiplier_service = rospy.Service('follow_joint_trajectory_speed_multiplier',
                                                          FollowJointTrajectorySpeedMultiplier,
                                                          self.set_speed_multiplier)
        rospy.loginfo("Started FollowJointTrajectoryAction speed multiplier service")

    def _send_feedback(self, index):
        # Start sending feedback only after 2 points are sent and ignore last waypoint, which is sentinel.
        if index >= 1 and index + 1 < self._waypoints_num():
            rospy.logdebug("Sending feedback")
            ur_state = self.command_sender.last_ur_state
            feedback = FollowJointTrajectoryFeedback(
                joint_names=REQUIRED_JOINT_SEQUENCE,
                actual=JointTrajectoryPoint(
                    positions=self.positions[index - 1],
                    velocities=self.velocities[index - 1]
                ),
                desired=JointTrajectoryPoint(
                    positions=self.positions[index],
                    velocities=self.velocities[index]
                )
            )
            self._as.publish_feedback(feedback)
        else:
            rospy.logdebug("Ignoring sending feedback")
        return

    def set_speed_multiplier(self, request):
        # type: (FollowJointTrajectorySpeedMultiplierRequest) -> FollowJointTrajectorySpeedMultiplierResponse
        if request.set_default:
            self.default_speed_multiplier = request.speed_multiplier
            rospy.loginfo("Default speed multiplier set to {}".format(self.default_speed_multiplier))
        self.next_speed_multiplier = request.speed_multiplier
        rospy.loginfo("Next speed multipler set to {}".format(self.next_speed_multiplier))
        return FollowJointTrajectorySpeedMultiplierResponse()

    def post_process_trajectory(self, trajectory):
        # type: (FollowJointTrajectoryGoal) -> FollowJointTrajectoryGoal
        rospy.logdebug("Post processing trajectory")
        new_trajectory = copy.deepcopy(trajectory)
        for point in new_trajectory.trajectory.points:
            new_point_velocities = list(point.velocities)
            for i in range(len(new_point_velocities)):
                new_point_velocities[i] = self.next_speed_multiplier * new_point_velocities[i]
            point.velocities = tuple(new_point_velocities)
            point.time_from_start = point.time_from_start * self.next_speed_multiplier
        rospy.logdebug("Original trajectory: \n{}".format(trajectory))
        rospy.logdebug("Post processed trajectory: \n{}".format(new_trajectory))
        return new_trajectory

    def _waypoints_num(self):
        assert len(self.positions) == len(self.times)
        assert len(self.times) == len(self.velocities)
        return len(self.positions)

    def _prepare_main_command(self):
        self.command_sender.clear_command()
        self.command_sender.command.input_int_register_0 = MoveURGoal.MOVEIT

    def _prepare_single_waypoint_command(self, index):
        self.command_sender.clear_command()
        command = self.command_sender.command
        command.input_int_register_1 = _get_urscript_index(index)
        for i in range(NUM_JOINTS):
            setattr(command, "input_double_register_{}".format(POSITIONS_OFFSET + i), self.positions[index][i])
        for i in range(NUM_JOINTS):
            setattr(command, "input_double_register_{}".format(VELOCITIES_OFFSET + i), self.velocities[index][i])
        setattr(command, "input_double_register_{}".format(TIME_OFFSET), self.times[index])

    def _send_waypoint(self, index):
        self.command_sender.wait_until(lambda state: _is_ready_for_waypoint(state, index))
        self._prepare_single_waypoint_command(index)
        rospy.logdebug("Sending waypoint #{}: positions={} velocities={} time={}".format(
            index, self.positions[index], self.velocities[index], self.times[index]))
        self.command_sender.send()

    def _send_waypoints(self):
        rospy.loginfo("Sending {} waypoints...".format(self._waypoints_num()))
        for i in range(self._waypoints_num()):
            is_preempted = self._as.is_preempt_requested()
            if is_preempted:
                rospy.loginfo('Preempted {}'.format(self.name))
                # Action has been preempted, but URscript expects next waypoint. In order to unblock it we need to send
                # expected waypoint which will be a sentinel.
                self.positions[i] = copy.copy(EMPTY_VALUES)
                self.velocities[i] = copy.copy(EMPTY_VALUES)
                self.times[i] = 0.0
            self._send_waypoint(i)
            self._send_feedback(i)
            if is_preempted:
                # We have sent sentinel waypoint so it's safe to return
                return
        rospy.loginfo("All waypoints sent successfully")
        self.success = True
        return

    def follow_joint_trajectory_received(self, trajectory):
        # type: (FollowJointTrajectoryGoal) -> None
        self.success = False
        rospy.loginfo("FollowJointTrajectoryActionGoal received")
        rospy.logdebug("Trajectory received: {}".format(trajectory))
        received_joint_sequence = trajectory.trajectory.joint_names
        map_joints_from_received_to_urscript = {}
        for i, joint in enumerate(received_joint_sequence):
            map_joints_from_received_to_urscript[i] = REQUIRED_JOINT_SEQUENCE.index(joint)
        positions = []
        velocities = []
        times = []

        new_trajectory = self.post_process_trajectory(trajectory)
        for point in new_trajectory.trajectory.points:
            pos = copy.copy(EMPTY_VALUES)
            vel = copy.copy(EMPTY_VALUES)
            for i in range(len(REQUIRED_JOINT_SEQUENCE)):
                mapped_index = map_joints_from_received_to_urscript[i]
                pos[mapped_index] = point.positions[i]
                vel[mapped_index] = point.velocities[i]
            positions.append(pos)
            velocities.append(vel)
            times.append(point.time_from_start.to_sec())
        positions.append(copy.copy(EMPTY_VALUES))
        velocities.append(copy.copy(EMPTY_VALUES))
        times.append(0.0)
        self.positions = positions
        self.velocities = velocities
        self.times = times

        self._prepare_main_command()
        try:
            self.command_sender.send_command(wait_for_completion=True, subcommand_cb=self._send_waypoints)
        except Exception as e:
            rospy.logwarn("Exception during command execution: {}".format(e))

        self.next_speed_multiplier = self.default_speed_multiplier
        if self.success:
            self._as.set_succeeded(FollowJointTrajectoryResult.SUCCESSFUL)
        else:
            self._as.set_aborted()
        rospy.logdebug("Finished processing FollowJoinTrajectory goal")
