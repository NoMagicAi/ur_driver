

# NoMagic UR RTDE driver

A new, RTDE based driver for UR10 robot arms from Universal Robots. It is designed to be run
as a separate ROS package.

## Installation

Copy this repository directly to your catkin working directory (e.g. by cloning it or using `git subtree`)
and make it with `catkin make`.

## Usage

This driver consists of two nodes:
 * `nomagic_ur_rtde_driver` - exposes FollowJoinTrajectory and MoveUR action servers
 * `nomagic_ur_state_publisher` - publishes state reported the robot.

For more details see section [Interfaces](#Interfaces).

In order to start the nodes, run:

```
rosrun nomagic_ur_driver nomagic_ur_rtde_driver.py
rosrun nomagic_ur_driver nomagic_ur_state_publisher.py _base_frame:=base_link _tool_frame:=tool0_controller
```

### Configuration

These ROS nodes use the following ROS parameters:
* `/galaxy/hardware/robot_ip` - robot IP

## Interfaces

### FollowJointTrajectory action server

Node `nomagic_ur_rtde_driver` exposes `FollowJointTrajectory` action server used by MoveIt (see
[documentation](https://moveit.ros.org/)). From MoveIt user perspective this is a drop-in replacement
for default implementation. See section [Internals](#Internals) for more details.

### MoveUR action server

Node `nomagic_ur_rtde_driver` exposes `MoveUR` action server, which allows users to perform operations
such as:
 * `movej()`
 * `movel()`
 * stop the program

This action server is available under name `move_robot` and expects `MoveURAction` message as input.

#### MoveJ

Calls `movej()` URscript function.

Parameters:
* `move_type=MOVE_JOINT`
* `target_joints`
* `velocity`
* `accelaration`

#### MoveL

Calls `movel()` URscript function.

Parameters:
* `move_type=MOVE_LINEAR`
* `target_pose`
* `velocity`
* `accelaration`

#### Stop program

Exits URscript program.

Parameters:
* `move_type=STOP_PROGRAM`

### UR state publisher

Node `nomagic_ur_state_publisher` publishes information about the robot state using ros topics:
* `/joint_states` - reports current state of robot joints; sends `sensor_msgs.msg.JointState` messages
* `/io_states` - reports current state of digital inputs (`standard` and `configurable`);
sends `nomagic_ur_driver.msg.IOState` messages.

## Internals

Functionality described above is implemented in two nodes - `nomagic_ur_rtde_driver` (exposes action servers)
and `nomagic_ur_state_publisher` (exposes robot state via ROS topics). Both of the use RTDE interface to
communicate with the robot.

All commands are executed by a single URscript. This allows us to avoid constant reloading of programs, which
can be very slow.

Note, that this approach implies that only a single command can be executed at a time. If user sends more
actions to be execute at once, they will fail with appropriate message.

### Optimizations

In order to improve reliability of communication between ROS nodes and URscript, we tried to keep amount
of data sent in both directions minimal. This implied that for `FollowJointTrajectory` actions,
instead of send each position separately, we only send waypoints (as planned by MoveIt) and perform interpolation
inside URscript. This helps us to perform more smooth movements and avoid protective stops in case communication
problems.

## Known issues

* Driver has some non-generic code for handling optoforce, grippers or succers. It should be generalized
and properly documented.
* MoveIt action can timeout, if it has been requested after certain period of inactivity
(https://github.com/NoMagicAi/gripper-ros/pull/485 should fix it)
