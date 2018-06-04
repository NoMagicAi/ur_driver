cmake_minimum_required(VERSION 2.8.3)
project(nomagic_ur_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
    roscpp
    genmsg
    actionlib_msgs
    actionlib
    std_msgs
    geometry_msgs
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

add_service_files(
    FILES
    FollowJointTrajectorySpeedMultiplier.srv
)

add_message_files(
    FILES
    ForceTorqueThreshold.msg
    IOState.msg
)

add_action_files(
    DIRECTORY
    action
    FILES
    MoveUR.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
)

catkin_package()