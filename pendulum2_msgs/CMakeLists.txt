cmake_minimum_required(VERSION 3.5)

project(pendulum2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)

set(msg_files
    "msg/JointState.msg"
    "msg/JointCommand.msg"
    "msg/JointCommandStamped.msg"
    "msg/PendulumTeleop.msg"
    )


add_message_files(
  ${msg_files}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
