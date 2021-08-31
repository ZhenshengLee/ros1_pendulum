cmake_minimum_required(VERSION 3.5)
project(pendulum_state_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  sensor_msgs
  pendulum2_msgs
)

add_executable(pendulum_state_publisher pendulum_state_publisher.cpp)
target_link_libraries(pendulum_state_publisher rclcpp pendulum2_msgs sensor_msgs ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp pendulum2_msgs sensor_msgs
  DEPENDS
)

install(TARGETS pendulum_state_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()
