cmake_minimum_required(VERSION 3.5)

project(pendulum_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rostest
  rosunit
  actionlib
  # lifecycle
  # lifecycle_msgs
)

set(PENDULUM_UTILS_LIB pendulum_utils)

add_library(${PENDULUM_UTILS_LIB} SHARED
  src/memory_lock.cpp
  src/rt_thread.cpp)

target_include_directories(${PENDULUM_UTILS_LIB}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PENDULUM_UTILS_LIB} ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PENDULUM_UTILS_LIB}
  # CATKIN_DEPENDS roscpp lifecycle_msgs
  DEPENDS
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PENDULUM_UTILS_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
