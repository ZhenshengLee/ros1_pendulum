cmake_minimum_required(VERSION 3.5)

project(pendulum_driver)

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
  sensor_msgs
  pendulum2_msgs
  lifecycle
  lifecycle_msgs
  pendulum_utils
)

set(dependencies
    roscpp
    lifecycle
    lifecycle_msgs
    sensor_msgs
    pendulum2_msgs
    pendulum_utils)

set(PENDULUM_DRIVER_LIB pendulum_driver)
add_library(${PENDULUM_DRIVER_LIB} SHARED
    src/pendulum_driver_node.cpp
    src/pendulum_driver.cpp
    src/pendulum_driver_config.cpp)

target_include_directories(${PENDULUM_DRIVER_LIB}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PENDULUM_DRIVER_LIB} ${dependencies} ${catkin_LIBRARIES})

# Generate standalone node executable
set(PENDULUM_DRIVER_EXE "${PENDULUM_DRIVER_LIB}_exe")
add_executable(${PENDULUM_DRIVER_EXE} src/pendulum_driver_node_main.cpp)
target_link_libraries(${PENDULUM_DRIVER_EXE} ${PENDULUM_DRIVER_LIB} ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PENDULUM_DRIVER_LIB}
  CATKIN_DEPENDS ${dependencies}
  DEPENDS
)

# if(BUILD_TESTING)
#   find_package(apex_test_tools REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   apex_test_tools_add_gtest(test_pendulum_driver test/test_pendulum_driver.cpp)
#   if(TARGET test_pendulum_driver)
#     target_link_libraries(test_pendulum_driver ${PENDULUM_DRIVER_LIB})
#   endif()
#   apex_test_tools_add_gtest(test_pendulum_driver_node test/test_pendulum_driver_node.cpp)
#   if(TARGET test_pendulum_driver_node)
#     target_link_libraries(test_pendulum_driver_node ${PENDULUM_DRIVER_LIB})
#   endif()

#   find_package(launch_testing_ament_cmake)
#   add_launch_test(
#       test/pendulum_driver_lifecycle.test.py
#       TARGET pendulum_driver_lifecycle.test
#       ENV
#       TIMEOUT 30
#   )
#   add_launch_test(
#       test/pendulum_driver_autostart.test.py
#       TARGET pendulum_driver_autostart.test
#       ENV
#       TIMEOUT 30
#   )
# endif()

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)

install(TARGETS ${PENDULUM_DRIVER_LIB} ${PENDULUM_DRIVER_EXE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
