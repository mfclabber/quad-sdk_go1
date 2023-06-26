cmake_minimum_required(VERSION 3.0.2)
project(robot_driver)

## Compile as C++14, supported in ROS Melodic and newer
# add_compile_options(-std=c++17 -fsanitize=address,undefined)
# add_link_options(-fsanitize=address,undefined)
add_compile_options(-std=c++17)
  
## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Setup MBLINK and MAVLINK
set(CMAKE_CXX_STANDARD 17)
set(MBLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/mblink/)

message(STATUS "Using architecture ${CMAKE_SYSTEM_PROCESSOR}")

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(GRMBLINK libgrmblink_tx2.a)
else()
  set(GRMBLINK libgrmblink_linux.a)
endif()

include_directories(${MBLINK_DIR}/include)
link_directories(${MBLINK_DIR}/lib)
#######
# LCM #
#######
set(LCM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk)

include_directories(${LCM_DIR}/include)
link_directories(${LCM_DIR}/lib "../unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a")
# set(EXTRA_LIBS -pthread libunitree_legged_sdk lcm)
set(EXTRA_LIBS -pthread lcm)

find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quad_msgs
  quad_utils
  eigen_conversions
  unitree_legged_msgs
  unitree_legged_real
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs quad_msgs quad_utils eigen_conversions
  DEPENDS EIGEN3
)

###########
##  Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}../unitree_legged_sdk/include
  ${CMAKE_SOURCE_DIR}../unitree_legged_sdk/lib
)

## Declare a C++ library for control
add_library(robot_driver
  src/robot_driver.cpp
  src/robot_driver_utils.cpp
  src/controllers/leg_controller.cpp
  src/controllers/inverse_dynamics_controller.cpp
  src/controllers/grf_pid_controller.cpp
  src/controllers/joint_controller.cpp
  src/estimators/state_estimator.cpp
  src/estimators/comp_filter_estimator.cpp
  src/estimators/ekf_estimator.cpp
  src/estimators/unitree_estimator.cpp
  src/hardware_interfaces/hardware_interface.cpp
  src/hardware_interfaces/spirit_interface.cpp
  # src/hardware_interfaces/a1_interface.cpp
  src/hardware_interfaces/go1_interface.cpp
  src/estimators/realsense_estimator.cpp
)
add_dependencies(robot_driver ${robot_driver_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_driver
  ${GRMBLINK}
  ${catkin_LIBRARIES}
  ${EXTRA_LIBS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(robot_driver_node src/robot_driver_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_driver_node
  robot_driver
	${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a
)

#############
## Testing ##
#############

## Google test file for robot_driver library
catkin_add_gtest(robot_driver_test
  test/test_robot_driver.cpp)
target_link_libraries(robot_driver_test robot_driver ${catkin_LIBRARIES})


