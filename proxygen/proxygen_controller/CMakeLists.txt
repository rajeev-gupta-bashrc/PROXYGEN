################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(proxygen_controller)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    proxygen_msgs
    robotis_manipulator
    proxygen_libs
    cmake_modules
)
find_package(Boost REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    proxygen_msgs
    robotis_manipulator
    proxygen_libs
    cmake_modules
  DEPENDS Boost
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(proxygen_controller src/proxygen_controller.cpp)
add_dependencies(proxygen_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(proxygen_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS proxygen_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES 99-open-manipulator-cdc.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
