################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(proxygen_libs)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    robotis_manipulator
    roscpp
    cmake_modules
    dynamixel_workbench_toolbox
)
find_package(Eigen3 REQUIRED)

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
  LIBRARIES proxygen_libs
  CATKIN_DEPENDS roscpp robotis_manipulator dynamixel_workbench_toolbox cmake_modules
  DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(proxygen_libs
  src/proxygen.cpp
  src/custom_trajectory.cpp
  src/dynamixel.cpp
  src/kinematics.cpp
)

add_dependencies(proxygen_libs ${catkin_EXPORTED_TARGETS})
target_link_libraries(proxygen_libs  ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS proxygen_libs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
