﻿################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(proxygen_control_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  proxygen_msgs
  cmake_modules
)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

set(${PROJECT_NAME}_UIS
  ui/main_window.ui
)

set(${PROJECT_NAME}_RESOURCES
  resources/images.qrc
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()



################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs proxygen_msgs
  DEPENDS EIGEN3
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${proxygen_control_gui_INCLUDE_DIRECTORIES})

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")


################################################################################
# Build
################################################################################
include_directories(
  include
  ${proxygen_control_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(proxygen_control_gui ${proxygen_control_gui_SRCS} ${proxygen_control_gui_MOCS} ${proxygen_control_gui_UIS_H} ${proxygen_control_gui_RCC})
add_dependencies(proxygen_control_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(proxygen_control_gui Qt5::Widgets ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

#################################################################################
## Install
#################################################################################
install(TARGETS proxygen_control_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resources ui launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
