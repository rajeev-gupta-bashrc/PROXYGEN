/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef proxygen_CONTROLLER_H_
#define proxygen_CONTROLLER_H_

#include <boost/thread.hpp>
#include <unistd.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "proxygen_libs/proxygen.h"
#include "proxygen_msgs/SetJointPosition.h"
#include "proxygen_msgs/SetKinematicsPose.h"
#include "proxygen_msgs/SetDrawingTrajectory.h"
#include "proxygen_msgs/SetActuatorState.h"
#include "proxygen_msgs/GetJointPosition.h"
#include "proxygen_msgs/GetKinematicsPose.h"
#include "proxygen_msgs/OpenManipulatorState.h"

namespace proxygen_controller
{
class OpenManipulatorController
{
 public:
  OpenManipulatorController(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorController();

  // update
  void publishCallback(const ros::TimerEvent&);
  void startTimerThread();
  static void *timerThread(void *param);
  void process(double time);
  double getControlPeriod(void){return control_period_;}

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  bool using_platform_;
  double control_period_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  // Thread parameter
  pthread_t timer_thread_;
  pthread_attr_t attr_;
  bool timer_thread_state_;

  // Robotis_manipulator related 
  OpenManipulator proxygen_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher proxygen_states_pub_;
  std::vector<ros::Publisher> proxygen_kinematics_pose_pub_;
  ros::Publisher proxygen_joint_states_pub_;
  std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  ros::Subscriber proxygen_option_sub_;

  void openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Servers and Callback Functions
  *****************************************************************************/
  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_pose_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_position_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_orientation_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_task_space_path_position_only_server_;
  ros::ServiceServer goal_task_space_path_orientation_only_server_;
  ros::ServiceServer goal_joint_space_path_from_present_server_;
  ros::ServiceServer goal_task_space_path_from_present_position_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_orientation_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_server_;
  ros::ServiceServer goal_tool_control_server_;
  ros::ServiceServer set_actuator_state_server_;
  ros::ServiceServer goal_drawing_trajectory_server_;
  ros::ServiceServer get_joint_position_server_;
  ros::ServiceServer get_kinematics_pose_server_;
  ros::ServiceServer set_joint_position_server_;
  ros::ServiceServer set_kinematics_pose_server_;

  bool goalJointSpacePathCallback(proxygen_msgs::SetJointPosition::Request  &req,
                                  proxygen_msgs::SetJointPosition::Response &res);

  bool goalJointSpacePathToKinematicsPoseCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                  proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsPositionCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                  proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsOrientationCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                  proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                 proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathPositionOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                             proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathOrientationOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathFromPresentCallback(proxygen_msgs::SetJointPosition::Request  &req,
                                             proxygen_msgs::SetJointPosition::Response &res);

  bool goalTaskSpacePathFromPresentCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                            proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentPositionOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                        proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentOrientationOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                           proxygen_msgs::SetKinematicsPose::Response &res);

  bool goalToolControlCallback(proxygen_msgs::SetJointPosition::Request  &req,
                               proxygen_msgs::SetJointPosition::Response &res);

  bool setActuatorStateCallback(proxygen_msgs::SetActuatorState::Request  &req,
                                proxygen_msgs::SetActuatorState::Response &res);

  bool goalDrawingTrajectoryCallback(proxygen_msgs::SetDrawingTrajectory::Request  &req,
                                     proxygen_msgs::SetDrawingTrajectory::Response &res);
};
}
#endif //proxygen_CONTROLLER_H_
