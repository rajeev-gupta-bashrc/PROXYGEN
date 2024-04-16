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

#include "proxygen_controller/proxygen_controller.h"

using namespace proxygen_controller;

OpenManipulatorController::OpenManipulatorController(std::string usb_port, std::string baud_rate)
: node_handle_(""),
  priv_node_handle_("~"),
  timer_thread_state_(false)
{
  /************************************************************
  ** Initialize ROS parameters
  ************************************************************/
  control_period_       = priv_node_handle_.param<double>("control_period", 0.010f);
  using_platform_       = priv_node_handle_.param<bool>("using_platform", false);

  /************************************************************
  ** Initialize variables
  ************************************************************/
  proxygen_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);

  if (using_platform_ == true) log::info("Succeeded to init " + priv_node_handle_.getNamespace());
  else if (using_platform_ == false) log::info("Ready to simulate " +  priv_node_handle_.getNamespace() + " on Gazebo");

  /************************************************************
  ** Initialize ROS publishers, subscribers and servers
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();
}

OpenManipulatorController::~OpenManipulatorController()
{
  timer_thread_state_ = false;
  pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
  log::info("Shutdown OpenManipulator Controller");
  proxygen_.disableAllActuator();
  ros::shutdown();
}

void OpenManipulatorController::startTimerThread()
{
  ////////////////////////////////////////////////////////////////////
  /// Use this when you want to increase the priority of threads.
  ////////////////////////////////////////////////////////////////////
  //  pthread_attr_t attr_;
  //  int error;
  //  struct sched_param param;
  //  pthread_attr_init(&attr_);

  //  error = pthread_attr_setschedpolicy(&attr_, SCHED_RR);
  //  if (error != 0)   log::error("pthread_attr_setschedpolicy error = ", (double)error);
  //  error = pthread_attr_setinheritsched(&attr_, PTHREAD_EXPLICIT_SCHED);
  //  if (error != 0)   log::error("pthread_attr_setinheritsched error = ", (double)error);

  //  memset(&param, 0, sizeof(param));
  //  param.sched_priority = 31;    // RT
  //  error = pthread_attr_setschedparam(&attr_, &param);
  //  if (error != 0)   log::error("pthread_attr_setschedparam error = ", (double)error);

  //  if ((error = pthread_create(&this->timer_thread_, &attr_, this->timerThread, this)) != 0)
  //  {
  //    log::error("Creating timer thread failed!!", (double)error);
  //    exit(-1);
  //  }
  // timer_thread_state_ = true;
  ////////////////////////////////////////////////////////////////////

  int error;
  if ((error = pthread_create(&this->timer_thread_, NULL, this->timerThread, this)) != 0)
  {
    log::error("Creating timer thread failed!!", (double)error);
    exit(-1);
  }
  timer_thread_state_ = true;
}

void *OpenManipulatorController::timerThread(void *param)
{
  OpenManipulatorController *controller = (OpenManipulatorController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(controller->timer_thread_state_)
  {
    next_time.tv_sec += (next_time.tv_nsec + ((int)(controller->getControlPeriod() * 1000)) * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ((int)(controller->getControlPeriod() * 1000)) * 1000000) % 1000000000;

    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    /////
    double delta_nsec = controller->getControlPeriod() - ((next_time.tv_sec - curr_time.tv_sec) + ((double)(next_time.tv_nsec - curr_time.tv_nsec)*0.000000001));
    //log::info("control time : ", controller->getControlPeriod() - delta_nsec);
    if (delta_nsec > controller->getControlPeriod())
    {
      //log::warn("Over the control time : ", delta_nsec);
      next_time = curr_time;
    }
    else
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    /////
  }
  return 0;
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorController::initPublisher()
{
  // ros message publisher
  auto om_tools_name = proxygen_.getManipulator()->getAllToolComponentName();

  for (auto const& name:om_tools_name)
  {
    ros::Publisher pb;
    pb = node_handle_.advertise<proxygen_msgs::KinematicsPose>(name + "/kinematics_pose", 10);
    proxygen_kinematics_pose_pub_.push_back(pb);
  }
  proxygen_states_pub_ = node_handle_.advertise<proxygen_msgs::OpenManipulatorState>("states", 10);

  if (using_platform_ == true)
  {
    proxygen_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
  }
  else
  {
    auto gazebo_joints_name = proxygen_.getManipulator()->getAllActiveJointComponentName();
    gazebo_joints_name.reserve(gazebo_joints_name.size() + om_tools_name.size());
    gazebo_joints_name.insert(gazebo_joints_name.end(), om_tools_name.begin(), om_tools_name.end());

    for (auto const& name:gazebo_joints_name)
    {
      ros::Publisher pb;
      pb = node_handle_.advertise<std_msgs::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
  }
}
void OpenManipulatorController::initSubscriber()
{
  // ros message subscriber
  proxygen_option_sub_ = node_handle_.subscribe("option", 10, &OpenManipulatorController::openManipulatorOptionCallback, this);
}

void OpenManipulatorController::initServer()
{
  goal_joint_space_path_server_                     = node_handle_.advertiseService("goal_joint_space_path", &OpenManipulatorController::goalJointSpacePathCallback, this);
  goal_joint_space_path_to_kinematics_pose_server_  = node_handle_.advertiseService("goal_joint_space_path_to_kinematics_pose", &OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback, this);
  goal_joint_space_path_to_kinematics_position_server_  = node_handle_.advertiseService("goal_joint_space_path_to_kinematics_position", &OpenManipulatorController::goalJointSpacePathToKinematicsPositionCallback, this);
  goal_joint_space_path_to_kinematics_orientation_server_  = node_handle_.advertiseService("goal_joint_space_path_to_kinematics_orientation", &OpenManipulatorController::goalJointSpacePathToKinematicsOrientationCallback, this);

  goal_task_space_path_server_                  = node_handle_.advertiseService("goal_task_space_path", &OpenManipulatorController::goalTaskSpacePathCallback, this);
  goal_task_space_path_position_only_server_    = node_handle_.advertiseService("goal_task_space_path_position_only", &OpenManipulatorController::goalTaskSpacePathPositionOnlyCallback, this);
  goal_task_space_path_orientation_only_server_ = node_handle_.advertiseService("goal_task_space_path_orientation_only", &OpenManipulatorController::goalTaskSpacePathOrientationOnlyCallback, this);

  goal_joint_space_path_from_present_server_      = node_handle_.advertiseService("goal_joint_space_path_from_present", &OpenManipulatorController::goalJointSpacePathFromPresentCallback, this);

  goal_task_space_path_from_present_server_                   = node_handle_.advertiseService("goal_task_space_path_from_present", &OpenManipulatorController::goalTaskSpacePathFromPresentCallback, this);
  goal_task_space_path_from_present_position_only_server_     = node_handle_.advertiseService("goal_task_space_path_from_present_position_only", &OpenManipulatorController::goalTaskSpacePathFromPresentPositionOnlyCallback, this);
  goal_task_space_path_from_present_orientation_only_server_  = node_handle_.advertiseService("goal_task_space_path_from_present_orientation_only", &OpenManipulatorController::goalTaskSpacePathFromPresentOrientationOnlyCallback, this);

  goal_tool_control_server_         = node_handle_.advertiseService("goal_tool_control", &OpenManipulatorController::goalToolControlCallback, this);
  set_actuator_state_server_        = node_handle_.advertiseService("set_actuator_state", &OpenManipulatorController::setActuatorStateCallback, this);
  goal_drawing_trajectory_server_   = node_handle_.advertiseService("goal_drawing_trajectory", &OpenManipulatorController::goalDrawingTrajectoryCallback, this);
}

/*****************************************************************************
** Callback Functions for ROS Subscribers
*****************************************************************************/
void OpenManipulatorController::openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "print_proxygen_setting")
    proxygen_.printManipulatorSetting();
}

/*****************************************************************************
** Callback Functions for ROS Servers
*****************************************************************************/
bool OpenManipulatorController::goalJointSpacePathCallback(proxygen_msgs::SetJointPosition::Request  &req,
                                                           proxygen_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for (int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  if (!proxygen_.makeJointTrajectory(target_angle, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                           proxygen_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  if (!proxygen_.makeJointTrajectory(req.end_effector_name, target_pose, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsPositionCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                               proxygen_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  if (!proxygen_.makeJointTrajectory(req.end_effector_name, target_pose.position, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsOrientationCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                                  proxygen_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  if (!proxygen_.makeJointTrajectory(req.end_effector_name, target_pose.orientation, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                          proxygen_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  if (!proxygen_.makeTaskTrajectory(req.end_effector_name, target_pose, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalTaskSpacePathPositionOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                      proxygen_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  if (!proxygen_.makeTaskTrajectory(req.end_effector_name, position, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalTaskSpacePathOrientationOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                         proxygen_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  if (!proxygen_.makeTaskTrajectory(req.end_effector_name, orientation, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalJointSpacePathFromPresentCallback(proxygen_msgs::SetJointPosition::Request  &req,
                                                                      proxygen_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for (int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  if (!proxygen_.makeJointTrajectoryFromPresentPosition(target_angle, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                     proxygen_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  if (!proxygen_.makeTaskTrajectoryFromPresentPose(req.planning_group, target_pose, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentPositionOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                                 proxygen_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  if (!proxygen_.makeTaskTrajectoryFromPresentPose(req.planning_group, position, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentOrientationOnlyCallback(proxygen_msgs::SetKinematicsPose::Request  &req,
                                                                                    proxygen_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                        req.kinematics_pose.pose.orientation.x,
                        req.kinematics_pose.pose.orientation.y,
                        req.kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  if (!proxygen_.makeTaskTrajectoryFromPresentPose(req.planning_group, orientation, req.path_time))
    res.is_planned = false;
  else
    res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalToolControlCallback(proxygen_msgs::SetJointPosition::Request  &req,
                                                        proxygen_msgs::SetJointPosition::Response &res)
{
  for (int i = 0; i < req.joint_position.joint_name.size(); i ++)
  {
    if (!proxygen_.makeToolTrajectory(req.joint_position.joint_name.at(i), req.joint_position.position.at(i)))
      res.is_planned = false;
    else
      res.is_planned = true;
  }

  return true;
}

bool OpenManipulatorController::setActuatorStateCallback(proxygen_msgs::SetActuatorState::Request  &req,
                                                         proxygen_msgs::SetActuatorState::Response &res)
{
  if (req.set_actuator_state == true) // enable actuators
  {
    log::println("Wait a second for actuator enable", "GREEN");
    timer_thread_state_ = false;
    pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
    proxygen_.enableAllActuator();
    startTimerThread();
  }
  else // disable actuators
  {
    log::println("Wait a second for actuator disable", "GREEN");
    timer_thread_state_ = false;
    pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
    proxygen_.disableAllActuator();
    startTimerThread();
  }

  res.is_planned = true;

  return true;
}

bool OpenManipulatorController::goalDrawingTrajectoryCallback(proxygen_msgs::SetDrawingTrajectory::Request  &req,
                                                              proxygen_msgs::SetDrawingTrajectory::Response &res)
{
  try
  {
    if (req.drawing_trajectory_name == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;

      if (!proxygen_.makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req.end_effector_name, p_draw_circle_arg, req.path_time))
        res.is_planned = false;
      else
        res.is_planned = true;
    }
    else if (req.drawing_trajectory_name == "line")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = req.param[0]; // x axis (m)
      draw_line_arg.kinematic.position(1) = req.param[1]; // y axis (m)
      draw_line_arg.kinematic.position(2) = req.param[2]; // z axis (m)
      void *p_draw_line_arg = &draw_line_arg;
      
      if (!proxygen_.makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, req.end_effector_name, p_draw_line_arg, req.path_time))
        res.is_planned = false;
      else
        res.is_planned = true;
    }
    else if (req.drawing_trajectory_name == "rhombus")
    {
      double draw_rhombus_arg[3];
      draw_rhombus_arg[0] = req.param[0];  // radius (m)
      draw_rhombus_arg[1] = req.param[1];  // revolution (rev)
      draw_rhombus_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_rhombus_arg = &draw_rhombus_arg;

      if (!proxygen_.makeCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, req.end_effector_name, p_draw_rhombus_arg, req.path_time))
        res.is_planned = false;
      else
        res.is_planned = true;
    }
    else if (req.drawing_trajectory_name == "heart")
    {
      double draw_heart_arg[3];
      draw_heart_arg[0] = req.param[0];  // radius (m)
      draw_heart_arg[1] = req.param[1];  // revolution (rev)
      draw_heart_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_heart_arg = &draw_heart_arg;

      if (!proxygen_.makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, req.end_effector_name, p_draw_heart_arg, req.path_time))
        res.is_planned = false;
      else
        res.is_planned = true;
    }

    return true;
  }
  catch ( ros::Exception &e )
  {
    log::error("Creation the custom trajectory is failed!");
  }

  return true;
}

/********************************************************************************
** Callback function for process timer
********************************************************************************/
void OpenManipulatorController::process(double time)
{
  proxygen_.processOpenManipulator(time);
}

/********************************************************************************
** Callback function for publish timer
********************************************************************************/
void OpenManipulatorController::publishCallback(const ros::TimerEvent&)
{
  if (using_platform_ == true)  publishJointStates();
  else publishGazeboCommand();

  publishOpenManipulatorStates();
  publishKinematicsPose();
}

void OpenManipulatorController::publishOpenManipulatorStates()
{
  proxygen_msgs::OpenManipulatorState msg;
  if (proxygen_.getMovingState())
    msg.proxygen_moving_state = msg.IS_MOVING;
  else
    msg.proxygen_moving_state = msg.STOPPED;

  if (proxygen_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.proxygen_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.proxygen_actuator_state = msg.ACTUATOR_DISABLED;

  proxygen_states_pub_.publish(msg);
}

void OpenManipulatorController::publishKinematicsPose()
{
  proxygen_msgs::KinematicsPose msg;
  auto om_tools_name = proxygen_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (auto const& tools:om_tools_name)
  {
    KinematicPose pose = proxygen_.getKinematicPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    proxygen_kinematics_pose_pub_.at(index).publish(msg);
    index++;
  }
}

void OpenManipulatorController::publishJointStates()
{
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();

  auto joints_name = proxygen_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = proxygen_.getManipulator()->getAllToolComponentName();

  auto joint_value = proxygen_.getAllActiveJointValue();
  auto tool_value = proxygen_.getAllToolValue();

  for (uint8_t i = 0; i < joints_name.size(); i ++)
  {
    msg.name.push_back(joints_name.at(i));

    msg.position.push_back(joint_value.at(i).position);
    msg.velocity.push_back(joint_value.at(i).velocity);
    msg.effort.push_back(joint_value.at(i).effort);
  }

  for (uint8_t i = 0; i < tool_name.size(); i ++)
  {
    msg.name.push_back(tool_name.at(i));

    msg.position.push_back(tool_value.at(i).position);
    msg.velocity.push_back(0.0f);
    msg.effort.push_back(0.0f);
  }
  proxygen_joint_states_pub_.publish(msg);
}

void OpenManipulatorController::publishGazeboCommand()
{
  JointWaypoint joint_value = proxygen_.getAllActiveJointValue();
  JointWaypoint tool_value = proxygen_.getAllToolValue();

  for (uint8_t i = 0; i < joint_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = joint_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(i).publish(msg);
  }

  for (uint8_t i = 0; i < tool_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = tool_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(joint_value.size() + i).publish(msg);
  }
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "proxygen_controller");
  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc = 3)
  {
    usb_port = argv[1];
    baud_rate = argv[2];
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());
  }
  else
  {
    log::error("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 1;
  }

  OpenManipulatorController om_controller(usb_port, baud_rate);

  // update
  om_controller.startTimerThread();
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(om_controller.getControlPeriod()), &OpenManipulatorController::publishCallback, &om_controller);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
