#!/usr/bin/env python

import sys
import rospy
from proxygen_msgs.srv import *
from std_msgs.msg import Float64


def set_position_client(x, y, z, time):
    service_name = 'goal_task_space_path_position_only'

    rospy.wait_for_service(service_name)

    try:
        set_position = rospy.ServiceProxy(service_name, SetKinematicsPose)

        arg = SetKinematicsPoseRequest()
        arg.end_effector_name = 'gripper'
        arg.kinematics_pose.pose.position.x = x
        arg.kinematics_pose.pose.position.y = y
        arg.kinematics_pose.pose.position.z = z
        arg.path_time = time
        resp1 = set_position(arg)
        if resp1:
            print ('End effector is going to %f, %f, %f position :)', x, y, z)
        else: 
            print('Unable to move end-effector :(')

    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def go_to_default_posn():
    service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

    service_request = SetJointPositionRequest()
    service_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4", "gripper"]
    service_request.joint_position.position = [0.0, 0.0, 0.0, -0.3, 0.0]
    # 1.08
    service_request.path_time = 2.0

    service_response = service_client(service_request)

    if service_response.is_planned:
        print('Successfully set joint positions.')
    else:
        print('Failed to set joint positions.')
    

'''def close_gripper():
    if(gripper_pub == None):
        rospy.loginfo("gripper publisher not correctly initiated")
    
def open_gripper():
    if(gripper_pub == None):
        rospy.loginfo("gripper publisher not correctly initiated")'''
    
def pick_and_place(obj_posn, goal_posn):
    go_to_default_posn()
    sleep()
    set_position_client(obj_posn[0], obj_posn[1], obj_posn[2], 2)
    sleep()
    go_to_default_posn()
    sleep()
    set_position_client(goal_posn[0], goal_posn[1], goal_posn[2], 2)
    sleep()
    go_to_default_posn()

def sleep():
    r = rospy.Rate(1)
    for i in range(5):
        r.sleep()

if __name__ == "__main__":
    rospy.init_node('open_manipulaotr_controller')
    obj_posn = [0.012, 0.189, 0.03]
    goal_posn = [0.012, -0.189, 0.03]
    pick_and_place(obj_posn, goal_posn)
