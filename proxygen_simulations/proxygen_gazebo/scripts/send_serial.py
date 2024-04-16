#!/usr/bin/env python

import sys, math
import rospy
from proxygen_msgs.srv import *
from std_msgs.msg import Float64
import serial
import time
from sensor_msgs.msg import JointState
import cv2
from ultralytics import YOLO
import pyrealsense2
# from realsense_depth import*
# dc=DepthCamera()
# ret,depth_frame,color_frame =dc.get_frame()
import math
from arm_class import MARS_ARM

# model = YOLO('~/Desktop/Flipkart1/best50.pt')
joint_pos = [0, 0, 0, 0]
# arduino = serial.Serial(port='COM4', baudrate=9600, timeout=3)
gripper_pub = None


def joint_states_callback(data):
    global joint_values

    # Get the joint names and positions from the message
    joint_names = data.name
    joint_positions = data.position

    # Loop through the joint names and positions to find 'joint1', 'joint2', 'joint3', and 'joint4'
    for name, position in zip(joint_names, joint_positions):
        if name == 'joint1':
            joint_pos[0] = round(position*180/math.pi, 2)
        elif name == 'joint2':
            joint_pos[1] = round(position*180/math.pi, 2)
        elif name == 'joint3':
            joint_pos[2] = round(position*180/math.pi, 2)
        elif name == 'joint4':
            joint_pos[3] = round(position*180/math.pi, 2)   


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
            print (f'End effector is going to {x}, {y}, {z} position :)')
        else: 
            print('Unable to move end-effector : ')

    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def go_to_default_posn():
    service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

    service_request = SetJointPositionRequest()
    service_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4", "gripper"]
    service_request.joint_position.position = [0.0, 0.0, -0.26, 0.0, 0.0]
    # 1.08
    service_request.path_time = 2.0

    service_response = service_client(service_request)

    if service_response.is_planned:
        print('Successfully set joint positions.')
    else:
        print('Failed to set joint positions.')
    

def close_gripper():
    if(gripper_pub == None):
        rospy.loginfo("gripper publisher not correctly initiated")
    
def open_gripper():
    if(gripper_pub == None):
        rospy.loginfo("gripper publisher not correctly initiated")

def sleep():
    r = rospy.Rate(1)
    for i in range(5):
        r.sleep()

def pick_and_place_package(obj_posn, goal_posn):
    global arm
    go_to_default_posn()
    input("movement log to send value to serial")
    arm.go_to_default_angle(0)
    input("movement log  arm has reached")
    
    set_position_client(obj_posn[0], obj_posn[1], obj_posn[2], 2)
    input("movement log to send value to serial")
    arm.go_to_desired_angle(joint_pos, 255)
    
    set_position_client(obj_posn[0], obj_posn[1], round(obj_posn[2]-0.06, 2), 2)
    input("movement log to send value to serial")
    arm.go_to_desired_angle(joint_pos, 255)
    # while(input("Press 1 to press the box or Enter if arm has gripped the box") == '1'):
    #     new_pos = joint_pos
    #     new_pos[2]+=0.01
    #     arm.go_to_desired_angle(new_pos, 255)
    go_to_default_posn()
    input("movement log to send value to serial")
    arm.go_to_default_angle(255)
    input("movement log  arm has reached")
    
    # to move robotic arm to goal posn
    set_position_client(goal_posn[0], goal_posn[1], goal_posn[2], 2)
    input("movement log to send value to serial")
    arm.go_to_desired_angle(joint_pos, 255)
    input("movement log  arm has reached")
    
    go_to_default_posn()
    arm.go_to_default_angle0(0)
    input("movement log  arm has reached")
    
    
if __name__ == "__main__":
    rospy.init_node('open_manipulator_controller')
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    arm = MARS_ARM()
    obj_posns = [[0.432, 0.0, 0.10], [0.254, 0.204, 0.13], [0.432, 0.0, 0.7]]
    goal_posn = [0.378, -0.274, 0.156]
    # to move robotic arm to object posn
    for obj_posn in obj_posns:
        pick_and_place_package(obj_posn, goal_posn)
        print("package movement performed!! \n\n")
    
    # for i in range(100):
    #     xyz = [round(float(x), 4) for x in input("enter xyz point: ").split()]
    #     if len(xyz) == 3:
    #         set_position_client(xyz[0], xyz[1], xyz[2], 2)
    #         input("Hit enter to move bot")
    #         arm.go_to_desired_angle(joint_pos, 255)
    #         # pick_and_place_package(xyz, goal_posn)
    #         # print("package movement performed!! \n\n")
    #     else:
    #         pass
    #         # print("I want only 3 float values :) ")
    
    
    rospy.spin()