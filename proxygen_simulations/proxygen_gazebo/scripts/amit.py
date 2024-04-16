#!/usr/bin/env python

import sys
import rospy
from proxygen_msgs.srv import *
from std_msgs.msg import Float64
import serial
import time
from sensor_msgs.msg import JointState
import cv2
from ultralytics import YOLO
import math
import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)



        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()


dc=DepthCamera()
ret,depth_frame,color_frame =dc.get_frame()

model = YOLO('/home/rajeev-gupta/grid_ws/src/proxygen_simulations/proxygen_gazebo/scripts/best50.pt')
joint_pos = []
arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=3)

obj_posn = [0, 0, 0]
goal_posn = [0, 0, 0]

def write_joint_val():
    arduino.write(joint_pos[0])
    sleep(1)
    arduino.write(joint_pos[1])
    sleep(1)
    arduino.write(joint_pos[2])
    sleep(1)
    arduino.write(joint_pos[3])
    sleep(1)
    arduino.write(joint_pos[4])
    sleep(1)

def joint_states_callback(data):
    joint_pos = data.position
    


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
            print('Unable to move end-effector : ')

    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def go_to_default_posn():
    service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

    service_request = SetJointPositionRequest()
    service_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4", "gripper"]
    service_request.joint_position.position = [0.0, 0.0, 0.0, 1.08, 0.0]
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
    write_joint_val()
    sleep()
    set_position_client(obj_posn[0], obj_posn[1], obj_posn[2], 2)
    write_joint_val()
    sleep()
    go_to_default_posn()
    write_joint_val()
    sleep()
    set_position_client(goal_posn[0], goal_posn[1], goal_posn[2], 2)
    write_joint_val()
    sleep()
    go_to_default_posn()

def sleep():
    r = rospy.Rate(1)
    for i in range(5):
        r.sleep()

if __name__ == "__main__":
    rospy.init_node('open_manipulaotr_controller')
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    results = model(color_frame,save=True,show=True)

    for r in results:
        xyxy = r.boxes.xyxy
        print(xyxy)  # r.xyxy contains the bounding box coordinates
        x = (xyxy[0][0] + xyxy[0][2])/2
        y = (xyxy[0][1] + xyxy[0][3])/2
        print(int(x)*((133)/640))
        print(int(y)*((100)/480))
        obj_posn[0] = int(x)*((1.33)/640)
        obj_posn[1] = int(y)*((1.00)/480)
        pixel_point=(int(x),int(y))
        cv2.circle(color_frame, pixel_point, 4, (0, 0, 255))
        distance = depth_frame[pixel_point[1],pixel_point[0]]
        obj_posn[2] = distance
        print(distance)
        break
    pick_and_place(obj_posn, goal_posn)
    rospy.spin()