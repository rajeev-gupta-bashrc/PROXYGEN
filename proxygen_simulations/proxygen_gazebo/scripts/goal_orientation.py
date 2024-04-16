#!/usr/bin/env python

import sys
import rospy, tf
from proxygen_msgs.srv import *

def set_position_client(r, p, y, time):
    service_name = 'goal_task_space_path_position_only'
    quaternion = tf.transformations.quaternion_from_euler(r, p, y)
    print(quaternion)
    rospy.wait_for_service(service_name)

    try:
        set_position = rospy.ServiceProxy(service_name, SetKinematicsPose)

        arg = SetKinematicsPoseRequest()
        arg.end_effector_name = 'gripper'
        arg.kinematics_pose.pose.orientation.x = quaternion[0]
        arg.kinematics_pose.pose.orientation.y = quaternion[1]
        arg.kinematics_pose.pose.orientation.z = quaternion[2]
        arg.kinematics_pose.pose.orientation.w = quaternion[3]

        # arg.kinematics_pose.pose.orientation.x = -2.198136348161608e-06
        # arg.kinematics_pose.pose.orientation.y = 0.15374071741104944
        # arg.kinematics_pose.pose.orientation.z = -3.224614963243735e-07
        # arg.kinematics_pose.pose.orientation.w = 0.9881112244099852

        arg.path_time = time
        resp1 = set_position(arg)
        print ('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
        return False

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    r = 0.0
    p = 0.0
    y = 0.0
    time = 2
    print( "Requesting [%s, %s, %s]"%(r, p, y))
    response = set_position_client(r, p, y, time)
    print( "[%s %s %s] returns [%s]"%(r, p, y, response))

