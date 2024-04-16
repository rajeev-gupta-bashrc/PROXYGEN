import rospy
from proxygen_msgs.srv import *


def main():
    rospy.init_node('aayen')

    service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

    service_request = SetJointPositionRequest()
    service_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4", "gripper"]
    service_request.joint_position.position = [0.0, 0.0, 0.0, 1.57, 0.0]
    service_request.path_time = 2.0

    service_response = service_client(service_request)

    if service_response.is_planned:
        print('Successfully set joint positions.')
    else:
        print('Failed to set joint positions.')


if __name__ == '__main__':
    main()
