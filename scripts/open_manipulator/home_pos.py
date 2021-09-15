#!/usr/bin/env python3

import sys
import rospy
from open_manipulator_msgs.srv import *

def set_home_position():
    service_name = '/goal_joint_space_path'

    rospy.wait_for_service(service_name)

    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)

        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.0, -1.51, 0.0, 1.53]
        
        arg = SetJointPositionRequest()
        arg.joint_position.joint_name = joint_name
        arg.joint_position.position = joint_angle
        arg.path_time = 2.0
        resp1 = set_position(arg)
        print('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: ()".format(e))
        return False


if __name__ == "__main__":

    response = set_home_position()

    print("Home returns {}".format( response))