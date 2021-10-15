#!/usr/bin/env python3

import sys
import rospy
import math
import geometry_msgs
from open_manipulator_msgs.srv import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def set_home_position():
    service_name = '/goal_task_space_path_orientation_only'

    rospy.wait_for_service(service_name)

    try:
        set_position = rospy.ServiceProxy(service_name, SetKinematicsPose)

        input_angle = -108
        input_angle = input_angle * -1
        input_angle = input_angle + 90
        
        # X, Y, Z, W
        e = euler_from_quaternion([0.000028, -0.15, -0.0015, 0.99] )
        print(e)
        q = quaternion_from_euler(e[0], e[1], e[2])
        q = quaternion_from_euler(e[0], math.radians(input_angle), e[2])

        print("Q x {} y {} z {} w {}".format(q[0], q[1], q[2], q[3]))

        arg = SetKinematicsPoseRequest()
        arg.end_effector_name = 'gripper'
        
        arg.kinematics_pose.pose.orientation.x = q[0]
        arg.kinematics_pose.pose.orientation.y = q[1]
        arg.kinematics_pose.pose.orientation.z = q[2]
        arg.kinematics_pose.pose.orientation.w = q[3]
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