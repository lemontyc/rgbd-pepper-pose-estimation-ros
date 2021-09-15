#!/usr/bin/env python3

import sys
import rospy
from open_manipulator_msgs.srv import *

def set_position_client(x, y, z, time):
    service_name = '/goal_joint_space_path'

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
        print('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: ()".format(e))
        return False

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 5:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        time = float(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)
    print ("Requesting [{}, {}, {}]".format(x, y, z))
    response = set_position_client(x, y, z, time)
    print("[{} {} {}] returns {}".format(x, y, z, response))