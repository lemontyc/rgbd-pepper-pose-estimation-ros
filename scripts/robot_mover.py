#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import OpenManipulatorState, KinematicsPose
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
import pprint
class Robot:
    def __init__(self, topics, services):
        self.states                 = OpenManipulatorState()
        self.gripper_pose           = KinematicsPose()
        self.pepper_bboxes          = BoundingBoxArray()

        self.sub_states         = rospy.Subscriber(topics[0], OpenManipulatorState, self.states_callback)
        self.sub_gripper_pose   = rospy.Subscriber(topics[1], KinematicsPose, self.gripper_pose_callback)
        self.sub_pepper_bboxes  = rospy.Subscriber(topics[2], BoundingBoxArray, self.pepper_bboxes_callback)

        self.ser_task_space_position        = rospy.wait_for_service(services[0])
        self.ser_task_space_orientation     = rospy.wait_for_service(services[1])
        self.ser_joint_space                = rospy.wait_for_service(services[2])

        # self.timer_reset = rospy.Timer(rospy.Duration(15), self.timer_reset_callback)

        try:
            self.set_task_space_position    = rospy.ServiceProxy(services[0], SetKinematicsPose)
        except rospy.ServiceException as e:
            print ("Service startup failed: ({})".format(e))

        try:
            self.set_task_space_orientation   = rospy.ServiceProxy(services[1], SetKinematicsPose)
        except rospy.ServiceException as e:
            print ("Service startup failed: ({})".format(e))

        try:
            self.set_joint_space_position   = rospy.ServiceProxy(services[2], SetJointPosition)
        except rospy.ServiceException as e:
            print ("Service startup failed: ({})".format(e))

        self.shutdown = rospy.on_shutdown(self.shutdown_callback)




    def states_callback(self, data):
        self.states = data

    def gripper_pose_callback(self, data):
        self.gripper_pose = data

    def pepper_bboxes_callback(self, data):
        self.pepper_bboxes = data
        targets = BoundingBoxArray()
        targets_counter = [0,0] # 0: Fruit, 1: Peduncle

        if self.pepper_bboxes.boxes and "STOPPED" in self.states.open_manipulator_moving_state:
            # rospy.loginfo("{} new boxes received".format(len(self.pepper_bboxes.boxes)))

            for object in self.pepper_bboxes.boxes:
                target = BoundingBox()
                # Search for peduncle. If Pepper.value > 0, pepper.value is the bbox index of its peduncle
                if object.value > 0.0:
                    target = self.pepper_bboxes.boxes[int(object.value)]
                    targets_counter[1] = targets_counter[1] + 1
                    targets.boxes.append(target)
                else:
                    if object.label != 2:
                        # If no peduncle was found, move towards the pepper
                        target = object
                        targets_counter[0] = targets_counter[0] + 1
                        targets.boxes.append(target)
            rospy.loginfo("{} objects, {} targets: {} peduncles and {} fruits".format(len(self.pepper_bboxes.boxes), len(targets.boxes), targets_counter[1], targets_counter[0]))
            # pprint.pprint(targets)

            for count, target in enumerate(targets.boxes):
                # If target is a peduncle, it has an orientation, rotate accordingly
                if target.label == 2:
                    self.rotate_gripper(count, target.pose.orientation)

                self.move_to_coordinates(count, [   round(target.pose.position.x, 3),
                                                    round(target.pose.position.y, 3),
                                                    round(target.pose.position.z, 3)])
                
                self.home_position()

    def move_to_coordinates(self, count, coordinates):
        
        new_coordinates = SetKinematicsPoseRequest()
        new_coordinates.end_effector_name = "gripper"
        new_coordinates.path_time = 2.5
        
        
        new_coordinates.kinematics_pose.pose.position.x = coordinates[0]
        new_coordinates.kinematics_pose.pose.position.y = coordinates[1]
        new_coordinates.kinematics_pose.pose.position.z = coordinates[2]
    
        try:
            rospy.loginfo("    Trying to reach target {} at x: {} y: {} z: {}".format(count,
                new_coordinates.kinematics_pose.pose.position.x,
                new_coordinates.kinematics_pose.pose.position.y,
                new_coordinates.kinematics_pose.pose.position.z
            ))
            # Try to reach coordinates without any path planning
            resp = self.set_task_space_position(new_coordinates)
            # rospy.loginfo(resp)
            if resp.is_planned == True:
                rospy.sleep(2.5)
                rospy.loginfo("        Successfully reached all coordinates\n")
            else:
                rospy.loginfo("        Failed to achieve x,y,z coordinates, trying with x and y")
                # Try to reach Y and x Coordinates
                # Obtain current gripper position and only set x and y
                new_coordinates.kinematics_pose.pose = self.gripper_pose.pose
                new_coordinates.kinematics_pose.pose.position.x = coordinates[0]
                new_coordinates.kinematics_pose.pose.position.y = coordinates[1]
                resp = self.set_task_space_position(new_coordinates)
                # rospy.loginfo(resp)
                if resp.is_planned == True:
                    rospy.sleep(2.5)
                    rospy.loginfo("            Trying to achieve z coordinate")
                    new_coordinates.kinematics_pose.pose = self.gripper_pose.pose
                    new_coordinates.kinematics_pose.pose.position.z = coordinates[2]
                    resp = self.set_task_space_position(new_coordinates)
                    # rospy.loginfo(resp)
                    if resp.is_planned == True:
                        rospy.sleep(2.5)
                        rospy.loginfo("                Successfully reached all coordinates\n")
                    else:
                        rospy.loginfo("                Trying to achieve z with pseudo path planning")
                        
                        divider = 4.0
                        increment =  round(abs(coordinates[2] - self.gripper_pose.pose.position.z )/divider, 3)
                        for i in range(int(divider)):
                            # Get current gripper position
                            new_coordinates.kinematics_pose.pose = self.gripper_pose.pose
                            if coordinates[2] - self.gripper_pose.pose.position.z > 0:
                                new_coordinates.kinematics_pose.pose.position.z = new_coordinates.kinematics_pose.pose.position.z + increment
                            else:
                                new_coordinates.kinematics_pose.pose.position.z = new_coordinates.kinematics_pose.pose.position.z - increment
                            resp = self.set_task_space_position(new_coordinates)
                            rospy.loginfo("                    Trying to reach z {}".format(new_coordinates.kinematics_pose.pose.position.z))
                            if resp.is_planned == True:
                                rospy.sleep(2.5)
                                rospy.loginfo("                        Successfully reached {} point\n".format(i))
                            else:
                                rospy.loginfo("                        Unable to reach {} point {}\n".format(i, new_coordinates.kinematics_pose.pose.position.z))
                        new_coordinates.kinematics_pose.pose = self.gripper_pose.pose
                        new_coordinates.kinematics_pose.pose.position.z = new_coordinates.kinematics_pose.pose.position.z + increment
                        resp = self.set_task_space_position(new_coordinates)

                else: 
                    rospy.loginfo("            Unable to reach x and y coordinates\n")

            # new_coordinates.kinematics_pose.pose.position.z = coordinates[2]
            # resp = self.set_task_space_position(new_coordinates)
            # rospy.loginfo(resp)
            # if resp.is_planned == True:
                
            #     rospy.sleep(2)

        except rospy.ServiceException as e:
            print ("Service call failed: ()".format(e))

    def rotate_gripper(self, count, orientation):
        new_orientation = SetKinematicsPoseRequest()
        new_orientation.end_effector_name = "gripper"
        new_orientation.path_time = 3.0

        new_orientation.kinematics_pose.pose.orientation = orientation

        try:
            rospy.loginfo("    Trying to reach orientation of {} at x: {} y: {} z: {} w: {}".format(count,
               new_orientation.kinematics_pose.pose.orientation.x,
               new_orientation.kinematics_pose.pose.orientation.y,
               new_orientation.kinematics_pose.pose.orientation.z,
               new_orientation.kinematics_pose.pose.orientation.w,
            ))
            # Try to reach coordinates without any path planning
            resp = self.set_task_space_orientation(new_orientation)
            # rospy.loginfo(resp)
            if resp.is_planned == True:
                rospy.sleep(2.5)
                rospy.loginfo("        Successfully reached orientation\n")
            else:
                rospy.loginfo("        Unable to reach orientation\n")
        except rospy.ServiceException as e:
            print ("Service call failed: ()".format(e))

    def home_position(self):
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.0, -1.052, 0.377, 0.703]
        try:
            arg = SetJointPositionRequest()
            arg.joint_position.joint_name   = joint_name
            arg.joint_position.position     = joint_angle
            arg.path_time                   = 2.5
            resp = self.set_joint_space_position(arg)
            rospy.sleep(2.5)
            rospy.loginfo("        Robot succesfully sent to home position\n\n")
        except rospy.ServiceException as e:
            print ("Service call failed: ({})".format(e))

    def shutdown_callback(self):
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.0, 0.127, 0.29, 1.144]

        try:
            arg = SetJointPositionRequest()
            arg.joint_position.joint_name   = joint_name
            arg.joint_position.position     = joint_angle
            arg.path_time                   = 2.5
            resp = self.set_joint_space_position(arg)
            rospy.loginfo("Robot succesfully sent to shutdown position")
        except rospy.ServiceException as e:
            print ("Service call failed: ({})".format(e))

def main():
    states_topic                = '/states'
    gripper_pose_topic          = '/gripper/kinematics_pose'
    pepper_bboxes               = '/peppers/bbox_3D'
    topics                      = [states_topic, gripper_pose_topic, pepper_bboxes]

    robot_task_space_position_service       =   '/goal_task_space_path_position_only'
    robot_task_space_orientation_service    =   '/goal_task_space_path_orientation_only'
    robot_joint_space_service               =   '/goal_joint_space_path'
    services                                = [robot_task_space_position_service, robot_task_space_orientation_service, robot_joint_space_service]

    node = Robot(topics, services)

    rospy.spin()


if __name__ == '__main__':
    node_name = 'robot_mover'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")