#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import OpenManipulatorState, KinematicsPose
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

class Robot:
    def __init__(self, topics, robot_service):
        self.states                 = OpenManipulatorState()
        self.gripper_pose           = KinematicsPose()
        self.pepper_bboxes          = BoundingBoxArray()
        self.saved_pepper_bbox      = BoundingBox()
        self.saved_peduncle_bbox    = None
        self.harvest = 0
        self.last_response = False
        self.first_run  = False

        self.sub_states         = rospy.Subscriber(topics[0], OpenManipulatorState, self.states_callback)
        self.sub_gripper_pose   = rospy.Subscriber(topics[1], KinematicsPose, self.gripper_pose_callback)
        self.sub_pepper_bboxes  = rospy.Subscriber(topics[2], BoundingBoxArray, self.pepper_bboxes_callback)

        self.ser_robot  = rospy.wait_for_service(robot_service)

        self.timer_reset = rospy.Timer(rospy.Duration(15), self.timer_reset_callback)
        try:
            self.set_position = rospy.ServiceProxy(robot_service, SetKinematicsPose)
        except rospy.ServiceException as e:
            print ("Service call failed: ()".format(e))

    def states_callback(self, data):
        self.states = data
        
        # If manipulator has stopped moving, try to harvest
        if "ACTUATOR_ENABLED" in self.states.open_manipulator_actuator_state:
            if "STOPPED" in self.states.open_manipulator_moving_state:
                # If there are still peppers
                if len(self.pepper_bboxes.boxes):
                    self.saved_pepper_bbox = self.pepper_bboxes.boxes[self.harvest]

                    self.saved_peduncle_bbox = BoundingBox()
                    # Check for peduncle associated to pepper
                    # The bbox.value corresponds to the bbox index of a peduncle
                    if self.saved_pepper_bbox.value > 0:
                        self.saved_peduncle_bbox = self.pepper_bboxes.boxes[int(self.saved_pepper_bbox.value)]
                    # Touch the pepper
                    else:
                        self.saved_peduncle_bbox = self.saved_pepper_bbox
                    # print(" {} {}".format(self.saved_peduncle_bbox.pose.position.x, self.gripper_pose.pose.position.x))
                    # print(abs(self.saved_peduncle_bbox.pose.position.x - self.gripper_pose.pose.position.x))
                    if not self.first_run:
                        self.first_run  = True
                        try:
                            new_coordinates = SetKinematicsPoseRequest()
                            new_coordinates.end_effector_name = "gripper"
                            new_coordinates.path_time = 3.0

                            new_coordinates.kinematics_pose.pose.position = self.saved_peduncle_bbox.pose.position

                            self.last_response = self.set_position(new_coordinates)
                            print(" {} {}".format(new_coordinates.kinematics_pose.pose.position , self.last_response))
                        # if self.last_response.is_planned:
                        #     # rospy.signal_shutdown("Coordinate found")
                        except rospy.ServiceException as e:
                            print ("Service call failed: ()".format(e))

                    if not self.last_response.is_planned:
                        try:
                            new_coordinates = SetKinematicsPoseRequest()
                            new_coordinates.end_effector_name = "gripper"
                            new_coordinates.path_time = 2.0

                            new_coordinates.kinematics_pose.pose.position = self.saved_peduncle_bbox.pose.position

                            self.last_response = self.set_position(new_coordinates)
                            print(" {} {}".format(new_coordinates.kinematics_pose.pose.position , self.last_response))
                        # if self.last_response.is_planned:
                        #     # rospy.signal_shutdown("Coordinate found")
                        except rospy.ServiceException as e:
                            print ("Service call failed: ()".format(e))


    def gripper_pose_callback(self, data):
        self.gripper_pose = data
    
    def pepper_bboxes_callback(self, data):
        self.pepper_bboxes = data

    def timer_reset_callback(self, event):
        self.last_response.is_planned = False

    def timer_reset_callback(self, event):
        self.last_response.is_planned = False

def main():
    states_topic        = '/states'
    gripper_pose_topic  = '/gripper/kinematics_pose'
    pepper_bboxes       = '/peppers/bbox_3D'
    topics = [states_topic, gripper_pose_topic, pepper_bboxes]

    robot_service       =   '/goal_task_space_path_position_only'

    node = Robot(topics, robot_service)

    rospy.spin()


if __name__ == '__main__':
    node_name = 'robot_mover'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")