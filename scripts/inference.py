#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as msg_Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from modules.peppers import Peppers
from modules.gui import Windows
from modules.utils import *


class ImageInference:
    def __init__(self, m_rcnn_path, m_rcnn_json_path, topics, rgb_image_pub):
        self.m_rcnn_path        = m_rcnn_path
        self.m_rcnn_json_path   = m_rcnn_json_path
        
        self.color_image_np        = []
        self.color_image           = msg_Image()
        

        self.bridge = CvBridge()

        self.peppers = Peppers(self.m_rcnn_path, self.m_rcnn_json_path)
        self.gui = Windows(self.peppers.expected)

        self.sub_color_image    = rospy.Subscriber(topics[0], msg_Image, self.save_color_image_callback)
        
        self.pub_rgb_image      = rospy.Publisher(rgb_image_pub, msg_Image, queue_size=10)

        self.first_run()
        self.m_rcnn_initialized = False
        self.timer_json = rospy.Timer(rospy.Duration(0.1), self.timer_json_callback)

    def first_run(self):
        rospy.loginfo("First Run")
        delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
        delete_all(self.m_rcnn_path, 'input')
        rospy.loginfo("Waiting for Mask-RCNN Initialization")

    def save_color_image_callback(self, data):
        # Save image in ROS format
        self.color_image = data
        # Produce image files:
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        self.color_image_frame_number = data.header.seq
        self.color_image_np = np.asanyarray(cv_image)
        # Save test image as a file
        save_image(self.color_image_np, str(data.header.seq), self.m_rcnn_path, 'input')

        if not self.m_rcnn_initialized:
            # Try to read bbox JSON
            self.peppers.read_JSON()
            # If JSON was succesfully read, set m_rcnn_initialized to True
            if self.peppers.json_file:
                delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
                self.peppers.json_file = []
                self.m_rcnn_initialized = True
                
                rospy.loginfo("Mask-RCNN Initialized")
                

    def timer_json_callback(self, event):
        if self.m_rcnn_initialized:
            # Try to read bbox JSON
            self.peppers.read_JSON()
            # If JSON was succesfully read, continue
            if self.peppers.json_file:
                # Only parse data if read file matches saved color image
                # Prevents desynch
                if self.peppers.read_json_data(self.color_image.header.seq):
                    # Delete json file
                    rospy.loginfo("Read {}.json".format(self.color_image.header.seq))
                    delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
                    self.peppers.parse_json_data()
                    self.gui.draw_all_objects_bbox(self.color_image_np, self.peppers.complete_pepper_list, (199, 240, 218), (129, 176, 247), 2)
                    self.peppers.filter_peppers(100)
                    self.peppers.find_peduncles()
                    self.gui.draw_all_objects_bbox(self.color_image_np, self.peppers.final_pepper_list, (57, 219, 98), (10, 88, 204),4)
                    self.peppers.compute_angle()
                    self.gui.draw_angles(self.color_image_np, self.peppers.final_pepper_list)

                    imgMsg = self.bridge.cv2_to_imgmsg(self.color_image_np, "rgb8")
                    self.pub_rgb_image.publish(imgMsg)


def main():
    M_RCNN_PATH = '/home/luis/GitHub/rgbd-pepper-pose-estimation/Mask_RCNN/src/Mask_RCNN/datasets/process'
    M_RCNN_JSON_PATH = 'boxes'

    rgb_image_topic     = 'rgb_image'
    depth_image_topic   = 'depth_image'
    depth_info_topic    = 'depth_camera_info'
    topics = [rgb_image_topic, depth_image_topic, depth_info_topic]

    rgb_image_pub = 'rgb_image_infered'

    node = ImageInference(M_RCNN_PATH, M_RCNN_JSON_PATH, topics, rgb_image_pub)
    rospy.spin()

if __name__ == '__main__':
    node_name = 'image_inference'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")