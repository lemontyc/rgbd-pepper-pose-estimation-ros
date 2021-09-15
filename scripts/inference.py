#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as msg_Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from modules.peppers import Peppers
from modules.utils import *

class ImageInference:
    def __init__(self, m_rcnn_path, m_rcnn_json_path, topics):
        self.m_rcnn_path        = m_rcnn_path
        self.m_rcnn_json_path   = m_rcnn_json_path
        
        self.color_image        = []
        
        self.bridge = CvBridge()

        self.sub_color_image    = rospy.Subscriber(topics[0], msg_Image, self.save_color_image_callback)

        self.first_run()
        self.m_rcnn_initialized = False

    def first_run(self):
        rospy.loginfo("First Run")
        delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
        delete_all(self.m_rcnn_path, 'input')

    def save_color_image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        self.color_image = np.asanyarray(cv_image)
        # Save test image as a file
        save_image(self.color_image, str(data.header.seq), self.m_rcnn_path, 'input')



def main():
    M_RCNN_PATH = '/home/luis/GitHub/rgbd-pepper-pose-estimation/Mask_RCNN/src/Mask_RCNN/datasets/process'
    M_RCNN_JSON_PATH = 'boxes'

    rgb_image_topic     = 'rgb_image'
    depth_image_topic   = 'depth_image'
    depth_info_topic    = 'depth_camera_info'
    topics = [rgb_image_topic, depth_image_topic, depth_info_topic]

    node = ImageInference(M_RCNN_PATH, M_RCNN_JSON_PATH, topics)
    rospy.spin()

if __name__ == '__main__':
    node_name = 'image_inference'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")