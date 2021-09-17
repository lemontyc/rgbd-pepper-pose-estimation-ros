#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from jsk_recognition_msgs.msg import BoundingBoxArray
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from modules.peppers import Peppers
from modules.gui import Windows
from modules.realsense import Realsense
from modules.utils import *



class ImageInference:
    def __init__(self, m_rcnn_path, m_rcnn_json_path, topics, publishers):
        self.m_rcnn_path        = m_rcnn_path
        self.m_rcnn_json_path   = m_rcnn_json_path
        
        # color_image_np: Used to draw 2D bboxes with openCV
        # color_image_frame_number: Allows to sync pepper detection with the correct frame
        self.color_image_np        = []
        self.depth_image_np         = []
        self.color_image_frame_number = 0

        
        
        # Used to convert between ROS Image msg to Numpy arrays
        self.bridge = CvBridge()

        # peppers: Class with multiple functions to parse JSON information
        # gui: Class with functions to draw 2D bboxes
        self.peppers    = Peppers(self.m_rcnn_path, self.m_rcnn_json_path)
        self.gui        = Windows(self.peppers.expected)
        self.camera     = Realsense()

        # Subscribes to topic that extracts images
        self.sub_color_image        = rospy.Subscriber(topics[0], msg_Image, self.save_color_image_callback)
        self.sub_depth_image        = rospy.Subscriber(topics[1], msg_Image, self.depth_image_callback)
        self.sub_depth_camera_info  = rospy.Subscriber(topics[2], CameraInfo, self.depth_camera_info_callback)

        # Will publish color image with bboxes and angle here
        self.pub_rgb_image      = rospy.Publisher(publishers[0], msg_Image, queue_size=10)
        # will publish final list of bboxes here
        self.pub_bbox_3d        = rospy.Publisher(publishers[1], BoundingBoxArray, queue_size=10)

        # Clean up the Mask R-CNN file system 
        self.first_run()
        self.m_rcnn_initialized = False
        # This callback will constantly try to read jsons if mask R-CNN is initialized
        self.timer_json = rospy.Timer(rospy.Duration(0.1), self.timer_json_callback)

    def first_run(self):
        rospy.loginfo("First Run")
        delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
        delete_all(self.m_rcnn_path, 'input')
        rospy.loginfo("Waiting for Mask-RCNN Initialization")

    def save_color_image_callback(self, data):
        # Get image from image_extractor node, save id:
        self.color_image_frame_number = data.header.seq
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        self.color_image_np = np.asanyarray(cv_image)
        # Save test image as a file
        save_image(self.color_image_np, str(data.header.seq), self.m_rcnn_path, 'input')

        if not self.m_rcnn_initialized:
            # Try to read bbox JSON
            self.peppers.read_JSON()
            # If JSON was succesfully read, it means Mask R-CNN is working
            if self.peppers.json_file:
                delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
                self.peppers.json_file = []
                self.m_rcnn_initialized = True
                rospy.loginfo("Mask-RCNN Initialized")
                

    def depth_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.depth_image_np = np.asanyarray(cv_image)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def depth_camera_info_callback(self, cameraInfo):
        self.camera.set_depth_info(cameraInfo)

    def timer_json_callback(self, event):
        bboxes = BoundingBoxArray()
        if self.m_rcnn_initialized:
            # Try to read bbox JSON
            self.peppers.read_JSON()
            # If JSON was succesfully read, continue
            if self.peppers.json_file:
                # Only parse data if read file matches saved color image
                # Prevents desynch 
                #TODO: There might be a bug here
                if self.peppers.read_json_data(self.color_image_frame_number):
                    # Delete json file
                    rospy.loginfo("Read {}.json".format(self.color_image_frame_number))
                    delete_all(self.m_rcnn_path, self.m_rcnn_json_path)
                    self.peppers.parse_json_data()
                    # Draw all detected objects here
                    self.gui.draw_all_objects_bbox(self.color_image_np, self.peppers.complete_pepper_list, (199, 240, 218), (129, 176, 247), 2)
                    self.peppers.filter_peppers(100)
                    self.peppers.find_peduncles()
                    # Redraw only the peppers that passed the filter
                    self.gui.draw_all_objects_bbox(self.color_image_np, self.peppers.final_pepper_list, (57, 219, 98), (10, 88, 204),4)
                    # Compute angle of peppers that passed the filter if peduncles are found
                    self.peppers.compute_angle()
                    self.gui.draw_angles(self.color_image_np, self.peppers.final_pepper_list)
                    # Convert openCV image to ROS Image and publish
                    imgMsg = self.bridge.cv2_to_imgmsg(self.color_image_np, "rgb8")
                    self.pub_rgb_image.publish(imgMsg)
                    # Obtain x, y, and z coordinates in "link1" frame_id (base of robot arm)
                    # Fills bboxes which is a 3D bbox array for visualization in RVIZ
                    self.camera.obtain_coordinates(self.peppers, self.depth_image_np, bboxes)

                    bboxes.header.frame_id = "link1"
                    
                    self.pub_bbox_3d.publish(bboxes)


def main():
    M_RCNN_PATH = '/home/luis/GitHub/rgbd-pepper-pose-estimation/Mask_RCNN/src/Mask_RCNN/datasets/process'
    M_RCNN_JSON_PATH = 'boxes'

    rgb_image_topic         = 'rgb_image'
    depth_image_topic       = 'depth_image'
    depth_camera_info_topic = 'depth_camera_info'
    topics = [rgb_image_topic, depth_image_topic, depth_camera_info_topic]

    rgb_image_pub       = 'rgb_image_infered'
    bbox_3d_pub         = 'bbox_3D'
    publishers = [rgb_image_pub, bbox_3d_pub]
    

    node = ImageInference(M_RCNN_PATH, M_RCNN_JSON_PATH, topics, publishers)
    rospy.spin()

if __name__ == '__main__':
    node_name = 'image_inference'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")