#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError




class ImageExtractor:
    def __init__(self, rgb_image_topic, depth_image_topic, depth_info_topic, 
                        rgb_image_pub, depth_image_pub, depth_info_pub):
        self.color_image = msg_Image()
        self.depth_image = msg_Image()
        self.depth_info  = CameraInfo()

        self.sub_color_image    = rospy.Subscriber(rgb_image_topic, msg_Image, self.save_color_image_callback)
        self.sub_depth_image    = rospy.Subscriber(depth_image_topic, msg_Image, self.save_depth_image_callback)
        self.sub_depth_info     = rospy.Subscriber(depth_info_topic, CameraInfo, self.save_depth_info_callback)

        self.pub_color_image    = rospy.Publisher(rgb_image_pub, msg_Image, queue_size=10)
        self.pub_depth_image    = rospy.Publisher(depth_image_pub, msg_Image, queue_size=10)
        self.pub_depth_info     = rospy.Publisher(depth_info_pub, CameraInfo, queue_size=10)


        self.timer  = rospy.Timer(rospy.Duration(5), self.timer_callback)

    def save_color_image_callback(self, data):
        self.color_image = data
    
    def save_depth_image_callback(self, data):
        self.depth_image = data

    def save_depth_info_callback(self, data):
        self.depth_info  = data

    def timer_callback(self, event):
        self.pub_color_image.publish(self.color_image)
        self.pub_depth_image.publish(self.depth_image)
        self.pub_depth_info.publish(self.depth_info)

def main():
    rgb_image_topic     = '/camera/color/image_raw'
    depth_image_topic   = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic    = '/camera/aligned_depth_to_color/camera_info'

    rgb_image_pub       = 'rgb_image'
    depth_image_pub     = 'depth_image'
    depth_info_pub      = 'depth_camera_info'

    node = ImageExtractor(rgb_image_topic, depth_image_topic, depth_info_topic, 
                            rgb_image_pub, depth_image_pub, depth_info_pub)
    rospy.spin()

if __name__ == '__main__':
    node_name = 'image_extractor'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")