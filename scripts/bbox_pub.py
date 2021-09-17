#!/usr/bin/env python3

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
from tf2_geometry_msgs import PointStamped
import tf2_ros


class bbox:
    def __init__(self, bbox_pub):
        self.pub_bbox_pub   = rospy.Publisher(bbox_pub, BoundingBoxArray, queue_size=10)
        self.timer_pub      = rospy.Timer(rospy.Duration(0.1), self.timer_pub)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def timer_pub(self, event):
        pepper_bboxes = BoundingBoxArray()
        pepper = BoundingBox()
        pepper.header.frame_id = "camera_color_optical_frame"
        pepper.pose.position.x = -0.005565765857696533
        pepper.pose.position.y = 0.11262673187255859
        pepper.pose.position.z = 0.3868384704589844
        pepper.pose.orientation.w = 1
        pepper.label = 1
        pepper.dimensions.x = 0.07
        pepper.dimensions.y = 0.07
        pepper.dimensions.z = 0.07

        temp_point = PointStamped()
        pepper_converted = BoundingBox()
        temp_point.header.frame_id = 'camera_color_optical_frame'
        temp_point.header.stamp = rospy.Time.now()
        temp_point.point.x = -0.005565765857696533
        temp_point.point.y = 0.11262673187255859
        temp_point.point.z = 0.3868384704589844
        converted_point = self.tfBuffer.transform(temp_point, 'link1', rospy.Duration(1.0))

        pepper_converted.header.frame_id = "link1"
        pepper_converted.pose.position.x = converted_point.point.x
        pepper_converted.pose.position.y = converted_point.point.y
        pepper_converted.pose.position.z = converted_point.point.z
        pepper_converted.pose.orientation.w = 1
        pepper_converted.label = 2
        pepper_converted.dimensions.x = 0.03
        pepper_converted.dimensions.y = 0.03
        pepper_converted.dimensions.z = 0.03

        
        pepper_bboxes.header.frame_id = "camera_color_optical_frame"

        pepper_bboxes.boxes.append(pepper)
        pepper_bboxes.boxes.append(pepper_converted)
        print(len(pepper_bboxes.boxes))

        self.pub_bbox_pub.publish(pepper_bboxes)





def main():
    bbox_pub = 'bbox_3d'
    node = bbox(bbox_pub)
    rospy.spin()

if __name__ == '__main__':
    node_name = 'bbox_tester'
    rospy.init_node(node_name, anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Exception thrown")