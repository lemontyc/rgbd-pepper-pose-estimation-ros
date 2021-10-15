import tf2_ros
import rospy
from cv_bridge import CvBridgeError
import cv2
import math
from tf2_geometry_msgs import PointStamped
from tf.transformations import quaternion_from_euler
from jsk_recognition_msgs.msg import BoundingBox
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2



class Realsense:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.intrinsics = None
        

    def set_depth_info(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def tf_optical_frame_to_link1(self, coordinates):
        temp_point = PointStamped()
        converted_point = [0,0,0]
        try:
            temp_point.header.frame_id = 'camera_color_optical_frame'
            temp_point.header.stamp = rospy.Time.now()
            temp_point.point.x = coordinates[0]
            temp_point.point.y = coordinates[1]
            temp_point.point.z = coordinates[2]
            
            converted_point = self.tfBuffer.transform(temp_point, 'link1', rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            raise

        return [converted_point.point.x, converted_point.point.y, converted_point.point.z]

    def convert_to_meters(self, coordinates):
        return [coordinates[0]/1000.0, coordinates[1]/1000.0, coordinates[2]/1000.0]

    def obtain_coordinates(self, peppers, depth_image_np, bboxes):
            try:
                if self.intrinsics:
                    # Get pepper depth coordinates at pepper bbox center
                    for pepper, pepper_data in peppers.final_pepper_list["peppers"].items():
                        bbox = BoundingBox()
                        pepper_2d_data  = pepper_data["2d_info"]["fruit"]

                        xmin_depth = int((pepper_2d_data["x_min"] * peppers.expected))
                        ymin_depth = int((pepper_2d_data["y_min"] * peppers.expected))
                        xmax_depth = int((pepper_2d_data["x_max"] * peppers.expected))
                        ymax_depth = int((pepper_2d_data["y_max"] * peppers.expected))

                        # Get depth values at the bbox location
                        bbox_area_depth = depth_image_np[ymin_depth:ymax_depth,  xmin_depth:xmax_depth]
                        # Obtain average depth value, this will be used to obtain the centroid coordinates
                        bbox_centroid_pixel_depth_value,_,_,_ = cv2.mean(bbox_area_depth)

                        # Get the centroid of the bbox
                        pepper_center      = [pepper_data["2d_info"]["fruit"]["center"]["x"], pepper_data["2d_info"]["fruit"]["center"]["y"]]
                        # Get x,y,z coordinates in mm.
                        coordinates = rs2.rs2_deproject_pixel_to_point(self.intrinsics, pepper_center, bbox_centroid_pixel_depth_value)

                        coordinates = self.convert_to_meters(coordinates)
                        # Convert from camera frame to robot frame
                        coordinates = self.tf_optical_frame_to_link1(coordinates)
                        
                        bbox.header.frame_id = "link1"
                        bbox.pose.position.x = coordinates[0] 
                        bbox.pose.position.y = coordinates[1] + 0.04
                        bbox.pose.position.z = coordinates[2]
                        bbox.pose.orientation.w = 1
                        
                        bbox.dimensions.z = (xmax_depth - xmin_depth)/2000.0 
                        bbox.dimensions.x = (ymax_depth - ymin_depth)/2000.0
                        bbox.dimensions.y = (bbox.dimensions.x + bbox.dimensions.z)/2.0
                        bbox.label = 1 # 1 for fruit, 2 for peduncle

                        # If peduncle exist, get depth coordinates at peduncle bbox center
                        if "peduncle" in pepper_data["2d_info"]:
                            # Mark pepper value with the bbox index of its peduncle
                            bbox.value = int(len(bboxes.boxes)) + 1
                            bboxes.boxes.append(bbox)
                            # Reset bbox for peduncle
                            bbox = BoundingBox()
                            peduncle_2d_data  = pepper_data["2d_info"]["peduncle"]
                            
                            # Get angle if exists
                            pepper_angle = pepper_data["2d_info"]["angle"]
                            pepper_angle = pepper_angle * -1
                            pepper_angle = pepper_angle + 90
                            q = quaternion_from_euler(0, math.radians(pepper_angle), 0)
                            bbox.pose.orientation.x = q[0]
                            bbox.pose.orientation.y = q[1]
                            bbox.pose.orientation.z = q[2]
                            bbox.pose.orientation.w = q[3]
                            
                            # print(bbox.pose.orientation)

                            xmin_depth = int((peduncle_2d_data["x_min"] * peppers.expected))
                            ymin_depth = int((peduncle_2d_data["y_min"] * peppers.expected))
                            xmax_depth = int((peduncle_2d_data["x_max"] * peppers.expected))
                            ymax_depth = int((peduncle_2d_data["y_max"] * peppers.expected))

                            # Get depth values at the bbox location
                            bbox_area_depth = depth_image_np[ymin_depth:ymax_depth,  xmin_depth:xmax_depth]
                            # Obtain average depth value, this will be used to obtain the centroid coordinates
                            bbox_centroid_pixel_depth_value,_,_,_ = cv2.mean(bbox_area_depth)

                            # Get the centroid of the bbox
                            pepper_center      = [pepper_data["2d_info"]["peduncle"]["center"]["x"], pepper_data["2d_info"]["peduncle"]["center"]["y"]]
                            # Get x,y,z coordinates in mm.
                            coordinates = rs2.rs2_deproject_pixel_to_point(self.intrinsics, pepper_center, bbox_centroid_pixel_depth_value)

                            coordinates = self.convert_to_meters(coordinates)
                            # Convert from camera frame to robot frame
                            coordinates = self.tf_optical_frame_to_link1(coordinates)
                            
                            bbox.header.frame_id = "link1"
                            bbox.pose.position.x = coordinates[0]
                            bbox.pose.position.y = bboxes.boxes[-1].pose.position.y 
                            bbox.pose.position.z = coordinates[2]

                            bbox.dimensions.z = (ymax_depth - ymin_depth)/2000.0
                            bbox.dimensions.x = (xmax_depth - xmin_depth)/2000.0 
                            bbox.dimensions.y = (bbox.dimensions.x + bbox.dimensions.z)/2.0
                            bbox.label = 2 # 1 for fruit, 2 for peduncle

                        bboxes.boxes.append(bbox)
                        
                        

                        


            except ValueError as e:
                return
