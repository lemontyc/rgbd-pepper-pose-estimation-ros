import cv2
import numpy as np
import time
import math


def nothing(x):
    pass

class Windows():
    def __init__(self, expected):
       self.expected = expected

    def draw_all_objects_bbox(self, saved_color_image, objects, pepper_color, peduncle_color, size):
        for key, value in objects.items():
            
            for bbox_number, bbox in value.items():
                info_2d = bbox['2d_info']
                if key == 'peppers':
                    # This validation is necessary as final_pepper_list has different structure
                    # than comple_pepper_list
                    if 'fruit' in bbox['2d_info']:
                        info_2d = bbox['2d_info']['fruit']
                    cv2.rectangle(saved_color_image,    (int(info_2d["x_min"] * self.expected), int(info_2d["y_min"] * self.expected)), 
                                                        (int(info_2d["x_max"] * self.expected), int(info_2d["y_max"] * self.expected)), 
                                                        pepper_color, size)
                    if 'peduncle' in bbox['2d_info']:
                        info_2d = bbox['2d_info']['peduncle']
                        cv2.rectangle(saved_color_image,    (int(info_2d["x_min"] * self.expected), int(info_2d["y_min"] * self.expected)), 
                                                            (int(info_2d["x_max"] * self.expected), int(info_2d["y_max"] * self.expected)), 
                                                            peduncle_color, size)
                if key == 'peduncles':
                    cv2.rectangle(saved_color_image,    (int(info_2d["x_min"] * self.expected), int(info_2d["y_min"] * self.expected)), 
                                                        (int(info_2d["x_max"] * self.expected), int(info_2d["y_max"] * self.expected)), 
                                                        peduncle_color, size)

    def draw_angles(self, saved_color_image, final_pepper_list):
        for pepper, pepper_data in final_pepper_list["peppers"].items():
            if "angle" in pepper_data["2d_info"]:
                angle = pepper_data["2d_info"]["angle"]
                
                pepper_center      = (pepper_data["2d_info"]["fruit"]["center"]["x"], pepper_data["2d_info"]["fruit"]["center"]["y"])
                peduncle_center    = (pepper_data["2d_info"]["peduncle"]["center"]["x"], pepper_data["2d_info"]["peduncle"]["center"]["y"])
                
                length = math.sqrt( ((pepper_center[0] - peduncle_center[0]) ** 2 )+((pepper_center[1] - peduncle_center[1]) ** 2) )

                line_end = (int(pepper_center[0] + length * math.cos(angle * math.pi / 180)), int(pepper_center[1] + length * math.sin(angle * math.pi / 180)))

                cv2.line(saved_color_image, pepper_center, line_end, (235, 213, 52), 4)
    