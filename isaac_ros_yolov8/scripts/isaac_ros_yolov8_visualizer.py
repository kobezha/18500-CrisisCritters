#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# This script listens for images and object detections on the image,
# then renders the output boxes on top of the image and publishes
# the result as an image message

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.executors import ExternalShutdownException
import random
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import pyrealsense2
from std_msgs.msg import String

class States(Enum):
    SEARCH = "Search"
    FOUND = "Found"
    INVESTIGATE = "Investigate"

visual_flag = True 
verbose = False 

names = {
        0: 'person',
        1: 'bicycle',
        2: 'car',
        3: 'motorcycle',
        4: 'airplane',
        5: 'bus',
        6: 'train',
        7: 'truck',
        8: 'boat',
        9: 'traffic light',
        10: 'fire hydrant',
        11: 'stop sign',
        12: 'parking meter',
        13: 'bench',
        14: 'bird',
        15: 'cat',
        16: 'dog',
        17: 'horse',
        18: 'sheep',
        19: 'cow',
        20: 'elephant',
        21: 'bear',
        22: 'zebra',
        23: 'giraffe',
        24: 'backpack',
        25: 'umbrella',
        26: 'handbag',
        27: 'tie',
        28: 'suitcase',
        29: 'frisbee',
        30: 'skis',
        31: 'snowboard',
        32: 'sports ball',
        33: 'kite',
        34: 'baseball bat',
        35: 'baseball glove',
        36: 'skateboard',
        37: 'surfboard',
        38: 'tennis racket',
        39: 'bottle',
        40: 'wine glass',
        41: 'cup',
        42: 'fork',
        43: 'knife',
        44: 'spoon',
        45: 'bowl',
        46: 'banana',
        47: 'apple',
        48: 'sandwich',
        49: 'orange',
        50: 'broccoli',
        51: 'carrot',
        52: 'hot dog',
        53: 'pizza',
        54: 'donut',
        55: 'cake',
        56: 'chair',
        57: 'couch',
        58: 'potted plant',
        59: 'bed',
        60: 'dining table',
        61: 'toilet',
        62: 'tv',
        63: 'laptop',
        64: 'mouse',
        65: 'remote',
        66: 'keyboard',
        67: 'cell phone',
        68: 'microwave',
        69: 'oven',
        70: 'toaster',
        71: 'sink',
        72: 'refrigerator',
        73: 'book',
        74: 'clock',
        75: 'vase',
        76: 'scissors',
        77: 'teddy bear',
        78: 'hair drier',
        79: 'toothbrush',
}

class Yolov8Visualizer(Node):
    QUEUE_SIZE = 10
    color = (0, 255, 0)
    bbox_thickness = 2
    

    def __init__(self):
        super().__init__('yolov8_visualizer',namespace = "")

        self.current_state = States.SEARCH
        
        #current target location for locking in
        self.current_target_x = -1
        self.current_target_y = -1
        self.target_delta_threshold = 50

        #img size for depth images 
        self.img_width = 640
        self.img_height = 480

        #cvbridge for converting from cv images to ros images 
        self._bridge = cv_bridge.CvBridge()

        #various pubs and subs 
        self._processed_image_pub = self.create_publisher(
            Image, 'yolov8_processed_image',  self.QUEUE_SIZE)

        self._detections_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            'detections_output')
        self._image_subscription = message_filters.Subscriber(
            self,
            Image,
            'image')

        self._depth_subscription = message_filters.Subscriber(
            self,
            Image,
            'camera/depth/image_rect_raw')
        
        self._hexapod_commands_pub = self.create_publisher(
            String, 'hexapod_commands', self.QUEUE_SIZE)
        
        self._pose_subscription = message_filters.Subscriber(
            self,
            PoseStamped,
            'visual_slam/tracking/vo_pose')
        

    
        #synchronizer syncs the detections and color/depth images based on timestamp 
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self._detections_subscription, self._image_subscription,self._depth_subscription],self.QUEUE_SIZE,0.5,allow_headerless=True)

        self.time_synchronizer.registerCallback(self.main_callback)
        
        #self.time_synchronizer = message_filters.TimeSynchronizer([self._depth_subscription,self._detections_subscription],self.QUEUE_SIZE)
        #self.time_synchronizer.registerCallback(self.dummy_callback)

    def dummy_callback(self,depth_msg,detect_msg):
        self.get_logger().info(f'Dummmy Callback')
    
    def send_heartbeat(self):
        if verbose: self.get_logger().info(f'Sending heartbeats to hexapods')

    def adjust_heading(self,center_x,center_y,depth_val):
        command = String()
        # Rotate such that person is in center of screen
        if center_x < (640 * 1 / 4):  # Person is in left quarter of screen
            command.data = "turn_left"
        elif center_x > (640 * 3 / 4):  # Person is in right quarter of screen
            command.data = "turn_right"
        else:
            # Keep walking towards person until it is within 300
            if depth_val > 500:
                command.data = "move_forward"
            else:
                command.data = "stop_moving"
        return command 

    def found_person(self):
        if verbose: self.get_logger().info(f'PERSON LOCATED!')
        #send found message to other hexapods 
        #sound buzzer
        #play message through speaker?

    def search_behavior(self, depth_img, cv2_img):
        #basic behavior, keep walking forward, if obstacle, turn right
        centerx = self.img_width/2
        centery = self.img_height/2 
        rightx = self.img_width * (3/4)
        righty = self.img_height/2 
        leftx = self.img_width * (1/4)
        lefty = self.img_height/2

        obstacle_distance = 350
        
        #depth_val = depth_img[int(centery)][int(centerx)]
        depth_val_central = self.calculate_depth_avg(depth_img,centerx,centery)
        depth_val_right = self.calculate_depth_avg(depth_img,rightx,righty)
        depth_val_left = self.calculate_depth_avg(depth_img,leftx,lefty)
        self.get_logger().info(f'Depth: ({depth_val_left},{depth_val_central},{depth_val_right})')

        command = String()
        #obstacle close by, turn right
        if depth_val_central < obstacle_distance:
            command.data = "turn_right"
            if verbose: self.get_logger().info(f'OBSTACLE DETECTED CENTRAL, TURNING RIGHT')
        elif depth_val_right < obstacle_distance:
            command.data = "turn_left"
            if verbose: self.get_logger().info(f'OBSTACLE DETECTED RIGHT, TURNING LEFT')
        elif depth_val_left < obstacle_distance:
            command.data = "turn_right" 
            if verbose: self.get_logger().info(f'OBSTACLE DETECTED LEFT, TURNING RIGHT')
        else:
            command.data = "move_forward"
            
        self._hexapod_commands_pub.publish(command)
    
    def calculate_depth_avg(self, depth_img, center_x, center_y):
        dirs = [(x, y) for x in range(-5, 6) for y in range(-5, 6)]
        total = 0
        numValid = 0 

        for dir in dirs:
            newx, newy = (int(center_x) + dir[0]),(int(center_y) + dir[1])

            if (0<=newx<self.img_width) and (0<=newy<self.img_height):
                pixel_depth = depth_img[newx][newy]
            else:
                pixel_depth = 0

            if pixel_depth != 0:
                total += pixel_depth
                numValid += 1
       
        if numValid != 0:
            return total/numValid
        else:
            return 0

    def main_callback(self, detections_msg, img_msg, depth_msg):
        txt_color = (255, 0, 255)

        cv2_img = self._bridge.imgmsg_to_cv2(img_msg)
        depth_img = self._bridge.imgmsg_to_cv2(depth_msg)
        if self.current_state == States.SEARCH:
            person_detected = False
    
            for detection in detections_msg.detections:
                center_x = detection.bbox.center.position.x
                center_y = detection.bbox.center.position.y
                width = detection.bbox.size_x
                height = detection.bbox.size_y

                label = names[int(detection.results[0].hypothesis.class_id)]
                conf_score = detection.results[0].hypothesis.score
                
                # If person is detected with center within camera view
                if (label == "person" and conf_score > 0.80 and 0 < center_y < self.img_height and 0 < center_x < self.img_width ):
                    if verbose: self.get_logger().info(f'DETECTED PERSON AT ({center_x},{center_y}) STATE: SEARCH -> INVESTIGATE')
                    person_detected = True 
                    self.current_state = States.INVESTIGATE
            if not person_detected:
                #send search algorithm movement commands 
                self.search_behavior(depth_img,cv2_img)
                

            processed_img = self._bridge.cv2_to_imgmsg(
                cv2_img, encoding=img_msg.encoding)
            self._processed_image_pub.publish(processed_img)

        elif self.current_state == States.INVESTIGATE:
            person_detected = False 

            for detection in detections_msg.detections:
                center_x = detection.bbox.center.position.x
                center_y = detection.bbox.center.position.y
                width = detection.bbox.size_x
                height = detection.bbox.size_y

                label = names[int(detection.results[0].hypothesis.class_id)]
                conf_score = detection.results[0].hypothesis.score
                
                #depth_val = depth_img[int(center_y)][int(center_x)]
                depth_val = self.calculate_depth_avg(depth_img,center_x,center_y)
                
                #in INVESTIGATE state we move closer to the detected person
                if (label == "person" and conf_score > 0.80 and 0 < center_y < 480 and 0 < center_x < 640 and depth_val != 0):


                    if self.current_target_x == -1 and self.current_target_y == -1:
                        if verbose: self.get_logger().info(f'Locking onto person at cx: {center_x} cy: {center_y} depth = {depth_val}')
                        self.current_target_x, self.current_target_y = center_x, center_y
                        person_detected = True 
                    elif (abs(self.current_target_x - center_x) < self.target_delta_threshold) and (abs(self.current_target_y - center_y) < self.target_delta_threshold):
                        if verbose: self.get_logger().info(f'PERSON DETECTED AT cx: {center_x} cy: {center_y} depth = {depth_val}')
                        command = self.adjust_heading(center_x,center_y,depth_val)
                        self._hexapod_commands_pub.publish(command)
                        person_detected = True 
                        
                        #CHECK IF STATE TRANSITION NEEDED
                        if (command.data == "stop_moving"):
                            self.current_state = States.FOUND 
                            if verbose: self.get_logger().info(f'(STATE) INVESTIGATE -> FOUND')
                    else:
                        pass


                    #ONLY DISPLAY BOUNDING BOXES FOR HUMANS THAT HIT THRESHOLD
                    min_pt = (round(center_x - (width / 2.0)),
                            round(center_y - (height / 2.0)))
                    max_pt = (round(center_x + (width / 2.0)),
                            round(center_y + (height / 2.0)))

                    lw = max(round((img_msg.height + img_msg.width) / 2 * 0.003), 2)  # line width
                    tf = max(lw - 1, 1)  # font thickness
                    label = f'{label} {conf_score:.2f}'
                    # text width, height
                    w, h = cv2.getTextSize(label, 0, fontScale=lw / 3, thickness=tf)[0]
                    outside = min_pt[1] - h >= 3

                    cv2.rectangle(cv2_img, min_pt, max_pt,
                                self.color, self.bbox_thickness)
                    cv2.putText(cv2_img, label, (min_pt[0], min_pt[1]-2 if outside else min_pt[1]+h+2),
                                0, lw / 3, txt_color, thickness=tf, lineType=cv2.LINE_AA)
                    
            #if no person detected, go back to search state 
            if not person_detected and self.current_state == States.INVESTIGATE:
                self.current_state = States.SEARCH
                self.current_target_x = -1
                self.current_target_y = -1
                if verbose: self.get_logger().info(f'(STATE) INVESTIGATE -> SEARCH')

            processed_img = self._bridge.cv2_to_imgmsg(
                cv2_img, encoding=img_msg.encoding)
            self._processed_image_pub.publish(processed_img)

        elif (self.current_state == States.FOUND):
            self.found_person()
            self.current_target_x = -1
            self.current_target_y = -1

            #we could also save the image to send back 
            
            #still visualize image
            processed_img = self._bridge.cv2_to_imgmsg(
                cv2_img, encoding=img_msg.encoding)
            self._processed_image_pub.publish(processed_img)



def main():
    rclpy.init()
    visualizer = Yolov8Visualizer()

    try:
        rclpy.spin(visualizer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
