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
from enum import Enum
import message_filters
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

class States(Enum):
    SEARCH = "Search"
    FOUND = "Found"


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


class Controller(Node):
    QUEUE_SIZE = 10
    color = (0, 255, 0)
    bbox_thickness = 2

    def __init__(self):
        super().__init__('controller',namespace = "")
        self.currentState = States.SEARCH
        self._bridge = cv_bridge.CvBridge()

        self._hexapod_commands_pub = self.create_publisher(
            String, 
            'hexapod_commands',
            self.QUEUE_SIZE)

        self._detections_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            'detections_output')
        self._image_subscription = message_filters.Subscriber(
            self,
            Image,
            'image')

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            self.QUEUE_SIZE)

        self.time_synchronizer.registerCallback(self.detections_callback)

    def detections_callback(self, detections_msg, img_msg):
        if self.currentState != States.SEARCH:
            self._detections_subscription.pause()
            self._image_subscription.pause()
            return

        for detection in detections_msg.detections:
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y

            label = names[int(detection.results[0].hypothesis.class_id)]
            conf_score = detection.results[0].hypothesis.score
            
            #only print if person is detected 
            if (label == "person" and conf_score > 0.80):
                self.get_logger().info(f'label: {label} cx: {center_x} cy: {center_y}')
                self.currentState == States.FOUND
                
    
    def run(self):
        while True:
            if (self.currentState == States.SEARCH):
                """ 
                Check subscriptions are active?

                Do random movement algorithm
                  - Use depth camera / ultrasonic to check which directions are blocked
                    - Can scan 4 directions (N, E, S, W) or 8 (NE, SE, SW, NW)
                  - Randomly choose one of the unblocked directions
                    - If no unblocked directions, signal error? Or scan more directions
                TODO: Unable to implement above algorithm until we can command how much to turn
                (ie. 90°)
                
                """
                # Check if path forward is blocked
                # TODO: Need to query ultrasonic or camera to get distance_ahead 
                if (self.distance_ahead > 10):  # Freenove returns length in cm
                    # Keep walking forwards!
                    self._hexapod_commands_pub.publish("move_forward")
                else:
                    # TODO: Query ultrasonic / camera again
                    while (self.distance_ahead < 10):
                        # Keep turning right until we have distance again
                        self._hexapod_commands_pub.publish("turn_right")

                
            elif (self.currentState == States.FOUND):

                #target found, stop moving first
                self._hexapod_commands_pub.publish("stop_moving")
                #enable buzzer command?
                self._hexapod_commands_pub.publish("enable_buzzer")

                #TODO: communication with other hexapods 
                # ie. Confidence of object. Location?
                pass
        


def main():
    rclpy.init()
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
