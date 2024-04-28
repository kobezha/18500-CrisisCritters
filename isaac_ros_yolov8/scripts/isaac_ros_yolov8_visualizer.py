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
from geometry_msgs.msg import PoseStamped
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import pyrealsense2
from std_msgs.msg import String
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

class States(Enum):
    SEARCH = "Search"
    FOUND = "Found"
    INVESTIGATE = "Investigate"

class SearchState(Enum):  # Nested FSM for Searching
    READY = "Ready"  # Ready to search a new grid
    TURNING = "Turning"  # Currently turning to search a new grid
    MOVING = "Moving"  # Moving towards new grid

class SquareType(Enum):
    VISITED = '.'
    EMPTY = ' '
    BLOCKED = '*'
    TARGET = ''
    HEXAPOD = 'ඬ'

class Direction(Enum):
    NORTH = (-1, 0, 0)  # d_row, d_col, compass heading in degrees
    EAST = (0, 1, 90)
    SOUTH = (1, 0, 180)
    WEST = (0, -1, 270)


visual_flag = True 
verbose = True
hardcode = True  # Hardcode turn directions instead of using vo_pose from VSLAM

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

        # Search state variables
        self.search_state = SearchState.READY
        self.grid = [[SquareType.HEXAPOD]]
        self.moving_start = None  # Timestamp for when Hexapod starts moving
        self.turning_start = None
        self.turning_time = None
        self.turn_90 = 6  # TODO: Need to tune

        # TODO: Make sure the distance moved here is covered by depth threshold
        self.obstacle_threshold = 600
        self.moving_time = 12  # Length of one square in walking time

        self.curr_dir = Direction.NORTH
        self.next_dir = Direction.NORTH
        self.compass_heading = 0  # Degrees
        self.heading_tolerance = 10  # Degrees

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
        
        self._directions_subscription = message_filters.Subscriber(
            self,
            PoseStamped,
            'visual_slam/tracking/vo_pose')

        self._hexapod_commands_pub = self.create_publisher(
            String, 'hexapod_commands', self.QUEUE_SIZE)
        
        if not hardcode:
            self._directions_subscription.registerCallback(self.directions_callback)

    
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
            # Keep walking towards person until it is within 500
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

    def _find_obj(self, grid, obj):
        """
        Find index of object
        grid: List[List[SquareType]]
        obj: SquareType

        Return: List[Tuple[int, int]]
        """
        res = []
        for row in range(len(grid)):
            for col in range(len(grid[0])):
                if grid[row][col] == obj:
                    res.append((row, col))
        return res
    
    def directions_callback(self, msg):
        """
        Convert Quaternion to yaw angle. Gyro frequency is 200Hz, so this is called 200 times a second
        Cite: https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        """
        q = msg.pose.orientation  # Quaternion for orientation
        yaw_angle = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        # Convert from radians to compass bearing degrees
        yaw_angle = -(yaw_angle * 180.0 / math.pi) % 360
        self.compass_heading = yaw_angle
        # self.get_logger().info(f"New angle: {yaw_angle}")
        return
    
    def search_behavior(self, depth_img, cv2_img):
        """ Pseudocode:
        Randomly choose a Direction (NESW) and check not BLOCKED in grid
         - Keep looping until we find unblocked grid
        Turn to this direction and wait for depth image in new callback
        If at boundary of grid: Expand the 2D array to have a new row/column
        Check if any obstacles within square length ahead
         - If yes, then mark new Square as BLOCKED
        Move_forward a Square length (command_publish)
         - Use time.time() to determine when to stop moving
         - Once time_elapsed is reached, then stop and mark new grid as HEXAPOD
        If no issue: Append HEXAPOD to current Square, replace previous Square with HEXAPOD->VISITED
        """

        centerx, centery = self.img_width/2, self.img_height/2 
        leftx, rightx = self.img_width * (1/4), self.img_width * (3/4)
        
        depth_val_left = self.calculate_depth_avg(depth_img, leftx, centery)
        depth_val_central = self.calculate_depth_avg(depth_img, centerx, centery)
        depth_val_right = self.calculate_depth_avg(depth_img, rightx, centery)

        self.get_logger().info(f"{depth_val_left, depth_val_central, depth_val_right}")
        return

        command = String()

        # Get current coordinates of hexapod robot
        curr_coords = self._find_obj(self.grid, SquareType.HEXAPOD)
        assert(len(curr_coords) == 1)  # TODO: Make sure this is me! Not another Hexapod
        curr_row, curr_col = curr_coords[0]

        if self.search_state == SearchState.READY:  # Robot ready to search new square
            directions = [direction for direction in Direction]  # N, E, S, W
            while True:
                # Choose a compass direction to turn to
                assert(len(directions) > 0)
                next_dir = random.choice(directions)
                directions.remove(next_dir)
                d_row, d_col = next_dir.value[:2]

                # Check that this direction is valid to explore
                new_row, new_col = curr_row + d_row, curr_col + d_col

                if new_row < 0:  # Expand grid NORTH
                    self.grid.insert(0, [SquareType.EMPTY for i in range(len(self.grid[0]))])
                elif len(self.grid) <= new_row:  # Expand SOUTH
                    self.grid.append([SquareType.EMPTY for i in range(len(self.grid[0]))])
                elif new_col < 0: # Expand WEST
                    self.grid = [[SquareType.EMPTY] + row for row in self.grid]
                elif len(self.grid[0]) <= new_col:  # Expand EAST
                    self.grid = [row + [SquareType.EMPTY] for row in self.grid]
                elif self.grid[new_row][new_col] != SquareType.BLOCKED:
                    # This is a vacant square in the grid. We should explore it
                    # TODO (OPTIMIZE): Skip previously visited squares
                    pass
                else:
                    # This loops until we find a valid direction to explore
                    continue
                #  Made it here! This is a valid direction
                self.next_dir = next_dir
                self.search_state = SearchState.TURNING
                if verbose: 
                    self.get_logger().info(f"Current map: {self.grid}")
                    self.get_logger().info(f"Turning to {self.next_dir}")
                return

        elif self.search_state == SearchState.TURNING:
            """
            Compass heading is assigned from directions_callback
            """
            target_heading = self.next_dir.value[2]
            if hardcode:
                if self.turning_start is not None:  # Already turning. Check elapsed time
                    elapsed_time = time.time() - self.turning_start
                    if elapsed_time > self.turn_time:  # Finished turn. 
                        command.data = "stop_moving"
                        self.search_state = SearchState.MOVING
                        self.turning_start = None
                        self.curr_dir = self.next_dir
                        if verbose: self.get_logger().info(f"TURNING -> MOVING")
                    else:
                        command = self.last_command
                else:  # Initiate turning
                    current_heading = self.curr_dir.value[2]
                    error = (target_heading - current_heading) % 360
                    if target_heading == current_heading:  # Already in direction we want
                        command.data = "stop_moving"
                        self.search_state = SearchState.MOVING
                        if verbose: self.get_logger().info(f"TURNING -> MOVING")
                    elif error == 90:  # Turn right 90
                        command.data = "turn_right"
                        self.turn_time = self.turn_90
                        self.turning_start = time.time()
                    elif error == 180:  # Turn right 180
                        command.data = "turn_right"
                        self.turn_time = 2 * self.turn_90
                        self.turning_start = time.time()
                    else:  # Turn left 90
                        assert error == 270
                        command.data = "turn_left"
                        self.turn_time = self.turn_90
                        self.turning_start = time.time()
            else:  # Using compass_heading, which is assigned from directions_callback by vo_pose
                error = target_heading - self.compass_heading
                if abs(error) < self.heading_tolerance:
                    command.data = "stop_moving"
                    self.search_state = SearchState.MOVING
                    if verbose: self.get_logger().info(f"TURNING -> MOVING")
                elif (error % 360) <= 180:
                    command.data = "turn_right"
                else:  # (error % 360) > 180
                    command.data = "turn_left"
            self._hexapod_commands_pub.publish(command)
            self.last_command = command
            return
        
        elif self.search_state == SearchState.MOVING:
            command.data = "move_forward"
            d_row, d_col = self.next_dir.value[:2]
            new_row, new_col = curr_row + d_row, curr_col + d_col

            if self.moving_start is None:  # Haven't started moving yet
                # Check if path is clear
                centerx, centery = self.img_width/2, self.img_height/2 
                leftx, rightx = self.img_width * (1/4), self.img_width * (3/4)
                
                depth_val_left = self.calculate_depth_avg(depth_img, leftx, centery)
                depth_val_central = self.calculate_depth_avg(depth_img, centerx, centery)
                depth_val_right = self.calculate_depth_avg(depth_img, rightx, centery)

                if (max(depth_val_left, depth_val_central, depth_val_right) < self.obstacle_threshold):
                    # Square is blocked. Update the internal grid and search again
                    self.grid[new_row][new_col] = SquareType.BLOCKED
                    self.get_logger().info("Blocked square")
                    self.search_state = SearchState.READY
                else:
                    # Square is clear! Move into new square
                    self.moving_start = time.time()
            else:  # Already moving. Check if we should stop
                elapsed_time = time.time() - self.moving_start
                # if verbose: self.get_logger().info(f"Elapsed since {self.moving_start}: {elapsed_time}")
                if elapsed_time > self.moving_time:  
                    # We've reached new square! Update the grid
                    self.grid[curr_row][curr_col] = SquareType.VISITED
                    self.grid[new_row][new_col] = SquareType.HEXAPOD

                    # Stop moving and restart the search from new square
                    command.data = "stop_moving"
                    self.moving_start = None
                    self.search_state = SearchState.READY
            self._hexapod_commands_pub.publish(command)
            return
    
    def calculate_depth_avg(self, depth_img, center_x, center_y):
        dirs = [(x, y) for x in range(-5, 6) for y in range(-5, 6)]
        total = 0
        numValid = 0 

        for dir in dirs:
            newx, newy = (int(center_x) + dir[0]),(int(center_y) + dir[1])

            if (0<=newx<self.img_width) and (0<=newy<self.img_height):
                pixel_depth = depth_img[newy][newx]  # (row, col)
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
        # self.get_logger().info(f"main_callback called")
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
                    pass
                    # TODO: Uncomment when we want tracking again
                    # if verbose: self.get_logger().info(f'DETECTED PERSON AT ({center_x},{center_y}) STATE: SEARCH -> INVESTIGATE')
                    # person_detected = True 
                    # self.current_state = States.INVESTIGATE
            #if not person_detected:
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
                        if verbose: self.get_logger().info(f'Locking onto person at cx: {center_x:.0f} cy: {center_y:.0f} depth = {depth_val:.0f}')  # TODO: Make depth readings ints?
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

    #executor = MultiThreadedExecutor(num_threads=2)
    executor = SingleThreadedExecutor()

    executor.add_node(visualizer)

    try:
        #rclpy.spin(visualizer)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        #graceful shutdown
        executor.shutdown()
        visualizer.destroy_node()
        rclpy.shutdown()
 
#    try:
#        rclpy.spin(visualizer)
#    except (KeyboardInterrupt, ExternalShutdownException):
#        pass
#    finally:
#        visualizer.destroy_node()
#        rclpy.shutdown()
#
    

if __name__ == '__main__':
    main()
