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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

#launch description for hexapod system, removes any visualization to improve performance
def generate_launch_description():
    yolov8_dir = get_package_share_directory('isaac_ros_yolov8')
    vslam_dir = get_package_share_directory('isaac_ros_visual_slam')

    ld = LaunchDescription()

    hexapod_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolov8_dir,
                         'launch/isaac_ros_yolov8_visualize.launch.py')
        )
    )
    
    #launches realsense camera as well
    hexapod_vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vslam_dir,
                         'launch/isaac_ros_visual_slam_realsense.launch.py')
        )
    )

    ld.add_action(hexapod_detection_launch)
    ld.add_action(hexapod_vslam_launch)



    return ld
