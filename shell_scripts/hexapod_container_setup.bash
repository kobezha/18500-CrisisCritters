#!/bin/bash
pip3 install pyrealsense2
pip3 install colcon-clean
sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-tensor-rt ros-humble-isaac-ros-dnn-image-encoder
sudo apt-get remove -y ros-humble-isaac-ros-yolov8
sudo apt install x11-apps
sudo apt install tmux
colcon clean workspace
colcon build
source install/setup.bash
