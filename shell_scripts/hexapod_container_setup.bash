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

build_yolo_package() {
	cd /workspaces/isaac_ros-dev/src
	colcon build --packages-select isaac_ros_yolov8
	ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_visualize.launch.py model_file_path:=/tmp/yolov8s.onnx engine_file_path:=/tmp/yolov8s.plan input_binding_names:=['images'] output_binding_names:=['output0'] network_image_width:=640 network_image_height:=640 force_engine_update:=False image_mean:=[0.0,0.0,0.0] image_stddev:=[1.0,1.0,1.0] input_image_width:=640 input_image_height:=640 confidence_threshold:=0.25 nms_threshold:=0.45
}

