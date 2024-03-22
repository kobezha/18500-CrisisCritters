from setuptools import find_packages
from setuptools import setup

setup(
    name='isaac_ros_yolov8',
    version='2.1.0',
    packages=find_packages(
        include=('isaac_ros_yolov8', 'isaac_ros_yolov8.*')),
)
