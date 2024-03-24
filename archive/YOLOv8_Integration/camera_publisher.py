#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher:
    def __init__(self):
        rospy.init_node('usb_camera_publisher', anonymous=True)
        self.window_title = "USB Camera"
        self.camera_id = "/dev/video2"
        self.video_capture = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        self.bridge = CvBridge()


        self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)

        #if not using callbacks 
        #self.rate = rospy.Rate(1) #Rate of 1Hz, frames would only be published at a rate of 1Hz 
        #self.publish_frames()

        self.timer = rospy.Timer(rospy.Duration(1.0 / 5), self.timer_callback)  # Timer callback every 5 seconds


    def timer_callback(self, event):
        if self.video_capture.isOpened():
            ret_val, frame = self.video_capture.read()
            if ret_val:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(image_msg)
            else:
                rospy.logwarn("Failed to read frame from the camera.")
        else:
            rospy.logerr("Unable to open camera")


if __name__ == '__main__':
    try:
        CameraPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
