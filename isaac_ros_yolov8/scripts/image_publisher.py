#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

window_title = "USB Camera"

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher", namespace ="")
        self.get_logger().info('ImagePublisher INITIALIZED...') 
        self.bridge = CvBridge()
        self.camera_id = "/dev/video0"
        #try:
        #   self.cap = cv2.VideoCapture(0)
        #except:
        #    self.cap = cv2.VideoCapture(1)
        self.pub = self.create_publisher(Image, "/image", 10)
        #self.rgb8pub = self.create_publisher(Image, "/image/rgb", 10)
        #self.bgr8pub = self.create_publisher(Image, "/image/bgr", 10)
        #self.mono8pub = self.create_publisher(Image, "/image/mono", 10)


        #self.run()

    def run(self):
        
        self.video_capture = cv2.VideoCapture(self.camera_id,cv2.CAP_V4L2)
        
        if self.video_capture.isOpened():
            self.get_logger().info("Video Capture Opened!") 
            try:
                window_handle = cv2.namedWindow(
                        window_title, cv2.WINDOW_AUTOSIZE)

                while True:
                    r, frame = self.video_capture.read()
                    if not r:
                        return

                    if cv2.getWindowProperty(window_title,cv2.WND_PROP_AUTOSIZE) >= 0:
                        cv2.imshow(window_title,frame)
                    else: 
                        self.get_logger().info("window property error") 
                        break

                    keyCode = cv2.waitKey(10) & 0xFF

                    if keyCode == 27 or keyCode == ord('q'):
                        break

                    #self.get_logger().info("ImagePublisher running") 
                    #print("Converting frame...")
                    self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            finally:

               self.get_logger().info("CAMERA SHUTTING DOWN") 
               video_capture.release()
               cv2.destroyAllWindows()
        else:
            self.get_logger().info("Unable to open camera")


        #except CvBridgeError as e:
         #   print(e)
            #self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    ip = ImagePublisher()
    #rclpy.spin(ip)
    #print("Publishing...")
    ip.get_logger().info("STARTING IP RUN") 
    ip.run()
    ip.get_logger().info("OUT OF IP RUN") 

    #ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
