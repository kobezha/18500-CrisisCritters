#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher {
public:
    CameraPublisher() {
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_pub = it.advertise("camera/image", 10);
        timer = nh.createTimer(ros::Duration(1.0 / 5), &CameraPublisher::timerCallback, this); // Timer callback every 5 seconds
        camera_id = "/dev/video2";
        video_capture.open(camera_id, cv::CAP_V4L2);
        if (!video_capture.isOpened()) {
            ROS_ERROR("Unable to open camera");
            return;
        }
    }

    ~CameraPublisher() {
        video_capture.release();
    }

private:
    ros::Timer timer;
    image_transport::Publisher image_pub;
    cv::VideoCapture video_capture;
    std::string camera_id;

    void timerCallback(const ros::TimerEvent& event) {
        cv::Mat frame;
        if (video_capture.isOpened()) {
            if (video_capture.read(frame)) {
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                image_pub.publish(msg);
            } else {
                ROS_WARN("Failed to read frame from the camera.");
            }
        } else {
            ROS_ERROR("Unable to open camera");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "usb_camera_publisher");
    CameraPublisher camera_publisher;
    ros::spin();
    return 0;
}
