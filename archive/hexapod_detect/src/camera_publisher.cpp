#include "rclcpp/rclcpp.hpp"

//could be image.msg, double check
#include "sensor_msgs/msg/Image.hpp"
#include <chrono>

//for image processing
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher_: public rclcpp:: Node
{
  protected:
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
    std::string camera_id_;

  public:
    //constructor 
    CameraPublisher_(): Node("camera") {
        
      //create a shared ptr to pass to img transport with an empty deleter function 
      node_handle_(std::shared_ptr<CamPublisher_>(this,[] (auto *) {})); 
      image_transport_(node_handle_);
      image_publisher_ = image_transport.advertise("image",10);

      timer_ = node_handle_.createTimer(ros::Duration(1.0 / 5), &CameraPublisher::timerCallback, this); // Timer callback every 5 seconds
      //topic_image = "image"
      // quality of service object, only last 10 messages are kept in history
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      camera_id_ = "/dev/video2";
      video_capture_.open(camera_id_, cv::CAP_V4L2);
      if (!video_capture_.isOpened()) {
          //ROS_ERROR("Unable to open camera");
          RCLCPP_ERROR(this->get_logger(), "Unable to open camera");

          return;
      }
    }

    ~CameraPublisher_() {
        video_capture_.release();
    }

  private:
    void timerCallback(const ros::TimerEvent& event) {
      cv::Mat frame;
      sensor_msgs::msg::Image msg;

      if (video_capture_.isOpened()) {
          if (video_capture_.read(frame)) {
              sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
              image_publisher_.publish(msg);
          } else {
              RCLCPP_WARN(this->get_logger(), "Failed to read frame from the camera.");

          }
      } else {
          RCLCPP_ERROR(this->get_logger(), "Unable to open camera");

      }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto camera_publisher = std::make_shared<CameraPublisher>();
    rclcpp::spin(camera_publisher);
    rclcpp::shutdown();
    return 0;
}
