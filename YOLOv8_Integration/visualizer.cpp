#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>



using namespace std::placeholders;

class Yolov8Visualizer : public rclcpp::Node
{
public:
    Yolov8Visualizer() : Node("yolov8_visualizer")
    {
        processed_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("yolov8_processed_image", 10);

        /*
        subscribe to detections_output topic 
        queue_size is how many messages can be buffered before they are dropped 
        third argument specifies the callback function that will be called when there are updates to the detections_output topic
        bind function binds the detections_callback function to this instance of Yolov8Visualizer (specified by this)
        _1 is a placeholder that represents the first argument passed to the callback function.
        */
        detections_subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "detections_output", 10, std::bind(&Yolov8Visualizer::detections_callback, this, _1));

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10, std::bind(&Yolov8Visualizer::image_callback, this, _1));
    }

private:
    void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_msg)
    {
        if (!cv_image_.empty())
        {
            cv::Mat cv_image = cv_image_;
            for (const auto &detection : detections_msg->detections)
            {
                auto center_x = detection.bbox.center.position.x;
                auto center_y = detection.bbox.center.position.y;
                auto width = detection.bbox.size_x;
                auto height = detection.bbox.size_y;

                auto label = names.at(static_cast<int>(detection.results[0].hypothesis.class_id));
                auto conf_score = detection.results[0].hypothesis.score;
                label = label + " " + std::to_string(conf_score);

                auto min_pt = cv::Point(center_x - (width / 2.0), center_y - (height / 2.0));
                auto max_pt = cv::Point(center_x + (width / 2.0), center_y + (height / 2.0));

                auto lw = std::max(static_cast<int>((cv_image.rows + cv_image.cols) / 2 * 0.003), 2); // line width
                auto tf = std::max(lw - 1, 1);                                                         // font thickness
                // text width, height
                int baseline = 0;
                auto text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, lw / 3, tf, &baseline);
                auto w = text_size.width;
                auto h = text_size.height;

                auto outside = min_pt.y - h >= 3;

                cv::rectangle(cv_image, min_pt, max_pt, cv::Scalar(0, 255, 0), 2);
                cv::putText(cv_image, label, cv::Point(min_pt.x, outside ? min_pt.y - 2 : min_pt.y + h + 2),
                            cv::FONT_HERSHEY_SIMPLEX, lw / 3, cv::Scalar(255, 0, 255), tf, cv::LINE_AA);
            }

            sensor_msgs::msg::Image::SharedPtr processed_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
            processed_image_pub_->publish(*processed_img);
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv_image_ = cv_ptr->image;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    cv::Mat cv_image_;

    std::map<int, std::string> names = {
        {0, "person"},
        {1, "bicycle"},
        {2, "car"},
        {3, "motorcycle"},
        {4, "airplane"},
        {5, "bus"},
        {6, "train"},
        {7, "truck"},
        {8, "boat"},
        {9, "traffic light"},
        {10, "fire hydrant"},
        {11, "stop sign"},
        {12, "parking meter"},
        {13, "bench"},
        {14, "bird"},
        {15, "cat"},
        {16, "dog"},
        {17, "horse"},
        {18, "sheep"},
        {19, "cow"},
        {20, "elephant"},
        {21, "bear"},
        {22, "zebra"},
        {23, "giraffe"},
        {24, "backpack"},
        {25, "umbrella"},
        {26, "handbag"},
        {27, "tie"},
        {28, "suitcase"},
        {29, "frisbee"},
        {30, "skis"},
        {31, "snowboard"},
        {32, "sports ball"},
        {33, "kite"},
        {34, "baseball bat"},
        {35, "baseball glove"},
        {36, "skateboard"},
        {37, "surfboard"},
        {38, "tennis racket"},
        {39, "bottle"},
        {40, "wine glass"},
        {41, "cup"},
        {42, "fork"},
        {43, "knife"},
        {44, "spoon"},
        {45, "bowl"},
        {46, "banana"},
        {47, "apple"},
        {48, "sandwich"},
        {49, "orange"},
        {50, "broccoli"},
        {51, "carrot"},
        {52, "hot dog"},
        {53, "pizza"},
        {54, "donut"},
        {55, "cake"},
        {56, "chair"},
        {57, "couch"},
        {58, "potted plant"},
        {59, "bed"},
        {60, "dining table"},
        {61, "toilet"},
        {62, "tv"},
        {63, "laptop"},
        {64, "mouse"},
        {65, "remote"},
        {66, "keyboard"},
        {67, "cell phone"},
        {68, "microwave"},
        {69, "oven"},
        {70, "toaster"},
        {71, "sink"},
        {72, "refrigerator"},
        {73, "book"},
        {74, "clock"},
        {75, "vase"},
        {76, "scissors"},
        {77, "teddy bear"},
        {78, "hair drier"},
        {79, "toothbrush"}};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Yolov8Visualizer>());
    rclcpp::shutdown();
    return 0;
}