#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode()
        : Node("image_conversion_node"), image_mode_(2) {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/converted_image");

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        // Subscriber and publisher
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10,
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        // Service to set the mode
        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_image_mode",
            std::bind(&ImageConversionNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "ImageConversionNode is running");
    }

private:
    void setModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        image_mode_ = request->data ? 1 : 2;
        response->success = true;
        response->message = image_mode_ == 1 ? "Mode set to Grayscale" : "Mode set to Color";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat processed_image;

            if (image_mode_ == 1) { // Grayscale
                cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_BGR2GRAY);
                cv_ptr->image = processed_image;
                cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
            }

            // Publish the processed image
            image_pub_->publish(*cv_ptr->toImageMsg());
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    int image_mode_; // 1 for grayscale, 2 for color

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
