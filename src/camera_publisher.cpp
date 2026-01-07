#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

class CameraPublisherNode : public rclcpp::Node
{
public:
  CameraPublisherNode() : Node("camera_publisher")
  {
    // Declare parameters
    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<int>("port_no", 0);
    this->declare_parameter<int>("frame_width", 640);
    this->declare_parameter<int>("frame_height", 480);
    this->declare_parameter<double>("publish_frequency", 30.0);

    // Get parameters
    frame_id_ = this->get_parameter("frame_id").as_string();
    port_no_ = this->get_parameter("port_no").as_int();
    frame_width_ = this->get_parameter("frame_width").as_int();
    frame_height_ = this->get_parameter("frame_height").as_int();
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();

    // Create publishers
    // rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    // qos.reliable();
    image_raw_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(frame_id_ + "/image", 10);

    // Initialize OpenCV video capture
    cap_.open(port_no_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera using V4L2");
        rclcpp::shutdown();
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);

    // Timer for publishing frames
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_frequency_),
        std::bind(&CameraPublisherNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Camera Publisher Node has started");
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (!frame.empty())
    {
      // Convert OpenCV frame to ROS Image message
      auto raw_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      raw_image_msg->header.stamp = this->now();
      raw_image_msg->header.frame_id = frame_id_;

      // Publish raw and compressed images
      image_raw_publisher_->publish(*raw_image_msg);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
    }
  }

  // Node parameters
  std::string frame_id_;
  int port_no_;
  int frame_width_;
  int frame_height_;
  double publish_frequency_;

  cv::VideoCapture cap_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisherNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}