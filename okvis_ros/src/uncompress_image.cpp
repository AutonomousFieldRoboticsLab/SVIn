#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#if defined(ROS2_JAZZY)
  #include <cv_bridge/cv_bridge.hpp>
#elif defined(ROS2_HUMBLE)
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

class UncompressImage : public rclcpp::Node {
 public:
  UncompressImage(const rclcpp::NodeOptions& options) : Node("uncompress_image", options) {
    this->get_parameter("compressed_img_topic", compressed_img_topic_);
    this->get_parameter("ouput_img_topic", uncompressed_img_topic_);

    compressed_img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        compressed_img_topic_, 1000, std::bind(&UncompressImage::compressedImageCallback, this, std::placeholders::_1));
    output_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(uncompressed_img_topic_, 100);
  }

 private:
  std::string compressed_img_topic_, uncompressed_img_topic_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_img_pub_;

  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed_img_msg_ptr) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Got Compressed image");

    rclcpp::Time stamp = compressed_img_msg_ptr->header.stamp;
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = compressed_img_msg_ptr->header.frame_id;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(compressed_img_msg_ptr);

    output_img_pub_->publish(*cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<UncompressImage>(options));

  return 0;
}