#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    image_sync_policy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
                                                        sensor_msgs::msg::CompressedImage>
    compressed_image_sync_policy;

class StereoSync : public rclcpp::Node {
 public:
  explicit StereoSync(const rclcpp::NodeOptions& options) : Node("stereo_sync", options) {
    previous_stamp_ = rclcpp::Time(0, 0);

    this->get_parameter("left_img_topic", left_img_topic_);
    this->get_parameter("right_img_topic", right_img_topic_);
    this->get_parameter("compressed", compressed_image_);
    this->get_parameter("config_filename", config_file_);

    cv::FileStorage fsSettings(config_file_, cv::FileStorage::READ);

    if (!fsSettings.isOpened()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "ERROR: Wrong path to settings\n");
    }

    float frame_rate = static_cast<float>(fsSettings["camera_params"]["camera_rate"]);
    float max_time_diff_between_msgs = 0.5f / frame_rate;
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting Age Penalty to:  " << max_time_diff_between_msgs);
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    static constexpr size_t kImageSynchronizerQueueSize = 10u;

    if (compressed_image_) {
      left_compressed_img_topic_ = left_img_topic_ + "/compressed";
      right_compressed_img_topic_ = right_img_topic_ + "/compressed";

      left_compressed_img_sub_.subscribe(this, left_compressed_img_topic_, rmw_qos_profile);
      right_compressed_img_sub_.subscribe(this, right_compressed_img_topic_, rmw_qos_profile);
      compressed_image_synchronizer_ = std::make_unique<message_filters::Synchronizer<compressed_image_sync_policy>>(
          compressed_image_sync_policy(kImageSynchronizerQueueSize),
          left_compressed_img_sub_,
          right_compressed_img_sub_);
      compressed_image_synchronizer_->registerCallback(
          std::bind(&StereoSync::compressedImageCallback, this, std::placeholders::_1, std::placeholders::_2));
      compressed_image_synchronizer_->setAgePenalty(max_time_diff_between_msgs);
    } else {
      left_img_sub_.subscribe(this, left_img_topic_, rmw_qos_profile);
      right_img_sub_.subscribe(this, right_img_topic_, rmw_qos_profile);
      image_synchonizer_ = std::make_unique<message_filters::Synchronizer<image_sync_policy>>(
          image_sync_policy(kImageSynchronizerQueueSize), left_img_sub_, right_img_sub_);
      image_synchonizer_->registerCallback(
          std::bind(&StereoSync::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
      image_synchonizer_->setAgePenalty(max_time_diff_between_msgs);
    }

    left_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 100);
    right_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", 100);
  }

 private:
  std::string left_img_topic_{}, right_img_topic_{};
  std::string left_compressed_img_topic_{}, right_compressed_img_topic_{};
  std::string config_file_{};
  bool compressed_image_{};

  rclcpp::Time previous_stamp_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_img_sub_, right_img_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> left_compressed_img_sub_, right_compressed_img_sub_;

  std::unique_ptr<message_filters::Synchronizer<image_sync_policy>> image_synchonizer_;
  std::unique_ptr<message_filters::Synchronizer<compressed_image_sync_policy>> compressed_image_synchronizer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub_, right_img_pub_;

  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& left_msg,
                               const sensor_msgs::msg::CompressedImage::ConstSharedPtr& right_msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Inside stereo compressed image callback");

    rclcpp::Time stamp = rclcpp::Time(left_msg->header.stamp.sec, left_msg->header.stamp.nanosec) +
                         (rclcpp::Time(right_msg->header.stamp.sec, right_msg->header.stamp.nanosec) -
                          rclcpp::Time(left_msg->header.stamp.sec, left_msg->header.stamp.nanosec)) *
                             0.5;
    std_msgs::msg::Header left_header, right_header;
    left_header.stamp = stamp;
    right_header.stamp = stamp;
    left_header.frame_id = left_msg->header.frame_id;
    right_header.frame_id = right_msg->header.frame_id;

    if (previous_stamp_ >= stamp) {
      return;
    }

    cv_bridge::CvImageConstPtr left_cv_ptr, right_cv_ptr;
    left_cv_ptr = cv_bridge::toCvCopy(left_msg);
    right_cv_ptr = cv_bridge::toCvCopy(right_msg);

    left_img_pub_->publish(*left_cv_ptr->toImageMsg());
    right_img_pub_->publish(*right_cv_ptr->toImageMsg());

    previous_stamp_ = stamp;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Inside stereo image callback");

    rclcpp::Time stamp = rclcpp::Time(left_msg->header.stamp.sec, left_msg->header.stamp.nanosec) +
                         (rclcpp::Time(right_msg->header.stamp.sec, right_msg->header.stamp.nanosec) -
                          rclcpp::Time(left_msg->header.stamp.sec, left_msg->header.stamp.nanosec)) *
                             0.5;
    std_msgs::msg::Header left_header, right_header;
    left_header.stamp = stamp;
    right_header.stamp = stamp;
    left_header.frame_id = left_msg->header.frame_id;
    right_header.frame_id = right_msg->header.frame_id;

    if (previous_stamp_ >= stamp) {
      return;
    }

    left_img_pub_->publish(*left_msg);
    right_img_pub_->publish(*right_msg);

    previous_stamp_ = stamp;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<StereoSync>(options));

  return 0;
}