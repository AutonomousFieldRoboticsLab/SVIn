#include "pose_graph/Subscriber.h"

#include <ros/console.h>

#include <memory>
#include <utility>

Subscriber::Subscriber(ros::NodeHandle& nh, const Parameters& params) : params_(params) {
  // TODO(bjoshi): pass as params from roslaunch file
  kf_image_topic_ = "/okvis_node/keyframe_imageL";
  kf_pose_topic_ = "/okvis_node/keyframe_pose";
  kf_points_topic_ = "/okvis_node/keyframe_points";
  svin_reloc_odom_topic_ = "/okvis_node/relocalization_odometry";
  svin_health_topic_ = "/svin_health";
  last_image_time_ = -1;

  setNodeHandle(nh);
}

void Subscriber::init(ros::NodeHandle& nh, const Parameters& params) {
  params_ = params;
  kf_image_topic_ = "/okvis_node/keyframe_imageL";
  kf_pose_topic_ = "/okvis_node/keyframe_pose";
  kf_points_topic_ = "/okvis_node/keyframe_points";
  svin_reloc_odom_topic_ = "/okvis_node/relocalization_odometry";
  svin_health_topic_ = "/svin_health";
  last_image_time_ = -1;

  setNodeHandle(nh);
}

void Subscriber::setNodeHandle(ros::NodeHandle& nh) {
  nh_ = &nh;
  if (it_) it_.reset();
  it_ = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(std::move(*nh_)));

  sub_kf_image_ =
      it_->subscribe(kf_image_topic_, 500, std::bind(&Subscriber::keyframeImageCallback, this, std::placeholders::_1));

  sub_orig_image_ =
      it_->subscribe("/cam0/image_raw", 500, std::bind(&Subscriber::imageCallback, this, std::placeholders::_1));
  sub_kf_points_ = nh_->subscribe(kf_points_topic_, 500, &Subscriber::keyframePointsCallback, this);

  sub_kf_pose_ = nh_->subscribe(kf_pose_topic_, 500, &Subscriber::keyframePoseCallback, this);
  // sub_svin_relocalization_odom_ =
  //     nh_->subscribe(svin_reloc_odom_topic_, 500, &Subscriber::svinRelocalizationOdomCallback, this);

  if (params_.use_health_)
    sub_svin_health_ = nh_->subscribe(svin_health_topic_, 500, &Subscriber::svinHealthCallback, this);
}

void Subscriber::keyframeImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::lock_guard<std::mutex> lock(measurement_mutex_);
  kf_image_buffer_.push(msg);
  last_image_time_ = msg->header.stamp.toSec();
}

void Subscriber::keyframePointsCallback(const sensor_msgs::PointCloudConstPtr& msg) {
  std::lock_guard<std::mutex> l(measurement_mutex_);
  kf_pcl_buffer_.push(msg);
}

void Subscriber::keyframePoseCallback(const nav_msgs::OdometryConstPtr& msg) {
  std::lock_guard<std::mutex> l(measurement_mutex_);
  kf_pose_buffer_.push(msg);
}

// void Subscriber::svinRelocalizationOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
//   svin_relocalization_odom_ = msg;
// }
void Subscriber::svinHealthCallback(const okvis_ros::SvinHealthConstPtr& msg) {
  std::lock_guard<std::mutex> l(measurement_mutex_);
  svin_health_buffer_.push(msg);
}

void Subscriber::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::lock_guard<std::mutex> lock(measurement_mutex_);
  orig_image_buffer_.push(msg);
}

const cv::Mat Subscriber::readRosImage(const sensor_msgs::ImageConstPtr& img_msg) const {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  const cv::Mat img_const = cv_ptr->image;  // Don't modify shared image in ROS.
  cv::Mat converted_img(img_const.size(), CV_8U);
  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    // LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    // LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    return converted_img;
  } else {
    ROS_ERROR_STREAM_COND(cv_ptr->encoding != sensor_msgs::image_encodings::MONO8,
                          "Expected image with MONO8, BGR8, or RGB8 encoding."
                          "Add in here more conversions if you wish.");
    return img_const;
  }
}

void Subscriber::getSyncMeasurements(sensor_msgs::ImageConstPtr& kf_image_msg,
                                     nav_msgs::OdometryConstPtr& kf_odom_msg,
                                     sensor_msgs::PointCloudConstPtr& kf_points_msg,
                                     okvis_ros::SvinHealthConstPtr& svin_health_msg) {
  // timestamp synchronization
  {
    std::lock_guard<std::mutex> l(measurement_mutex_);
    if (!kf_image_buffer_.empty() && !kf_pcl_buffer_.empty() && !kf_pose_buffer_.empty() &&
        (!params_.use_health_ || !svin_health_buffer_.empty())) {  // TODO(bjoshi): this condition does not look good.
      if (kf_image_buffer_.front()->header.stamp.toSec() > kf_pose_buffer_.front()->header.stamp.toSec()) {
        kf_pose_buffer_.pop();
        printf("Throw away pose at beginning\n");
      } else if (kf_image_buffer_.front()->header.stamp.toSec() > kf_pcl_buffer_.front()->header.stamp.toSec()) {
        kf_pcl_buffer_.pop();
        printf("Throw away pointcloud at beginning\n");

      } else if (params_.use_health_ &&
                 kf_image_buffer_.front()->header.stamp.toSec() > svin_health_buffer_.front()->header.stamp.toSec()) {
        svin_health_buffer_.pop();
        printf("Throw away health at beginning\n");
      } else if (kf_image_buffer_.back()->header.stamp.toSec() >= kf_pose_buffer_.front()->header.stamp.toSec() &&
                 kf_pcl_buffer_.back()->header.stamp.toSec() >= kf_pose_buffer_.front()->header.stamp.toSec() &&
                 (!params_.use_health_ ||
                  svin_health_buffer_.back()->header.stamp.toSec() >= kf_pose_buffer_.front()->header.stamp.toSec())) {
        kf_odom_msg = kf_pose_buffer_.front();
        kf_pose_buffer_.pop();
        while (!kf_pose_buffer_.empty()) kf_pose_buffer_.pop();
        while (kf_image_buffer_.front()->header.stamp.toSec() < kf_odom_msg->header.stamp.toSec())
          kf_image_buffer_.pop();
        kf_image_msg = kf_image_buffer_.front();
        kf_image_buffer_.pop();

        while (kf_pcl_buffer_.front()->header.stamp.toSec() < kf_odom_msg->header.stamp.toSec()) kf_pcl_buffer_.pop();
        kf_points_msg = kf_pcl_buffer_.front();
        kf_pcl_buffer_.pop();

        if (params_.use_health_) {
          while (svin_health_buffer_.front()->header.stamp.toSec() < kf_odom_msg->header.stamp.toSec()) {
            svin_health_buffer_.pop();
          }
          svin_health_msg = svin_health_buffer_.front();
          svin_health_buffer_.pop();
        }
      }
    }
  }
}

const cv::Mat Subscriber::getCorrespondingImage(const uint64_t& ros_stamp) {
  while (!orig_image_buffer_.empty() && orig_image_buffer_.front()->header.stamp.toNSec() < ros_stamp) {
    orig_image_buffer_.pop();
  }

  sensor_msgs::ImageConstPtr img_msg = orig_image_buffer_.front();
  orig_image_buffer_.pop();

  uint64_t diff = abs(static_cast<int64_t>(img_msg->header.stamp.toNSec()) - static_cast<int64_t>(ros_stamp));
  assert(diff < 10000000);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  const cv::Mat img_const = cv_ptr->image;  // Don't modify shared image in ROS.
  cv::Mat converted_img;
  if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    // LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2BGR);
    return converted_img;
  } else {
    return img_const;
  }
}
