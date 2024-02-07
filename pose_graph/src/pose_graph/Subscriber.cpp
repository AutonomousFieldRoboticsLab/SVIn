#include "pose_graph/Subscriber.h"

#include <memory>
#include <opencv2/core/eigen.hpp>
#include <utility>
#include <vector>

#include "utils/Statistics.h"
#include "utils/Timer.h"
#include "utils/Utils.h"
#include "utils/UtilsOpenCV.h"

Subscriber::Subscriber(std::shared_ptr<rclcpp::Node> node, Parameters& params) : node_(node), params_(params) {
  // TODO(bjoshi): pass as params from roslaunch file
  kf_image_topic_ = "/keyframe_imageL";
  kf_pose_topic_ = "/keyframe_pose";
  kf_points_topic_ = "/keyframe_points";
  svin_reloc_odom_topic_ = "/relocalization_odometry";
  svin_health_topic_ = "/svin_health";
  primitive_estimator_topic_ = "/aqua_primitive_estimator/odometry";
  last_image_time_ = -1;
  raw_image_topic_ = "/cam0/image_raw";

  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  keyframe_image_subscriber_.subscribe(node_, kf_image_topic_, rmw_qos_profile);
  keyframe_points_subscriber_.subscribe(node_, kf_points_topic_, rmw_qos_profile);
  keyframe_pose_subscriber_.subscribe(node_, kf_pose_topic_, rmw_qos_profile);
  svin_health_subscriber_.subscribe(node_, svin_health_topic_, rmw_qos_profile);

  static constexpr size_t kMaxKeyframeSynchronizerQueueSize = 10u;
  sync_keyframe_ = std::make_unique<message_filters::Synchronizer<keyframe_sync_policy>>(
      keyframe_sync_policy(kMaxKeyframeSynchronizerQueueSize),
      keyframe_image_subscriber_,
      keyframe_pose_subscriber_,
      keyframe_points_subscriber_,
      svin_health_subscriber_);
  sync_keyframe_->registerCallback(std::bind(&Subscriber::keyframeCallback,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2,
                                             std::placeholders::_3,
                                             std::placeholders::_4));

  if (params_.global_mapping_params_.enabled) {
    sub_orig_image_ = node_->create_subscription<sensor_msgs::msg::Image>(
        raw_image_topic_, 100, std::bind(&Subscriber::imageCallback, this, std::placeholders::_1));
  }
  if (params_.health_params_.enabled) {
    sub_primitive_estimator_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        primitive_estimator_topic_,
        100,
        std::bind(&Subscriber::primitiveEstimatorCallback, this, std::placeholders::_1));
  }
}

void Subscriber::primitiveEstimatorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!primitive_estimator_callback_ && params_.health_params_.enabled) {
    LOG_EVERY_N(ERROR, 100) << "Primitive estimator callback not set";
  } else if (primitive_estimator_callback_) {
    Eigen::Matrix4d pose = Utils::rosPoseToMatrix(msg->pose.pose);
    cv::Mat cv_pose;
    cv::eigen2cv(pose, cv_pose);
    auto pose_with_timestamp =
        std::make_unique<std::pair<Timestamp, cv::Mat>>(std::make_pair(Utils::getHeaderStamp(msg->header), cv_pose));
    primitive_estimator_callback_(std::move(pose_with_timestamp));
  }
}

void Subscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg) {
  cv::Mat image = UtilsOpenCV::readRosImage(image_msg, false);
  auto image_with_timestamp =
      std::make_unique<std::pair<Timestamp, cv::Mat>>(std::make_pair(Utils::getHeaderStamp(image_msg->header), image));
  // raw image callback is not compulsory
  if (raw_image_callback_) {
    raw_image_callback_(std::move(image_with_timestamp));
  } else {
    LOG_EVERY_N(WARNING, 100) << "Raw image callback not set";
  }
}

nav_msgs::msg::Odometry::ConstSharedPtr Subscriber::getPrimitiveEstimatorPose(const int64_t& ros_stamp) {
  nav_msgs::msg::Odometry::ConstSharedPtr prim_estimator_pose = nullptr;
  // 25ms sync period while using primitive estimator publish rate of 20Hz
  while (!prim_estimator_odom_buffer_.empty() &&
         Utils::getHeaderStamp(prim_estimator_odom_buffer_.front()->header) < (ros_stamp - 25000000)) {
    prim_estimator_pose = prim_estimator_odom_buffer_.front();
    prim_estimator_odom_buffer_.pop();
  }

  if (!prim_estimator_odom_buffer_.empty()) {
    prim_estimator_pose = prim_estimator_odom_buffer_.front();
    prim_estimator_odom_buffer_.pop();
  }

  return prim_estimator_pose;
}

void Subscriber::getPrimitiveEstimatorPoses(const int64_t& ros_stamp,
                                            std::vector<nav_msgs::msg::Odometry::ConstSharedPtr>& poses) {
  nav_msgs::msg::Odometry::ConstSharedPtr prim_estimator_pose = nullptr;
  // 25ms sync period while using primitive estimator publish rate of 20Hz

  while (!prim_estimator_odom_buffer_.empty() &&
         Utils::getHeaderStamp(prim_estimator_odom_buffer_.front()->header) < (ros_stamp - 25000000)) {
    prim_estimator_odom_buffer_.pop();
  }

  while (!prim_estimator_odom_buffer_.empty()) {
    poses.emplace_back(prim_estimator_odom_buffer_.front());
    prim_estimator_odom_buffer_.pop();
  }
}

void Subscriber::keyframeCallback(const sensor_msgs::msg::Image::ConstSharedPtr kf_image_msg,
                                  const nav_msgs::msg::Odometry::ConstSharedPtr kf_odom,
                                  const sensor_msgs::msg::PointCloud::ConstSharedPtr kf_points,
                                  const okvis_ros::msg::SvinHealth::ConstSharedPtr svin_health) {
  TrackingInfo tracking_info(Utils::getHeaderStamp(kf_odom->header),
                             svin_health->num_tracked_kps,
                             svin_health->new_kps,
                             svin_health->kps_per_quadrant,
                             svin_health->covisibilities,
                             svin_health->response_strengths,
                             svin_health->quality);

  Eigen::Vector3d translation =
      Eigen::Vector3d(kf_odom->pose.pose.position.x, kf_odom->pose.pose.position.y, kf_odom->pose.pose.position.z);
  Eigen::Matrix3d rotation = Eigen::Quaterniond(kf_odom->pose.pose.orientation.w,
                                                kf_odom->pose.pose.orientation.x,
                                                kf_odom->pose.pose.orientation.y,
                                                kf_odom->pose.pose.orientation.z)
                                 .toRotationMatrix();

  std::vector<cv::Point3f> keyframe_points;
  std::vector<cv::KeyPoint> keypoint_observations;
  std::vector<Eigen::Vector3i> point_ids;
  std::vector<int64_t> landmark_ids;
  std::vector<std::vector<int64_t>> kf_covisibilities;

  int64_t keyframe_index = -1;
  cv::Mat kf_image = UtilsOpenCV::readRosImage(kf_image_msg);

  for (unsigned int i = 0; i < kf_points->points.size(); i++) {
    keyframe_index = kf_points->channels[i].values[4];

    cv::Point3f point_3d(kf_points->points[i].x, kf_points->points[i].y, kf_points->points[i].z);
    keyframe_points.push_back(point_3d);

    // @Reloc landmarkId, poseId or MultiFrameId,  keypointIdx
    int64_t landmark_id = kf_points->channels[i].values[0];
    Eigen::Vector3i point_id(
        kf_points->channels[i].values[0], kf_points->channels[i].values[1], kf_points->channels[i].values[2]);
    point_ids.push_back(point_id);

    cv::KeyPoint p_2d_uv;
    p_2d_uv.pt.x = kf_points->channels[i].values[5];
    p_2d_uv.pt.y = kf_points->channels[i].values[6];
    p_2d_uv.size = kf_points->channels[i].values[7];
    p_2d_uv.angle = kf_points->channels[i].values[8];
    p_2d_uv.octave = kf_points->channels[i].values[9];
    p_2d_uv.response = kf_points->channels[i].values[10];
    p_2d_uv.class_id = kf_points->channels[i].values[11];

    keypoint_observations.push_back(p_2d_uv);

    std::vector<int64_t> covisible_kfs;
    for (size_t sz = 12; sz < kf_points->channels[i].values.size(); sz++) {
      int observed_kf_index = kf_points->channels[i].values[sz];
      if (observed_kf_index != keyframe_index) {
        covisible_kfs.push_back(observed_kf_index);
      }
    }
    kf_covisibilities.push_back(covisible_kfs);
  }

  if (keyframe_index != -1) {
    std::unique_ptr<KeyframeInfo> keyframe_info = std::make_unique<KeyframeInfo>(keyframe_index,
                                                                                 kf_image,
                                                                                 translation,
                                                                                 rotation,
                                                                                 tracking_info,
                                                                                 keyframe_points,
                                                                                 keypoint_observations,
                                                                                 point_ids,
                                                                                 kf_covisibilities);

    keyframe_callback_(std::move(keyframe_info));
  } else {
    LOG(WARNING) << "Skipping keyframe. Does not contain any triangulated points.";
  }
}
