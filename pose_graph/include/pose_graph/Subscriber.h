#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <okvis_ros/msg/svin_health.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <string>
#include <vector>

#include "common/Definitions.h"
#include "pose_graph/Parameters.h"

class Subscriber {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Subscriber(std::shared_ptr<rclcpp::Node> node, Parameters& params);  // NOLINT
  Subscriber();                                                        // NOLINT

  ~Subscriber() = default;

  nav_msgs::msg::Odometry::ConstSharedPtr getPrimitiveEstimatorPose(const uint64_t& ros_stamp);
  void getPrimitiveEstimatorPoses(const uint64_t& ros_stamp,
                                  std::vector<nav_msgs::msg::Odometry::ConstSharedPtr>& poses);  // NOLINT

 private:
  std::mutex measurement_mutex_;

  std::shared_ptr<rclcpp::Node> node_;  // The node handle of the node that will subscribe to the topic.

  Parameters params_;  // The parameters of the node.

  // Subscriber to the keyframe points, image, pose and svin health.
  message_filters::Subscriber<sensor_msgs::msg::Image> keyframe_image_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> keyframe_pose_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud> keyframe_points_subscriber_;
  message_filters::Subscriber<okvis_ros::msg::SvinHealth> svin_health_subscriber_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                    nav_msgs::msg::Odometry,
                                                    sensor_msgs::msg::PointCloud,
                                                    okvis_ros::msg::SvinHealth>
      keyframe_sync_policy;

  std::unique_ptr<message_filters::Synchronizer<keyframe_sync_policy>> sync_keyframe_;

  // List of other subscribers.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      sub_svin_relocalization_odom_;  // Subscriber to the relocalization odometry.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_orig_image_;  // Subscriber to the original image.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      sub_primitive_estimator_;  // Subscriber to the primitive estimator odometry.

  // Callback functions.
  void svinRelocalizationOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void primitiveEstimatorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void keyframeCallback(const sensor_msgs::msg::Image::ConstSharedPtr kf_image_msg,
                        const nav_msgs::msg::Odometry::ConstSharedPtr kf_odom,
                        const sensor_msgs::msg::PointCloud::ConstSharedPtr kf_points,
                        const okvis_ros::msg::SvinHealth::ConstSharedPtr svin_health);

  std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> prim_estimator_odom_buffer_;

  // Topic names.
  std::string kf_image_topic_;             // The topic name of the keyframe image.
  std::string kf_pose_topic_;              // The topic name of the keyframe pose.
  std::string kf_points_topic_;            // The topic name of the keyframe points.
  std::string svin_reloc_odom_topic_;      // The topic name of the relocalization odometry.
  std::string svin_health_topic_;          // The topic name of the health of the SVIn.
  std::string primitive_estimator_topic_;  // The topic name of the primitive estimator odometry.
  std::string raw_image_topic_;            // The topic name of the original image.

  double last_image_time_;                // The time of the last image.
  double last_primitive_estimator_time_;  // The time of the last primitive estimator odometry.

  KeyframeCallback keyframe_callback_;          // The callback function for the keyframe.
  CVMatCallback primitive_estimator_callback_;  // The callback function for the pose.
  CVMatCallback raw_image_callback_;            // The callback function for the original_image.

 public:
  inline double getLatestPrimitiveEstimatorTime() const { return last_primitive_estimator_time_; }
  inline void registerKeyframeCallback(const KeyframeCallback& callback) { keyframe_callback_ = callback; }
  inline void registerImageCallback(const CVMatCallback& callback) { raw_image_callback_ = callback; }
  inline void registerPrimitiveEstimatorCallback(const CVMatCallback& callback) {
    primitive_estimator_callback_ = callback;
  }  // NOLINT
};
