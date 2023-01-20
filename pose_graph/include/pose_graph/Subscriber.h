#pragma once

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/Odometry.h>
#include <okvis_ros/SvinHealth.h>
#include <sensor_msgs/PointCloud.h>

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "common/Definitions.h"
#include "pose_graph/Parameters.h"

class Subscriber {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Subscriber(ros::NodeHandle& nh, Parameters& params);  // NOLINT
  Subscriber() {}

  ~Subscriber() = default;

  void setNodeHandle(ros::NodeHandle& nh);  // NOLINT

  nav_msgs::OdometryConstPtr getPrimitiveEstimatorPose(const uint64_t& ros_stamp);
  void getPrimitiveEstimatorPoses(const uint64_t& ros_stamp, std::vector<nav_msgs::OdometryConstPtr>& poses);  // NOLINT

 private:
  std::mutex measurement_mutex_;

  ros::NodeHandle* nh_;  // The node handle of the node that will subscribe to the topic.
  std::unique_ptr<image_transport::ImageTransport> it_;  // The image transport.

  Parameters params_;  // The parameters of the node.

  // Subscriber to the keyframe points, image, pose and svin health.
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber keyframe_image_subscriber_;
  message_filters::Subscriber<nav_msgs::Odometry> keyframe_pose_subscriber_;
  message_filters::Subscriber<sensor_msgs::PointCloud> keyframe_points_subscriber_;
  message_filters::Subscriber<okvis_ros::SvinHealth> svin_health_subscriber_;

  typedef message_filters::sync_policies::
      ExactTime<sensor_msgs::Image, nav_msgs::Odometry, sensor_msgs::PointCloud, okvis_ros::SvinHealth>
          keyframe_sync_policy;

  std::unique_ptr<message_filters::Synchronizer<keyframe_sync_policy>> sync_keyframe_;

  // List of other subscribers.
  ros::Subscriber sub_svin_relocalization_odom_;  // Subscriber to the relocalization odometry.
  image_transport::Subscriber sub_orig_image_;    // Subscriber to the original image.
  ros::Subscriber sub_primitive_estimator_;       // Subscriber to the primitive estimator odometry.

  // Callback functions.
  void svinRelocalizationOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void primitiveEstimatorCallback(const nav_msgs::OdometryConstPtr& msg);
  void keyframeCallback(const sensor_msgs::ImageConstPtr& kf_image_msg,
                        const nav_msgs::OdometryConstPtr& kf_odom,
                        const sensor_msgs::PointCloudConstPtr& kf_points,
                        const okvis_ros::SvinHealthConstPtr& svin_health);

  std::queue<nav_msgs::OdometryConstPtr> prim_estimator_odom_buffer_;

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

  KeyframeCallback keyframe_callback_;         // The callback function for the keyframe.
  PoseCallback primitive_estimator_callback_;  // The callback function for the pose.
  ImageCallback raw_image_callback_;           // The callback function for the original_image.

 public:
  inline double getLatestPrimitiveEstimatorTime() const { return last_primitive_estimator_time_; }
  inline void registerKeyframeCallback(const KeyframeCallback& callback) { keyframe_callback_ = callback; }
  inline void registerImageCallback(const ImageCallback& callback) { raw_image_callback_ = callback; }
};
