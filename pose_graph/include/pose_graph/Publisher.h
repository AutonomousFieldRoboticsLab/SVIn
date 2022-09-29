#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "pose_graph/Parameters.h"

class Publisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Publisher(ros::NodeHandle& nh_private);
  ~Publisher() = default;

  void kfMatchedPointCloudCallback(const sensor_msgs::PointCloud& pointcloud);
  void publishGlobalMap(const sensor_msgs::PointCloud2& pointcloud);
  void publishPath(const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Publisher& pub) const;
  void publishPath(const nav_msgs::Path& path, const ros::Publisher& pub) const;
  void publishOdometry(const nav_msgs::Odometry& odometry, const ros::Publisher& pub) const;

  void publishKeyframePath(const std::pair<ros::Time, Eigen::Matrix4d>& keyframe_pose);
  void publishLoopClosurePath(const std::vector<std::pair<ros::Time, Eigen::Matrix4d>>& loop_closure_poses);

 private:
  ros::Publisher pub_matched_points_;  // Publish keyframe matched points
  ros::Publisher pub_gloal_map_;       // Publishes sparse global map deformed after loop closure

  ros::Publisher pub_primitive_estimator_path_;  // Publishes the path of the primitive estimator (debugging)
  ros::Publisher pub_robust_path_;      // Publishes the path of the robust estimator without loop closure (debugging)
  ros::Publisher pub_robust_odometry_;  // Publishes robust switching odometry (debugging)
  ros::Publisher pub_primitive_odometry_;  // Publishes primitive estimator odometry (debugging)
  ros::Publisher pub_loop_closure_path_;   // Publishes loop closure path
  ros::Publisher pub_kf_connections_;      // Publisher keyframe connections

  nav_msgs::Path loop_closure_path_;
};
