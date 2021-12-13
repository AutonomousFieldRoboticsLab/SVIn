#include "pose_graph/Publisher.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <vector>

Publisher::Publisher() : nh_private_("~") {}

void Publisher::setParameters(const Parameters& params) { params_ = params; }

void Publisher::setPublishers() {
  // Publishers
  pub_matched_points_ = nh_private_.advertise<sensor_msgs::PointCloud>("match_points", 100);
  pub_gloal_map_ = nh_private_.advertise<sensor_msgs::PointCloud2>("global_map", 2);

  pub_primitive_estimator_path_ = nh_private_.advertise<nav_msgs::Path>("primitive_estimator_path", 2);
  pub_odometry_ = nh_private_.advertise<nav_msgs::Odometry>("debug_odometry", 2);
}

void Publisher::kfMatchedPointCloudCallback(const sensor_msgs::PointCloud& msg) { pub_matched_points_.publish(msg); }

void Publisher::publishGlobalMap(const sensor_msgs::PointCloud2& cloud) { pub_gloal_map_.publish(cloud); }

void Publisher::publishPrimitiveEstimatorPath(const std::vector<geometry_msgs::PoseStamped>& prim_estimator_poses) {
  nav_msgs::Path path;
  path.header.frame_id = prim_estimator_poses.back().header.frame_id;
  path.header.stamp = prim_estimator_poses.back().header.stamp;
  path.header.seq = prim_estimator_poses.size() + 1;
  path.poses = prim_estimator_poses;
  pub_primitive_estimator_path_.publish(path);
}

void Publisher::publishOdometry(const nav_msgs::Odometry& odom) { pub_odometry_.publish(odom); }