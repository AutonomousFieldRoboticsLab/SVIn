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
  pub_uber_path_ = nh_private_.advertise<nav_msgs::Path>("uber_path", 2);
  pub_uber_odometry_ = nh_private_.advertise<nav_msgs::Odometry>("uber_odometry", 2);
  pub_prim_odometry_ = nh_private_.advertise<nav_msgs::Odometry>("prim_odometry", 2);
}

void Publisher::kfMatchedPointCloudCallback(const sensor_msgs::PointCloud& msg) { pub_matched_points_.publish(msg); }

void Publisher::publishGlobalMap(const sensor_msgs::PointCloud2& cloud) { pub_gloal_map_.publish(cloud); }

void Publisher::publishPath(const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Publisher& pub) const {
  nav_msgs::Path path;
  path.header.frame_id = poses.back().header.frame_id;
  path.header.stamp = poses.back().header.stamp;
  path.header.seq = poses.size() + 1;
  path.poses = poses;
  pub.publish(path);
}

void Publisher::publishOdometry(const nav_msgs::Odometry& odom, const ros::Publisher& publisher) const {
  publisher.publish(odom);
}
