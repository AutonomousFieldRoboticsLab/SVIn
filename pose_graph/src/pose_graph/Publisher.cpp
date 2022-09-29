#include "pose_graph/Publisher.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

Publisher::Publisher(ros::NodeHandle& nh) {
  // Publishers
  pub_matched_points_ = nh.advertise<sensor_msgs::PointCloud>("match_points", 100);
  pub_gloal_map_ = nh.advertise<sensor_msgs::PointCloud2>("global_map", 2);

  pub_primitive_estimator_path_ = nh.advertise<nav_msgs::Path>("primitive_estimator_path", 2);
  pub_robust_path_ = nh.advertise<nav_msgs::Path>("uber_path", 2);
  pub_robust_odometry_ = nh.advertise<nav_msgs::Odometry>("uber_odometry", 2);
  pub_primitive_odometry_ = nh.advertise<nav_msgs::Odometry>("prim_odometry", 2);

  pub_loop_closure_path_ = nh.advertise<nav_msgs::Path>("loop_closure_path", 100);
  pub_kf_connections_ = nh.advertise<visualization_msgs::MarkerArray>("kf_connections", 1000);
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

void Publisher::publishPath(const nav_msgs::Path& path, const ros::Publisher& publisher) const {
  publisher.publish(path);
}

void Publisher::publishOdometry(const nav_msgs::Odometry& odom, const ros::Publisher& publisher) const {
  publisher.publish(odom);
}

void Publisher::publishKeyframePath(const std::pair<ros::Time, Eigen::Matrix4d>& kf_pose) {
  Eigen::Matrix3d rot = kf_pose.second.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rot);
  Eigen::Vector3d trans = kf_pose.second.block<3, 1>(0, 3);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = kf_pose.first;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = trans.x();
  pose_stamped.pose.position.y = trans.y();
  pose_stamped.pose.position.z = trans.z();
  pose_stamped.pose.orientation.x = quat.x();
  pose_stamped.pose.orientation.y = quat.y();
  pose_stamped.pose.orientation.z = quat.z();
  pose_stamped.pose.orientation.w = quat.w();

  loop_closure_path_.poses.push_back(pose_stamped);
  loop_closure_path_.header = pose_stamped.header;

  publishPath(loop_closure_path_, pub_loop_closure_path_);
}

void Publisher::publishLoopClosurePath(const std::vector<std::pair<ros::Time, Eigen::Matrix4d>>& loop_closure_poses) {
  loop_closure_path_.poses.clear();
  for (auto kf_pose : loop_closure_poses) {
    Eigen::Matrix3d rot = kf_pose.second.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rot);
    Eigen::Vector3d trans = kf_pose.second.block<3, 1>(0, 3);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = kf_pose.first;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = trans.x();
    pose_stamped.pose.position.y = trans.y();
    pose_stamped.pose.position.z = trans.z();
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();

    loop_closure_path_.poses.push_back(pose_stamped);
    loop_closure_path_.header = pose_stamped.header;
  }
}
