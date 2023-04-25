#include "pose_graph/Publisher.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "utils/Utils.h"

Publisher::Publisher(ros::NodeHandle& nh, bool debug_mode) : debug_mode_(debug_mode) {
  // Publishers
  pub_matched_points_ = nh.advertise<sensor_msgs::PointCloud>("match_points", 100);
  pub_gloal_map_ = nh.advertise<sensor_msgs::PointCloud2>("global_map", 2);

  pub_robust_path_ = nh.advertise<nav_msgs::Path>("uber_path", 2);
  pub_robust_odometry_ = nh.advertise<nav_msgs::Odometry>("uber_odometry", 2);

  if (debug_mode_) {
    pub_primitive_estimator_path_ = nh.advertise<nav_msgs::Path>("primitive_estimator_path", 2);
    pub_primitive_odometry_ = nh.advertise<nav_msgs::Odometry>("prim_odometry", 2);
  }

  pub_loop_closure_path_ = nh.advertise<nav_msgs::Path>("loop_closure_path", 100);
  pub_kf_connections_ = nh.advertise<visualization_msgs::MarkerArray>("kf_connections", 1000);

  camera_pose_visualizer_ = std::make_unique<CameraPoseVisualization>(1.0, 0.0, 0.0, 1.0);
  camera_pose_visualizer_->setScale(0.4);
  camera_pose_visualizer_->setLineWidth(0.04);
  pub_visualization_ = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1000);
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

void Publisher::publishKeyframePath(const std::pair<Timestamp, Eigen::Matrix4d>& kf_pose,
                                    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& loop_closure_edge) {
  Eigen::Matrix3d rot = kf_pose.second.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rot);
  Eigen::Vector3d trans = kf_pose.second.block<3, 1>(0, 3);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = Utility::toRosTime(kf_pose.first);
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = trans.x();
  pose_stamped.pose.position.y = trans.y();
  pose_stamped.pose.position.z = trans.z();
  pose_stamped.pose.orientation.x = quat.x();
  pose_stamped.pose.orientation.y = quat.y();
  pose_stamped.pose.orientation.z = quat.z();
  pose_stamped.pose.orientation.w = quat.w();

  loop_closure_traj_.poses.push_back(pose_stamped);
  loop_closure_traj_.header = pose_stamped.header;

  publishPath(loop_closure_traj_, pub_loop_closure_path_);

  camera_pose_visualizer_->clearCameraPoseMarkers();
  camera_pose_visualizer_->add_pose(trans, quat);
  if (!loop_closure_edge.first.isZero() || !loop_closure_edge.second.isZero()) {
    camera_pose_visualizer_->add_loopedge(loop_closure_edge.first, loop_closure_edge.second);
  }
  camera_pose_visualizer_->publish_by(pub_visualization_, pose_stamped.header);
}

void Publisher::publishLoopClosurePath(
    const std::vector<std::pair<Timestamp, Eigen::Matrix4d>>& loop_closure_poses,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& loop_closure_edges) {
  loop_closure_traj_.poses.clear();
  camera_pose_visualizer_->reset();
  for (auto kf_pose : loop_closure_poses) {
    Eigen::Matrix3d rot = kf_pose.second.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rot);
    Eigen::Vector3d trans = kf_pose.second.block<3, 1>(0, 3);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = Utility::toRosTime(kf_pose.first);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = trans.x();
    pose_stamped.pose.position.y = trans.y();
    pose_stamped.pose.position.z = trans.z();
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();

    loop_closure_traj_.poses.push_back(pose_stamped);
    loop_closure_traj_.header = pose_stamped.header;
  }

  for (auto loop_closure_edge : loop_closure_edges) {
    camera_pose_visualizer_->add_loopedge(loop_closure_edge.first, loop_closure_edge.second);
  }

  camera_pose_visualizer_->publish_by(pub_visualization_, loop_closure_traj_.header);
}

void Publisher::setGlobalPointCloudFunction(const PointCloudCallback& global_pointcloud_callback) {
  pointcloud_callback_ = global_pointcloud_callback;
}

void Publisher::updatePublishGlobalMap(const ros::TimerEvent& event) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_callback_(global_map_pcl);
  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(*global_map_pcl, pcl_msg);
  pcl_msg.header.frame_id = "world";
  pcl_msg.header.stamp = ros::Time::now();
  publishGlobalMap(pcl_msg);
}

bool Publisher::savePointCloud(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
  ROS_INFO_STREAM("!! Saving Point Cloud !!");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_callback_(pointcloud);

  std::string pkg_path = ros::package::getPath("pose_graph");
  std::string pointcloud_file = pkg_path + "/reconstruction_results/pointcloud.ply";

  pcl::io::savePLYFileBinary(pointcloud_file, *pointcloud);
  response.success = true;
  response.message = "Saving Point Cloud ";
  return true;
}

void Publisher::saveTrajectory(const std::string& filename) const {
  std::ofstream loop_path_file(filename, std::ios::out);
  loop_path_file.setf(std::ios::fixed, std::ios::floatfield);
  loop_path_file.precision(9);
  loop_path_file << "#timestamp tx ty tz qx qy qz qw" << std::endl;
  for (geometry_msgs::PoseStamped keyframe_pose : loop_closure_traj_.poses) {
    geometry_msgs::Quaternion quat = keyframe_pose.pose.orientation;
    geometry_msgs::Point pos = keyframe_pose.pose.position;
    loop_path_file << keyframe_pose.header.stamp << " " << pos.x << " " << pos.y << " " << pos.z << " " << quat.x << " "
                   << quat.y << " " << quat.z << " " << quat.w << std::endl;
  }
  loop_path_file.close();
}

void Publisher::publishPrimitiveEstimator(const std::pair<Timestamp, Eigen::Matrix4d>& primitive_estimator_pose) {
  Eigen::Matrix3d rot = primitive_estimator_pose.second.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rot);
  Eigen::Vector3d trans = primitive_estimator_pose.second.block<3, 1>(0, 3);

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Pose pose;
  pose_stamped.header.stamp = Utility::toRosTime(primitive_estimator_pose.first);
  pose_stamped.header.frame_id = "world";
  pose.position.x = trans.x();
  pose.position.y = trans.y();
  pose.position.z = trans.z();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  pose_stamped.pose = pose;

  primitive_estimator_traj_.poses.push_back(pose_stamped);
  primitive_estimator_traj_.header = pose_stamped.header;

  publishPath(primitive_estimator_traj_, pub_primitive_estimator_path_);
  nav_msgs::Odometry prim_odom;
  prim_odom.header = pose_stamped.header;
  prim_odom.pose.pose = pose;
  publishOdometry(prim_odom, pub_primitive_odometry_);
}
