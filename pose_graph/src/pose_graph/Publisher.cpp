#include "pose_graph/Publisher.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <boost/filesystem.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

#include "utils/Utils.h"

Publisher::Publisher(std::shared_ptr<rclcpp::Node> node, bool debug_mode) : node_(node), debug_mode_(debug_mode) {
  // Publishers
  // pub_matched_points_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("match_points", 100);
  pub_gloal_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 2);

  pub_robust_path_ = node_->create_publisher<nav_msgs::msg::Path>("uber_path", 2);
  pub_robust_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("uber_odometry", 2);

  if (debug_mode_) {
    pub_primitive_estimator_path_ = node_->create_publisher<nav_msgs::msg::Path>("primitive_estimator_path", 2);
    pub_primitive_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("prim_odometry", 2);
  }

  pub_loop_closure_path_ = node_->create_publisher<nav_msgs::msg::Path>("loop_closure_path", 100);
  // pub_kf_connections_ = nh.advertise<visualization_msgs::msg::MarkerArray>("kf_connections", 1000);

  camera_pose_visualizer_ = std::make_unique<CameraPoseVisualization>(1.0, 0.0, 0.0, 1.0);
  camera_pose_visualizer_->setScale(0.4);
  camera_pose_visualizer_->setLineWidth(0.04);
  pub_visualization_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization", 100);
}

// void Publisher::kfMatchedPointCloudCallback(const sensor_msgs::msg::PointCloud& msg) {
//   pub_matched_points_->publish(msg);
// }

void Publisher::publishGlobalMap(const sensor_msgs::msg::PointCloud2& cloud) { pub_gloal_map_->publish(cloud); }

void Publisher::publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                            const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = poses.back().header.frame_id;
  path.header.stamp = poses.back().header.stamp;
  path.poses = poses;
  pub->publish(path);
}

void Publisher::publishPath(const nav_msgs::msg::Path& path,
                            const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher) const {
  publisher->publish(path);
}

void Publisher::publishOdometry(const nav_msgs::msg::Odometry& odom,
                                const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher) const {
  publisher->publish(odom);
}

void Publisher::publishKeyframePath(const std::pair<Timestamp, Eigen::Matrix4d>& kf_pose,
                                    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& loop_closure_edge) {
  Eigen::Matrix3d rot = kf_pose.second.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rot);
  Eigen::Vector3d trans = kf_pose.second.block<3, 1>(0, 3);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = Utils::toRosTime(kf_pose.first);
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
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = Utils::toRosTime(kf_pose.first);
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

void Publisher::updatePublishGlobalMap() {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_callback_(global_map_pcl);
  sensor_msgs::msg::PointCloud2 pcl_msg;
  pcl::toROSMsg(*global_map_pcl, pcl_msg);
  pcl_msg.header.frame_id = "world";
  pcl_msg.header.stamp = node_->now();
  publishGlobalMap(pcl_msg);
}

bool Publisher::savePointCloud(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(node_->get_logger(), "!! Saving Point Cloud !!");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_callback_(pointcloud);

  // Determine source directory of this file
  std::filesystem::path this_file(__FILE__);
  std::filesystem::path posegraph_dir = this_file.parent_path().parent_path().parent_path();
  std::string posegraph_dir_str = posegraph_dir.string();
  std::string pointcloud_dir = posegraph_dir_str + "/reconstruction_results";
  // std::string pointcloud_dir = "/tmp/reconstruction_results/pointcloud.ply";
  if (!std::filesystem::exists(pointcloud_dir)) {
    std::filesystem::create_directories(pointcloud_dir);
  }

  // Generate timestamped filename
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  localtime_r(&now_time, &now_tm);
  std::stringstream filename_ss;
  filename_ss << pointcloud_dir << "/pointcloud_"
              << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S") << ".ply";
  std::string pointcloud_file = filename_ss.str();

  // Save the point cloud to a PLY file
  // Save point cloud
  int ret = pcl::io::savePLYFileBinary(pointcloud_file, *pointcloud);
  if (ret < 0) {
    response->success = false;
    response->message = "Failed to save PLY file.";
    RCLCPP_ERROR(node_->get_logger(), "Failed to save point cloud to: %s", pointcloud_file.c_str());
    return true;
  }

  response->success = true;
  response->message = "Saved to: " + pointcloud_file;
  RCLCPP_INFO(node_->get_logger(), "PointCloud saved to: %s", pointcloud_file.c_str());
  return true;
}

void Publisher::saveTrajectory(const std::string& filename) const {
  std::ofstream loop_path_file(filename, std::ios::out);
  loop_path_file.setf(std::ios::fixed, std::ios::floatfield);
  loop_path_file.precision(9);
  loop_path_file << "#timestamp tx ty tz qx qy qz qw" << std::endl;
  for (geometry_msgs::msg::PoseStamped keyframe_pose : loop_closure_traj_.poses) {
    geometry_msgs::msg::Quaternion quat = keyframe_pose.pose.orientation;
    geometry_msgs::msg::Point pos = keyframe_pose.pose.position;
    loop_path_file << keyframe_pose.header.stamp.sec << "." << keyframe_pose.header.stamp.nanosec << " " << pos.x << " "
                   << pos.y << " " << pos.z << " " << quat.x << " " << quat.y << " " << quat.z << " " << quat.w
                   << std::endl;
  }
  loop_path_file.close();
}

bool Publisher::saveKeyframes(const std::string& filename, const std::vector<KeyframeDump>& keyframes) const {
  std::ofstream ofs(filename, std::ios::out);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open keyframes file: %s", filename.c_str());
    return false;
  }
  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.precision(9);
  ofs << "#ID, timestamp, qx, qy, qz, qw, tx, ty, tz" << std::endl;
  for (const auto& kf : keyframes) {
    // Convert Timestamp (ns) to sec.nsec with 9-digit zero-padded nanoseconds
    const uint64_t sec_part = static_cast<uint64_t>(kf.stamp) / 1000000000ULL;
    const uint64_t nsec_part = static_cast<uint64_t>(kf.stamp) % 1000000000ULL;
    ofs << kf.id << ", " << sec_part << "." << std::setw(9) << std::setfill('0') << nsec_part
        << std::setfill(' ')  // reset fill for subsequent fields
        << ", " << kf.qx << ", " << kf.qy << ", " << kf.qz << ", " << kf.qw
        << ", " << kf.tx << ", " << kf.ty << ", " << kf.tz << std::endl;
  }
  ofs.close();
  RCLCPP_INFO(node_->get_logger(), "Keyframes saved to: %s", filename.c_str());
  return true;
}

void Publisher::publishPrimitiveEstimator(const std::pair<Timestamp, Eigen::Matrix4d>& primitive_estimator_pose) {
  Eigen::Matrix3d rot = primitive_estimator_pose.second.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rot);
  Eigen::Vector3d trans = primitive_estimator_pose.second.block<3, 1>(0, 3);

  geometry_msgs::msg::PoseStamped pose_stamped;
  geometry_msgs::msg::Pose pose;
  pose_stamped.header.stamp = Utils::toRosTime(primitive_estimator_pose.first);
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
  nav_msgs::msg::Odometry prim_odom;
  prim_odom.header = pose_stamped.header;
  prim_odom.pose.pose = pose;
  publishOdometry(prim_odom, pub_primitive_odometry_);
}
