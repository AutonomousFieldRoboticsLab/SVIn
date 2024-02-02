#pragma once

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

#include "common/Definitions.h"
#include "pose_graph/Parameters.h"
#include "utils/CameraPoseVisualization.h"

class Publisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Publisher(std::shared_ptr<rclcpp::Node> node, bool debug_mode);

  ~Publisher() = default;

  void kfMatchedPointCloudCallback(const sensor_msgs::msg::PointCloud& pointcloud);
  void publishGlobalMap(const sensor_msgs::msg::PointCloud2& pointcloud);
  void publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                   const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) const;
  void publishPath(const nav_msgs::msg::Path& path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) const;
  void publishOdometry(const nav_msgs::msg::Odometry& odometry,
                       const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub) const;

  void publishKeyframePath(const std::pair<Timestamp, Eigen::Matrix4d>& keyframe_pose,
                           const std::pair<Eigen::Vector3d, Eigen::Vector3d>& loop_closure_edge);
  void publishLoopClosurePath(const std::vector<std::pair<Timestamp, Eigen::Matrix4d>>& loop_closure_poses,
                              const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& loop_closure_edges);
  void updatePublishGlobalMap();

  void setGlobalPointCloudFunction(const PointCloudCallback& global_pointcloud_callback);
  bool savePointCloud(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      const std::shared_ptr<std_srvs::srv::Trigger::Response> response);  // NOLINT

  void saveTrajectory(const std::string& filename) const;
  void publishPrimitiveEstimator(const std::pair<Timestamp, Eigen::Matrix4d>& primitive_estimator_pose);

 private:
  std::shared_ptr<rclcpp::Node> node_;

  bool debug_mode_;  // If true, publish additional topics for debugging
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_gloal_map_;  // Publishes sparse global map deformed after loop closure
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      pub_primitive_estimator_path_;  // Publishes the path of the primitive estimator (debugging)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pub_primitive_odometry_;  // Publishes primitive estimator odometry (debugging)

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      pub_robust_path_;  // Publishes the path of the robust estimator without loop closure (debugging)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pub_robust_odometry_;  // Publishes robust switching odometry (debugging)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_loop_closure_path_;  // Publishes loop closure path
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_visualization_;  // Publishes visualization markers for rviz

  nav_msgs::msg::Path loop_closure_traj_;         // Stores the loop closure path
  nav_msgs::msg::Path primitive_estimator_traj_;  // Stores the primitive estimator path

  std::unique_ptr<CameraPoseVisualization> camera_pose_visualizer_;

  PointCloudCallback pointcloud_callback_;
};
