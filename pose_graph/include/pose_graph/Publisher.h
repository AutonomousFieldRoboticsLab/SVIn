#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "common/Definitions.h"
#include "pose_graph/Parameters.h"
#include "utils/CameraPoseVisualization.h"

class Publisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Publisher(ros::NodeHandle& nh_private, bool debug_mode);

  ~Publisher() = default;

  void kfMatchedPointCloudCallback(const sensor_msgs::PointCloud& pointcloud);
  void publishGlobalMap(const sensor_msgs::PointCloud2& pointcloud);
  void publishPath(const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Publisher& pub) const;
  void publishPath(const nav_msgs::Path& path, const ros::Publisher& pub) const;
  void publishOdometry(const nav_msgs::Odometry& odometry, const ros::Publisher& pub) const;

  void publishKeyframePath(const std::pair<Timestamp, Eigen::Matrix4d>& keyframe_pose,
                           const std::pair<Eigen::Vector3d, Eigen::Vector3d>& loop_closure_edge);
  void publishLoopClosurePath(const std::vector<std::pair<Timestamp, Eigen::Matrix4d>>& loop_closure_poses,
                              const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& loop_closure_edges);
  void updatePublishGlobalMap(const ros::TimerEvent& event);

  void setGlobalPointCloudFunction(const PointCloudCallback& global_pointcloud_callback);
  bool savePointCloud(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);  // NOLINT

  void saveTrajectory(const std::string& filename) const;
  void publishPrimitiveEstimator(const std::pair<Timestamp, Eigen::Matrix4d>& primitive_estimator_pose);

 private:
  bool debug_mode_;                    // If true, publish additional topics for debugging
  ros::Publisher pub_matched_points_;  // Publish keyframe matched points
  ros::Publisher pub_gloal_map_;       // Publishes sparse global map deformed after loop closure

  ros::Publisher pub_primitive_estimator_path_;  // Publishes the path of the primitive estimator (debugging)
  ros::Publisher pub_primitive_odometry_;        // Publishes primitive estimator odometry (debugging)

  ros::Publisher pub_robust_path_;        // Publishes the path of the robust estimator without loop closure (debugging)
  ros::Publisher pub_robust_odometry_;    // Publishes robust switching odometry (debugging)
  ros::Publisher pub_loop_closure_path_;  // Publishes loop closure path
  ros::Publisher pub_kf_connections_;     // Publisher keyframe connections
  ros::Publisher pub_visualization_;      // Publishes visualization markers for rviz

  nav_msgs::Path loop_closure_traj_;         // Stores the loop closure path
  nav_msgs::Path primitive_estimator_traj_;  // Stores the primitive estimator path

  std::unique_ptr<CameraPoseVisualization> camera_pose_visualizer_;

  PointCloudCallback pointcloud_callback_;
};
