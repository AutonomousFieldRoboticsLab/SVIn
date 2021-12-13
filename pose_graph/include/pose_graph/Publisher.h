#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <memory>

#include "pose_graph/Parameters.h"

class Publisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Publisher();
  ~Publisher() = default;

  void setPublishers();
  void setParameters(const Parameters& parameters);

  void kfMatchedPointCloudCallback(const sensor_msgs::PointCloud& pointcloud);
  void publishGlobalMap(const sensor_msgs::PointCloud2& pointcloud);
  void publishPrimitiveEstimatorPath(const std::vector<geometry_msgs::PoseStamped>& prim_estimator_poses);
  void publishOdometry(const nav_msgs::Odometry& odometry);

 private:
  ros::NodeHandle nh_private_;
  Parameters params_;

  ros::Publisher pub_matched_points_;
  ros::Publisher pub_gloal_map_;
  ros::Publisher pub_primitive_estimator_path_;
  ros::Publisher pub_odometry_;
};
