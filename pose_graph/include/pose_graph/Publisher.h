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
  void publishPath(const std::vector<geometry_msgs::PoseStamped>& prim_estimator_poses,
                   const ros::Publisher& pub) const;
  void publishOdometry(const nav_msgs::Odometry& odometry, const ros::Publisher& pub) const;

 private:
  ros::NodeHandle nh_private_;
  Parameters params_;

  ros::Publisher pub_matched_points_;
  ros::Publisher pub_gloal_map_;

  // TODO(bjoshi): Make this private later and create getters
 public:
  ros::Publisher pub_primitive_estimator_path_;
  ros::Publisher pub_uber_path_;
  ros::Publisher pub_uber_odometry_;
  ros::Publisher pub_prim_odometry_;
};
