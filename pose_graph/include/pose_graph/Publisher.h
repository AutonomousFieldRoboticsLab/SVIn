#pragma once

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

 private:
  ros::NodeHandle nh_private_;
  Parameters params_;

  ros::Publisher pub_matched_points_;
};