#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <string>

class Parameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Parameters();
  ~Parameters() = default;

  Eigen::Vector3d tic_;
  Eigen::Matrix3d qic_;

  std::string brief_pattern_file_;

  std::string svin_w_loop_path_;

  int fast_relocalization_;

  int min_loop_num_;

  // projection matrix
  double p_fx;
  double p_fy;
  double p_cx;
  double p_cy;

  std::vector<double> distortion_coeffs_;

  bool use_health_;
  std::string vocabulary_file_;

  // for visulization
  double camera_visual_size_;

 public:
  void loadParameters(const ros::NodeHandle& nh);
};