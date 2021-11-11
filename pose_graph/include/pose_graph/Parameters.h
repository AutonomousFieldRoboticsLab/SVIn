#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

class Parameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

  bool use_health_;
  std::string vocabulary_file_;

  // for visulization
  double camera_visual_size_;

  cv::Mat distortion_coeffs_;
  uint16_t image_width_;
  uint16_t image_height_;
  cv::Mat cam0_undistort_map_x_, cam0_undistort_map_y_;

 public:
  void loadParameters(const ros::NodeHandle& nh);
};
