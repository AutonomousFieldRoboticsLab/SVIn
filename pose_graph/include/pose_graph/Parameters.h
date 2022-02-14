#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

struct LoopClosureParams {
  bool loop_closure_enabled;
  double pnp_reprojection_thresh;
  double pnp_ransac_iterations;
  int min_correspondences;
};

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
  double resize_factor_;
  Eigen::Matrix4d T_imu_cam0_;
  Eigen::Matrix4d T_body_imu_;

  uint16_t tracked_kypoints_threshold_;
  double wait_for_keyframe_time_;
  uint16_t consecutive_good_keyframes_threshold_;

  bool debug_image_;
  double image_delay_;

  LoopClosureParams loop_closure_params_;

 public:
  void loadParameters(const ros::NodeHandle& nh);
};
