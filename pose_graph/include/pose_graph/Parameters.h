#pragma once

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

struct LoopClosureParams {
  bool enabled;
  double pnp_reprojection_thresh;
  double pnp_ransac_iterations;
  int min_correspondences;
};

struct HealthParams {
  bool enabled = false;  // by default, health monitoring is disabled
  uint16_t consecutive_keyframes;
  uint16_t min_tracked_keypoints;
  uint16_t kps_per_quadrant;
  float kf_wait_time;
};

struct GlobalMappingParams {
  bool enabled = false;  // by default global mapping is disabled
  double min_lmk_quality = 0.001;
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
  double p_fx_;
  double p_fy_;
  double p_cx_;
  double p_cy_;

  std::string vocabulary_file_;

  // for visulization
  double camera_visual_size_;

  cv::Mat distortion_coeffs_;
  uint16_t image_width_;
  uint16_t image_height_;
  double resize_factor_;
  Eigen::Matrix4d T_imu_cam0_;
  Eigen::Matrix4d T_body_imu_;

  bool debug_mode_;
  std::string debug_output_path_;

  double image_delay_;

  // loop closure parameters
  LoopClosureParams loop_closure_params_;

  // health monitoring parameters
  HealthParams health_params_;

  // Global Mapping Parameters
  GlobalMappingParams global_mapping_params_;

 public:
  void loadParameters(const std::string& config_file);
};
