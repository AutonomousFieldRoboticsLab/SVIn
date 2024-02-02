#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "common/Definitions.h"

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

  std::string vocabulary_file_;

  // for visulization
  double camera_visual_size_;

  double resize_factor_;
  Eigen::Matrix4d T_body_imu_;

  // The first camera in the camera is used as refrence
  CameraCalibration camera_calibration_;

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
  bool getCalibrationViaConfig(CameraCalibration& calibration, cv::FileNode camera_node);
};