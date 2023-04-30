#pragma once

#include <Eigen/Core>
#include <deque>

#include "common/Definitions.h"
#include "pose_graph/Parameters.h"
/*
    This class contains functionality related to switching estimator.
*/
class SwitchingEstimator {
 private:
  TrackingStatus tracking_status_;
  HealthParams health_params_;
  Eigen::Matrix4d T_body_imu_;  // Transformation from IMU to body frame
  Eigen::Matrix4d T_imu_cam0_;  // Transformation from left camera (used for Keyframes) to IMU frame

  double forward_velocity_;  // Forward velocity of the vehicle
  double lateral_velocity_;  // Lateral velocity of the vehicle

  uint16_t consecutive_tracking_failures_;
  uint16_t consecutive_tracking_successes_;

  Eigen::Matrix4d init_t_w_prim_;
  Eigen::Matrix4d init_t_w_vio_;

  // All the pose are in Body Frame for uniformity
  Eigen::Matrix4d switch_t_w_prim_;
  Eigen::Matrix4d switch_t_w_vio_;
  Eigen::Matrix4d switch_t_w_robust_;

  Eigen::Matrix4d current_primitive_pose_;
  Eigen::Matrix4d current_vio_pose_;
  Eigen::Matrix4d current_robust_pose_;

  Timestamp last_vio_keyframe_time_ = -1;
  Timestamp last_primitive_estimator_time_ = -1;
  uint32_t primitive_estimator_kfs_ = 0;
  uint32_t latest_kf_index = 0;

  bool update_vio_keyframe_pose_ = false;
  bool update_primitive_estimator_pose_ = false;

  std::deque<std::pair<Timestamp, Eigen::Matrix4d>> primitive_estimator_poses_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SwitchingEstimator(Parameters& params);
  ~SwitchingEstimator() = default;

  void addKeyframeInfo(KeyframeInfo& keyframe_info);

  void performHealthCheck(TrackingInfo& tracking_info, std::string& status_message);
  bool checkTrackingInfo(TrackingInfo& tracking_info, std::string& status_message);

  void updateVIOKeyframePose(Timestamp time, Eigen::Vector3d& translation, Eigen::Matrix3d& rotation);
  void addPrimitiveEstimatorPose(Timestamp timestamp, Eigen::Matrix4d& primitive_estimator_pose);

  bool getRobustPose(Timestamp& stamp, Eigen::Vector3d& translation, Eigen::Matrix3d& rotation);
  bool getLatestPrimitiveEstimatorPose(std::pair<Timestamp, Eigen::Matrix4d>& primitive_pose);
  uint32_t getPrimitiveKFCount();

  void getPrimitiveEstimatorPoses(std::vector<std::pair<Timestamp, Eigen::Matrix4d>>& primitive_poses);
};
