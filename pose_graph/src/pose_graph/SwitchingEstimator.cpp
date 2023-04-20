#include "pose_graph/SwitchingEstimator.h"

#include <glog/logging.h>

#include <algorithm>
#include <numeric>

SwitchingEstimator::SwitchingEstimator(Parameters& params) {
  health_params_ = params.health_params_;
  T_body_imu_ = params.T_body_imu_;
  T_imu_cam0_ = params.T_imu_cam0_;

  init_t_w_prim_ = Eigen::Matrix4d::Zero();
  switch_t_w_prim_ = Eigen::Matrix4d::Zero();

  tracking_status_ = TrackingStatus::NOT_INITIALIZED;

  consecutive_tracking_failures_ = 0;
  consecutive_tracking_successes_ = 0;
}

void SwitchingEstimator::addKeyframeInfo(KeyframeInfo& keyframe_info) {
  std::string message;
  bool vio_health_ok = performHealthCheck(keyframe_info.tracking_info_, message);
  updateVIOKeyframePose(keyframe_info.timestamp_.toNSec(), keyframe_info.translation_, keyframe_info.rotation_);
}

bool SwitchingEstimator::performHealthCheck(TrackingInfo& tracking_info, std::string& status_message) {
  bool tracking_info_check = checkTrackingInfo(tracking_info, status_message);
  if (!tracking_info_check) {
    consecutive_tracking_failures_ += 1;
    consecutive_tracking_successes_ = 0;
  } else {
    consecutive_tracking_successes_ += 1;
    consecutive_tracking_failures_ = 0;
  }
  return tracking_info_check;
}

bool SwitchingEstimator::checkTrackingInfo(TrackingInfo& tracking_info, std::string& error_message) {
  std::stringstream ss;
  std::setprecision(5);

  if (tracking_info.num_tracked_keypoints_ < health_params_.min_tracked_keypoints) {
    ss << "Not enough triangulated keypoints: " << tracking_info.num_tracked_keypoints_ << std::endl;
    error_message = ss.str();
    return false;
  }

  std::vector<int> keypoints_per_quadrant = tracking_info.keypoints_per_quadrant_;

  bool quadrant_check = std::all_of(keypoints_per_quadrant.begin(), keypoints_per_quadrant.end(), [&](int kp_count) {
    return kp_count >= health_params_.kps_per_quadrant;
  });

  if (!quadrant_check && *std::max_element(keypoints_per_quadrant.begin(), keypoints_per_quadrant.end()) <=
                             10.0 * health_params_.kps_per_quadrant) {
    ss << "Not enough keypoints per quadrant:  [" << keypoints_per_quadrant[0] << ", " << keypoints_per_quadrant[1]
       << ", " << keypoints_per_quadrant[2] << ", " << keypoints_per_quadrant[3] << "]" << std::endl;
    error_message = ss.str();
    return false;
  }

  float new_detected_keypoints_ratio =
      static_cast<float>(tracking_info.num_new_keypoints_) / static_cast<float>(tracking_info.num_tracked_keypoints_);

  if (new_detected_keypoints_ratio >= 0.75) {
    ss << "Too many new keypoints: " << new_detected_keypoints_ratio << std::endl;
    error_message = ss.str();
    return false;
  }

  double average_response = std::accumulate(tracking_info.keypoints_response_strengths_.begin(),
                                            tracking_info.keypoints_response_strengths_.end(),
                                            double(0.0)) /
                            static_cast<double>(tracking_info.keypoints_response_strengths_.size());
  float fraction_with_low_detector_response =
      std::count_if(tracking_info.keypoints_response_strengths_.begin(),
                    tracking_info.keypoints_response_strengths_.end(),
                    [&](double response) { return response < average_response; }) /
      static_cast<float>(tracking_info.keypoints_response_strengths_.size());

  if (fraction_with_low_detector_response >= 0.85) {
    ss << "Too many detectors with low response: " << fraction_with_low_detector_response << std::endl;
    error_message = ss.str();
    return false;
  }

  return true;
}

void SwitchingEstimator::addPrimitiveEstimatorPose(Timestamp timestamp, Eigen::Matrix4d& primitive_estimator_pose) {
  // Frame of reference is different for Robust Estimator and Primitive Estimator
  // The primitive estimator is at water depth from origin with arbitrary yaw
  // Needs to be in the body frame same as the VIO

  if (tracking_status_ == TrackingStatus::NOT_INITIALIZED) {
    init_t_w_prim_ = primitive_estimator_pose;
    switch_t_w_prim_ = primitive_estimator_pose;
    current_primitive_pose_ = init_t_w_vio_ * init_t_w_prim_.inverse() * primitive_estimator_pose;
    last_primitive_estimator_time_ = timestamp;
  } else if (timestamp > last_primitive_estimator_time_) {
    current_primitive_pose_ = init_t_w_vio_ * init_t_w_prim_.inverse() * primitive_estimator_pose;
    last_primitive_estimator_time_ = timestamp;
  }
}

void SwitchingEstimator::updateVIOKeyframePose(Timestamp timestamp,
                                               Eigen::Vector3d& translation,
                                               Eigen::Matrix3d& rotation) {
  if (tracking_status_ == TrackingStatus::NOT_INITIALIZED) {
    Eigen::Matrix4d t_w_vio_cam = Eigen::Matrix4d::Identity();
    t_w_vio_cam.block<3, 3>(0, 0) = rotation;
    t_w_vio_cam.block<3, 1>(0, 3) = translation;

    // Pose of body frame in World frame using VIO
    Eigen::Matrix4d t_w_vio_body = t_w_vio_cam * T_imu_cam0_.inverse() * T_body_imu_.inverse();
    init_t_w_vio_ = t_w_vio_body;
    switch_t_w_vio_ = t_w_vio_body;
    switch_t_w_robust_ = t_w_vio_body;  // Initially using VIO as robust estimator
    current_vio_pose_ = t_w_vio_body;
    current_robust_pose_ = t_w_vio_body;
    last_vio_keyframe_time_ = timestamp;
    tracking_status_ = TrackingStatus::TRACKING_VIO;
  } else {
  }
}

bool SwitchingEstimator::getRobustPose(Timestamp time, Eigen::Matrix4d& robust_pose) {
  // If OKVIS do not produce keyframe for a while, switch to
  // primitive estimator
  if (static_cast<float>(time - last_vio_keyframe_time_) * 1e-9 > health_params_.kf_wait_time &&
      tracking_status_ != TrackingStatus::NOT_INITIALIZED) {
    if (tracking_status_ == TrackingStatus::TRACKING_VIO) {
      switch_t_w_vio_ = current_vio_pose_;
      switch_t_w_prim_ = current_primitive_pose_;
      switch_t_w_robust_ = current_robust_pose_;

      tracking_status_ = TrackingStatus::TRACKING_PRIMITIVE_ESTIMATOR;
      LOG(INFO) << "!!!!!!!!Switching to Primitive Estimator !!!!!!!!!!";
    }

    current_robust_pose_ = switch_t_w_robust_ * switch_t_w_prim_.inverse() * current_primitive_pose_;
  } else {
  }

  robust_pose = current_robust_pose_ * T_body_imu_ * T_imu_cam0_;

  return true;
}
