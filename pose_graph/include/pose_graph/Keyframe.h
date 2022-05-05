#ifndef POSE_GRAPH_KEYFRAME_INFORMATION_H_
#define POSE_GRAPH_KEYFRAME_INFORMATION_H_

#include <Eigen/Core>
#include <iostream>
#include <opencv2/core.hpp>


struct TrackingInfo {
 public:
  TrackingInfo() = default;

  TrackingInfo(const Timestamp& timestamp,
               const bool is_tracking_ok,
               const uint32_t num_tracked_keypoints,
               const uint32_t num_new_keypoints,
               const std::vector<uint32_t>& keypoints_per_quadrant,
               const std::vector<int32_t>& covisibilities,
               const std::vector<double>& keypoints_response_strengths,
               const std::vector<double>& points_quality)
      : is_tracking_ok_(is_tracking_ok_),
        num_tracked_keypoints_(num_tracked_keypoints),
        num_new_keypoints_(num_new_keypoints),
        keypoints_per_quadrant_(keypoints_per_quadrant),
        covisibilities_(covisibilities),
        keypoints_response_strengths_(keypoints_response_strengths),
        points_quality_(points_quality){};

  bool is_tracking_ok_;
  uint32_t num_tracked_keypoints_;
  uint32_t num_new_keypoints_;
  std::vector<uint32_t> keypoints_per_quadrant_;
  std::vector<int32_t> covisibilities_;
  std::vector<double> keypoints_response_strengths_;
  std::vector<double> points_quality_;
};

struct KeyframeInfo {
 public:
  KeyframeInfo() = default;

  KeyframeInfo(const cv::Mat& image,
               const Eigen::Vector3d& translation,
               const Eigen::Matrix3d rotation,
               const TrackingInfo& tracking_info,
               const std::vector<Eigen::Vector3d>& points_3d,
               const std::vector<cv::KeyPoint>& keypoints,
               const std::vector<Eigen::Vector3d>& points_ids)
      : translation_(translation),
        rotation_(rotation),
        tracking_info_(tracking_info),
        points_3d_(points_3d),
        keypoints_(keypoints),
        point_ids_(points_ids){};

  KeyframeInfo(const int64_t timestamp,
               const cv::Mat& image,
               const Eigen::Vector3d& translation,
               const Eigen::Matrix3d rotation,
               const std::vector<Eigen::Vector3d>& points_3d,
               const std::vector<cv::KeyPoint>& keypoints,
               const std::vector<Eigen::Vector3d>& points_ids)
      : timestamp_(timestamp),
        translation_(translation),
        rotation_(rotation),
        points_3d_(points_3d),
        keypoints_(keypoints),
        point_ids_(points_ids){};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_;
  cv::Mat keyframe_image_;
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;
  TrackingInfo tracking_info_;
  std::vector<Eigen::Vector3d> points_3d_;
  std::vector<cv::KeyPoint> keypoints_;
  // @Reloc: landmarkId, mfId, keypointIdx related to each point
  std::vector<Eigen::Vector3d> point_ids_;
};

#endif  // #POSE_GRAPH_KEYFRAME_INFORMATION_H_
