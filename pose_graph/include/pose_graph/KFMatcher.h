#pragma once

#include <brisk/brisk.h>
#include <brisk/internal/hamming.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "DBoW/DBoW2.h"
#include "DVision/DVision.h"
#include "pose_graph/Parameters.h"
#include "utils/Utils.h"
#include "utils/tic_toc.h"

class BriefExtractor {
 public:
  virtual void operator()(const cv::Mat& im,
                          vector<cv::KeyPoint>& keys,                          // NOLINT
                          vector<DVision::BRIEF::bitset>& descriptors) const;  // NOLINT
  explicit BriefExtractor(const std::string& pattern_file);

  DVision::BRIEF m_brief;
};

class KFMatcher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KFMatcher(double _time_stamp,
            vector<Eigen::Vector3d>& _point_ids,  // NOLINT
            int _index,
            Eigen::Vector3d& _svin_T_w_i,        // NOLINT
            Eigen::Matrix3d& _svin_R_w_i,        // NOLINT
            cv::Mat& _image,                     // NOLINT
            vector<cv::Point3f>& _point_3d,      // NOLINT
            vector<cv::KeyPoint>& _point_2d_uv,  // NOLINT
            map<KFMatcher*, int>& KFcounter,     // NOLINT
            int _sequence,
            BriefVocabulary* vocBrief,
            const Parameters& params,
            const bool vio_keyframe = true);

  KFMatcher(double _time_stamp,
            int _index,
            Eigen::Vector3d& _svin_T_w_i,     // NOLINT
            Eigen::Matrix3d& _svin_R_w_i,     // NOLINT
            map<KFMatcher*, int>& KFcounter,  // NOLINT
            int _sequence,
            const Parameters& params,
            const bool is_vio_keyframe = false);

  bool findConnection(KFMatcher* old_kf);
  void computeWindowBRIEFPoint();
  void computeBRIEFPoint();

  int HammingDis(const DVision::BRIEF::bitset& a, const DVision::BRIEF::bitset& b);
  bool searchInAera(const DVision::BRIEF::bitset window_descriptor,
                    const std::vector<DVision::BRIEF::bitset>& descriptors_old,
                    const std::vector<cv::KeyPoint>& keypoints_old,
                    const std::vector<cv::KeyPoint>& keypoints_old_norm,
                    cv::Point2f& best_match,                            // NOLINT
                    cv::Point2f& best_match_norm);                      // NOLINT
  void searchByBRIEFDes(std::vector<cv::Point2f>& matched_2d_old,       // NOLINT
                        std::vector<cv::Point2f>& matched_2d_old_norm,  // NOLINT
                        std::vector<uchar>& status,                     // NOLINT
                        const std::vector<DVision::BRIEF::bitset>& descriptors_old,
                        const std::vector<cv::KeyPoint>& keypoints_old,
                        const std::vector<cv::KeyPoint>& keypoints_old_norm);
  void PnPRANSAC(const vector<cv::Point2f>& matched_2d_old_norm,
                 const std::vector<cv::Point3f>& matched_3d,
                 std::vector<uchar>& status,                           // NOLINT
                 Eigen::Vector3d& PnP_T_old,                           // NOLINT
                 Eigen::Matrix3d& PnP_R_old);                          // NOLINT
  void getSVInPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i);  // NOLINT
  void getPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i);      // NOLINT
  void updatePose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i);
  void updateSVInPose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i);  // NOLINT
  void updateLoop(Eigen::Matrix<double, 8, 1>& _loop_info);                           // NOLINT

  Eigen::Vector3d getLoopRelativeT();
  double getLoopRelativeYaw();
  Eigen::Quaterniond getLoopRelativeQ();

  void computeBoW();

  void project_normal(Eigen::Vector2d kp, Eigen::Vector3d& point3d) const;  // NOLINT

  void updateConnections();
  int searchByBoW(KFMatcher* old_kf, vector<cv::Point3f>& vpMatches12, vector<bool>& vbMatched2);  // NOLINT

  // For BRISK Descriptor
  void searchByBRISKDescriptor(std::vector<cv::Point2f>& matched_2d_old,  // NOLINT
                               std::vector<uchar>& status,                // NOLINT
                               const cv::Mat& descriptors_old,
                               const std::vector<cv::KeyPoint>& keypoints_old);
  bool matchBrisk(const cv::Mat& window_descriptor,
                  const cv::Mat& descriptors_old,
                  const std::vector<cv::KeyPoint>& keypoints_old,
                  cv::Point2f& best_match);  // NOLINT
  void computeBRISKPoint();
  double brisk_distance(const cv::Mat& a, const cv::Mat& b);
  std::vector<cv::KeyPoint> brisk_keypoints;
  cv::Mat brisk_descriptors;
  cv::Mat window_brisk_descriptors;

  double time_stamp;
  int index;
  int local_index;
  Eigen::Vector3d svin_T_w_i;
  Eigen::Matrix3d svin_R_w_i;
  Eigen::Vector3d T_w_i;
  Eigen::Matrix3d R_w_i;
  Eigen::Vector3d origin_svin_T;
  Eigen::Matrix3d origin_svin_R;
  cv::Mat image;
  vector<cv::Point3f> point_3d;
  vector<cv::KeyPoint> point_2d_uv;
  vector<Eigen::Vector3d> point_ids_;

  vector<cv::KeyPoint> keypoints;
  vector<cv::KeyPoint> keypoints_norm;
  vector<cv::KeyPoint> window_keypoints_norm;
  vector<cv::KeyPoint> window_keypoints;
  vector<DVision::BRIEF::bitset> brief_descriptors;
  vector<DVision::BRIEF::bitset> window_brief_descriptors;
  bool has_fast_point;
  int sequence;

  bool has_loop;
  int loop_index;
  Eigen::Matrix<double, 8, 1> loop_info;

  map<KFMatcher*, int> KFcounter_;
  map<KFMatcher*, int> mConnectedKeyFrameWeights;

  // BoW
  BriefVocabulary* voc;
  DBoW2::BowVector bowVec;
  DBoW2::FeatureVector featVec;

  static const int TH_LOW;
  static const int TH_HIGH;

  // In OKVIS, these parameters are read from config file. It's same as config_StereoRigV2.yaml.
  static const size_t briskDetectionOctaves_;             ///< The set number of brisk octaves.
  static const double briskDetectionThreshold_;           ///< The set BRISK detection threshold.
  static const double briskDetectionAbsoluteThreshold_;   ///< The set BRISK absolute detection threshold.
  static const size_t briskDetectionMaximumKeypoints_;    ///< The set maximum number of keypoints.
  static const bool briskDescriptionRotationInvariance_;  ///< The set rotation invariance setting.
  static const bool briskDescriptionScaleInvariance_;     ///< The set scale invariance setting.
  static const double briskMatchingThreshold_;            ///< The set BRISK matching threshold.

  Parameters params_;
  // ros::Publisher pubMatchedPoints;

  typedef std::function<void(const sensor_msgs::PointCloud& pointcloud)> PointCloudCallback;

  void setRelocalizationPCLCallback(const PointCloudCallback& pointcloud_callback);
  PointCloudCallback relocalization_pcl_callback_;

  // Bharat - easy check to see if this keyframe comes from primitive estimator
  bool is_vio_keyframe_;
};
