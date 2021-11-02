#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "parameters.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"

#include <brisk/brisk.h>
#include <brisk/internal/hamming.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

using namespace Eigen;
using namespace std;
using namespace DVision;

class BriefExtractor {
 public:
  virtual void operator()(const cv::Mat& im, vector<cv::KeyPoint>& keys, vector<BRIEF::bitset>& descriptors) const;
  BriefExtractor(const std::string& pattern_file);

  DVision::BRIEF m_brief;
};

class KFMatcher {
 public:
  KFMatcher(double _time_stamp,
            vector<Eigen::Vector3d>& _point_ids,
            int _index,
            Vector3d& _svin_T_w_i,
            Matrix3d& _svin_R_w_i,
            cv::Mat& _image,
            vector<cv::Point3f>& _point_3d,
            vector<cv::KeyPoint>& _point_2d_uv,
            map<KFMatcher*, int>& KFcounter,
            int _sequence,
            BriefVocabulary* vocBrief);

  bool findConnection(KFMatcher* old_kf);
  void computeWindowBRIEFPoint();
  void computeBRIEFPoint();

  int HammingDis(const BRIEF::bitset& a, const BRIEF::bitset& b);
  bool searchInAera(const BRIEF::bitset window_descriptor,
                    const std::vector<BRIEF::bitset>& descriptors_old,
                    const std::vector<cv::KeyPoint>& keypoints_old,
                    const std::vector<cv::KeyPoint>& keypoints_old_norm,
                    cv::Point2f& best_match,
                    cv::Point2f& best_match_norm);
  void searchByBRIEFDes(std::vector<cv::Point2f>& matched_2d_old,
                        std::vector<cv::Point2f>& matched_2d_old_norm,
                        std::vector<uchar>& status,
                        const std::vector<BRIEF::bitset>& descriptors_old,
                        const std::vector<cv::KeyPoint>& keypoints_old,
                        const std::vector<cv::KeyPoint>& keypoints_old_norm);
  void PnPRANSAC(const vector<cv::Point2f>& matched_2d_old_norm,
                 const std::vector<cv::Point3f>& matched_3d,
                 std::vector<uchar>& status,
                 Eigen::Vector3d& PnP_T_old,
                 Eigen::Matrix3d& PnP_R_old);
  void getSVInPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i);
  void getPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i);
  void updatePose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i);
  void updateSVInPose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i);
  void updateLoop(Eigen::Matrix<double, 8, 1>& _loop_info);

  Eigen::Vector3d getLoopRelativeT();
  double getLoopRelativeYaw();
  Eigen::Quaterniond getLoopRelativeQ();

  void computeBoW();

  void project_normal(Eigen::Vector2d kp, Eigen::Vector3d& point3d) const;

  void updateConnections();
  int searchByBoW(KFMatcher* old_kf, vector<cv::Point3f>& vpMatches12, vector<bool>& vbMatched2);

  // For BRISK Descriptor
  void searchByBRISKDescriptor(std::vector<cv::Point2f>& matched_2d_old,
                               std::vector<uchar>& status,
                               const cv::Mat& descriptors_old,
                               const std::vector<cv::KeyPoint>& keypoints_old);
  bool matchBrisk(const cv::Mat& window_descriptor,
                  const cv::Mat& descriptors_old,
                  const std::vector<cv::KeyPoint>& keypoints_old,
                  cv::Point2f& best_match);

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
  vector<BRIEF::bitset> brief_descriptors;
  vector<BRIEF::bitset> window_brief_descriptors;
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
};
