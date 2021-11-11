#pragma once

#include <assert.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdio.h>

#include <eigen3/Eigen/Dense>
#include <list>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "DBoW/DBoW2.h"
#include "DBoW/TemplatedDatabase.h"
#include "DBoW/TemplatedVocabulary.h"
#include "DVision/DVision.h"
#include "pose_graph/KFMatcher.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"

#define SHOW_S_EDGE false
#define SHOW_L_EDGE true
#define SAVE_LOOP_PATH true

using namespace DVision;  // NOLINT
using namespace DBoW2;    // NOLINT

class LoopClosing {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoopClosing();
  ~LoopClosing();
  void setPublishers(ros::NodeHandle& n);  // NOLINT
  void addKFToPoseGraph(KFMatcher* cur_kf, bool flag_detect_loop);

  void updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1>& _loop_info);  // NOLINT
  KFMatcher* getKFPtr(int index);

  void setBriefVocAndDB(BriefVocabulary* vocabulary, BriefDatabase database);

  nav_msgs::Path path[10];
  nav_msgs::Path base_path;
  CameraPoseVisualization* posegraph_visualization;

  void publish();
  // Relocalization
  Vector3d t_drift;
  double yaw_drift;
  Matrix3d r_drift;

  Vector3d w_t_svin;
  Matrix3d w_r_svin;

  typedef std::function<void(const uint64_t& time)> EventCallback;

  EventCallback loop_closure_optimization_callback_;
  void registerLoopClosureOptimizationCallback(const EventCallback& finish_optimization_callback);

 private:
  int detectLoop(KFMatcher* keyframe, int frame_index);
  void addKeyFrameIntoVoc(KFMatcher* keyframe);
  void optimize4DoFPoseGraph();
  void updatePath();
  list<KFMatcher*> keyframelist;
  std::mutex kflistMutex_;
  std::mutex optimizationMutex_;
  std::mutex pathMutex_;
  std::mutex driftMutex_;
  std::thread t_optimization;
  std::queue<int> optimizationBuffer_;

  int global_index;
  int sequence_cnt;
  vector<bool> sequence_loop;
  map<int, cv::Mat> image_pool;
  int earliest_loop_index;
  int base_sequence;

  BriefDatabase db;
  BriefVocabulary* voc;

  ros::Publisher pubPoseGraphPath;
  ros::Publisher pubBasePath;
  ros::Publisher pubPoseGraph;
  ros::Publisher pubPath[10];

  std::string svin_output_file_;
  bool is_fast_localization_;

 public:
  void set_svin_results_file(const std::string& svin_output_file);
  void set_fast_relocalization(const bool localization_flag);
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
    return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
    return angle_degrees + T(360.0);
  else
    return angle_degrees;
}

class AngleLocalParameterization {
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
  }
};

template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9]) {
  T y = yaw / T(180.0) * T(M_PI);
  T p = pitch / T(180.0) * T(M_PI);
  T r = roll / T(180.0) * T(M_PI);

  R[0] = cos(y) * cos(p);
  R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
  R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
  R[3] = sin(y) * cos(p);
  R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
  R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
  R[6] = -sin(p);
  R[7] = cos(p) * sin(r);
  R[8] = cos(p) * cos(r);
}

template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9]) {
  inv_R[0] = R[0];
  inv_R[1] = R[3];
  inv_R[2] = R[6];
  inv_R[3] = R[1];
  inv_R[4] = R[4];
  inv_R[5] = R[7];
  inv_R[6] = R[2];
  inv_R[7] = R[5];
  inv_R[8] = R[8];
}

template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3]) {
  r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
  r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
  r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
}

struct FourDOFError {
  FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
      : t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i) {}

  template <typename T>
  bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const {
    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    // euler to rotation
    T w_R_i[9];
    YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
    // rotation transpose
    T i_R_w[9];
    RotationMatrixTranspose(w_R_i, i_R_w);
    // rotation matrix rotate point
    T t_i_ij[3];
    RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x));
    residuals[1] = (t_i_ij[1] - T(t_y));
    residuals[2] = (t_i_ij[2] - T(t_z));
    residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

    return true;
  }

  static ceres::CostFunction* Create(const double t_x,
                                     const double t_y,
                                     const double t_z,
                                     const double relative_yaw,
                                     const double pitch_i,
                                     const double roll_i) {
    return (new ceres::AutoDiffCostFunction<FourDOFError, 4, 1, 3, 1, 3>(
        new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
  }

  double t_x, t_y, t_z;
  double relative_yaw, pitch_i, roll_i;
};

struct FourDOFWeightError {
  FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
      : t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i) {
    weight = 1;
  }

  template <typename T>
  bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const {
    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    // euler to rotation
    T w_R_i[9];
    YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
    // rotation transpose
    T i_R_w[9];
    RotationMatrixTranspose(w_R_i, i_R_w);
    // rotation matrix rotate point
    T t_i_ij[3];
    RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
    residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
    residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
    residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

    return true;
  }

  static ceres::CostFunction* Create(const double t_x,
                                     const double t_y,
                                     const double t_z,
                                     const double relative_yaw,
                                     const double pitch_i,
                                     const double roll_i) {
    return (new ceres::AutoDiffCostFunction<FourDOFWeightError, 4, 1, 3, 1, 3>(
        new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
  }

  double t_x, t_y, t_z;
  double relative_yaw, pitch_i, roll_i;
  double weight;
};
