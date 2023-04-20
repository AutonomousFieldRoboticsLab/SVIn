#pragma once

#include <Eigen/Core>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "common/Definitions.h"

struct Observation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Observation(uint64_t kf_indx, const Eigen::Vector3d& local_pos, double quality, const Eigen::Vector3d& color)
      : kf_indx_(kf_indx), local_pos_(local_pos), quality_(quality), color_(color) {}

  Observation() : kf_indx_(0), local_pos_(Eigen::Vector3d::Zero()), quality_(0), color_(Eigen::Vector3d::Zero()) {}

  uint64_t kf_indx_;
  Eigen::Vector3d local_pos_;
  double quality_;
  Eigen::Vector3d color_;
};

struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Constructor.
   * @param id        ID of the point. E.g. landmark ID.
   * @param point     Homogeneous coordinate of the point.
   * @param quality   Quality of the point. Usually between 0 and 1.
   */
  Landmark(uint64_t id, const Eigen::Vector3d& point, double quality, const Eigen::Vector3d& color)
      : id_(id), point_(point), quality_(quality), color_(color) {}
  Landmark() : id_(0), point_(Eigen::Vector3d::Zero()), quality_(0), color_(Eigen::Vector3d::Zero()) {}

  uint64_t id_;            //< ID of the point. E.g. landmark ID.
  Eigen::Vector3d point_;  //< coordinate of the point.
  double quality_;         //< Quality of the point. Usually between 0 and 1.
  Eigen::Vector3d color_;  //< Color of the point.

  std::unordered_map<uint64_t, Observation> keyframe_observations_;  // < Keyframe index, observation

  uint64_t latest_kf_obs_;

  void updateObservation(uint64_t keyframe_id,
                         const Eigen::Vector3d& position,
                         double quality,
                         const Eigen::Vector3d& color);
};

std::ostream& operator<<(std::ostream& os, const Landmark& landmark);

typedef std::unordered_map<uint64_t,
                           Landmark,
                           std::hash<uint64_t>,
                           std::equal_to<uint64_t>,
                           Eigen::aligned_allocator<std::pair<const uint64_t, Landmark>>>
    LandmarkMap;

class GlobalMap {
 private:
  // Map of all points.

  LandmarkMap map_points_;
  uint64_t last_loop_closure_optimization_time_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GlobalMap();
  virtual ~GlobalMap() = default;

  void addLandmark(const Eigen::Vector3d& global_pos,
                   uint64_t landmark_id,
                   double quality,
                   uint64_t keyframe_id,
                   const Eigen::Vector3d& local_pos,
                   const Eigen::Vector3d& color);

  void updateLandmark(uint64_t landmark_id,
                      const Eigen::Vector3d& global_pos,
                      double quality,
                      const Eigen::Vector3d& color);

  LandmarkMap getMapPoints() const { return map_points_; }

  void loopClosureOptimizationFinishCallback(const Timestamp time);
  bool loop_closure_optimization_finished_;  // this flag is set to true when loop closure optimization is done
};
