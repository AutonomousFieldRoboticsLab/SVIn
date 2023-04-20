#include "pose_graph/GlobalMapping.h"

#include <ros/console.h>

#include <utility>

std::ostream& operator<<(std::ostream& os, const Landmark& landmark) {
  os << "Landmark: " << landmark.id_ << " at " << landmark.point_.transpose() << " with quality: " << landmark.quality_;
  os << "Total obs: " << landmark.keyframe_observations_.size()
     << "with latest at kf_indx: " << landmark.latest_kf_obs_;
  return os;
}

void Landmark::updateObservation(uint64_t keyframe_id,
                                 const Eigen::Vector3d& position,
                                 double quality,
                                 const Eigen::Vector3d& color) {
  if (keyframe_observations_.find(keyframe_id) != keyframe_observations_.end()) {
    std::cout << "Multiple observation of landmark: " << id_ << " in keyframe: " << keyframe_id << std::endl;
    return;
  }

  Observation observation(keyframe_id, position, quality, color);
  keyframe_observations_.insert(std::make_pair(keyframe_id, observation));
  latest_kf_obs_ = keyframe_id;
}

GlobalMap::GlobalMap() {
  loop_closure_optimization_finished_ = false;
  last_loop_closure_optimization_time_ = 0;
}

void GlobalMap::addLandmark(const Eigen::Vector3d& global_pos,
                            uint64_t landmark_id,
                            double quality,
                            uint64_t keyframe_id,
                            const Eigen::Vector3d& local_pos,
                            const Eigen::Vector3d& color) {
  if (!map_points_.count(landmark_id)) {  // If the point is not in the map, add it.
    Landmark point_landmark = Landmark(landmark_id, global_pos, quality, color);
    point_landmark.updateObservation(keyframe_id, local_pos, quality, color);
    map_points_.insert(std::make_pair(landmark_id, point_landmark));
  } else {
    Landmark& point_landmark = map_points_.at(landmark_id);
    point_landmark.color_ = color;
    point_landmark.point_ = global_pos;
    point_landmark.quality_ = quality;
    point_landmark.updateObservation(keyframe_id, local_pos, quality, color);
  }
}

void GlobalMap::updateLandmark(uint64_t landmark_id,
                               const Eigen::Vector3d& global_pos,
                               double quality,
                               const Eigen::Vector3d& color) {
  if (map_points_.count(landmark_id)) {
    Landmark& point_landmark = map_points_.at(landmark_id);
    point_landmark.color_ = color;
    point_landmark.point_ = global_pos;
    point_landmark.quality_ = quality;
  }
}

void GlobalMap::loopClosureOptimizationFinishCallback(const Timestamp optimization_finish_time) {
  last_loop_closure_optimization_time_ = optimization_finish_time;
  loop_closure_optimization_finished_ = true;
  // LOG(WARN) << "Loop Closure Optimization finished at: " << optimization_finish_time);
}
