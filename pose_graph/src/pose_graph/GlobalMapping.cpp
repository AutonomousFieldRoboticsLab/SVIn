#include "pose_graph/GlobalMapping.h"

#include <glog/logging.h>

#include <utility>
#include <algorithm>

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
    LOG(WARNING) << "Multiple observation of landmark: " << id_ << " in keyframe: " << keyframe_id;
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

bool GlobalMap::saveKeyframeObservationsToFile(const std::string& file_path) const {
  std::ofstream ofs(file_path, std::ios::out);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Failed to open file for writing keyframe observations: " << file_path;
    return false;
  }
  ofs << "# landmark_id, [landmark_x, landmark_y, landmark_z], [keyframe_id1, keyframe_id2, ...]" << std::endl;

  for (const auto& kv : map_points_) {
    const uint64_t landmark_id = kv.first;
    const Landmark& lm = kv.second;
    // Collect and sort keyframe ids for deterministic output
    std::vector<uint64_t> observing_kf_ids;
    observing_kf_ids.reserve(lm.keyframe_observations_.size());
    for (const auto& obs_kv : lm.keyframe_observations_) {
      observing_kf_ids.push_back(obs_kv.first);
    }
    std::sort(observing_kf_ids.begin(), observing_kf_ids.end());

    ofs << landmark_id << ", [" 
        << lm.point_.x() << ", " 
        << lm.point_.y() << ", " 
        << lm.point_.z() << "], [";
    for (size_t i = 0; i < observing_kf_ids.size(); ++i) {
      ofs << observing_kf_ids[i];
      if (i + 1 < observing_kf_ids.size()) ofs << ", ";
    }
    ofs << "]" << std::endl;
  }

  ofs.close();
  LOG(INFO) << "Saved keyframe observations to: " << file_path;
  return true;
}
