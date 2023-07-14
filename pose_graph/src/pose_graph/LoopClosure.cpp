#include "pose_graph/LoopClosure.h"

#include <Eigen/SVD>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <map>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <utility>
#include <vector>

#include "utils/Statistics.h"
#include "utils/UtilsOpenCV.h"

LoopClosure::LoopClosure(Parameters& params)
    : params_(params),
      pose_graph_(nullptr),
      global_map_(nullptr),
      keyframe_tracking_queue_("keyframe_queue"),
      raw_image_buffer_(kBufferLengthNs),
      primitive_estimator_poses_buffer_(kBufferLengthNs) {
  frame_index_ = 0;
  last_translation_ = Eigen::Vector3d(-100, -100, -100);

  setup();
  shutdown_ = false;
}

void LoopClosure::setup() {
  pose_graph_ = std::unique_ptr<PoseGraph>(new PoseGraph());

  pose_graph_->set_fast_relocalization(params_.fast_relocalization_);

  if (params_.global_mapping_params_.enabled) {
    global_map_ = std::unique_ptr<GlobalMap>(new GlobalMap());
    pose_graph_->setLoopClosureOptimizationCallback(
        std::bind(&GlobalMap::loopClosureOptimizationFinishCallback, global_map_.get(), std::placeholders::_1));
  }

  if (params_.health_params_.enabled) {
    switching_estimator_ = std::unique_ptr<SwitchingEstimator>(new SwitchingEstimator(params_));
  }

  pose_graph_->startOptimizationThread();

  // Loading vocabulary
  voc_ = new BriefVocabulary(params_.vocabulary_file_);
  BriefDatabase db;
  db.setVocabulary(*voc_, false, 0);
  LOG(INFO) << "Vocabulary loaded!";
  pose_graph_->setBriefVocAndDB(voc_, db);
}

void LoopClosure::run() {
  while (!shutdown_) {
    std::unique_ptr<KeyframeInfo> keyframe_info = nullptr;
    bool queue_state = keyframe_tracking_queue_.popBlocking(keyframe_info);
    if (queue_state) {
      CHECK(keyframe_info);

      std::map<Keyframe*, int> KFcounter;

      for (size_t i = 0; i < keyframe_info->keyfame_points_.size(); ++i) {
        double quality = static_cast<double>(keyframe_info->tracking_info_.points_quality_[i]);
        for (auto observed_kf_index : keyframe_info->point_covisibilities_[i]) {
          observed_kf_index += prim_estimator_keyframes_;
          if (kfMapper_.find(observed_kf_index) != kfMapper_.end()) {
            Keyframe* observed_kf =
                kfMapper_.find(observed_kf_index)->second;  // Keyframe where this point_3d has been observed
            KFcounter[observed_kf]++;
          }
        }
      }

      Keyframe* keyframe = new Keyframe(keyframe_info->timestamp_.toNSec(),
                                        keyframe_info->keypoint_ids_,
                                        keyframe_info->keyframe_index_,
                                        keyframe_info->translation_,
                                        keyframe_info->rotation_,
                                        keyframe_info->keyframe_image_,
                                        keyframe_info->keyfame_points_,
                                        keyframe_info->cv_keypoints_,
                                        KFcounter,
                                        sequence_,
                                        voc_,
                                        params_);
      kfMapper_.insert(std::make_pair(keyframe_info->keyframe_index_, keyframe));
      pose_graph_->addKFToPoseGraph(keyframe, 1);

      if (params_.global_mapping_params_.enabled) {
        cv::Mat original_color_image;
        if (!raw_image_buffer_.getNearestValueToTime(keyframe_info->timestamp_.toNSec(), &original_color_image)) {
          LOG(WARNING) << "Could not find color image for keyframe with timestamp "
                       << keyframe_info->timestamp_.toNSec();
        } else {
          if (params_.resize_factor_ != 0) {
            cv::resize(original_color_image,
                       original_color_image,
                       cv::Size(params_.image_width_, params_.image_height_),
                       cv::INTER_LINEAR);
          }
          if (kfMapper_.find(keyframe_info->keyframe_index_) != kfMapper_.end()) {
            addPointsToGlobalMap(keyframe_info->keyframe_index_,
                                 original_color_image,
                                 keyframe_info->rotation_,
                                 keyframe_info->translation_,
                                 keyframe_info->keyfame_points_,
                                 keyframe_info->tracking_info_.points_quality_,
                                 keyframe_info->keypoint_ids_,

                                 keyframe_info->cv_keypoints_);
          } else {
            LOG(WARNING) << "Keyframe not found";
          }
        }
      }
    }
    frame_index_++;
  }
}

void LoopClosure::getGlobalMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud) {
  // only update the global map if the pose graph optimization is finished after loop closure
  if (global_map_->loop_closure_optimization_finished_) {
    updateGlobalMap();
    global_map_->loop_closure_optimization_finished_ = false;
  }
  getGlobalPointCloud(pointcloud);
}

void LoopClosure::getGlobalPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud) {
  for (auto point_landmark_map : global_map_->getMapPoints()) {
    Landmark point_landmark = point_landmark_map.second;
    Eigen::Vector3d global_pos = point_landmark.point_;
    Eigen::Vector3d color = point_landmark.color_;
    double quality = point_landmark.quality_;

    if (quality > params_.global_mapping_params_.min_lmk_quality) {
      pcl::PointXYZRGB point;
      point.x = global_pos(0);
      point.y = global_pos(1);
      point.z = global_pos(2);
      point.r = static_cast<uint8_t>(color.x());
      point.g = static_cast<uint8_t>(color.y());
      point.b = static_cast<uint8_t>(color.z());
      pointcloud->push_back(point);
    }
  }
}

void LoopClosure::addPointsToGlobalMap(const int64_t keyframe_index,
                                       const cv::Mat& color_image,
                                       const Eigen::Matrix3d& camera_rotation,
                                       const Eigen::Vector3d& camera_translation,
                                       const std::vector<cv::Point3f>& keyframe_points,
                                       const std::vector<float>& point_qualities,
                                       const std::vector<Eigen::Vector3i>& point_ids,
                                       const std::vector<cv::KeyPoint>& cv_keypoints) {
  for (size_t i = 0; i < keyframe_points.size(); ++i) {
    float quality = point_qualities[i];
    if (quality < params_.global_mapping_params_.min_lmk_quality) continue;
    Eigen::Vector3d global_point_position(keyframe_points[i].x, keyframe_points[i].y, keyframe_points[i].z);
    Eigen::Vector3d point_cam_frame = camera_rotation.transpose() * (global_point_position - camera_translation);

    Keyframe* kf = kfMapper_.find(keyframe_index)->second;
    Eigen::Matrix3d R_w_kf;
    Eigen::Vector3d T_w_kf;
    kf->getPose(T_w_kf, R_w_kf);
    global_point_position = R_w_kf * point_cam_frame + T_w_kf;

    cv::KeyPoint image_point = cv_keypoints[i];
    cv::Vec3b color =
        color_image.at<cv::Vec3b>(static_cast<uint16_t>(image_point.pt.y), static_cast<uint16_t>(image_point.pt.x));
    // bgr to rgb
    Eigen::Vector3d color_eigen(
        static_cast<double>(color[2]), static_cast<double>(color[1]), static_cast<double>(color[0]));
    uint64_t landmark_id = static_cast<uint64_t>(point_ids[i].x());

    global_map_->addLandmark(global_point_position, landmark_id, quality, keyframe_index, point_cam_frame, color_eigen);
  }
}

void LoopClosure::updateGlobalMap() {
  for (auto point_landmark_map : global_map_->getMapPoints()) {
    uint64_t landmark_id = point_landmark_map.first;
    Landmark point_landmark = point_landmark_map.second;

    Eigen::Vector3d point_3d = Eigen::Vector3d::Zero();
    Eigen::Vector3d color = Eigen::Vector3d::Zero();
    double quality = 0.0;
    uint64_t total_observations = 0;
    for (auto kf_observation : point_landmark.keyframe_observations_) {
      uint64_t kf_id = kf_observation.first;
      Observation obs = kf_observation.second;
      Eigen::Vector3d local_pos = obs.local_pos_;
      double kf_quality = obs.quality_;

      // Converting to global coordinates
      if (kfMapper_.find(kf_id) == kfMapper_.end()) {
        LOG(WARNING) << "Keyframe not found";
        continue;
      }

      // The keyframe poses are already updated in PoseGraph Optimization as
      // pointers are passed
      Keyframe* kf = kfMapper_.find(kf_id)->second;
      Eigen::Matrix3d R_kf_w;
      Eigen::Vector3d T_kf_w;
      kf->getPose(T_kf_w, R_kf_w);

      Eigen::Vector3d global_pos = R_kf_w * local_pos + T_kf_w;
      point_3d = point_3d + global_pos * kf_quality;
      color = color + obs.color_ * kf_quality;
      quality = quality + kf_quality;
      total_observations += 1;
    }

    point_3d = point_3d / quality;
    color = color / quality;
    quality = quality / total_observations;

    global_map_->updateLandmark(landmark_id, point_3d, quality, color);
  }
}

void LoopClosure::updatePrimiteEstimatorTrajectory(const nav_msgs::OdometryConstPtr& pose_msg) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = pose_msg->header;
  pose_stamped.header.seq = primitive_estimator_poses_.size() + 1;
  pose_stamped.pose = Utility::matrixToRosPose(init_t_w_svin_ * init_t_w_prim_.inverse() *
                                               Utility::rosPoseToMatrix(pose_msg->pose.pose) * params_.T_body_imu_ *
                                               params_.T_imu_cam0_);
  // primitive_estimator_poses_.push_back(pose_stamped);
}

bool LoopClosure::healthCheck(const TrackingInfo& tracking_info, boost::optional<std::string> error_message) {
  std::stringstream ss;
  std::setprecision(5);

  HealthParams health_params = params_.health_params_;
  if (tracking_info.num_tracked_keypoints_ < health_params.min_tracked_keypoints) {
    ss << "Not enough triangulated keypoints: " << tracking_info.num_tracked_keypoints_ << std::endl;
    error_message = ss.str();
    return false;
  }

  std::vector<int> keypoints_per_quadrant = tracking_info.keypoints_per_quadrant_;

  bool quadrant_check = std::all_of(keypoints_per_quadrant.begin(), keypoints_per_quadrant.end(), [&](int kp_count) {
    return kp_count >= health_params.kps_per_quadrant;
  });

  if (!quadrant_check && *std::max_element(keypoints_per_quadrant.begin(), keypoints_per_quadrant.end()) <=
                             10.0 * health_params.kps_per_quadrant) {
    ss << "Not enough keypoints per quadrant:  [" << keypoints_per_quadrant[0] << ", " << keypoints_per_quadrant[1]
       << ", " << keypoints_per_quadrant[2] << ", " << keypoints_per_quadrant[3] << "]" << std::endl;
    error_message = ss.str();
    return false;
  }

  float new_detected_keypoints_ratio =
      static_cast<float>(tracking_info.num_new_keypoints_) / static_cast<float>(tracking_info.num_tracked_keypoints_);

  if (new_detected_keypoints_ratio >= 0.75) {
    if (error_message) {
      ss << "Too many new keypoints: " << new_detected_keypoints_ratio << std::endl;
      error_message = ss.str();
      return false;
    }
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
    if (error_message) {
      ss << "Too many detectors with low response: " << fraction_with_low_detector_response << std::endl;
      error_message = ss.str();
      return false;
    }
  }

  return true;
}

void LoopClosure::shutdown() {
  LOG_IF(ERROR, shutdown_) << "Shutdown requested, but PoseGraph modile was already shutdown.";
  keyframe_tracking_queue_.shutdown();
  shutdown_ = true;
  LOG(INFO) << "Shutting down PoseGraph module.";
}

void LoopClosure::setKeyframePoseCallback(const KeframeWithLoopClosureCallback& keyframe_pose_callback) {
  pose_graph_->setKeyframePoseCallback(keyframe_pose_callback);
}
void LoopClosure::setLoopClosureCallback(const PathWithLoopClosureCallback& loop_closure_callback) {
  pose_graph_->setLoopClosureCallback(loop_closure_callback);
}