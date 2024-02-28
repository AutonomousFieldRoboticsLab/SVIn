#include "pose_graph/Parameters.h"

#include <glog/logging.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <filesystem>
#include <fstream>
#include <string>

#include "utils/Utils.h"

Parameters::Parameters() {
  // default values
  tic_ = Eigen::Vector3d::Zero();
  qic_ = Eigen::Matrix3d::Identity();
  health_params_.min_tracked_keypoints = 8;
  health_params_.kf_wait_time = 0.5;
  health_params_.consecutive_keyframes = 5;
  debug_mode_ = false;
  image_delay_ = 0.0;

  // Enable loop closure by default
  loop_closure_params_.enabled = true;
  loop_closure_params_.min_correspondences = 25;
  loop_closure_params_.pnp_reprojection_thresh = 20.0;
  loop_closure_params_.pnp_ransac_iterations = 100;
  resize_factor_ = 1.0;
}

void Parameters::loadParameters(const std::string& config_file) {
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

  if (!fsSettings.isOpened()) {
    LOG(FATAL) << "ERROR: Wrong path to settings" << std::endl;
  }

  camera_visual_size_ = fsSettings["visualize_camera_size"];

  std::string pkg_path = ros::package::getPath("pose_graph");

  vocabulary_file_ = pkg_path + "/Vocabulary/brief_k10L6.bin";
  std::cout << "vocabulary_file" << vocabulary_file_ << std::endl;

  brief_pattern_file_ = pkg_path + "/Vocabulary/brief_pattern.yml";

  if (fsSettings["loop_closure_params"]["enable"].isInt()) {
    loop_closure_params_.enabled = static_cast<int>(fsSettings["loop_closure_params"]["enable"]);
    LOG(INFO) << "loop_closure_params.enable: " << loop_closure_params_.enabled;

    if (fsSettings["loop_closure_params"]["min_correspondences"].isInt() ||
        fsSettings["loop_closure_params"]["min_correspondences"].isReal()) {
      loop_closure_params_.min_correspondences =
          static_cast<int>(fsSettings["loop_closure_params"]["min_correspondences"]);
      LOG(INFO) << "Num of matched keypoints for Loop Detection:" << loop_closure_params_.min_correspondences;
    }

    if (fsSettings["loop_closure_params"]["pnp_reprojection_threshold"].isReal() ||
        fsSettings["loop_closure_params"]["pnp_reprojection_threshold"].isInt()) {
      loop_closure_params_.pnp_reprojection_thresh =
          static_cast<double>(fsSettings["loop_closure_params"]["pnp_reprojection_threshold"]);
      LOG(INFO) << "PnP reprojection threshold: " << loop_closure_params_.pnp_reprojection_thresh;
    }

    if (fsSettings["loop_closure_params"]["pnp_ransac_iterations"].isInt() ||
        fsSettings["loop_closure_params"]["pnp_ransac_iterations"].isReal()) {
      loop_closure_params_.pnp_ransac_iterations =
          static_cast<int>(fsSettings["loop_closure_params"]["pnp_ransac_iterations"]);
      LOG(INFO) << "PnP ransac iterations: " << loop_closure_params_.pnp_ransac_iterations;
    }
    if (fsSettings["loop_closure_params"]["keyframe_queue"].isInt() ||
        fsSettings["loop_closure_params"]["keyframe_queue"].isReal()) {
      loop_closure_params_.keyframe_queue_size = static_cast<int>(fsSettings["loop_closure_params"]["keyframe_queue"]);
      LOG(INFO) << "Keyframe Queue Size: " << loop_closure_params_.keyframe_queue_size;
    }
    if (fsSettings["loop_closure_params"]["max_yaw_diff"].isInt() ||
        fsSettings["loop_closure_params"]["max_yaw_diff"].isReal()) {
      loop_closure_params_.max_relative_yaw = static_cast<int>(fsSettings["loop_closure_params"]["max_yaw_diff"]);
      LOG(INFO) << "Max Relative Yaw: " << loop_closure_params_.max_relative_yaw;
    }
    if (fsSettings["loop_closure_params"]["max_position_diff"].isInt() ||
        fsSettings["loop_closure_params"]["max_position_diff"].isReal()) {
      loop_closure_params_.max_relative_distance =
          static_cast<int>(fsSettings["loop_closure_params"]["max_position_diff"]);
      LOG(INFO) << "Max Relative Distance: " << loop_closure_params_.max_relative_distance;
    }
  }

  if (fsSettings["output_params"]["output_dir"].isString()) {
    output_path_ = static_cast<std::string>(fsSettings["output_params"]["output_dir"]);
    debug_output_path_ = output_path_ + "/debug_output";
    LOG(INFO) << "Output folder: " << output_path_;
    if (fsSettings["output_params"]["debug"].isInt()) {
      debug_mode_ = static_cast<int>(fsSettings["output_params"]["debug"]);
    }
  }

  if (fsSettings["global_map_params"]["enable"].isInt()) {
    global_mapping_params_.enabled = static_cast<int>(fsSettings["global_map_params"]["enable"]);
    LOG(INFO) << "global_map.enable: " << global_mapping_params_.enabled;

    if (fsSettings["global_map_params"]["min_landmark_quality"].isInt() ||
        fsSettings["global_map_params"]["min_landmark_quality"].isReal()) {
      global_mapping_params_.min_lmk_quality =
          static_cast<double>(fsSettings["global_map_params"]["min_landmark_quality"]);
      LOG(INFO) << "Minimum landmark quality to add to global map:" << global_mapping_params_.min_lmk_quality;
    }
  }

  if (fsSettings["health"]["enable"].isInt()) {
    health_params_.enabled = static_cast<int>(fsSettings["health"]["enable"]);
    LOG(INFO) << "health_params_.enable: " << health_params_.enabled;

    if (fsSettings["health"]["min_keypoints"].isInt() || fsSettings["health"]["min_keypoints"].isReal()) {
      health_params_.min_tracked_keypoints = static_cast<int>(fsSettings["health"]["min_keypoints"]);
      LOG(INFO) << "health_params_.min_tracked_keypoints :" << health_params_.min_tracked_keypoints;
    }

    if (fsSettings["health"]["consecutive_keyframes"].isReal() ||
        fsSettings["health"]["consecutive_keyframes"].isInt()) {
      health_params_.consecutive_keyframes = static_cast<double>(fsSettings["health"]["consecutive_keyframes"]);
      LOG(INFO) << "health_params_.consecutive keyframes check " << health_params_.consecutive_keyframes;
    }

    if (fsSettings["health"]["keyframe_wait_time"].isInt() || fsSettings["health"]["keyframe_wait_time"].isReal()) {
      health_params_.kf_wait_time = static_cast<float>(fsSettings["health"]["keyframe_wait_time"]);
      LOG(INFO) << "health_params_.keyframe wait time: " << health_params_.kf_wait_time;
    }

    if (fsSettings["health"]["kps_per_quadrant"].isInt() || fsSettings["health"]["kps_per_quadrant"].isReal()) {
      health_params_.kps_per_quadrant = static_cast<int>(fsSettings["health"]["kps_per_quadrant"]);
      LOG(INFO) << "health_params_.keypoints_per_quadrant: " << health_params_.kps_per_quadrant;
    }
  }

  fast_relocalization_ = fsSettings["fast_relocalization"];

  std::string results_path = pkg_path + "/svin_results/";
  if (!boost::filesystem::exists(results_path)) {
    boost::filesystem::create_directory(results_path);
  }
  svin_w_loop_path_ = results_path + "svin_" + Utils::getTimeStr() + ".txt";

  std::cout << "SVIN Result path: " << svin_w_loop_path_ << std::endl;

  // Read config file parameters
  if (fsSettings["resizeFactor"].isReal() || fsSettings["resizeFactor"].isInt()) {
    resize_factor_ = static_cast<double>(fsSettings["resizeFactor"]);
  }

  bool calibration_valid = getCalibrationViaConfig(camera_calibration_, fsSettings["cameras"]);
  if (!calibration_valid) {
    LOG(FATAL) << "Calibration not found in config file. Please provide calibration in config file.";
  }
  camera_calibration_.print();
  fsSettings.release();
}

// Get the camera calibration via the configuration file.
bool Parameters::getCalibrationViaConfig(CameraCalibration& calib, cv::FileNode camera_node) {
  bool got_calibration = false;
  // first check if calibration is available in config file
  if (camera_node.isSeq() && camera_node.size() > 0) {
    size_t cam_idx = 0;
    cv::FileNodeIterator it = camera_node.begin();
    if ((*it).isMap() && (*it)["T_SC"].isSeq() && (*it)["image_dimension"].isSeq() &&
        (*it)["image_dimension"].size() == 2 && (*it)["distortion_coefficients"].isSeq() &&
        (*it)["distortion_coefficients"].size() >= 4 && (*it)["distortion_type"].isString() &&
        (*it)["focal_length"].isSeq() && (*it)["focal_length"].size() == 2 && (*it)["principal_point"].isSeq() &&
        (*it)["principal_point"].size() == 2) {
      LOG(INFO) << "Found calibration in configuration file for camera " << cam_idx;
      got_calibration = true;
    } else {
      LOG(WARNING) << "Found incomplete calibration in configuration file for camera " << cam_idx
                   << ". Will not use the calibration from the configuration file.";
      return false;
    }
  } else {
    LOG(INFO) << "Did not find a calibration in the configuration file.";
  }
  if (got_calibration) {
    cv::FileNodeIterator it = camera_node.begin();

    cv::FileNode T_SC_node = (*it)["T_SC"];
    cv::FileNode image_dimension_node = (*it)["image_dimension"];
    cv::FileNode distortion_coefficient_node = (*it)["distortion_coefficients"];
    cv::FileNode focal_length_node = (*it)["focal_length"];
    cv::FileNode principal_point_node = (*it)["principal_point"];

    // extrinsics
    calib.T_imu_cam0_ << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3], T_SC_node[4], T_SC_node[5],
        T_SC_node[6], T_SC_node[7], T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11], T_SC_node[12],
        T_SC_node[13], T_SC_node[14], T_SC_node[15];

    calib.image_dimension_ << image_dimension_node[0], image_dimension_node[1];
    calib.image_dimension_(0) = static_cast<int>(static_cast<double>(calib.image_dimension_(0)) * resize_factor_);
    calib.image_dimension_(1) = static_cast<int>(static_cast<double>(calib.image_dimension_(1)) * resize_factor_);
    LOG(WARNING) << "Resize Factor: " << resize_factor_;
    LOG(WARNING) << calib.image_dimension_;

    calib.distortion_coefficients_ = cv::Mat::zeros(distortion_coefficient_node.size(), 1, CV_64F);
    for (size_t i = 0; i < distortion_coefficient_node.size(); ++i) {
      calib.distortion_coefficients_.at<double>(i, 0) = distortion_coefficient_node[i];
    }

    // Changing focal_length and principal_point accord to image resizeFactor
    calib.focal_length_ << focal_length_node[0], focal_length_node[1];
    calib.focal_length_ = calib.focal_length_ * resize_factor_;

    calib.principal_point_ << principal_point_node[0], principal_point_node[1];
    calib.principal_point_ = calib.principal_point_ * resize_factor_;

    calib.distortion_type_ = (std::string)((*it)["distortion_type"]);
  }
  return got_calibration;
}
