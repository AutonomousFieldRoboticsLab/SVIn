#include "pose_graph/Parameters.h"

#include <glog/logging.h>
#include <ros/package.h>

#include <fstream>
#include <opencv2/calib3d.hpp>
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
  }

  if (fsSettings["debug"]["enable"].isInt()) {
    debug_mode_ = static_cast<int>(fsSettings["debug"]["enable"]);
    if (fsSettings["debug"]["output_dir"].isString()) {
      debug_output_path_ = static_cast<std::string>(fsSettings["debug"]["output_dir"]);
      LOG(INFO) << "Debug output folder: " << debug_output_path_;
    } else {
      LOG(WARNING) << "Could not found output folder for debug images.";
      LOG(WARNING) << "setting debug mode to false";
      debug_mode_ = false;
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

  svin_w_loop_path_ = pkg_path + "/svin_results/svin_" + Utility::getTimeStr() + ".txt";

  std::cout << "SVIN Result path: " << svin_w_loop_path_ << std::endl;

  // Read config file parameters
  resize_factor_ = static_cast<double>(fsSettings["resizeFactor"]);

  cv::FileNode fnode = fsSettings["focal_length"];
  p_fx_ = static_cast<double>(fnode[0]);
  p_fy_ = static_cast<double>(fnode[1]);

  fnode = fsSettings["principal_point"];
  p_cx_ = static_cast<double>(fnode[0]);
  p_cy_ = static_cast<double>(fnode[1]);

  image_height_ = static_cast<int>(fsSettings["image_height"]);
  image_width_ = static_cast<int>(fsSettings["image_width"]);

  std::cout << "focal_length: " << p_fx_ << " " << p_fy_ << std::endl;
  std::cout << "principal points: " << p_cx_ << " " << p_cy_ << std::endl;

  if (resize_factor_ != 1.0) {
    p_fx_ *= resize_factor_;
    p_fy_ *= resize_factor_;
    p_cx_ *= resize_factor_;
    p_cy_ *= resize_factor_;
    image_height_ = static_cast<int>(static_cast<double>(image_height_) * resize_factor_);
    image_width_ = static_cast<int>(static_cast<double>(image_width_) * resize_factor_);
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = p_fx_;
  K.at<double>(1, 1) = p_fy_;
  K.at<double>(0, 2) = p_cx_;
  K.at<double>(1, 2) = p_cy_;

  cv::FileNode dnode = fsSettings["distortion_coefficients"];
  distortion_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
  if (dnode.isSeq()) {
    distortion_coeffs_.at<double>(0, 0) = static_cast<double>(dnode[0]);
    distortion_coeffs_.at<double>(1, 0) = static_cast<double>(dnode[1]);
    distortion_coeffs_.at<double>(2, 0) = static_cast<double>(dnode[2]);
    distortion_coeffs_.at<double>(3, 0) = static_cast<double>(dnode[3]);
  }

  cv::Size image_size(image_width_, image_height_);
  LOG(INFO) << "distortion_coefficients: " << distortion_coeffs_;
  LOG(INFO) << "camera_matrix : \n" << K;

  cv::FileNode t_s_c_node = fsSettings["T_SC"];
  T_imu_cam0_ = Eigen::Matrix4d::Identity();
  if (t_s_c_node.isSeq()) {
    T_imu_cam0_(0, 0) = static_cast<double>(t_s_c_node[0]);
    T_imu_cam0_(0, 1) = static_cast<double>(t_s_c_node[1]);
    T_imu_cam0_(0, 2) = static_cast<double>(t_s_c_node[2]);
    T_imu_cam0_(0, 3) = static_cast<double>(t_s_c_node[3]);
    T_imu_cam0_(1, 0) = static_cast<double>(t_s_c_node[4]);
    T_imu_cam0_(1, 1) = static_cast<double>(t_s_c_node[5]);
    T_imu_cam0_(1, 2) = static_cast<double>(t_s_c_node[6]);
    T_imu_cam0_(1, 3) = static_cast<double>(t_s_c_node[7]);
    T_imu_cam0_(2, 0) = static_cast<double>(t_s_c_node[8]);
    T_imu_cam0_(2, 1) = static_cast<double>(t_s_c_node[9]);
    T_imu_cam0_(2, 2) = static_cast<double>(t_s_c_node[10]);
    T_imu_cam0_(2, 3) = static_cast<double>(t_s_c_node[11]);
  }

  LOG(INFO) << "T_imu_cam0: \n" << T_imu_cam0_;

  cv::FileNode t_bs_node = fsSettings["T_BS"];
  T_body_imu_ = Eigen::Matrix4d::Identity();
  if (t_bs_node.isSeq()) {
    T_body_imu_(0, 0) = static_cast<double>(t_bs_node[0]);
    T_body_imu_(0, 1) = static_cast<double>(t_bs_node[1]);
    T_body_imu_(0, 2) = static_cast<double>(t_bs_node[2]);
    T_body_imu_(0, 3) = static_cast<double>(t_bs_node[3]);
    T_body_imu_(1, 0) = static_cast<double>(t_bs_node[4]);
    T_body_imu_(1, 1) = static_cast<double>(t_bs_node[5]);
    T_body_imu_(1, 2) = static_cast<double>(t_bs_node[6]);
    T_body_imu_(1, 3) = static_cast<double>(t_bs_node[7]);
    T_body_imu_(2, 0) = static_cast<double>(t_bs_node[8]);
    T_body_imu_(2, 1) = static_cast<double>(t_bs_node[9]);
    T_body_imu_(2, 2) = static_cast<double>(t_bs_node[10]);
    T_body_imu_(2, 3) = static_cast<double>(t_bs_node[11]);
  }

  LOG(INFO) << "T_BS: \n" << T_body_imu_;

  fsSettings.release();
}
