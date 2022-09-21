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
  debug_image_ = false;
  image_delay_ = 0.0;

  loop_closure_params_.loop_closure_enabled = true;
  loop_closure_params_.min_correspondences = 25;
  loop_closure_params_.pnp_reprojection_thresh = 20.0;
  loop_closure_params_.pnp_ransac_iterations = 100;

  min_landmark_quality_ = 0.001;
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
    loop_closure_params_.loop_closure_enabled = static_cast<int>(fsSettings["loop_closure_params"]["enable"]);
    LOG(INFO) << "loop_closure_params.enable: " << loop_closure_params_.loop_closure_enabled;

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

  fast_relocalization_ = fsSettings["fast_relocalization"];

  svin_w_loop_path_ = pkg_path + "/svin_results/svin_" + Utility::getTimeStr() + ".txt";

  std::cout << "SVIN Result path: " << svin_w_loop_path_ << std::endl;
  std::ofstream fout(svin_w_loop_path_, std::ios::out);
  fout.close();

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
  cv::initUndistortRectifyMap(
      K, distortion_coeffs_, cv::Mat(), K, image_size, CV_32FC1, cam0_undistort_map_x_, cam0_undistort_map_y_);

  cv::FileNode t_s_c_node = fsSettings["T_S_C"];
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

  LOG(INFO) << "T_imu_cam0: " << T_imu_cam0_;

  fsSettings.release();
}
