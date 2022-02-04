#include "pose_graph/Parameters.h"

#include <ros/package.h>

#include <fstream>
#include <opencv2/calib3d.hpp>
#include <string>

#include "utils/Utils.h"

Parameters::Parameters() {
  // default values
  use_health_ = false;
  tic_ = Eigen::Vector3d::Zero();
  qic_ = Eigen::Matrix3d::Identity();
  tracked_kypoints_threshold_ = 8;
  wait_for_keyframe_time_ = 0.5;
  consecutive_good_keyframes_threshold_ = 5;
  debug_image_ = false;
}

void Parameters::loadParameters(const ros::NodeHandle& nh) {
  // Optional connection to svin_health
  // nh.getParam("use_health", use_health_);

  // ROS_INFO_STREAM("use_health: " << use_health_);
  std::string config_file;
  nh.getParam("config_file", config_file);
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  camera_visual_size_ = fsSettings["visualize_camera_size"];

  std::string image_topic;

  // Read config file parameters
  resize_factor_ = static_cast<double>(fsSettings["resizeFactor"]);

  cv::FileNode fnode = fsSettings["projection_matrix"];
  p_fx = static_cast<double>(fnode["fx"]);
  p_fy = static_cast<double>(fnode["fy"]);
  p_cx = static_cast<double>(fnode["cx"]);
  p_cy = static_cast<double>(fnode["cy"]);

  if (resize_factor_ != 1.0) {
    p_fx = p_fx * resize_factor_;
    p_fy = p_fy * resize_factor_;
    p_cx = p_cx * resize_factor_;
    p_cy = p_cy * resize_factor_;
  }
  std::cout << "projection_matrix: " << p_fx << " " << p_fy << " " << p_cx << " " << p_cy << std::endl;
  std::string pkg_path = ros::package::getPath("pose_graph");

  vocabulary_file_ = pkg_path + "/Vocabulary/brief_k10L6.bin";
  std::cout << "vocabulary_file" << vocabulary_file_ << std::endl;

  brief_pattern_file_ = pkg_path + "/Vocabulary/brief_pattern.yml";

  min_loop_num_ = fsSettings["min_loop_num"];
  std::cout << "Num of matched keypoints for Loop Detection: " << min_loop_num_ << std::endl;

  fast_relocalization_ = fsSettings["fast_relocalization"];

  svin_w_loop_path_ = pkg_path + "/svin_results/svin_" + Utility::getTimeStr() + ".txt";

  std::cout << "SVIN Result path: " << svin_w_loop_path_ << std::endl;
  std::ofstream fout(svin_w_loop_path_, std::ios::out);
  fout.close();

  bool is_stereo = static_cast<int>(fsSettings["is_stereo"]);
  cv::FileNode cam0_node = fsSettings["cam0"];

  image_height_ = static_cast<int>(cam0_node["height"]);
  image_width_ = static_cast<int>(cam0_node["width"]);

  cv::Mat P = cv::Mat::eye(3, 3, CV_64F);
  P.at<double>(0, 0) = p_fx / resize_factor_;
  P.at<double>(1, 1) = p_fy / resize_factor_;
  P.at<double>(0, 2) = p_cx / resize_factor_;
  P.at<double>(1, 2) = p_cy / resize_factor_;

  cv::FileNode dnode = cam0_node["D"];
  distortion_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
  if (dnode.isSeq()) {
    distortion_coeffs_.at<double>(0, 0) = static_cast<double>(dnode[0]);
    distortion_coeffs_.at<double>(1, 0) = static_cast<double>(dnode[1]);
    distortion_coeffs_.at<double>(2, 0) = static_cast<double>(dnode[2]);
    distortion_coeffs_.at<double>(3, 0) = static_cast<double>(dnode[3]);
  }

  cv::FileNode rnode = cam0_node["R"];
  cv::FileNode knode = cam0_node["K"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

  if (is_stereo) {
    if (knode.isSeq()) {
      K.at<double>(0, 0) = static_cast<double>(knode[0]);
      K.at<double>(1, 1) = static_cast<double>(knode[4]);
      K.at<double>(0, 2) = static_cast<double>(knode[2]);
      K.at<double>(1, 2) = static_cast<double>(knode[5]);
    }

    if (rnode.isSeq()) {
      R.at<double>(0, 0) = static_cast<double>(rnode[0]);
      R.at<double>(1, 0) = static_cast<double>(rnode[1]);
      R.at<double>(2, 0) = static_cast<double>(rnode[2]);
      R.at<double>(0, 1) = static_cast<double>(rnode[3]);
      R.at<double>(1, 1) = static_cast<double>(rnode[4]);
      R.at<double>(2, 1) = static_cast<double>(rnode[5]);
      R.at<double>(0, 2) = static_cast<double>(rnode[6]);
      R.at<double>(1, 2) = static_cast<double>(rnode[7]);
      R.at<double>(2, 2) = static_cast<double>(rnode[8]);
    }
  }
  cv::Size image_size(image_width_, image_height_);

  ROS_INFO_STREAM("distortion_coefficients: " << distortion_coeffs_);
  ROS_INFO_STREAM("projection_matrix: " << P);
  ROS_INFO_STREAM("camera_matrix: " << K);
  ROS_INFO_STREAM("rectification_matrix: " << R);
  if (is_stereo) {
    cv::initUndistortRectifyMap(
        K, distortion_coeffs_, R, P, image_size, CV_32FC1, cam0_undistort_map_x_, cam0_undistort_map_y_);
  } else {
    cv::initUndistortRectifyMap(
        P, distortion_coeffs_, cv::Mat(), P, image_size, CV_32FC1, cam0_undistort_map_x_, cam0_undistort_map_y_);
  }

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

  ROS_INFO_STREAM("T_imu_cam0: " << T_imu_cam0_);

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

  ROS_INFO_STREAM("T_BS: " << T_body_imu_);

  cv::FileNode use_health_node = fsSettings["use_health"];
  if (use_health_node.isInt()) {
    use_health_ = static_cast<int>(use_health_node);
    ROS_INFO_STREAM("use_health: " << use_health_);
  }

  if (use_health_) {
    cv::FileNode kps_node = fsSettings["keypoints_threshold"];
    if (kps_node.isInt()) {
      tracked_kypoints_threshold_ = static_cast<int>(kps_node);
      ROS_INFO_STREAM("tracked_kypoints_threshold: " << tracked_kypoints_threshold_);
    }

    cv::FileNode kf_wait_time_node = fsSettings["switching_time"];
    if (kf_wait_time_node.isReal()) {
      wait_for_keyframe_time_ = static_cast<double>(kf_wait_time_node);
      ROS_INFO_STREAM("wait_for_keyframe_time: " << wait_for_keyframe_time_);
    }

    cv::FileNode kf_success_node = fsSettings["consecutive_good_keyframes"];
    if (kf_success_node.isInt()) {
      consecutive_good_keyframes_threshold_ = static_cast<int>(kf_success_node);
      ROS_INFO_STREAM("consecutive_good_keyframes_threshold: " << consecutive_good_keyframes_threshold_);
    }
  }

  cv::FileNode debug_image_node = fsSettings["debug_image"];
  if (debug_image_node.isInt()) {
    debug_image_ = static_cast<int>(debug_image_node);
    ROS_INFO_STREAM("debug_image: " << debug_image_);
  }

  fsSettings.release();
}
