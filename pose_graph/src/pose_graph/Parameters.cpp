#include "pose_graph/Parameters.h"

#include <ros/package.h>

#include <fstream>
#include <opencv2/calib3d.hpp>
#include <string>

#include "utility/utility.h"

Parameters::Parameters() {
  // default values
  use_health_ = false;
  tic_ = Eigen::Vector3d::Zero();
  qic_ = Eigen::Matrix3d::Identity();
}

void Parameters::loadParameters(const ros::NodeHandle& nh) {
  // Optional connection to svin_health
  nh.getParam("use_health", use_health_);

  ROS_ERROR_STREAM("use_health: " << use_health_);
  std::string config_file;
  nh.getParam("config_file", config_file);
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  camera_visual_size_ = fsSettings["visualize_camera_size"];

  std::string pkg_path = ros::package::getPath("pose_graph");

  vocabulary_file_ = pkg_path + "/Vocabulary/brief_k10L6.bin";
  std::cout << "vocabulary_file" << vocabulary_file_ << std::endl;

  brief_pattern_file_ = pkg_path + "/Vocabulary/brief_pattern.yml";

  min_loop_num_ = fsSettings["min_loop_num"];
  std::cout << "Num of matched keypoints for Loop Detection: " << min_loop_num_ << std::endl;

  ransac_reproj_threshold_ = static_cast<double>(fsSettings["ransac_reproj_threshold"]);
  std::cout << "Ransac reprojection threshold: " << ransac_reproj_threshold_ << std::endl;

  fast_relocalization_ = fsSettings["fast_relocalization"];

  svin_w_loop_path_ = pkg_path + "/svin_results/svin_" + Utility::getTimeStr() + ".txt";

  std::cout << "SVIN Result path: " << svin_w_loop_path_ << std::endl;
  std::ofstream fout(svin_w_loop_path_, std::ios::out);
  fout.close();

  // Read config file parameters
  resize_factor_ = static_cast<double>(fsSettings["resizeFactor"]);

  cv::FileNode fnode = fsSettings["focal_length"];
  p_fx = static_cast<double>(fnode[0]);
  p_fy = static_cast<double>(fnode[1]);

  fnode = fsSettings["principal_point"];
  p_cx = static_cast<double>(fnode[0]);
  p_cy = static_cast<double>(fnode[1]);

  image_height_ = static_cast<int>(fsSettings["image_height"]);
  image_width_ = static_cast<int>(fsSettings["image_width"]);

  std::cout << "focal_length: " << p_fx << " " << p_fy << std::endl;
  std::cout << "principal points: " << p_cx << " " << p_cy << std::endl;

  if (resize_factor_ != 1.0) {
    p_fx *= resize_factor_;
    p_fy *= resize_factor_;
    p_cx *= resize_factor_;
    p_cy *= resize_factor_;
    image_height_ = static_cast<int>(static_cast<double>(image_height_) * resize_factor_);
    image_width_ = static_cast<int>(static_cast<double>(image_width_) * resize_factor_);
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = p_fx;
  K.at<double>(1, 1) = p_fy;
  K.at<double>(0, 2) = p_cx;
  K.at<double>(1, 2) = p_cy;

  cv::FileNode dnode = fsSettings["distortion_coefficients"];
  distortion_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
  if (dnode.isSeq()) {
    distortion_coeffs_.at<double>(0, 0) = static_cast<double>(dnode[0]);
    distortion_coeffs_.at<double>(1, 0) = static_cast<double>(dnode[1]);
    distortion_coeffs_.at<double>(2, 0) = static_cast<double>(dnode[2]);
    distortion_coeffs_.at<double>(3, 0) = static_cast<double>(dnode[3]);
  }

  cv::Size image_size(image_width_, image_height_);

  ROS_INFO_STREAM("distortion_coefficients: " << distortion_coeffs_);
  ROS_INFO_STREAM("camera_matrix : \n" << K);
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

  ROS_INFO_STREAM("T_imu_cam0: " << T_imu_cam0_);

  fsSettings.release();
}
