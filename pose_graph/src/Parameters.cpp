#include "pose_graph/Parameters.h"

#include <ros/package.h>

#include <fstream>

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

  std::string config_file;
  nh.getParam("config_file", config_file);
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  camera_visual_size_ = fsSettings["visualize_camera_size"];

  std::string image_topic;

  // Read config file parameters
  double resize_factor = static_cast<double>(fsSettings["resizeFactor"]);

  cv::FileNode fnode = fsSettings["projection_matrix"];
  p_fx = static_cast<double>(fnode["fx"]);
  p_fy = static_cast<double>(fnode["fy"]);
  p_cx = static_cast<double>(fnode["cx"]);
  p_cy = static_cast<double>(fnode["cy"]);

  if (resize_factor != 1.0) {
    p_fx = p_fx * resize_factor;
    p_fy = p_fy * resize_factor;
    p_cx = p_cx * resize_factor;
    p_cy = p_cy * resize_factor;
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
  fsSettings.release();
}