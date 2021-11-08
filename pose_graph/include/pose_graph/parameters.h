#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <string>

extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

extern ros::Publisher pubMatchedPoints;

extern std::string BRIEF_PATTERN_FILE;

extern std::string SVIN_W_LOOP_PATH;

extern int FAST_RELOCALIZATION;

extern int MIN_LOOP_NUM;
// projection matrix
extern double p_fx;
extern double p_fy;
extern double p_cx;
extern double p_cy;
