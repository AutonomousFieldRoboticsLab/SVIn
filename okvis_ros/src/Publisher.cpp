/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Apr 27, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Publisher.cpp
 * @brief Source file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>

#if defined(ROS2_JAZZY)
  #include <cv_bridge/cv_bridge.hpp>
#elif defined(ROS2_HUMBLE)
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

#include <algorithm>
#include <list>
#include <memory>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Publisher.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <string>
#include <vector>

#include "okvis_ros/msg/svin_health.hpp"
/// \brief okvis Main namespace of this package.
namespace okvis {

Publisher::~Publisher() {
  // close file
  if (csvLandmarksFile_) {
    // write down also the current landmarks
    if (csvLandmarksFile_->good()) {
      for (size_t l = 0; l < pointsMatched2_.size(); ++l) {
        Eigen::Vector4d landmark = pointsMatched2_.at(l).point;
        *csvLandmarksFile_ << std::setprecision(19) << pointsMatched2_.at(l).id << ", " << std::scientific
                           << std::setprecision(18) << landmark[0] << ", " << landmark[1] << ", " << landmark[2] << ", "
                           << landmark[3] << ", " << pointsMatched2_.at(l).quality << std::endl;
      }
    }
    csvLandmarksFile_->close();
  }
  if (csvFile_) csvFile_->close();
}

// Constructor. Calls setNodeHandle().
Publisher::Publisher(std::shared_ptr<rclcpp::Node> node) : node_(node), pubTf_(node), image_transport_(node) {
  // pubStereoMatched_ = nh_->advertise<sensor_msgs::PointCloud2>("okvis_stereo_matched", 1);   // Sharmin
  pubKeyframeImageL_ = node_->create_publisher<sensor_msgs::msg::Image>("keyframe_imageL", 10);      // Sharmin
  pubKeyframePose_ = node_->create_publisher<nav_msgs::msg::Odometry>("keyframe_pose", 10);          // Sharmin
  pubKeyframePoints_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("keyframe_points", 1);  // Sharmin
  pubSvinHealth = node_->create_publisher<okvis_ros::msg::SvinHealth>("svin_health", 1);  // Sharmin: SVIN health

  pubPointsMatched_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("okvis_points_matched", 1);
  pubPointsUnmatched_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("okvis_points_unmatched", 1);
  pubPointsTransferred_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("okvis_points_transferred", 1);
  pubObometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("okvis_odometry", 1);
  pubPath_ = node_->create_publisher<nav_msgs::msg::Path>("okvis_path", 1);
  pubTransform_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>("okvis_transform", 1);
  pubMesh_ = node_->create_publisher<visualization_msgs::msg::Marker>("okvis_mesh", 0);
  // where to get the mesh from
  std::string mesh_file;
  if (node_->get_parameter("mesh_file", mesh_file)) {
    meshMsg_.mesh_resource = "package://okvis_ros/meshes/" + mesh_file;
  } else {
    LOG(INFO) << "no mesh found for visualisation, set ros param mesh_file, if desired";
    meshMsg_.mesh_resource = "";
  }

  static_tf_published_ = false;
}

void Publisher::publishKeyframeAsCallback(const okvis::Time& t,
                                          const cv::Mat& imageL,
                                          const okvis::kinematics::Transformation& T_WCa,
                                          std::vector<std::list<std::vector<double>>>& keyframePoints) {
  const okvis::kinematics::Transformation& T_Wc_W = parameters_.publishing.T_Wc_W;
  const okvis::kinematics::Transformation& T_WcCa = T_Wc_W * T_WCa;

  // publish keyframe
  std_msgs::msg::Header img_header;
  img_header.stamp = rclcpp::Time(t.sec, t.nsec);
  img_header.frame_id = "okvis_keyframe";
  sensor_msgs::msg::Image::ConstSharedPtr msg = cv_bridge::CvImage(img_header, "mono8", imageL).toImageMsg();
  pubKeyframeImageL_->publish(*msg);

  // publish keyframe odometry
  nav_msgs::msg::Odometry odometry;

  odometry.header.frame_id = "world";
  odometry.header.stamp = rclcpp::Time(t.sec, t.nsec);
  // fill orientation
  Eigen::Quaterniond q = T_WcCa.q();
  odometry.pose.pose.orientation.x = q.x();
  odometry.pose.pose.orientation.y = q.y();
  odometry.pose.pose.orientation.z = q.z();
  odometry.pose.pose.orientation.w = q.w();
  // fill position
  Eigen::Vector3d r = T_WcCa.r();
  odometry.pose.pose.position.x = r[0];
  odometry.pose.pose.position.y = r[1];
  odometry.pose.pose.position.z = r[2];
  pubKeyframePose_->publish(odometry);

  // SVIN health
  okvis_ros::msg::SvinHealth svinInfo;
  svinInfo.header.stamp = rclcpp::Time(t.sec, t.nsec);
  svinInfo.header.frame_id = "world";
  int q00_counter = 0;
  int q01_counter = 0;
  int q10_counter = 0;
  int q11_counter = 0;
  int nrows = imageL.rows;
  int ncols = imageL.cols;

  // publish 3d-2d keyframe points
  sensor_msgs::msg::PointCloud point_cloud;
  point_cloud.header.frame_id = "world";
  point_cloud.header.stamp = rclcpp::Time(t.sec, t.nsec);

  uint32_t new_feature_keypoints = 0;
  for (std::vector<std::list<std::vector<double>>>::iterator it = keyframePoints.begin(); it != keyframePoints.end();
       it++) {
    std::list<std::vector<double>> ptList = *it;

    std::list<std::vector<double>>::iterator lit = ptList.begin();
    // Note: Every time you ptList.push_back(...) in ThreadedKFVio, you advance lit by 1.

    std::vector<double> pt3d = *lit;
    // std::cout<<"MapPoint 3d:"<< pt3d.at(0) << " , "<< pt3d.at(1)<< " , "<< pt3d.at(2) <<std::endl;

    Eigen::Vector4d pt4d_eigen(pt3d.at(0), pt3d.at(1), pt3d.at(2), 1.0);
    Eigen::Vector4d pt4d_Wc_eigen = T_Wc_W * pt4d_eigen;

    geometry_msgs::msg::Point32 p;  // 3d position of MapPoint in W coordinate
    p.x = pt4d_Wc_eigen[0];
    p.y = pt4d_Wc_eigen[1];
    p.z = pt4d_Wc_eigen[2];
    point_cloud.points.push_back(p);
    std::advance(lit, 1);  // advancing by 1 after getting the 3d W-coordinate

    // SVIN health
    svinInfo.points_3d.push_back(p);

    sensor_msgs::msg::ChannelFloat32 p_id_w_uv;
    std::vector<double> cvKeypoint_w_id = *lit;

    // @Reloc
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(0));  // landmark id
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(1));  // poseId or multiframeId
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(2));  // keypointIndex
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(3));  // quality

    // kf_index where the MapPoint has been observed and it's corresponding 2d position in image
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(4));   // kf_index
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(5));   // cv Point2f: x -> corres to column
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(6));   // cv Point2f: y -> corres to row
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(7));   // keypoint size
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(8));   // keypoint angle/orientation
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(9));   // keypoint octave
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(10));  // keypoint response
    p_id_w_uv.values.push_back(cvKeypoint_w_id.at(11));  // keypoint class_id

    // SVIN health
    int x_coord = cvKeypoint_w_id.at(5);
    int y_coord = cvKeypoint_w_id.at(6);
    // q00 = image[int(0):int(0.5*nrows), int(0):int(0.5*ncols)]
    // q01 = image[int(0):int(0.5*nrows), int(0.5*ncols):int(ncols)]
    // q10 = image[int(0.5*nrows):int(nrows), int(0):int(0.5*ncols)]
    // q11 = image[int(0.5*nrows):int(nrows), int(0.5*ncols):int(ncols)]
    if ((x_coord >= 0 && x_coord <= 0.5 * ncols) && (y_coord >= 0 && y_coord <= 0.5 * nrows)) {
      q00_counter++;
    } else if ((x_coord >= 0.5 * ncols && x_coord <= ncols) && (y_coord >= 0 && y_coord <= 0.5 * nrows)) {
      q01_counter++;
    } else if ((x_coord >= 0 && x_coord <= 0.5 * ncols) && (y_coord >= 0.5 * nrows && y_coord <= nrows)) {
      q10_counter++;
    } else if ((x_coord >= 0.5 * ncols && x_coord <= ncols) && (y_coord >= 0.5 * nrows && y_coord <= nrows)) {
      q11_counter++;
    }

    std::advance(lit, 1);  // advancing by 1 after getting the id and 2d image-coordinate and other cv::KeyPoint
                           // information (Total size 8)

    // Id of other kfs where this MapPoint has been observed.
    // i = 2, because: ptList has been advanced by 2.

    int covis = 0;
    while (lit != ptList.end()) {
      std::vector<double> kf_id = *lit;
      // std::cout<<"MapPoint observed ID:"<< kf_id.at(0) <<" In KF"<< cvKeypoint_w_id.at(3)<<std::endl;
      p_id_w_uv.values.push_back(kf_id.at(0));  // kf_index
      lit++;                                    // advancing by 1 after getting the kf_id
      covis++;
    }

    if (covis == 0) {
      new_feature_keypoints++;  // if 3D point is not observed by any other keyframe, it is a new feature
    }
    // SVIN health
    svinInfo.covisibilities.push_back(covis);
    svinInfo.quality.push_back(cvKeypoint_w_id.at(3));
    svinInfo.response_strengths.push_back(cvKeypoint_w_id.at(10));
    // This also works fine as above
    /*for (size_t i = 2; i < ptList.size(); i++){
            std::vector<double> kf_id = *lit;
            //std::cout<<"MapPoint observed ID:"<< kf_id.at(0) <<" In KF"<< cvKeypoint_w_id.at(0)<<std::endl;
            p_id_w_uv.values.push_back(kf_id.at(0));  // kf_index
            std::advance(lit, 1); // advancing by 1 after getting the kf_id
    }*/

    point_cloud.channels.push_back(p_id_w_uv);
  }
  pubKeyframePoints_->publish(point_cloud);

  // Sharmin: svin health. No. of minimum match required = 10
  if (pointsMatched_.size() <= 10) {
    svinInfo.is_tracking_ok = false;
  } else {
    svinInfo.is_tracking_ok = true;
  }
  svinInfo.num_tracked_kps = point_cloud.points.size();
  svinInfo.kps_per_quadrant.push_back(q00_counter);
  svinInfo.kps_per_quadrant.push_back(q01_counter);
  svinInfo.kps_per_quadrant.push_back(q10_counter);
  svinInfo.kps_per_quadrant.push_back(q11_counter);
  svinInfo.new_kps = new_feature_keypoints;

  pubSvinHealth->publish(svinInfo);
}
// *************** End ***********************//

// Write CSV header.
bool Publisher::writeCsvDescription() {
  if (!csvFile_) return false;
  if (!csvFile_->good()) return false;
  *csvFile_ << "timestamp"
            << ", "
            << "p_WS_W_x"
            << ", "
            << "p_WS_W_y"
            << ", "
            << "p_WS_W_z"
            << ", "
            << "q_WS_x"
            << ", "
            << "q_WS_y"
            << ", "
            << "q_WS_z"
            << ", "
            << "q_WS_w"
            << ", "
            << "v_WS_W_x"
            << ", "
            << "v_WS_W_y"
            << ", "
            << "v_WS_W_z"
            << ", "
            << "b_g_x"
            << ", "
            << "b_g_y"
            << ", "
            << "b_g_z"
            << ", "
            << "b_a_x"
            << ", "
            << "b_a_y"
            << ", "
            << "b_a_z" << std::endl;
  return true;
}

// Write CSV header for landmarks file.
bool Publisher::writeLandmarksCsvDescription() {
  if (!csvLandmarksFile_) return false;
  if (!csvLandmarksFile_->good()) return false;
  *csvLandmarksFile_ << ", "
                     << "id"
                     << ", "
                     << "l_x"
                     << ", "
                     << "l_y"
                     << ", "
                     << "l_z"
                     << ", "
                     << "l_w"
                     << ", "
                     << "quality, "
                     << "distance" << std::endl;
  return true;
}

// Set an odometry output CSV file.
bool Publisher::setCsvFile(std::fstream& csvFile) {
  if (csvFile_) {
    csvFile_->close();
  }
  csvFile_.reset(&csvFile);
  writeCsvDescription();
  return csvFile_->good();
}
// Set an odometry output CSV file.
bool Publisher::setCsvFile(std::string& csvFileName) {
  csvFile_.reset(new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeCsvDescription();
  return csvFile_->good();
}
// Set an odometry output CSV file.
bool Publisher::setCsvFile(std::string csvFileName) {
  csvFile_.reset(new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeCsvDescription();
  return csvFile_->good();
}

// Set a CVS file where the landmarks will be saved to.
bool Publisher::setLandmarksCsvFile(std::fstream& csvFile) {
  if (csvLandmarksFile_) {
    csvLandmarksFile_->close();
  }
  csvLandmarksFile_.reset(&csvFile);
  writeLandmarksCsvDescription();
  return csvLandmarksFile_->good();
}
// Set a CVS file where the landmarks will be saved to.
bool Publisher::setLandmarksCsvFile(std::string& csvFileName) {
  csvLandmarksFile_.reset(new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeLandmarksCsvDescription();
  return csvLandmarksFile_->good();
}
// Set a CVS file where the landmarks will be saved to.
bool Publisher::setLandmarksCsvFile(std::string csvFileName) {
  csvLandmarksFile_.reset(new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeLandmarksCsvDescription();
  return csvLandmarksFile_->good();
}

// Set the pose message that is published next.
void Publisher::setPose(const okvis::kinematics::Transformation& T_WS) {
  okvis::kinematics::Transformation T;
  if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    poseMsg_.child_frame_id = "sensor";
    T = parameters_.publishing.T_Wc_W * T_WS;
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    poseMsg_.child_frame_id = "body";
    T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();
  } else {
    LOG(ERROR) << "Pose frame does not exist for publishing. Choose 'S' or 'B'.";
    poseMsg_.child_frame_id = "body";
    T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();
  }

  poseMsg_.header.frame_id = "world";
  poseMsg_.header.stamp = _t;
  if ((node_->now().seconds() - _t.seconds()) > 10.0) poseMsg_.header.stamp = node_->now();

  // fill orientation
  Eigen::Quaterniond q = T.q();
  poseMsg_.transform.rotation.x = q.x();
  poseMsg_.transform.rotation.y = q.y();
  poseMsg_.transform.rotation.z = q.z();
  poseMsg_.transform.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = T.r();
  poseMsg_.transform.translation.x = r[0];
  poseMsg_.transform.translation.y = r[1];
  poseMsg_.transform.translation.z = r[2];

  // also do the mesh
  /*if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    meshMsg_.child_frame_id = "sensor";
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    meshMsg_.child_frame_id = "body";
  } else {
    meshMsg_.child_frame_id = "body";
  }*/
  meshMsg_.header.frame_id = "world";
  meshMsg_.header.stamp = _t;
  meshMsg_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  if ((node_->now().seconds() - _t.seconds()) > 10.0) meshMsg_.header.stamp = node_->now();

  // fill orientation
  meshMsg_.pose.orientation.x = q.x();
  meshMsg_.pose.orientation.y = q.y();
  meshMsg_.pose.orientation.z = q.z();
  meshMsg_.pose.orientation.w = q.w();

  // fill position
  meshMsg_.pose.position.x = r[0];
  meshMsg_.pose.position.y = r[1];
  meshMsg_.pose.position.z = r[2];

  // scale -- needed
  meshMsg_.scale.x = 1.0;
  meshMsg_.scale.y = 1.0;
  meshMsg_.scale.z = 1.0;

  meshMsg_.action = visualization_msgs::msg::Marker::ADD;
  meshMsg_.color.a = 1.0;  // Don't forget to set the alpha!
  meshMsg_.color.r = 1.0;
  meshMsg_.color.g = 1.0;
  meshMsg_.color.b = 1.0;

  // embedded material / colour
  // meshMsg_.mesh_use_embedded_materials = true;
}

// Set the odometry message that is published next.
// Modified by SHarmin
void Publisher::setOdometry(const okvis::kinematics::Transformation& T_WS,
                            const okvis::SpeedAndBiases& speedAndBiases,
                            const Eigen::Vector3d& omega_S) {
  // header.frame_id is the frame in which the pose is given. I.e. world frame in our case
  // child_frame_id is the frame in which the twist part of the odometry message is given.
  // see also nav_msgs/Odometry Message documentation

  odometryMsg_.header.stamp = _t;
  okvis::kinematics::Transformation T;  // the pose to be published. T_WS or T_WB depending on 'trackedBodyFrame'
  okvis::kinematics::Transformation
      reloc_T;  // Sharmin: the reloc pose to be published. T_WS or T_WB depending on 'trackedBodyFrame'
  Eigen::Vector3d omega_W = parameters_.publishing.T_Wc_W.C() * T_WS.C() * omega_S;
  Eigen::Vector3d t_W_ofFrame;  // lever arm in W-system
  Eigen::Vector3d v_W_ofFrame;  // velocity in W-system. v_S_in_W or v_B_in_W

  if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    odometryMsg_.header.frame_id = "world";
    T = parameters_.publishing.T_Wc_W * T_WS;
    t_W_ofFrame.setZero();  // r_SS_in_W
    v_W_ofFrame =
        parameters_.publishing.T_Wc_W.C() * speedAndBiases.head<3>();  // world-centric speedAndBiases.head<3>()
    // LOG (INFO) << "Tracked Body Frame: S";
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    odometryMsg_.header.frame_id = "world";
    T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();
    reloc_T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();  // Sharmin
    t_W_ofFrame = (parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse()).r() -
                  (parameters_.publishing.T_Wc_W * T_WS).r();  // r_BS_in_W
    v_W_ofFrame = parameters_.publishing.T_Wc_W.C() * speedAndBiases.head<3>() +
                  omega_W.cross(t_W_ofFrame);  // world-centric speedAndBiases.head<3>()
    // LOG (INFO) << "Tracked Body Frame: B";
    // LOG (INFO) << parameters_.publishing.T_Wc_W.C();
  } else {
    LOG(ERROR) << "Pose frame does not exist for publishing. Choose 'S' or 'B'.";
    odometryMsg_.header.frame_id = "world";
    T = parameters_.publishing.T_Wc_W * T_WS;
    t_W_ofFrame.setZero();  // r_SS_in_W
    v_W_ofFrame =
        parameters_.publishing.T_Wc_W.C() * speedAndBiases.head<3>();  // world-centric speedAndBiases.head<3>()
  }

  // fill orientation
  Eigen::Quaterniond q = T.q();
  odometryMsg_.pose.pose.orientation.x = q.x();
  odometryMsg_.pose.pose.orientation.y = q.y();
  odometryMsg_.pose.pose.orientation.z = q.z();
  odometryMsg_.pose.pose.orientation.w = q.w();

  // fill position
  Eigen::Vector3d r = T.r();
  odometryMsg_.pose.pose.position.x = r[0];
  odometryMsg_.pose.pose.position.y = r[1];
  odometryMsg_.pose.pose.position.z = r[2];

  nav_msgs::msg::Odometry odometry;  // Sharmin

  Eigen::Matrix3d C_v;
  Eigen::Matrix3d C_omega;
  if (parameters_.publishing.velocitiesFrame == FrameName::S) {
    C_v = (parameters_.publishing.T_Wc_W * T_WS).inverse().C();
    C_omega.setIdentity();
    odometryMsg_.child_frame_id = "sensor";
  } else if (parameters_.publishing.velocitiesFrame == FrameName::B) {
    C_v = (parameters_.imu.T_BS * T_WS.inverse()).C() * parameters_.publishing.T_Wc_W.inverse().C();
    C_omega = parameters_.imu.T_BS.C();
    odometryMsg_.child_frame_id = "body";
  } else if (parameters_.publishing.velocitiesFrame == FrameName::Wc) {
    C_v.setIdentity();
    C_omega = parameters_.publishing.T_Wc_W.C() * T_WS.C();
    odometryMsg_.child_frame_id = "world";
  } else {
    LOG(ERROR) << "Speeds frame does not exist for publishing. Choose 'S', 'B', or 'Wc'.";
    C_v = (parameters_.imu.T_BS * T_WS.inverse()).C() * parameters_.publishing.T_Wc_W.inverse().C();
    C_omega = parameters_.imu.T_BS.C();
    odometryMsg_.child_frame_id = "body";
  }

  // fill velocity
  Eigen::Vector3d v = C_v * v_W_ofFrame;  // v_S_in_'speedsInThisFrame' or v_B_in_'speedsInThisFrame'
  odometryMsg_.twist.twist.linear.x = v[0];
  odometryMsg_.twist.twist.linear.y = v[1];
  odometryMsg_.twist.twist.linear.z = v[2];

  // fill angular velocity
  Eigen::Vector3d omega = C_omega * omega_S;  // omega_in_'speedsInThisFrame'
  odometryMsg_.twist.twist.angular.x = omega[0];
  odometryMsg_.twist.twist.angular.y = omega[1];
  odometryMsg_.twist.twist.angular.z = omega[2];

  // linear acceleration ?? - would also need point of percussion mapping!!

  // *********Sharmin: For Reloc Pose ************************//
  // std::cout << driftCorrected_T.q().toRotationMatrix()<< " "<< driftCorrected_T.r() << std::endl;

  // std::cout << T.q().toRotationMatrix()<< " "<< T.r() << std::endl;
  // std::cout << reloc_T.q().toRotationMatrix()<< " "<< reloc_T.r() << std::endl;

  odometry.header.stamp = _t;
  odometry.header.frame_id = "world";

  // fill orientation
  Eigen::Quaterniond reloc_q = reloc_T.q();
  odometry.pose.pose.orientation.x = reloc_q.x();
  odometry.pose.pose.orientation.y = reloc_q.y();
  odometry.pose.pose.orientation.z = reloc_q.z();
  odometry.pose.pose.orientation.w = reloc_q.w();

  // fill position
  Eigen::Vector3d relo_r = reloc_T.r();
  odometry.pose.pose.position.x = relo_r[0];
  odometry.pose.pose.position.y = relo_r[1];
  odometry.pose.pose.position.z = relo_r[2];

  odometry.twist = odometryMsg_.twist;
}

// Set the points that are published next.
void Publisher::setPoints(const okvis::MapPointVector& pointsMatched,
                          const okvis::MapPointVector& pointsUnmatched,
                          const okvis::MapPointVector& pointsTransferred) {
  pointsMatched2_.clear();
  pointsMatched2_ = pointsMatched;
  pointsMatched_.clear();
  pointsUnmatched_.clear();
  pointsTransferred_.clear();

  // transform points into custom world frame:
  const Eigen::Matrix4d T_Wc_W = parameters_.publishing.T_Wc_W.T();

  for (size_t i = 0; i < pointsMatched.size(); ++i) {
    // check infinity
    if (fabs(static_cast<double>(pointsMatched[i].point[3])) < 1.0e-8) continue;

    // check quality
    if (pointsMatched[i].quality < parameters_.publishing.landmarkQualityThreshold) continue;

    pointsMatched_.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = T_Wc_W * pointsMatched[i].point;
    pointsMatched_.back().x = point[0] / point[3];
    pointsMatched_.back().y = point[1] / point[3];
    pointsMatched_.back().z = point[2] / point[3];
    pointsMatched_.back().g =
        255 * (std::min(parameters_.publishing.maxLandmarkQuality, static_cast<float>(pointsMatched[i].quality)) /
               parameters_.publishing.maxLandmarkQuality);

    // added by Sharmin
    if (csvLandmarksFile_)
      *csvLandmarksFile_ << ", " << pointsMatched.at(i).id << ", " << point[0] << ", " << point[1] << ", " << point[2]
                         << ", " << point[3] << ", " << pointsMatched.at(i).quality << std::endl;
  }
  pointsMatched_.header.frame_id = "world";

  std_msgs::msg::Header header;
  header.stamp = _t;
  pointsMatched_.header.stamp = pcl_conversions::toPCL(_t);

  for (size_t i = 0; i < pointsUnmatched.size(); ++i) {
    // check infinity
    if (fabs(static_cast<double>(pointsUnmatched[i].point[3])) < 1.0e-8) continue;

    // check quality
    if (pointsUnmatched[i].quality < parameters_.publishing.landmarkQualityThreshold) continue;

    pointsUnmatched_.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = T_Wc_W * pointsUnmatched[i].point;
    pointsUnmatched_.back().x = point[0] / point[3];
    pointsUnmatched_.back().y = point[1] / point[3];
    pointsUnmatched_.back().z = point[2] / point[3];
    pointsUnmatched_.back().b =
        255 * (std::min(parameters_.publishing.maxLandmarkQuality, static_cast<float>(pointsUnmatched[i].quality)) /
               parameters_.publishing.maxLandmarkQuality);
  }
  pointsUnmatched_.header.frame_id = "world";
  pointsUnmatched_.header.stamp = pcl_conversions::toPCL(_t);

  for (size_t i = 0; i < pointsTransferred.size(); ++i) {
    // check infinity

    if (fabs(static_cast<double>(pointsTransferred[i].point[3])) < 1.0e-10) continue;

    // check quality
    if (pointsTransferred[i].quality < parameters_.publishing.landmarkQualityThreshold) continue;

    pointsTransferred_.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = T_Wc_W * pointsTransferred[i].point;
    pointsTransferred_.back().x = point[0] / point[3];
    pointsTransferred_.back().y = point[1] / point[3];
    pointsTransferred_.back().z = point[2] / point[3];
    float intensity =
        std::min(parameters_.publishing.maxLandmarkQuality, static_cast<float>(pointsTransferred[i].quality)) /
        parameters_.publishing.maxLandmarkQuality;
    pointsTransferred_.back().r = 255 * intensity;
    pointsTransferred_.back().g = 255 * intensity;
    pointsTransferred_.back().b = 255 * intensity;
    // _omfile << point[0] << " " << point[1] << " " << point[2] << ";" <<std::endl;
  }
  pointsTransferred_.header.frame_id = "world";
  pointsTransferred_.header.seq = ctr2_++;

  pointsTransferred_.header.stamp = pcl_conversions::toPCL(_t);
}

// Publish the pose.
void Publisher::publishPose() {
  if ((_t.seconds() - lastOdometryTime2_.seconds()) < 1.0 / parameters_.publishing.publishRate)
    return;  // control the publish rate
  pubTf_.sendTransform(poseMsg_);
  if (!meshMsg_.mesh_resource.empty()) pubMesh_->publish(meshMsg_);  // publish stamped mesh

  publishSensorStaticTf();
  lastOdometryTime2_ = _t;  // remember
}

// Publish the last set odometry.
void Publisher::publishOdometry() {
  if ((_t.seconds() - lastOdometryTime_.seconds()) < 1.0 / parameters_.publishing.publishRate)
    return;  // control the publish rate
  pubObometry_->publish(odometryMsg_);

  /*** Added by Sharmin ****/
  static tf2_ros::TransformBroadcaster br(node_);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odometryMsg_.header.stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = odometryMsg_.pose.pose.position.x;
  transform.transform.translation.y = odometryMsg_.pose.pose.position.y;
  transform.transform.translation.z = odometryMsg_.pose.pose.position.z;
  transform.transform.rotation.x = odometryMsg_.pose.pose.orientation.x;
  transform.transform.rotation.y = odometryMsg_.pose.pose.orientation.y;
  transform.transform.rotation.z = odometryMsg_.pose.pose.orientation.z;
  transform.transform.rotation.w = odometryMsg_.pose.pose.orientation.w;

  br.sendTransform(transform);

  /*** End Added by Sharmin ****/

  if (!meshMsg_.mesh_resource.empty()) pubMesh_->publish(meshMsg_);  // publish stamped mesh
  lastOdometryTime_ = _t;                                            // remember
}

// Publish the T_WS transform.
void Publisher::publishTransform() {
  if ((_t.seconds() - lastTransfromTime_.seconds()) < 1.0 / parameters_.publishing.publishRate)
    return;                          // control the publish rate
  pubTransform_->publish(poseMsg_);  // publish stamped transform for MSF
  lastTransfromTime_ = _t;           // remember
}

// Set and publish pose.
void Publisher::publishStateAsCallback(const okvis::Time& t, const okvis::kinematics::Transformation& T_WS) {
  setTime(t);
  setPose(T_WS);  // TODO(hdamron): provide setters for this hack
  publishPose();
}
// Set and publish full state.
// Modified by SHarmin
void Publisher::publishFullStateAsCallback(const okvis::Time& t,
                                           const okvis::kinematics::Transformation& T_WS,
                                           const Eigen::Matrix<double, 9, 1>& speedAndBiases,
                                           const Eigen::Matrix<double, 3, 1>& omega_S
                                           //  const okvis::kinematics::Transformation& driftCorrected_T_WS
) {
  setTime(t);
  // Modified by Sharmin
  setOdometry(T_WS, speedAndBiases, omega_S);  // TODO(sharmin): provide setters for this hack
  setPath(T_WS);
  publishOdometry();
  publishTransform();
  publishPath();
}

// Set and write full state to CSV file.
void Publisher::csvSaveFullStateAsCallback(const okvis::Time& t,
                                           const okvis::kinematics::Transformation& T_WS,
                                           const Eigen::Matrix<double, 9, 1>& speedAndBiases,
                                           const Eigen::Matrix<double, 3, 1>& omega_S) {
  setTime(t);
  // Modified by Sharmin
  setOdometry(T_WS, speedAndBiases,
              omega_S);  // TODO(sharmin): provide setters for this hack
  if (!csvFile_) {
    LOG(WARNING) << "csvFile_ not ok";
  } else {
    // LOG(INFO)<<"filePtr: ok; ";
    if (csvFile_->good()) {
      // LOG(INFO)<<"file: good.";
      Eigen::Vector3d p_WS_W = T_WS.r();
      Eigen::Quaterniond q_WS = T_WS.q();
      std::stringstream time;
      time << t.sec << std::setw(9) << std::setfill('0') << t.nsec;
      *csvFile_ << time.str() << ", " << std::scientific << std::setprecision(18) << p_WS_W[0] << ", " << p_WS_W[1]
                << ", " << p_WS_W[2] << ", " << q_WS.x() << ", " << q_WS.y() << ", " << q_WS.z() << ", " << q_WS.w()
                << ", " << speedAndBiases[0] << ", " << speedAndBiases[1] << ", " << speedAndBiases[2] << ", "
                << speedAndBiases[3] << ", " << speedAndBiases[4] << ", " << speedAndBiases[5] << ", "
                << speedAndBiases[6] << ", " << speedAndBiases[7] << ", " << speedAndBiases[8] << std::endl;
    }
  }
}

// Set and write full state including camera extrinsics to file.
void Publisher::csvSaveFullStateWithExtrinsicsAsCallback(
    const okvis::Time& t,
    const okvis::kinematics::Transformation& T_WS,
    const Eigen::Matrix<double, 9, 1>& speedAndBiases,
    const Eigen::Matrix<double, 3, 1>& omega_S,
    const std::vector<okvis::kinematics::Transformation, Eigen::aligned_allocator<okvis::kinematics::Transformation>>&
        extrinsics) {
  setTime(t);
  // Modified by SHarmin
  setOdometry(T_WS, speedAndBiases,
              omega_S);  // TODO(sharmin): provide setters for this hack
  if (csvFile_) {
    if (csvFile_->good()) {
      Eigen::Vector3d p_WS_W = T_WS.r();
      Eigen::Quaterniond q_WS = T_WS.q();
      std::stringstream time;
      time << t.sec << std::setw(9) << std::setfill('0') << t.nsec;
      *csvFile_ << time.str() << ", " << std::scientific << std::setprecision(18) << p_WS_W[0] << ", " << p_WS_W[1]
                << ", " << p_WS_W[2] << ", " << q_WS.x() << ", " << q_WS.y() << ", " << q_WS.z() << ", " << q_WS.w()
                << ", " << speedAndBiases[0] << ", " << speedAndBiases[1] << ", " << speedAndBiases[2] << ", "
                << speedAndBiases[3] << ", " << speedAndBiases[4] << ", " << speedAndBiases[5] << ", "
                << speedAndBiases[6] << ", " << speedAndBiases[7] << ", " << speedAndBiases[8];
      for (size_t i = 0; i < extrinsics.size(); ++i) {
        Eigen::Vector3d p_SCi = extrinsics[i].r();
        Eigen::Quaterniond q_SCi = extrinsics[i].q();
        *csvFile_ << ", " << p_SCi[0] << ", " << p_SCi[1] << ", " << p_SCi[2] << ", " << q_SCi.x() << ", " << q_SCi.y()
                  << ", " << q_SCi.z() << ", " << q_SCi.w();
      }
      *csvFile_ << std::endl;
    }
  }
}

// Set and publish landmarks.
void Publisher::publishLandmarksAsCallback(const okvis::Time& /*t*/,
                                           const okvis::MapPointVector& actualLandmarks,
                                           const okvis::MapPointVector& transferredLandmarks) {
  // ROS_WARN("Landmarks Callback");
  if (parameters_.publishing.publishLandmarks) {
    okvis::MapPointVector empty;
    setPoints(actualLandmarks, empty, transferredLandmarks);
    publishPoints();
  }
}

// Set and write landmarks to file.
void Publisher::csvSaveLandmarksAsCallback(const okvis::Time& /*t*/,
                                           const okvis::MapPointVector& actualLandmarks,
                                           const okvis::MapPointVector& transferredLandmarks) {
  RCLCPP_WARN(node_->get_logger(), "Landmarks Callback csv");
  okvis::MapPointVector empty;
  setPoints(actualLandmarks, empty, transferredLandmarks);
  if (csvLandmarksFile_) {
    if (csvLandmarksFile_->good()) {
      for (size_t l = 0; l < actualLandmarks.size(); ++l) {
        Eigen::Vector4d landmark = actualLandmarks.at(l).point;
        *csvLandmarksFile_ << std::setprecision(19) << actualLandmarks.at(l).id << ", " << std::scientific
                           << std::setprecision(18) << landmark[0] << ", " << landmark[1] << ", " << landmark[2] << ", "
                           << landmark[3] << ", "
                           << actualLandmarks.at(l).quality
                           // << ", " << actualLandmarks.at(l).distance
                           << std::endl;
      }
    }
  }
}

// Publish the last set points.
void Publisher::publishPoints() {
  auto pcl_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(pointsMatched_, *pcl_msg);
  pcl_msg->header.frame_id = "world";

  pubPointsMatched_->publish(*pcl_msg);
  pcl::toROSMsg(pointsUnmatched_, *pcl_msg);
  pcl_msg->header.frame_id = "world";
  pubPointsUnmatched_->publish(*pcl_msg);
  pcl::toROSMsg(pointsTransferred_, *pcl_msg);
  pcl_msg->header.frame_id = "world";
  pubPointsTransferred_->publish(*pcl_msg);
}

// Set the images to be published next.
void Publisher::setImages(const std::vector<cv::Mat>& images) {
  // copy over
  images_.resize(images.size());
  for (size_t i = 0; i < images.size(); ++i) images_[i] = images[i];
}

// Add a pose to the path that is published next. The path contains a maximum of
// maxPathLength poses that are published. Once the
// maximum is reached, the last pose is copied in a new path message. The rest are deleted.
void Publisher::setPath(const okvis::kinematics::Transformation& T_WS) {
  if (path_.poses.size() >= parameters_.publishing.maxPathLength) {
    geometry_msgs::msg::PoseStamped lastPose = path_.poses.back();
    path_.poses.clear();
    path_.poses.reserve(parameters_.publishing.maxPathLength);
    path_.poses.push_back(lastPose);
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = _t;
  pose.header.frame_id = "world";
  okvis::kinematics::Transformation T = parameters_.publishing.T_Wc_W * T_WS;

  // put the path into the origin of the selected tracked frame
  if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    // nothing
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    T = T * parameters_.imu.T_BS.inverse();
  } else {
    LOG(ERROR) << "Pose frame does not exist for publishing. Choose 'S' or 'B'.";
    T = T * parameters_.imu.T_BS.inverse();
  }

  const Eigen::Vector3d& r = T.r();
  pose.pose.position.x = r[0];
  pose.pose.position.y = r[1];
  pose.pose.position.z = r[2];
  const Eigen::Quaterniond& q = T.q();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  path_.header.stamp = _t;
  path_.header.frame_id = "world";
  path_.poses.push_back(pose);
}

// Publish the last set images.
void Publisher::publishImages() {
  // advertise what's been missing:
  if (images_.size() != pubImagesVector_.size()) {
    pubImagesVector_.clear();
    for (size_t i = 0; i < images_.size(); ++i) {
      std::stringstream drawingNameStream;
      drawingNameStream << "okvis_drawing_" << i;
      pubImagesVector_.push_back(image_transport_.advertise(drawingNameStream.str(), 10));
    }
  }

  // publish:
  for (size_t i = 0; i < images_.size(); ++i) {
    sensor_msgs::msg::Image msg;
    std::stringstream cameraNameStream;
    cameraNameStream << "camera_" << i;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = cameraNameStream.str();
    sensor_msgs::fillImage(msg,
                           sensor_msgs::image_encodings::MONO8,
                           images_[i].rows,
                           images_[i].cols,
                           images_[i].step.buf[0],
                           images_[i].data);
    pubImagesVector_[i].publish(msg);
  }
}

// Publish the last set path.
void Publisher::publishPath() { pubPath_->publish(path_); }

void Publisher::publishDebugImageAsCallback(const okvis::Time& t, int i, const cv::Mat& image) {
  // publish debug image

  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
  msg->header.stamp = rclcpp::Time(t.sec, t.nsec);

  // Expand the number of publishers if needed
  for (int j = pubDebugImage_.size(); j <= i; j++) {
    std::stringstream topic;
    topic << "debug_image_" << j;
    pubDebugImage_.push_back(node_->create_publisher<sensor_msgs::msg::Image>(topic.str(), 10));
  }

  // pubDebugImage_[i].publish(msg);
  pubDebugImage_[i]->publish(*msg);
}

void Publisher::publishSensorStaticTf() {
  size_t num_cameras = parameters_.nCameraSystem.numCameras();

  if (!static_tf_published_) {
    for (size_t i = 0; i < num_cameras; ++i) {
      publishStaticTfCamera(i);
    }

    if (parameters_.sensorList.isSonarUsed) publishStaticTfSonar();

    static_tf_published_ = true;
  }
}
void Publisher::publishStaticTfCamera(size_t camera_index) {
  geometry_msgs::msg::Transform pose;
  okvis::cameras::NCameraSystem nCameraSystem = parameters_.nCameraSystem;

  std::string parent_frame_id;
  if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    parent_frame_id = "sensor";
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    parent_frame_id = "body";
  }

  std::shared_ptr<const okvis::kinematics::Transformation> T = nCameraSystem.T_SC(camera_index);

  okvis::kinematics::Transformation t_body_imu = parameters_.imu.T_BS;
  okvis::kinematics::Transformation result = t_body_imu * (*T);
  std::string child_frame_id = "cam" + std::to_string(camera_index);

  // fill orientation
  Eigen::Quaterniond q = result.q();
  pose.rotation.x = q.x();
  pose.rotation.y = q.y();
  pose.rotation.z = q.z();
  pose.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = result.r();
  pose.translation.x = r[0];
  pose.translation.y = r[1];
  pose.translation.z = r[2];

  publishStaticTf(pose, parent_frame_id, child_frame_id);
}

void Publisher::publishStaticTfSonar() {
  geometry_msgs::msg::Transform pose;  ///< Pose message.

  std::string parent_frame_id;
  if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    parent_frame_id = "sensor";
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    parent_frame_id = "body";
  }

  okvis::kinematics::Transformation T_SSo = parameters_.sonar.T_SSo;
  okvis::kinematics::Transformation t_body_imu = parameters_.imu.T_BS;
  okvis::kinematics::Transformation result = t_body_imu * T_SSo;
  std::string child_frame_id = "sonar";

  // fill orientation
  Eigen::Quaterniond q = result.q();
  pose.rotation.x = q.x();
  pose.rotation.y = q.y();
  pose.rotation.z = q.z();
  pose.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = result.r();
  pose.translation.x = r[0];
  pose.translation.y = r[1];
  pose.translation.z = r[2];

  publishStaticTf(pose, parent_frame_id, child_frame_id);
}

void Publisher::publishStaticTf(const geometry_msgs::msg::Transform& pose,
                                const std::string& parent_frame_id,
                                const std::string& child_frame_id) {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster(node_);
  geometry_msgs::msg::TransformStamped static_transform_stamped;
  static_transform_stamped.header.stamp = _t;
  if ((node_->get_clock()->now().seconds() - _t.seconds()) > 10.0)
    static_transform_stamped.header.stamp = node_->get_clock()->now();

  // TODO(Toni): Warning: using rclcpp::Time::now(), will that bring issues?
  static_transform_stamped.header.stamp = node_->now();
  static_transform_stamped.header.frame_id = parent_frame_id;
  static_transform_stamped.child_frame_id = child_frame_id;
  static_transform_stamped.transform = pose;
  static_broadcaster.sendTransform(static_transform_stamped);
}

}  // namespace okvis
