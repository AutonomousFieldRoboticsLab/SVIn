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
 *  Created on: Mar 23, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Subscriber.cpp
 * @brief Source file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>

#include <functional>
#include <memory>
#include <okvis/Subscriber.hpp>
#include <vector>

#define THRESHOLD_DATA_DELAY_WARNING 0.1  // in seconds

/// \brief okvis Main namespace of this package.
namespace okvis {

Subscriber::~Subscriber() { imgTransport_.release(); }

Subscriber::Subscriber(std::shared_ptr<rclcpp::Node> node,
                       okvis::VioInterface* vioInterfacePtr,
                       const okvis::VioParametersReader& param_reader)
    : node_(node), vioInterface_(vioInterfacePtr), imgTransport_(nullptr) {
  /// @Sharmin
  param_reader.getParameters(vioParameters_);

  imageSubscribers_.resize(vioParameters_.nCameraSystem.numCameras());

  imgTransport_ = std::make_unique<image_transport::ImageTransport>(node);

  // setup callback groups
  auto svin2_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = svin2_callback_group;

  // Set up Camera callbacks
  for (size_t i = 0; i < vioParameters_.nCameraSystem.numCameras(); ++i) {
    imageSubscribers_[i] =
        imgTransport_->subscribe("camera" + std::to_string(i),
                                 30 * vioParameters_.nCameraSystem.numCameras(),
                                 std::bind(&Subscriber::imageCallback, this, std::placeholders::_1, i));
  }

  // Set up IMU callback
  subImu_ = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 1000, std::bind(&Subscriber::imuCallback, this, std::placeholders::_1));
  //subImu_ = node->create_subscription<sensor_msgs::msg::Imu>(
  //    "imu", rclcpp::SensorDataQoS(), std::bind(&Subscriber::imuCallback, this, std::placeholders::_1), options);

  // Sharmin
  // if (vioParameters_.sensorList.isSonarUsed) {
  //   subSonarRange_ = nh_->subscribe("/imagenex831l/range", 1000, &Subscriber::sonarCallback, this);
  // }
  // Sharmin
  // if (vioParameters_.sensorList.isDepthUsed){
  // subDepth_ = nh_->subscribe("/bar30/depth", 1000, &Subscriber::depthCallback, this);
  // subDepth_ = nh_->subscribe("/aqua/state", 1000, &Subscriber::depthCallback, this); // Aqua depth topic
  // }

  // Sharmin
  // if (vioParameters_.relocParameters.isRelocalization) {
  //   std::cout << "Subscribing to /pose_graph/match_points topic" << std::endl;
  //   subReloPoints_ = node->create_subscription<sensor_msgs::msg::PointCloud>(
  //       "/pose_graph/match_points", 1000, std::bind(&Subscriber::relocCallback, this, std::placeholders::_1));
  // }

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  imgTransport_ = 0;

  imgLeftCounter = 0;   // @Sharmin
  imgRightCounter = 0;  // @Sharmin
  // Added by Sharmin
  if (vioParameters_.histogramParams.histogramMethod == HistogramMethod::CLAHE) {
    clahe = cv::createCLAHE();
    clahe->setClipLimit(vioParameters_.histogramParams.claheClipLimit);
    clahe->setTilesGridSize(
        cv::Size(vioParameters_.histogramParams.claheTilesGridSize, vioParameters_.histogramParams.claheTilesGridSize));

    std::cout << "Set Clahe Params " << vioParameters_.histogramParams.claheClipLimit << " "
              << vioParameters_.histogramParams.claheTilesGridSize << std::endl;
  }
}

// Hunter
void Subscriber::setT_Wc_W(okvis::kinematics::Transformation T_Wc_W) { vioParameters_.publishing.T_Wc_W = T_Wc_W; }

void Subscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, unsigned int cameraIndex) {
  const cv::Mat raw = readRosImage(msg);

  // resizing factor( e.g., with a factor = 0.8, an image will convert from 800x600 to 640x480)
  cv::Mat raw_resized;
  if (vioParameters_.miscParams.resizeFactor != 1.0) {
    cv::resize(
        raw, raw_resized, cv::Size(), vioParameters_.miscParams.resizeFactor, vioParameters_.miscParams.resizeFactor);
  } else {
    raw_resized = raw.clone();
  }

  cv::Mat filtered;
  if (vioParameters_.optimization.useMedianFilter) {
    cv::medianBlur(raw_resized, filtered, 3);
  } else {
    filtered = raw_resized.clone();
  }

  // Added by Sharmin for CLAHE
  cv::Mat histogram_equalized_image;
  if (vioParameters_.histogramParams.histogramMethod == HistogramMethod::CLAHE) {
    clahe->apply(filtered, histogram_equalized_image);
  } else if (vioParameters_.histogramParams.histogramMethod == HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(filtered, histogram_equalized_image);
  } else {
    histogram_equalized_image = filtered;
  }
  // End Added by Sharmin

  // adapt timestamp
  okvis::Time t(msg->header.stamp.sec, msg->header.stamp.nanosec);
  t -= okvis::Duration(vioParameters_.sensors_information.imageDelay);

  if (!vioInterface_->addImage(t, cameraIndex, histogram_equalized_image)) {
    LOG(WARNING) << "Frame delayed at time " << t;
  }
}

void Subscriber::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  vioInterface_->addImuMeasurement(
      okvis::Time(msg->header.stamp.sec, msg->header.stamp.nanosec),
      Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
      Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z));
}

// @Sharmin
void Subscriber::relocCallback(const sensor_msgs::msg::PointCloud::SharedPtr relo_msg) {
  std::vector<Eigen::Vector3d> matched_ids;
  okvis::kinematics::Transformation T_Wc_W = vioParameters_.publishing.T_Wc_W;
  // double frame_stamp = relo_msg->header.stamp.toSec();
  for (unsigned int i = 0; i < relo_msg->points.size(); i++) {
    // landmarkId, mfId/poseId, keypointIdx for Every Matched 3d points in Current frame
    Eigen::Vector4d pt_ids;
    pt_ids.x() = relo_msg->points[i].x;
    pt_ids.y() = relo_msg->points[i].y;
    pt_ids.z() = relo_msg->points[i].z;
    pt_ids.w() = 1.0;
    pt_ids = T_Wc_W.inverse() * pt_ids;  // Hunter: Transform reloc points
    matched_ids.push_back(pt_ids.segment<3>(0));
  }
  Eigen::Vector3d pos(
      relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
  Eigen::Quaterniond quat(relo_msg->channels[0].values[3],
                          relo_msg->channels[0].values[4],
                          relo_msg->channels[0].values[5],
                          relo_msg->channels[0].values[6]);
  // Eigen::Matrix3d relo_r = relo_q.toRotationMatrix();

  // Hunter: Transform reloc pose
  okvis::kinematics::Transformation pose_Wc(pos, quat);
  okvis::kinematics::Transformation pose_W = T_Wc_W.inverse() * pose_Wc;

  // estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);

  // vioInterface_->addRelocMeasurement(
  //     okvis::Time(relo_msg->header.stamp.sec, relo_msg->header.stamp.nanosec), matched_ids, pose_W.r(), pose_W.q());
}
// @Sharmin
// /*
// // Aqua depth topic subscription
// void Subscriber::depthCallback(const aquacore::StateMsg::ConstPtr& msg)
// {
//         vioInterface_->addDepthMeasurement(
//                                           okvis::Time(msg->header.stamp.sec, msg->header.stamp.nsec),
//                                           msg->Depth);
// }
// */

// /*
// // stereo rig depth topic subscription
// void Subscriber::depthCallback(const depth_node_py::Depth::ConstPtr& msg)
// {
//         vioInterface_->addDepthMeasurement(
//                                           okvis::Time(msg->header.stamp.sec, msg->header.stamp.nsec),
//                                           msg->depth);
// }
// * /

// @Sharmin
// void Subscriber::sonarCallback(const imagenex831l::ProcessedRange::ConstPtr& msg) {
//   double rangeResolution = msg->max_range / msg->intensity.size();
//   int max = 0;
//   int maxIndex = 0;

//   // @Sharmin: discarding as range was set higher (which introduced some noisy data) during data collection
//   for (unsigned int i = 0; i < msg->intensity.size() - 100; i++) {
//     if (msg->intensity[i] > max) {
//       max = msg->intensity[i];
//       maxIndex = i;
//     }
//   }

//   double range = (maxIndex + 1) * rangeResolution;
//   double heading = (msg->head_position * M_PI) / 180;

//   // No magic no!! within 4.5 meter
//   if (range < 4.5 && max > 10) {
//     vioInterface_->addSonarMeasurement(okvis::Time(msg->header.stamp.sec, msg->header.stamp.nsec), range, heading);
//   }
// }

const cv::Mat Subscriber::readRosImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) const {
  CHECK(img_msg);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
    RCLCPP_INFO(node_->get_logger(), "Received image - encoding: %s, width: %d, height: %d, size: %zu",
                img_msg->encoding.c_str(),
                img_msg->width,
                img_msg->height,
                img_msg->data.size());
  } catch (cv_bridge::Exception& exception) {
    RCLCPP_FATAL(node_->get_logger(), "cv_bridge exception: %s", exception.what());
    rclcpp::shutdown();
  }

  CHECK(cv_ptr);
  const cv::Mat img_const = cv_ptr->image;  // Don't modify shared image in ROS.
  cv::Mat converted_img(img_const.size(), CV_8U);
  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    // LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    // LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    return converted_img;
  } else {
    CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
        << "Expected image with MONO8, BGR8, or RGB8 encoding."
           "Add in here more conversions if you wish.";
    return img_const;
  }
}

}  // namespace okvis
