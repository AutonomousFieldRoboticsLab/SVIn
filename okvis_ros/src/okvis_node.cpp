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
 * @file okvis_node.cpp
 * @brief This file includes the ROS node implementation.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <stdlib.h>
#include <fstream>
#include <functional>
#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <image_transport/image_transport.h>
#include "sensor_msgs/Imu.h"

#include <glog/logging.h>

#include <okvis/Publisher.hpp>
#include <okvis/RosParametersReader.hpp>
#include <okvis/Subscriber.hpp>
#include <okvis/ThreadedKFVio.hpp>

// Hunter
#include <okvis_ros/OdometryTrigger.h>
#include <std_srvs/Trigger.h>

bool is_reloc = true;

namespace okvis {
void initEstimator(ThreadedKFVio* okvis_estimator, Publisher* publisher, VioParameters& parameters) {
  /****** Hunter: moved all okvis_estimator initialization here to be resetable ****/
  publisher->setParameters(parameters);  // pass the specified publishing stuff

  okvis_estimator->setFullStateCallback(std::bind(&okvis::Publisher::publishFullStateAsCallback,
                                                  publisher,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2,
                                                  std::placeholders::_3,
                                                  std::placeholders::_4,
                                                  std::placeholders::_5));
  okvis_estimator->setLandmarksCallback(std::bind(&okvis::Publisher::publishLandmarksAsCallback,
                                                  publisher,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2,
                                                  std::placeholders::_3));
  okvis_estimator->setStateCallback(
      std::bind(&okvis::Publisher::publishStateAsCallback, publisher, std::placeholders::_1, std::placeholders::_2));
  // okvis_estimator->setBlocking(true);
  // Sharmin
  // okvis_estimator->setStereoMatchCallback(std::bind(&okvis::Publisher::publishSteroPointCloudAsCallback,publisher,std::placeholders::_1,std::placeholders::_2));
  // Sharmin
  okvis_estimator->setKeyframeCallback(std::bind(&okvis::Publisher::publishKeyframeAsCallback,
                                                 publisher,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::placeholders::_4));

  // Hunter
  if (parameters.visualization.publishDebugImages) {
    okvis_estimator->setDebugImgCallback(std::bind(&okvis::Publisher::publishDebugImageAsCallback,
                                                   publisher,
                                                   std::placeholders::_1,
                                                   std::placeholders::_2,
                                                   std::placeholders::_3));
  }

  if (is_reloc) {
    okvis_estimator->setRelocRelativePoseCallback(std::bind(&okvis::Publisher::publishRelocRelativePoseAsCallback,
                                                            publisher,
                                                            std::placeholders::_1,
                                                            std::placeholders::_2,
                                                            std::placeholders::_3,
                                                            std::placeholders::_4,
                                                            std::placeholders::_5));
  }

  // Like okvis_node_synchronous to setup files to be written
  okvis_estimator->setImuCsvFile("imu_data.csv");
  for (size_t i = 0; i < 2; ++i) {
    std::stringstream num;
    num << i + 1;
    okvis_estimator->setTracksCsvFile(i, "slave" + num.str() + "_tracks.csv");
  }
}

okvis::kinematics::Transformation odometryToTransformation(const nav_msgs::Odometry& msg) {
  const geometry_msgs::Pose* pose = &msg.pose.pose;
  Eigen::Vector3d pos(pose->position.x, pose->position.y, pose->position.z);
  Eigen::Quaterniond quat(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);
  return okvis::kinematics::Transformation(pos, quat);
}

bool reset(ThreadedKFVio* okvis_estimator,
           Publisher* publisher,
           Subscriber* subscriber,
           VioParameters& parameters,
           const okvis::kinematics::Transformation& orig_T_Wc_W,
           okvis_ros::OdometryTrigger::Request& request,
           okvis_ros::OdometryTrigger::Response& response) {
  okvis_estimator->~ThreadedKFVio();
  parameters.publishing.T_Wc_W = odometryToTransformation(request.pose) * orig_T_Wc_W;
  new (okvis_estimator) ThreadedKFVio(parameters);
  initEstimator(okvis_estimator, publisher, parameters);
  subscriber->setT_Wc_W(parameters.publishing.T_Wc_W);
  response.success = true;
  return response.success;
}
bool resetZero(ThreadedKFVio* okvis_estimator,
               Publisher* publisher,
               Subscriber* subscriber,
               VioParameters& parameters,
               const okvis::kinematics::Transformation& orig_T_Wc_W,
               std_srvs::Trigger::Request& request,
               std_srvs::Trigger::Response& response) {
  okvis_ros::OdometryTrigger::Request odom_request;
  okvis_ros::OdometryTrigger::Response odom_response;
  odom_request.pose.pose.pose.orientation.w = 1.0;  // Call reset with identity transformation
  reset(okvis_estimator, publisher, subscriber, parameters, orig_T_Wc_W, odom_request, odom_response);
  response.success = odom_response.success;
  return response.success;
}
bool smoothReset(Publisher* publisher,
                 Subscriber* subscriber,
                 const okvis::kinematics::Transformation& orig_T_Wc_W,
                 okvis_ros::OdometryTrigger::Request& request,
                 okvis_ros::OdometryTrigger::Response& response) {
  okvis::kinematics::Transformation new_T_Wc_W = odometryToTransformation(request.pose) * orig_T_Wc_W;
  publisher->setT_Wc_W(new_T_Wc_W);
  subscriber->setT_Wc_W(new_T_Wc_W);
  response.success = true;
  return response.success;
}
}  // namespace okvis

int main(int argc, char** argv) {
  ros::init(argc, argv, "okvis_node");

  // set up the node
  ros::NodeHandle nh("okvis_node");

  // initialise logging
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // publisher
  okvis::Publisher publisher(nh);

  // read configuration file
  std::string configFilename;
  if (!nh.getParam("config_filename", configFilename)) {
    LOG(ERROR) << "Please specify filename of configuration!";
    return 1;
  }
  okvis::RosParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);
  okvis::kinematics::Transformation orig_T_Wc_W = parameters.publishing.T_Wc_W;

  okvis::ThreadedKFVio okvis_estimator(parameters);

  // Like okvis_node_synchronous to setup files to be written
  publisher.setCsvFile("okvis_estimator_output.csv");
  publisher.setLandmarksCsvFile("okvis_estimator_landmarks.csv");

  okvis::initEstimator(&okvis_estimator,
                       &publisher,
                       parameters);  // Hunter moved initialization of okvis_estimator to a function for resetability

  // subscriber
  okvis::Subscriber subscriber(nh, &okvis_estimator, vio_parameters_reader);

  ros::ServiceServer srvReset_, srvResetZero_, srvSmoothReset_;
  if (parameters.resetableParams.isResetable) {
    const boost::function<bool(okvis_ros::OdometryTrigger::Request&, okvis_ros::OdometryTrigger::Response&)>
        resetFunction = std::bind(&okvis::reset,
                                  &okvis_estimator,
                                  &publisher,
                                  &subscriber,
                                  parameters,
                                  orig_T_Wc_W,
                                  std::placeholders::_1,
                                  std::placeholders::_2);

    const boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> resetZeroFunction =
        std::bind(&okvis::resetZero,
                  &okvis_estimator,
                  &publisher,
                  &subscriber,
                  parameters,
                  orig_T_Wc_W,
                  std::placeholders::_1,
                  std::placeholders::_2);

    const boost::function<bool(okvis_ros::OdometryTrigger::Request&, okvis_ros::OdometryTrigger::Response&)>
        smoothResetFunction = std::bind(
            &okvis::smoothReset, &publisher, &subscriber, orig_T_Wc_W, std::placeholders::_1, std::placeholders::_2);

    srvReset_ = nh.advertiseService("reset", resetFunction);
    srvResetZero_ = nh.advertiseService("reset_zero", resetZeroFunction);
    srvSmoothReset_ = nh.advertiseService("smooth_reset", smoothResetFunction);
  }

  while (ros::ok()) {
    ros::spinOnce();
    okvis_estimator.display();
  }

  return 0;
}
