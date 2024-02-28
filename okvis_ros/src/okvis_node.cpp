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
#include <glog/logging.h>
#include <image_transport/image_transport.h>

#include <okvis/Publisher.hpp>
#include <okvis/RosParametersReader.hpp>
#include <okvis/Subscriber.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <string>

#include "sensor_msgs/Imu.h"

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

  // subscriber
  okvis::Subscriber subscriber(nh, &okvis_estimator, vio_parameters_reader);

  publisher.setParameters(parameters);  // pass the specified publishing stuff

  okvis_estimator.setFullStateCallback(std::bind(&okvis::Publisher::publishFullStateAsCallback,
                                                 publisher,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::placeholders::_4));
  okvis_estimator.setLandmarksCallback(std::bind(&okvis::Publisher::publishLandmarksAsCallback,
                                                 publisher,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3));
  okvis_estimator.setStateCallback(
      std::bind(&okvis::Publisher::publishStateAsCallback, publisher, std::placeholders::_1, std::placeholders::_2));
  // okvis_estimator->setBlocking(true);
  // Sharmin
  // okvis_estimator->setStereoMatchCallback(std::bind(&okvis::Publisher::publishSteroPointCloudAsCallback,publisher,std::placeholders::_1,std::placeholders::_2));
  // Sharmin
  okvis_estimator.setKeyframeCallback(std::bind(&okvis::Publisher::publishKeyframeAsCallback,
                                                publisher,
                                                std::placeholders::_1,
                                                std::placeholders::_2,
                                                std::placeholders::_3,
                                                std::placeholders::_4));

  // Hunter
  if (parameters.visualization.publishDebugImages) {
    okvis_estimator.setDebugImgCallback(std::bind(&okvis::Publisher::publishDebugImageAsCallback,
                                                  publisher,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2,
                                                  std::placeholders::_3));
  }

  // Like okvis_node_synchronous to setup files to be written
  okvis_estimator.setImuCsvFile("imu_data.csv");
  for (size_t i = 0; i < 2; ++i) {
    std::stringstream num;
    num << i + 1;
    okvis_estimator.setTracksCsvFile(i, "slave" + num.str() + "_tracks.csv");
  }

  while (ros::ok()) {
    ros::spinOnce();
    okvis_estimator.display();
  }

  return 0;
}
