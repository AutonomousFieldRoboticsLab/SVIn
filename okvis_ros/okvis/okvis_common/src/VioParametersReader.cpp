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
 *  Created on: Jun 17, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioParametersReader.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>

#include <algorithm>
#include <memory>
#include <okvis/VioParametersReader.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
VioParametersReader::VioParametersReader() : useDriver(false), readConfigFile_(false) {
  vioParameters_.publishing.publishRate = 0;
}

// The constructor. This calls readConfigFile().
VioParametersReader::VioParametersReader(const std::string& filename) {
  // reads
  readConfigFile(filename);
}

// Read and parse a config file.
void VioParametersReader::readConfigFile(const std::string& filename) {
  vioParameters_.optimization.useMedianFilter = false;
  vioParameters_.optimization.timeReserve.fromSec(0.005);

  // reads
  cv::FileStorage file(filename, cv::FileStorage::READ);

  OKVIS_ASSERT_TRUE(Exception, file.isOpened(), "Could not open config file: " << filename);
  LOG(INFO) << "Opened configuration file: " << filename;

  // number of keyframes
  if (file["numKeyframes"].isInt()) {
    file["numKeyframes"] >> vioParameters_.optimization.numKeyframes;
  } else {
    LOG(WARNING) << "numKeyframes parameter not provided. Setting to default numKeyframes=5.";
    vioParameters_.optimization.numKeyframes = 5;
  }
  // number of IMU frames
  if (file["numImuFrames"].isInt()) {
    file["numImuFrames"] >> vioParameters_.optimization.numImuFrames;
  } else {
    LOG(WARNING) << "numImuFrames parameter not provided. Setting to default numImuFrames=2.";
    vioParameters_.optimization.numImuFrames = 2;
  }
  // minimum ceres iterations
  if (file["ceres_options"]["minIterations"].isInt()) {
    file["ceres_options"]["minIterations"] >> vioParameters_.optimization.min_iterations;
  } else {
    LOG(WARNING) << "ceres_options: minIterations parameter not provided. Setting to default minIterations=1";
    vioParameters_.optimization.min_iterations = 1;
  }
  // maximum ceres iterations
  if (file["ceres_options"]["maxIterations"].isInt()) {
    file["ceres_options"]["maxIterations"] >> vioParameters_.optimization.max_iterations;
  } else {
    LOG(WARNING) << "ceres_options: maxIterations parameter not provided. Setting to default maxIterations=10.";
    vioParameters_.optimization.max_iterations = 10;
  }
  // ceres time limit
  if (file["ceres_options"]["timeLimit"].isReal()) {
    file["ceres_options"]["timeLimit"] >> vioParameters_.optimization.timeLimitForMatchingAndOptimization;
  } else {
    LOG(WARNING) << "ceres_options: timeLimit parameter not provided. Setting no time limit.";
    vioParameters_.optimization.timeLimitForMatchingAndOptimization = -1.0;
  }

  // do we use the direct driver?
  bool success = parseBoolean(file["useDriver"], useDriver);
  OKVIS_ASSERT_TRUE(Exception, success, "'useDriver' parameter missing in configuration file.");

  // display images?
  success = parseBoolean(file["displayImages"], vioParameters_.visualization.displayImages);
  OKVIS_ASSERT_TRUE(Exception, success, "'displayImages' parameter missing in configuration file.");

  // Hunter
  success = parseBoolean(file["publishDebugImages"], vioParameters_.visualization.publishDebugImages);
  if (!success)
    vioParameters_.visualization.publishDebugImages = false;  // Default to false for backwards compatibility

  // detection threshold
  success = file["detection_options"]["threshold"].isReal();
  OKVIS_ASSERT_TRUE(Exception, success, "'detection threshold' parameter missing in configuration file.");
  file["detection_options"]["threshold"] >> vioParameters_.optimization.detectionThreshold;

  // detection octaves
  success = file["detection_options"]["octaves"].isInt();
  OKVIS_ASSERT_TRUE(Exception, success, "'detection octaves' parameter missing in configuration file.");
  file["detection_options"]["octaves"] >> vioParameters_.optimization.detectionOctaves;
  OKVIS_ASSERT_TRUE(Exception, vioParameters_.optimization.detectionOctaves >= 0, "Invalid parameter value.");

  // maximum detections
  success = file["detection_options"]["maxNoKeypoints"].isInt();
  OKVIS_ASSERT_TRUE(Exception, success, "'detection maxNoKeypoints' parameter missing in configuration file.");
  file["detection_options"]["maxNoKeypoints"] >> vioParameters_.optimization.maxNoKeypoints;
  OKVIS_ASSERT_TRUE(Exception, vioParameters_.optimization.maxNoKeypoints >= 0, "Invalid parameter value.");

  // image delay
  success = file["imageDelay"].isReal();
  OKVIS_ASSERT_TRUE(Exception, success, "'imageDelay' parameter missing in configuration file.");
  file["imageDelay"] >> vioParameters_.sensors_information.imageDelay;
  LOG(INFO) << "imageDelay=" << vioParameters_.sensors_information.imageDelay;

  // camera rate
  success = file["camera_params"]["camera_rate"].isInt();
  OKVIS_ASSERT_TRUE(Exception, success, "'camera_params: camera_rate' parameter missing in configuration file.");
  file["camera_params"]["camera_rate"] >> vioParameters_.sensors_information.cameraRate;

  // timestamp tolerance
  if (file["camera_params"]["timestamp_tolerance"].isReal()) {
    file["camera_params"]["timestamp_tolerance"] >> vioParameters_.sensors_information.frameTimestampTolerance;
    OKVIS_ASSERT_TRUE(Exception,
                      vioParameters_.sensors_information.frameTimestampTolerance <
                          0.5 / vioParameters_.sensors_information.cameraRate,
                      "Timestamp tolerance for stereo frames is larger than half the time between frames.");
    OKVIS_ASSERT_TRUE(Exception,
                      vioParameters_.sensors_information.frameTimestampTolerance >= 0.0,
                      "Timestamp tolerance is smaller than 0");
  } else {
    vioParameters_.sensors_information.frameTimestampTolerance = 0.2 / vioParameters_.sensors_information.cameraRate;
    LOG(WARNING) << "No timestamp tolerance for stereo frames specified. Setting to "
                 << vioParameters_.sensors_information.frameTimestampTolerance;
  }

  // camera params
  if (file["camera_params"]["sigma_absolute_translation"].isReal()) {
    file["camera_params"]["sigma_absolute_translation"] >> vioParameters_.camera_extrinsics.sigma_absolute_translation;
  } else {
    vioParameters_.camera_extrinsics.sigma_absolute_translation = 0.0;
    LOG(WARNING) << "camera_params: sigma_absolute_translation parameter not provided. Setting to default 0.0";
  }
  if (file["camera_params"]["sigma_absolute_orientation"].isReal()) {
    file["camera_params"]["sigma_absolute_orientation"] >> vioParameters_.camera_extrinsics.sigma_absolute_orientation;
  } else {
    vioParameters_.camera_extrinsics.sigma_absolute_orientation = 0.0;
    LOG(WARNING) << "camera_params: sigma_absolute_orientation parameter not provided. Setting to default 0.0";
  }
  if (file["camera_params"]["sigma_c_relative_translation"].isReal()) {
    file["camera_params"]["sigma_c_relative_translation"] >>
        vioParameters_.camera_extrinsics.sigma_c_relative_translation;
  } else {
    vioParameters_.camera_extrinsics.sigma_c_relative_translation = 0.0;
    LOG(WARNING) << "camera_params: sigma_c_relative_translation parameter not provided. Setting to default 0.0";
  }
  if (file["camera_params"]["sigma_c_relative_orientation"].isReal()) {
    file["camera_params"]["sigma_c_relative_orientation"] >>
        vioParameters_.camera_extrinsics.sigma_c_relative_orientation;
  } else {
    vioParameters_.camera_extrinsics.sigma_c_relative_orientation = 0.0;
    LOG(WARNING) << "camera_params: sigma_c_relative_orientation parameter not provided. Setting to default 0.0";
  }

  if (file["publishing_options"]["publish_rate"].isInt()) {
    file["publishing_options"]["publish_rate"] >> vioParameters_.publishing.publishRate;
  }

  if (file["publishing_options"]["landmarkQualityThreshold"].isReal()) {
    file["publishing_options"]["landmarkQualityThreshold"] >> vioParameters_.publishing.landmarkQualityThreshold;
  }

  if (file["publishing_options"]["maximumLandmarkQuality"].isReal()) {
    file["publishing_options"]["maximumLandmarkQuality"] >> vioParameters_.publishing.maxLandmarkQuality;
  }

  if (file["publishing_options"]["maxPathLength"].isInt()) {
    vioParameters_.publishing.maxPathLength = static_cast<int>(file["publishing_options"]["maxPathLength"]);
  }

  parseBoolean(file["publishing_options"]["publishImuPropagatedState"],
               vioParameters_.publishing.publishImuPropagatedState);

  parseBoolean(file["publishing_options"]["publishLandmarks"], vioParameters_.publishing.publishLandmarks);

  cv::FileNode T_Wc_W_ = file["publishing_options"]["T_Wc_W"];
  if (T_Wc_W_.isSeq()) {
    Eigen::Matrix4d T_Wc_W_e;
    T_Wc_W_e << T_Wc_W_[0], T_Wc_W_[1], T_Wc_W_[2], T_Wc_W_[3], T_Wc_W_[4], T_Wc_W_[5], T_Wc_W_[6], T_Wc_W_[7],
        T_Wc_W_[8], T_Wc_W_[9], T_Wc_W_[10], T_Wc_W_[11], T_Wc_W_[12], T_Wc_W_[13], T_Wc_W_[14], T_Wc_W_[15];

    vioParameters_.publishing.T_Wc_W = okvis::kinematics::Transformation(T_Wc_W_e);
    std::stringstream s;
    s << vioParameters_.publishing.T_Wc_W.T();
    LOG(INFO) << "Custom World frame provided T_Wc_W=\n" << s.str();
  }

  if (file["publishing_options"]["trackedBodyFrame"].isString()) {
    std::string frame = (std::string)file["publishing_options"]["trackedBodyFrame"];
    // cut out first word. str currently contains everything including comments
    frame = frame.substr(0, frame.find(" "));
    if (frame.compare("B") == 0) {
      vioParameters_.publishing.trackedBodyFrame = FrameName::B;
    } else if (frame.compare("S") == 0) {
      vioParameters_.publishing.trackedBodyFrame = FrameName::S;
    } else {
      LOG(WARNING) << frame << " unknown/invalid frame for trackedBodyFrame, setting to B";
      vioParameters_.publishing.trackedBodyFrame = FrameName::B;
    }
  }

  if (file["publishing_options"]["velocitiesFrame"].isString()) {
    std::string frame = (std::string)file["publishing_options"]["velocitiesFrame"];
    // cut out first word. str currently contains everything including comments
    frame = frame.substr(0, frame.find(" "));
    if (frame.compare("B") == 0) {
      vioParameters_.publishing.velocitiesFrame = FrameName::B;
    } else if (frame.compare("S") == 0) {
      vioParameters_.publishing.velocitiesFrame = FrameName::S;
    } else if (frame.compare("Wc") == 0) {
      vioParameters_.publishing.velocitiesFrame = FrameName::Wc;
    } else {
      LOG(WARNING) << frame << " unknown/invalid frame for velocitiesFrame, setting to Wc";
      vioParameters_.publishing.velocitiesFrame = FrameName::Wc;
    }
  }

  // *****  Sharmin: Additional config ******//

  if (file["resizeFactor"].isReal()) {
    file["resizeFactor"] >> vioParameters_.miscParams.resizeFactor;
    LOG(INFO) << "Image will be resized with a factor: " << vioParameters_.miscParams.resizeFactor;
  } else {
    vioParameters_.miscParams.resizeFactor = 1.0;
  }

  success = parseBoolean(file["isSonarUsed"], vioParameters_.sensorList.isSonarUsed);
  OKVIS_ASSERT_TRUE(Exception, success, "'isSonarUsed' parameter missing in configuration file.");

  success = parseBoolean(file["isDepthUsed"], vioParameters_.sensorList.isDepthUsed);
  OKVIS_ASSERT_TRUE(Exception, success, "'isDepthUsed' parameter missing in configuration file.");

  if (file["histogramMethod"].isString()) {
    std::string histogram_method = static_cast<std::string>(file["histogramMethod"]);
    if (histogram_method == "NONE" || histogram_method == "none") {
      vioParameters_.histogramParams.histogramMethod = HistogramMethod::NONE;
    } else if (histogram_method == "HISTOGRAM" || histogram_method == "histogram") {
      vioParameters_.histogramParams.histogramMethod = HistogramMethod::HISTOGRAM;
    } else if (histogram_method == "CLAHE" || histogram_method == "clahe") {
      vioParameters_.histogramParams.histogramMethod = HistogramMethod::CLAHE;
      vioParameters_.histogramParams.claheClipLimit = 5.0;
      vioParameters_.histogramParams.claheTilesGridSize = 8;
      if (file["claheClipLimit"].isReal()) {
        vioParameters_.histogramParams.claheClipLimit = static_cast<double>(file["claheClipLimit"]);
      }
      if (file["claheTilesGridSize"].isInt()) {
        vioParameters_.histogramParams.claheTilesGridSize = static_cast<int>(file["claheTilesGridSize"]);
      }

      std::cout << "Read Clahe Params " << vioParameters_.histogramParams.claheClipLimit << " "
                << vioParameters_.histogramParams.claheTilesGridSize << std::endl;
    } else {
      LOG(WARNING) << histogram_method << " unknown/invalid histogramMethod, setting to NONE";
      vioParameters_.histogramParams.histogramMethod = HistogramMethod::NONE;
    }
  } else {
    LOG(WARNING) << "No histogramMethod specified in config file, setting to 'none'";
    vioParameters_.histogramParams.histogramMethod = HistogramMethod::NONE;
  }

  // sonar parameters

  if (vioParameters_.sensorList.isSonarUsed) {
    cv::FileNode T_SSo_ = file["sonar_params"]["T_SSo"];
    OKVIS_ASSERT_TRUE(
        Exception, T_SSo_.isSeq(), "'T_SSo_' parameter missing in the configuration file or in the wrong format.")

    Eigen::Matrix4d T_SSo_e;
    T_SSo_e << T_SSo_[0], T_SSo_[1], T_SSo_[2], T_SSo_[3], T_SSo_[4], T_SSo_[5], T_SSo_[6], T_SSo_[7], T_SSo_[8],
        T_SSo_[9], T_SSo_[10], T_SSo_[11], T_SSo_[12], T_SSo_[13], T_SSo_[14], T_SSo_[15];

    vioParameters_.sonar.T_SSo = okvis::kinematics::Transformation(T_SSo_e);
    std::stringstream ss;
    ss << vioParameters_.sonar.T_SSo.T();
    LOG(INFO) << "Sonar with transformation T_SSo=\n" << ss.str();
  }
  // *****  End Sharmin: Additional config ******//

  // Hunter: Resetable pose parameters
  success = parseBoolean(file["isResetable"], vioParameters_.resetableParams.isResetable);
  if (!success) vioParameters_.resetableParams.isResetable = false;  // Default to false for backwards compatibility

  // camera calibration
  std::vector<CameraCalibration, Eigen::aligned_allocator<CameraCalibration>> calibrations;
  if (!getCameraCalibration(calibrations, file)) LOG(FATAL) << "Did not find any calibration!";

  size_t camIdx = 0;
  for (size_t i = 0; i < calibrations.size(); ++i) {
    std::shared_ptr<const okvis::kinematics::Transformation> T_SC_okvis_ptr(
        new okvis::kinematics::Transformation(calibrations[i].T_SC.r(), calibrations[i].T_SC.q().normalized()));

    if (strcmp(calibrations[i].distortionType.c_str(), "equidistant") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::EquidistantDistortion(calibrations[i].distortionCoefficients[0],
                                                        calibrations[i].distortionCoefficients[1],
                                                        calibrations[i].distortionCoefficients[2],
                                                        calibrations[i].distortionCoefficients[3]) /*, id ?*/)),
          okvis::cameras::NCameraSystem::Equidistant /*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Equidistant pinhole camera " << camIdx << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential") == 0 ||
               strcmp(calibrations[i].distortionType.c_str(), "plumb_bob") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::RadialTangentialDistortion(calibrations[i].distortionCoefficients[0],
                                                             calibrations[i].distortionCoefficients[1],
                                                             calibrations[i].distortionCoefficients[2],
                                                             calibrations[i].distortionCoefficients[3]) /*, id ?*/)),
          okvis::cameras::NCameraSystem::RadialTangential /*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential8") == 0 ||
               strcmp(calibrations[i].distortionType.c_str(), "plumb_bob8") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::RadialTangentialDistortion8(calibrations[i].distortionCoefficients[0],
                                                              calibrations[i].distortionCoefficients[1],
                                                              calibrations[i].distortionCoefficients[2],
                                                              calibrations[i].distortionCoefficients[3],
                                                              calibrations[i].distortionCoefficients[4],
                                                              calibrations[i].distortionCoefficients[5],
                                                              calibrations[i].distortionCoefficients[6],
                                                              calibrations[i].distortionCoefficients[7]) /*, id ?*/)),
          okvis::cameras::NCameraSystem::RadialTangential8 /*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << calibrations[i].distortionType;
    }
    ++camIdx;
  }

  vioParameters_.sensors_information.imuIdx = 0;

  cv::FileNode T_BS_ = file["imu_params"]["T_BS"];
  OKVIS_ASSERT_TRUE(
      Exception, T_BS_.isSeq(), "'T_BS' parameter missing in the configuration file or in the wrong format.")

  Eigen::Matrix4d T_BS_e;
  T_BS_e << T_BS_[0], T_BS_[1], T_BS_[2], T_BS_[3], T_BS_[4], T_BS_[5], T_BS_[6], T_BS_[7], T_BS_[8], T_BS_[9],
      T_BS_[10], T_BS_[11], T_BS_[12], T_BS_[13], T_BS_[14], T_BS_[15];

  vioParameters_.imu.T_BS = okvis::kinematics::Transformation(T_BS_e);
  std::stringstream s;
  s << vioParameters_.imu.T_BS.T();
  LOG(INFO) << "IMU with transformation T_BS=\n" << s.str();

  // the IMU parameters
  cv::FileNode imu_params = file["imu_params"];
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["a_max"].isReal(), "'imu_params: a_max' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["g_max"].isReal(), "'imu_params: g_max' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_g_c"].isReal(), "'imu_params: sigma_g_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_a_c"].isReal(), "'imu_params: sigma_a_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_bg"].isReal(), "'imu_params: sigma_bg' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_ba"].isReal(), "'imu_params: sigma_ba' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception,
                    imu_params["sigma_gw_c"].isReal(),
                    "'imu_params: sigma_gw_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_g_c"].isReal(), "'imu_params: sigma_g_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["tau"].isReal(), "'imu_params: tau' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception, imu_params["g"].isReal(), "'imu_params: g' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception, imu_params["a0"].isSeq(), "'imu_params: a0' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["imu_rate"].isInt(), "'imu_params: imu_rate' parameter missing in configuration file.");
  imu_params["a_max"] >> vioParameters_.imu.a_max;
  imu_params["g_max"] >> vioParameters_.imu.g_max;
  imu_params["sigma_g_c"] >> vioParameters_.imu.sigma_g_c;
  imu_params["sigma_a_c"] >> vioParameters_.imu.sigma_a_c;
  imu_params["sigma_bg"] >> vioParameters_.imu.sigma_bg;
  imu_params["sigma_ba"] >> vioParameters_.imu.sigma_ba;
  imu_params["sigma_gw_c"] >> vioParameters_.imu.sigma_gw_c;
  imu_params["sigma_aw_c"] >> vioParameters_.imu.sigma_aw_c;
  imu_params["imu_rate"] >> vioParameters_.imu.rate;
  imu_params["tau"] >> vioParameters_.imu.tau;
  imu_params["g"] >> vioParameters_.imu.g;

  vioParameters_.imu.a0 = Eigen::Vector3d(static_cast<double>(imu_params["a0"][0]),
                                          static_cast<double>(imu_params["a0"][1]),
                                          static_cast<double>(imu_params["a0"][2]));

  readConfigFile_ = true;
}

// Parses booleans from a cv::FileNode. OpenCV sadly has no implementation like this.
bool VioParametersReader::parseBoolean(cv::FileNode node, bool& val) const {
  if (node.isInt()) {
    val = static_cast<int>(node) != 0;
    return true;
  }
  if (node.isString()) {
    std::string str = (std::string)(node);
    // cut out first word. str currently contains everything including comments
    str = str.substr(0, str.find(" "));
    // transform it to all lowercase
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    /* from yaml.org/type/bool.html:
     * Booleans are formatted as English words
     * (“true”/“false”, “yes”/“no” or “on”/“off”)
     * for readability and may be abbreviated as
     * a single character “y”/“n” or “Y”/“N”. */
    if (str.compare("false") == 0 || str.compare("no") == 0 || str.compare("n") == 0 || str.compare("off") == 0) {
      val = false;
      return true;
    }
    if (str.compare("true") == 0 || str.compare("yes") == 0 || str.compare("y") == 0 || str.compare("on") == 0) {
      val = true;
      return true;
    }
  }
  return false;
}

bool VioParametersReader::getCameraCalibration(
    std::vector<CameraCalibration, Eigen::aligned_allocator<CameraCalibration>>& calibrations,
    cv::FileStorage& configurationFile) {
  bool success = getCalibrationViaConfig(calibrations, configurationFile["cameras"]);

  return success;
}

// Get the camera calibration via the configuration file.
bool VioParametersReader::getCalibrationViaConfig(
    std::vector<CameraCalibration, Eigen::aligned_allocator<CameraCalibration>>& calibrations,
    cv::FileNode cameraNode) const {
  calibrations.clear();
  bool gotCalibration = false;
  // first check if calibration is available in config file
  if (cameraNode.isSeq() && cameraNode.size() > 0) {
    size_t camIdx = 0;
    for (cv::FileNodeIterator it = cameraNode.begin(); it != cameraNode.end(); ++it) {
      if ((*it).isMap() && (*it)["T_SC"].isSeq() && (*it)["image_dimension"].isSeq() &&
          (*it)["image_dimension"].size() == 2 && (*it)["distortion_coefficients"].isSeq() &&
          (*it)["distortion_coefficients"].size() >= 4 && (*it)["distortion_type"].isString() &&
          (*it)["focal_length"].isSeq() && (*it)["focal_length"].size() == 2 && (*it)["principal_point"].isSeq() &&
          (*it)["principal_point"].size() == 2) {
        LOG(INFO) << "Found calibration in configuration file for camera " << camIdx;
        gotCalibration = true;
      } else {
        LOG(WARNING) << "Found incomplete calibration in configuration file for camera " << camIdx
                     << ". Will not use the calibration from the configuration file.";
        return false;
      }
      ++camIdx;
    }
  } else {
    LOG(INFO) << "Did not find a calibration in the configuration file.";
  }
  if (gotCalibration) {
    for (cv::FileNodeIterator it = cameraNode.begin(); it != cameraNode.end(); ++it) {
      CameraCalibration calib;

      cv::FileNode T_SC_node = (*it)["T_SC"];
      cv::FileNode imageDimensionNode = (*it)["image_dimension"];
      cv::FileNode distortionCoefficientNode = (*it)["distortion_coefficients"];
      cv::FileNode focalLengthNode = (*it)["focal_length"];
      cv::FileNode principalPointNode = (*it)["principal_point"];

      // extrinsics
      Eigen::Matrix4d T_SC;
      T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3], T_SC_node[4], T_SC_node[5], T_SC_node[6],
          T_SC_node[7], T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11], T_SC_node[12], T_SC_node[13],
          T_SC_node[14], T_SC_node[15];
      calib.T_SC = okvis::kinematics::Transformation(T_SC);

      calib.imageDimension << imageDimensionNode[0], imageDimensionNode[1];
      calib.distortionCoefficients.resize(distortionCoefficientNode.size());
      for (size_t i = 0; i < distortionCoefficientNode.size(); ++i) {
        calib.distortionCoefficients[i] = distortionCoefficientNode[i];
      }

      // Changing focal_length and principal_point accord to image resizeFactor
      calib.focalLength << focalLengthNode[0], focalLengthNode[1];

      calib.focalLength(0) = calib.focalLength(0) * vioParameters_.miscParams.resizeFactor;
      calib.focalLength(1) = calib.focalLength(1) * vioParameters_.miscParams.resizeFactor;

      LOG(INFO) << "focal_length: " << calib.focalLength;

      calib.principalPoint << principalPointNode[0], principalPointNode[1];
      calib.principalPoint(0) = calib.principalPoint(0) * vioParameters_.miscParams.resizeFactor;
      calib.principalPoint(1) = calib.principalPoint(1) * vioParameters_.miscParams.resizeFactor;

      LOG(INFO) << "principal_point: " << calib.principalPoint;

      calib.distortionType = (std::string)((*it)["distortion_type"]);

      calibrations.push_back(calib);
    }
  }
  return gotCalibration;
}

}  // namespace okvis
