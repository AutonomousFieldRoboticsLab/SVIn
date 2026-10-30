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
 *  Created on: Sep 15, 2014
 *      Author: Pascal Gohl
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioVisualizer.cpp
 * @brief Source file for the VioVisualizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include "okvis/VioVisualizer.hpp"

#include <okvis/FrameTypedefs.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/kinematics/Transformation.hpp>

// cameras and distortions
#include <algorithm>
#include <memory>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <string>
#include <vector>

/// \brief okvis Main namespace of this package.
namespace okvis {

VioVisualizer::VioVisualizer(okvis::VioParameters& parameters) : parameters_(parameters) {
  if (parameters.nCameraSystem.numCameras() > 0) {
    init(parameters);
  }
}

VioVisualizer::~VioVisualizer() {}

void VioVisualizer::init(okvis::VioParameters& parameters) { parameters_ = parameters; }

cv::Mat VioVisualizer::drawMatches(VisualizationData::Ptr& data, size_t image_number) {
  std::shared_ptr<okvis::MultiFrame> keyframe = data->keyFrames;
  std::shared_ptr<okvis::MultiFrame> frame = data->currentFrames;

  if (keyframe == nullptr) return frame->image(image_number);

  // allocate an image
  const unsigned int im_cols = frame->image(image_number).cols;
  const unsigned int im_rows = frame->image(image_number).rows;
  const unsigned int rowJump = im_rows;

  cv::Mat outimg(2 * im_rows, im_cols, CV_8UC3);
  // copy current images Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
  cv::Mat current = outimg(cv::Rect(0, rowJump, im_cols, im_rows));
  cv::Mat actKeyframe = outimg(cv::Rect(0, 0, im_cols, im_rows));

  cv::cvtColor(frame->image(image_number), current, cv::COLOR_GRAY2BGR);
  cv::cvtColor(keyframe->image(image_number), actKeyframe, cv::COLOR_GRAY2BGR);

  // the keyframe trafo
  Eigen::Vector2d keypoint;
  Eigen::Vector4d landmark;
  okvis::kinematics::Transformation lastKeyframeT_CW =
      parameters_.nCameraSystem.T_SC(image_number)->inverse() * data->T_WS_keyFrame.inverse();

  // find distortion type
  okvis::cameras::NCameraSystem::DistortionType distortionType = parameters_.nCameraSystem.distortionType(0);
  for (size_t i = 1; i < parameters_.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == parameters_.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet");
  }

  for (auto it = data->observations.begin(); it != data->observations.end(); ++it) {
    if (it->cameraIdx != image_number) continue;

    cv::Scalar color;

    if (it->landmarkId != 0) {
      color = cv::Scalar(255, 0, 0);  // blue
    } else {
      color = cv::Scalar(0, 0, 255);  // red
    }

    // draw matches to keyframe
    keypoint = it->keypointMeasurement;
    if (fabs(it->landmark_W[3]) > 1.0e-8) {
      Eigen::Vector4d hPoint = it->landmark_W;
      if (it->isInitialized) {
        color = cv::Scalar(0, 255, 0);  // green
      } else {
        color = cv::Scalar(0, 255, 255);  // yellow
      }
      Eigen::Vector2d keyframePt;
      bool isVisibleInKeyframe = false;
      Eigen::Vector4d hP_C = lastKeyframeT_CW * hPoint;
      switch (distortionType) {
        case okvis::cameras::NCameraSystem::RadialTangential: {
          if (frame->geometryAs<okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>>(image_number)
                  ->projectHomogeneous(hP_C, &keyframePt) == okvis::cameras::CameraBase::ProjectionStatus::Successful)
            isVisibleInKeyframe = true;
          break;
        }
        case okvis::cameras::NCameraSystem::Equidistant: {
          if (frame->geometryAs<okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>>(image_number)
                  ->projectHomogeneous(hP_C, &keyframePt) == okvis::cameras::CameraBase::ProjectionStatus::Successful)
            isVisibleInKeyframe = true;
          break;
        }
        case okvis::cameras::NCameraSystem::RadialTangential8: {
          if (frame
                  ->geometryAs<okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8>>(image_number)
                  ->projectHomogeneous(hP_C, &keyframePt) == okvis::cameras::CameraBase::ProjectionStatus::Successful)
            isVisibleInKeyframe = true;
          break;
        }
        default:
          OKVIS_THROW(Exception, "Unsupported distortion type.")
          break;
      }
      if (fabs(hP_C[3]) > 1.0e-8) {
        if (hP_C[2] / hP_C[3] < 0.4) {
          isVisibleInKeyframe = false;
        }
      }

      if (isVisibleInKeyframe) {
        // found in the keyframe. draw line
        cv::line(outimg,
                 cv::Point2f(keyframePt[0], keyframePt[1]),
                 cv::Point2f(keypoint[0], keypoint[1] + rowJump),
                 color,
                 1,
                 cv::LINE_AA);
        cv::circle(
            actKeyframe, cv::Point2f(keyframePt[0], keyframePt[1]), 0.5 * it->keypointSize, color, 1, cv::LINE_AA);
      }
    }
    // draw keypoint
    const double r = 0.5 * it->keypointSize;
    cv::circle(current, cv::Point2f(keypoint[0], keypoint[1]), r, color, 1, cv::LINE_AA);
    cv::KeyPoint cvKeypoint;
    frame->getCvKeypoint(image_number, it->keypointIdx, cvKeypoint);
    const double angle = cvKeypoint.angle / 180.0 * M_PI;
    cv::line(outimg,
             cv::Point2f(keypoint[0], keypoint[1] + rowJump),
             cv::Point2f(keypoint[0], keypoint[1] + rowJump) + cv::Point2f(cos(angle), sin(angle)) * r,
             color,
             1,
             cv::LINE_AA);
  }
  return outimg;
}

cv::Mat VioVisualizer::drawKeypoints(VisualizationData::Ptr& data, size_t cameraIndex) {
  std::shared_ptr<okvis::MultiFrame> currentFrames = data->currentFrames;
  const cv::Mat currentImage = currentFrames->image(cameraIndex);

  cv::Mat outimg;
  cv::cvtColor(currentImage, outimg, cv::COLOR_GRAY2BGR);
  cv::Scalar greenColor(0, 255, 0);  // green

  cv::KeyPoint keypoint;
  for (size_t k = 0; k < currentFrames->numKeypoints(cameraIndex); ++k) {
    currentFrames->getCvKeypoint(cameraIndex, k, keypoint);

    double radius = keypoint.size;
    double angle = keypoint.angle / 180.0 * M_PI;

    cv::circle(outimg, keypoint.pt, radius, greenColor);
    cv::line(outimg,
             keypoint.pt,
             cv::Point2f(keypoint.pt.x + radius * cos(angle), keypoint.pt.y - radius * sin(angle)),
             greenColor);
  }

  return outimg;
}

void VioVisualizer::createDebugWindow() {
  cv::namedWindow(debugWindowName(), cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  cv::resizeWindow(debugWindowName(), 1000, 600);
}

cv::Mat VioVisualizer::composeOverview(const std::vector<cv::Mat>& images) {
  int imageWidth = 0;
  int imageHeight = 0;
  for (const auto& image : images) {
    imageWidth = std::max(imageWidth, image.cols);
    imageHeight = std::max(imageHeight, image.rows);
  }
  if (imageWidth == 0 || imageHeight == 0) return cv::Mat();

  const int cameras = static_cast<int>(images.size());
  const int columns = std::min(3, cameras);
  const int rows = (cameras + columns - 1) / columns;
  const int gap = 2;
  const double scale = std::min({1.0,
      double(1200 - gap * (columns - 1)) / (columns * imageWidth),
      double(720 - gap * (rows - 1)) / (rows * imageHeight)});
  const int tileWidth = std::max(1, int(imageWidth * scale));
  const int tileHeight = std::max(1, int(imageHeight * scale));
  cv::Mat canvas(rows * tileHeight + gap * (rows - 1),
                 columns * tileWidth + gap * (columns - 1), CV_8UC3, cv::Scalar::all(0));
  for (int camera = 0; camera < cameras; ++camera) {
    const int left = (camera % columns) * (tileWidth + gap);
    const int top = (camera / columns) * (tileHeight + gap);
    const auto& image = images[camera];
    if (!image.empty()) {
      const double fit = std::min(double(tileWidth) / image.cols, double(tileHeight) / image.rows);
      cv::Mat resized;
      cv::resize(image, resized, cv::Size(std::max(1, int(image.cols * fit)),
                                        std::max(1, int(image.rows * fit))),
                 0, 0, fit < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
      if (resized.channels() == 1) cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
      resized.copyTo(canvas(cv::Rect(left + (tileWidth - resized.cols) / 2,
                                     top + (tileHeight - resized.rows) / 2,
                                     resized.cols, resized.rows)));
    }
    const std::string label = "Camera " + std::to_string(camera);
    cv::putText(canvas, label, cv::Point(left + 7, top + 19), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar::all(0), 3, cv::LINE_AA);
    cv::putText(canvas, label, cv::Point(left + 7, top + 19), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar::all(255), 1, cv::LINE_AA);
  }
  return canvas;
}

void VioVisualizer::showDebugImages(VisualizationData::Ptr& data) {
  std::vector<cv::Mat> out_images(parameters_.nCameraSystem.numCameras());
  for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
    out_images[i] = drawMatches(data, i);
  }

  if (!debugWindowCreated_) {
    createDebugWindow();
    debugWindowCreated_ = true;
  }
  const cv::Mat overview = composeOverview(out_images);
  if (!overview.empty()) cv::imshow(debugWindowName(), overview);
  cv::waitKey(1);
}

} /* namespace okvis */
