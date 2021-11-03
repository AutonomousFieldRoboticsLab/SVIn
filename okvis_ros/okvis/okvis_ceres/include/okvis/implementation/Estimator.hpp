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
 *  Created on: Jan 10, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/Estimator.hpp
 * @brief Header implementation file for the Estimator class.
 * @author Stefan Leutenegger
 */

#include <map>
#include <memory>
#include <utility>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Add an observation to a landmark.
template <class GEOMETRY_TYPE>
::ceres::ResidualBlockId Estimator::addObservation(uint64_t landmarkId,
                                                   uint64_t poseId,
                                                   size_t camIdx,
                                                   size_t keypointIdx) {
  OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId), "landmark not added");

  // avoid double observations
  okvis::KeypointIdentifier kid(poseId, camIdx, keypointIdx);
  if (landmarksMap_.at(landmarkId).observations.find(kid) != landmarksMap_.at(landmarkId).observations.end()) {
    return NULL;
  }

  // get the keypoint measurement
  okvis::MultiFramePtr multiFramePtr = multiFramePtrMap_.at(poseId);
  Eigen::Vector2d measurement;
  multiFramePtr->getKeypoint(camIdx, keypointIdx, measurement);
  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  double size = 1.0;
  multiFramePtr->getKeypointSize(camIdx, keypointIdx, size);
  information *= 64.0 / (size * size);

  // create error term
  std::shared_ptr<ceres::ReprojectionError<GEOMETRY_TYPE>> reprojectionError(
      new ceres::ReprojectionError<GEOMETRY_TYPE>(
          multiFramePtr->template geometryAs<GEOMETRY_TYPE>(camIdx), camIdx, measurement, information));

  ::ceres::ResidualBlockId retVal = mapPtr_->addResidualBlock(
      reprojectionError,
      cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : NULL,
      mapPtr_->parameterBlockPtr(poseId),
      mapPtr_->parameterBlockPtr(landmarkId),
      mapPtr_->parameterBlockPtr(
          statesMap_.at(poseId).sensors.at(SensorStates::Camera).at(camIdx).at(CameraSensorStates::T_SCi).id));

  // remember
  landmarksMap_.at(landmarkId)
      .observations.insert(std::pair<okvis::KeypointIdentifier, uint64_t>(kid, reinterpret_cast<uint64_t>(retVal)));

  return retVal;
}

// Sharmin
// Add an observation to a landmark found in loop for relocalization.
template <class GEOMETRY_TYPE>
::ceres::ResidualBlockId Estimator::addRelocObservation(uint64_t landmarkId,
                                                        uint64_t poseId,
                                                        size_t camIdx,
                                                        size_t keypointIdx) {
  // OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId),
  //                     "landmark not added for relocalization");
  // SHarmin
  if (!isLandmarkAdded(landmarkId)) {
    std::cout << "Reloc matched landmark not added to Map. retruning Null" << std::endl;
    return NULL;
  } else {
    std::cout << "Reloc point found in the existing map" << std::endl;
  }

  // avoid double observations
  okvis::KeypointIdentifier kid(poseId, camIdx, keypointIdx);
  if (landmarksMap_.at(landmarkId).observations.find(kid) != landmarksMap_.at(landmarkId).observations.end()) {
    std::cout << "Double obser found for Reloc point. Returning..." << std::endl;
    return NULL;
  }

  std::cout << "Reloc: poseId " << poseId << "multiFramePtrMap_ size: " << multiFramePtrMap_.size() << std::endl;
  for (std::map<uint64_t, okvis::MultiFramePtr>::iterator i = multiFramePtrMap_.begin(); i != multiFramePtrMap_.end();
       i++) {
    std::cout << "Reloc: mfId " << i->first << std::endl;
  }
  if (multiFramePtrMap_.find(poseId) == multiFramePtrMap_.end()) return NULL;

  // get the keypoint measurement
  okvis::MultiFramePtr multiFramePtr = multiFramePtrMap_.at(poseId);
  std::cout << "Reloc: Found poseId" << std::endl;
  Eigen::Vector2d measurement;
  multiFramePtr->getKeypoint(camIdx, keypointIdx, measurement);
  std::cout << "Reloc: Got keypoint measurement" << std::endl;
  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  double size = 1.0;
  multiFramePtr->getKeypointSize(camIdx, keypointIdx, size);
  information *= 64.0 / (size * size);

  // create error term
  std::shared_ptr<ceres::ReprojectionError<GEOMETRY_TYPE>> reprojectionError(
      new ceres::ReprojectionError<GEOMETRY_TYPE>(
          multiFramePtr->template geometryAs<GEOMETRY_TYPE>(camIdx), camIdx, measurement, information));
  std::cout << "Reloc: created error term" << std::endl;

  ::ceres::ResidualBlockId retVal = mapPtr_->addResidualBlock(
      reprojectionError,
      cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : NULL,
      mapPtr_->parameterBlockPtr(poseId),      // TODO(sharmin): check if poseParameterBlock
      mapPtr_->parameterBlockPtr(landmarkId),  // TODO(sharmin): check if homogeneousPointParameterBlock
      mapPtr_->parameterBlockPtr(statesMap_.at(poseId)
                                     .sensors.at(SensorStates::Camera)
                                     .at(camIdx)
                                     .at(CameraSensorStates::T_SCi)
                                     .id));  // TODO(sharmin): check if extrinsicParameterBlock
  std::cout << "Error term added to ceres for Reloc" << std::endl;

  // remember
  // TODO(Sharmin): Is this observation needed to add?
  // landmarksMap_.at(landmarkId).observations.insert(
  //    std::pair<okvis::KeypointIdentifier, uint64_t>(
  //       kid, reinterpret_cast<uint64_t>(retVal)));

  return retVal;
  // return ::ceres::ResidualBlockId();
}

}  // namespace okvis
