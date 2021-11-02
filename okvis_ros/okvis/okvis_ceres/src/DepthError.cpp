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
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file DepthError.cpp
 * @brief Source file for the DepthError class.
 * @author Sharmin Rahman
 */

#include <okvis/ceres/DepthError.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement, variance and LandmarkSubset.
DepthError::DepthError(double depth, const information_t& information, double first_depth) {
  // LOG(INFO) << "DepthError initialization";
  setMeasurement(depth, first_depth);
  setInformation(information);
}

void DepthError::setInformation(const information_t& information) {
  information_ = information;
  covariance_ = 1 / information;
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  // Eigen::LLT<information_t> lltOfInformation(information_);
  // TODO @Sharmin: Check if it's correct
  _squareRootInformation = sqrt(information);
}

// This evaluates the error term and additionally computes the Jacobians.
bool DepthError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool DepthError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                              double* residuals,
                                              double** jacobians,
                                              double** jacobiansMinimal) const {
  okvis::kinematics::Transformation T_WS(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]));

  // For Stereo Rig V2
  okvis::kinematics::Transformation T_SD(Eigen::Vector3d(-0.019, 0.032, -0.282),
                                         Eigen::Quaterniond(-0.500, 0.500, 0.500, 0.500));
  // Transforming World frame to Depth frame. So that, in current W-frame is aligned with depth direction(=Gravity
  // direction).
  okvis::kinematics::Transformation T_WD = T_WS * T_SD;
  // Eigen::Vector4d depthPointW = T_WD * Eigen::Vector4d(0, 0, -1*depth_+ first_depth_, 1.0);
  Eigen::Vector3d T_DW = T_WD.inverse().r();

  // std::cout<<"T_WD: "<<T_WD.coeffs()<<std::endl;

  // compute error
  double error = 0.0;
  // Initial Displacement between depth and world frame
  double displacement = 0.032;

  // error = (T_DW[2] + displacement) - (-1*depth_+ first_depth_);
  // error = T_DW[2] - (-1*depth_+ first_depth_);
  // error = parameters[0][2] - depthPointW[2]/depthPointW[3];
  error = parameters[0][2] - (-1 * depth_ + first_depth_);

  /*
  std::cout<< "Z value in Depth frame: " << T_DW[2]<< std::endl;
  //std::cout<< "Z value in world frame: " << T_WD[2]<< std::endl;
  std::cout<< "Depth sensor: "<< -1*depth_+ first_depth_<< std::endl;


  std::cout<< " Depth Error:  " << error<< std::endl;
  */

  // weight:
  double weighted_error = _squareRootInformation * error;

  // assign:
  residuals[0] = weighted_error;

  // compute Jacobian...
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      // jacobians[0][0] = 1;

      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J0(jacobians[0]);
      Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J0_minimal;
      J0_minimal.setZero();
      J0_minimal(0, 2) = 1;

      J0_minimal = (_squareRootInformation * J0_minimal).eval();

      // hallucinate Jacobian w.r.t. state
      J0 = J0_minimal;

      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[0] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J0_minimal_mapped(jacobiansMinimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
