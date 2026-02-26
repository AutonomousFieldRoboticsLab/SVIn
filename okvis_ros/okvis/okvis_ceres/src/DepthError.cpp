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
#include <okvis/ceres/PoseManifold.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement, variance and LandmarkSubset.
DepthError::DepthError(double depth, const information_t& information, double first_depth, const okvis::kinematics::Transformation& T_SD)
    : T_SD_(T_SD) {
  // LOG(INFO) << "DepthError initialization";
  setMeasurement(depth, first_depth);
  setInformation(information);
  setTransformation(T_SD);
}

void DepthError::setInformation(const information_t& information) {
  information_ = information;
  covariance_ = 1 / information;
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  // Eigen::LLT<information_t> lltOfInformation(information_);
  // TODO(Sharmin): Check if it's correct
  _squareRootInformation = sqrt(information);
}

void DepthError::setTransformation(const okvis::kinematics::Transformation& T_SD) {
  T_SD_ = T_SD;
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

  // compute error
  double error = 0.0;

  // Example: first_depth = 5, current_depth = 10, parameters[0][2] = -5, error = -5 - (5-10)
  error = parameters[0][2] - (-1 * depth_ + first_depth_);

  // LOG(INFO) << std::fixed << std::setprecision(3)
  //           << " DepthError: parameters[0][2]: " << parameters[0][2]
  //           << " \n depth_: " << depth_ 
  //           << " \n first_depth_: " << first_depth_
  //           << " \n error: parameters[0][2] - (-1 * depth_ + first_depth_) = " << error;  

  // weight:
  double weighted_error = _squareRootInformation * error;

  // assign:
  residuals[0] = weighted_error;

  // compute Jacobian...
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {

      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J0(jacobians[0]);
      Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J0_minimal;
      J0_minimal.setZero();

      Eigen::Vector3d e3(0.0, 0.0, 1.0);  // Unit vector along z-axis

      // Position Jacobian ∂e/∂p = e₃ᵀ = [0, 0, 1]
      J0_minimal.block<1,3>(0,0) = e3.transpose();  // [0, 0, 1, ...]

      // Rotation Jacobian: ∂e/∂θ = e₃ᵀ·R_ItoW·[p_DinI×]
      // Eigen::Vector3d p_IinD = T_SD_.r();
      // Eigen::Vector3d p_DinI = T_SD_.C().transpose() * (-1 * p_IinD);
      // Eigen::Matrix3d R_ItoW = T_WS.C().transpose();
      // Skew-symmetric matrix [p_DinI]×
      // Eigen::Matrix3d p_DinI_skew;
      // p_DinI_skew <<        0,      -p_DinI(2),  p_DinI(1),
      //                p_DinI(2),            0,   -p_DinI(0),
      //               -p_DinI(1),     p_DinI(0),           0;

      // Compute rotation Jacobian
      // Eigen::RowVector3d J_rotation = e3.transpose() * R_ItoW * p_DinI_skew;
      // J0_minimal.block<1,3>(0,3) = J_rotation;  // [..., J_θx, J_θy, J_θz, 0]

      // Apply information weighting
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
