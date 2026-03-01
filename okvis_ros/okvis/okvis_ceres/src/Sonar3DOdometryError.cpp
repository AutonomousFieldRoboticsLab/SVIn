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
 * @file DvlError.cpp
 * @brief Source file for the DvlError class.
 * @author Chinmay 
 */

#include <okvis/ceres/Sonar3DOdometryError.hpp>
#include <okvis/ceres/PoseManifold.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement, variance and LandmarkSubset.
Sonar3DOdometryError::Sonar3DOdometryError(
    const okvis::kinematics::Transformation& T_L_old,
    const okvis::kinematics::Transformation& T_L_new,
    const Eigen::Matrix<double, 6, 1>& covariance_diag,
    const okvis::kinematics::Transformation& T_SL){ 

  // Compute relative transformation: motion from old to new in 3D Sonar frame 
  // T_Lk-1_Lk
  okvis::kinematics::Transformation T_relative = T_L_old.inverse() * T_L_new;
  
  setMeasurement(T_relative.r(), T_relative.q());
  setInformation(covariance_diag);
  setTransformation(T_SL);

  // --- Verification prints ---
  LOG(INFO) << "Inside Sonar3DOdometryError";
  LOG(INFO) << "[Sonar3DOdometryError] position_m (Sonar3DOdometry frame): ["
            << std::fixed << std::setprecision(4)
            << position_m_(0) << ", " << position_m_(1) << ", " << position_m_(2) << "] m";
  LOG(INFO) << "[Sonar3DOdometryError] orientation_m (Sonar3DOdometry frame): ["
            << orientation_m_.w() << ", " << orientation_m_.x() << ", " 
            << orientation_m_.y() << ", " << orientation_m_.z() << "]";
  LOG(INFO) << "[Sonar3DOdometryError] covariance diag: ["
            << covariance_.diagonal()(0) << ", "
            << covariance_.diagonal()(1) << ", "
            << covariance_.diagonal()(2) << "] (m/s)^2"
            << ", [" << covariance_.diagonal()(3) << ", "
            << covariance_.diagonal()(4) << ", "
            << covariance_.diagonal()(5) << "] (rad)^2";
  LOG(INFO) << "[Sonar3DOdometryError] information diag: ["
            << information_.diagonal()(0) << ", "
            << information_.diagonal()(1) << ", "
            << information_.diagonal()(2) << "]";
  LOG(INFO) << "[Sonar3DOdometryError] sqrtInfo diag: ["
            << _squareRootInformation.diagonal()(0) << ", "
            << _squareRootInformation.diagonal()(1) << ", "
            << _squareRootInformation.diagonal()(2) << "]";
  LOG(INFO) << "[Sonar3DOdometryError] T_SL translation: ["
            << T_SL_.r()(0) << ", " << T_SL_.r()(1) << ", " << T_SL_.r()(2) << "] m";
  LOG(INFO) << "[Sonar3DOdometryError] T_SL rotation C():\n" << T_SL_.C();  // --- Verification prints ---
}

// This evaluates the error term and additionally computes the Jacobians.
bool Sonar3DOdometryError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool Sonar3DOdometryError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                              double* residuals,
                                              double** jacobians,
                                              double** jacobiansMinimal) const {

  // --- Extract the two pose states ---
  // parameters[0]: T_WS_0 (old/reference)  [tx, ty, tz, qx, qy, qz, qw]
  // parameters[1]: T_WS_1 (new/current)    [tx, ty, tz, qx, qy, qz, qw]
  okvis::kinematics::Transformation T_WS_0(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]));
  okvis::kinematics::Transformation T_WS_1(
      Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2]),
      Eigen::Quaterniond(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]));

  // Decompose into basic rotation/translation components
  Eigen::Matrix3d C_WS_0 = T_WS_0.C();   // R_{W,S₀}
  Eigen::Vector3d t_WS_0 = T_WS_0.r();   // t_{W,S₀}
  Eigen::Matrix3d C_WS_1 = T_WS_1.C();   // R_{W,S₁}
  Eigen::Vector3d t_WS_1 = T_WS_1.r();   // t_{W,S₁}
  Eigen::Matrix3d C_SL   = T_SL_.C();     // R_{S,L}
  Eigen::Vector3d t_SL   = T_SL_.r();     // t_{S,L}
  Eigen::Matrix3d C_LS   = C_SL.transpose();
  Eigen::Vector3d t_LS   = -C_LS * t_SL;

  // Predicted relative transform in sonar(L) frame:
  //   T_pred = T_SL^{-1} * T_WS_0^{-1} * T_WS_1 * T_SL
  okvis::kinematics::Transformation T_LS = T_SL_.inverse();
  okvis::kinematics::Transformation T_pred = T_LS * T_WS_0.inverse() * T_WS_1 * T_SL_; 
  
  // Measured relative transform (stored from constructor)
  okvis::kinematics::Transformation T_meas(position_m_, orientation_m_);

  // Error: T_error = T_meas^{-1} * T_pred
  okvis::kinematics::Transformation T_error = T_meas.inverse() * T_pred;

  // Quaternion sign consistency
  Eigen::Quaterniond q_err = T_error.q();
  if (q_err.w() < 0.0) {
    q_err.coeffs() = -q_err.coeffs();
  }

  // Residual: e = sqrt_info * [position_error(3); orientation_error(3)]
  Eigen::Map<Eigen::Matrix<double, 6, 1>> error(residuals);
  error.head<3>() = T_error.r();
  // error.tail<3>() = 2.0 * q_err.vec(); // To use orientation error
  error.tail<3>().setZero(); // To disable orientation error, set to zero instead of 2.0*q_err.vec()
  
  error = _squareRootInformation * error;

  // --- Jacobians ---
  if (jacobians != NULL) {
  
    // Jacobian w.r.t. T_WS_0 — ZERO (not optimizing old state)
    if (jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J0(jacobians[0]);
      J0.setZero();
    }
    if (jacobiansMinimal != NULL && jacobiansMinimal[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J0min(jacobiansMinimal[0]);
      J0min.setZero();
    }

    // T_WS_1: position-only Jacobian
    // t_err = C_A * (C_1 * t_SL + t_1) + t_A
    // where C_A = (T_meas^{-1} * T_LS * T_WS_0^{-1}).C()
    //
    // ∂t_err/∂δt₁ = C_A              (3x3)
    // ∂t_err/∂δφ₁ = -C_A * [C_1 * t_SL]×  (3x3)  (left-multiply perturbation) (wrong)
    // ∂t_err/∂δφ₁ = -C_A * C_1 *[t_SL]×  (3x3)  (left-multiply perturbation)

    Eigen::Matrix3d C_A = (T_meas.inverse() * T_LS * T_WS_0.inverse()).C();

    Eigen::Matrix<double, 6, 6> J1_minimal;
    J1_minimal.setZero();
    // Position Jacobian w.r.t. T_WS_1
    // ∂t_err/∂δt₁ = C_A                          (3x3)
    // ∂t_err/∂δφ₁ = -C_A * C_WS_1 * [t_SL]×     (3x3)
    J1_minimal.topLeftCorner<3, 3>()  = _squareRootInformation.topLeftCorner<3, 3>() * C_A;
    J1_minimal.topRightCorner<3, 3>() = -_squareRootInformation.topLeftCorner<3, 3>() * C_A * C_WS_1
                                         * okvis::kinematics::crossMx(t_SL);

    // Uncomment error.tail<3>() = 2.0 * q_err.vec();                                      
    // Orientation Jacobian w.r.t. T_WS_1
    // ∂e_r/∂δt₁ = 0                              (3x3, already zero from setZero)
    // ∂e_r/∂δφ₁ = (w*I + [v]×) * R_SL^T             (3x3)
    // double w = q_err.w();
    // Eigen::Vector3d v = q_err.vec();
    // Eigen::Matrix3d wI_plus_vx = w * Eigen::Matrix3d::Identity()
    //                               + okvis::kinematics::crossMx(v);
    // J1_minimal.bottomRightCorner<3, 3>() = _squareRootInformation.bottomRightCorner<3, 3>()
    //                                         * wI_plus_vx
    //                                         * C_SL.transpose();

    // Now copy to output (after ALL blocks are filled)
    if (jacobiansMinimal != NULL && jacobiansMinimal[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J1min(jacobiansMinimal[1]);
      J1min = J1_minimal;
    }

    if (jacobians[1] != NULL) {
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseManifold::liftJacobian(parameters[1], J_lift.data());
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J1(jacobians[1]);
      J1 = J1_minimal * J_lift;
    }
  
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
