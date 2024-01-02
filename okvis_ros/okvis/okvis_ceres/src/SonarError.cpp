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
 * @file SonarError.cpp
 * @brief Source file for the SonarError class.
 * @author Sharmin Rahman
 */

#include <okvis/ceres/PoseManifold.hpp>
#include <okvis/ceres/SonarError.hpp>
#include <vector>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement and information matrix.
/*SonarError::SonarError(
    const Eigen::Vector4d & measurement, const information_t & information) {
  setMeasurement(measurement);
  setInformation(information);
}*/

// Construct with measurement, variance and LandmarkSubset.
SonarError::SonarError(const okvis::VioParameters& params,
                       double range,
                       double heading,
                       const information_t& information,
                       std::vector<Eigen::Vector3d> landmarkSubset)
    : params_(params) {
  setMeasurement(range, heading);
  setInformation(information);
  // computeCovarianceMatrix(landmarkSubset);
  setLandmarkSubset(landmarkSubset);
}

/*void SonarError::computeCovarianceMatrix(const std::vector<Eigen::Vector4d> &landmarkSubset) {
        Eigen::Matrix<double,3,3> covariance = Eigen::Matrix<double,3,3>::Zero();
        double mean[3] = {0, 0, 0};
        for (auto it = landmarkSubset.begin(); it != landmarkSubset.end(); ++it){
                mean[0] += (*it)[0] / (*it)[3]; // it[3] is always 1
                mean[1] += (*it)[1] / (*it)[3]; // it[3] is always 1
                mean[2] += (*it)[2] / (*it)[3]; // it[3] is always 1
        }
        mean[0] = mean[0] / landmarkSubset.size();
        mean[1] = mean[1] / landmarkSubset.size();
        mean[2] = mean[2] / landmarkSubset.size();

        for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                        covariance(i,j) = 0.0;
                        for (auto it = landmarkSubset.begin(); it != landmarkSubset.end(); ++it)
                                covariance(i,j) += (mean[i] - (*it)[0]) * (mean[j] - (*it)[1]);
                        covariance(i,j) /= landmarkSubset.size() - 1;
                }
        covariance_ = covariance;
        information_ = covariance.inverse();
        // perform the Cholesky decomposition on order to obtain the correct error weighting
        Eigen::LLT<information_t> lltOfInformation(information_);
        _squareRootInformation = lltOfInformation.matrixL().transpose();
        LOG(INFO) << covariance_;
        LOG(INFO) << information_;
        LOG(INFO) << _squareRootInformation;
  }*/

void SonarError::setInformation(const information_t& information) {
  information_ = information;
  covariance_ = 1 / information;
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  // Eigen::LLT<information_t> lltOfInformation(information_);
  // TODO(@sharmin): Check if it's correct
  _squareRootInformation = sqrt(information);
}

// This evaluates the error term and additionally computes the Jacobians.
bool SonarError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool SonarError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                              double* residuals,
                                              double** jacobians,
                                              double** jacobiansMinimal) const {
  // compute error
  Eigen::Vector3d mean;
  mean.setZero();
  double range_corrected = 0.0, error = 0.0;
  for (auto it = landmarkSubset_.begin(); it != landmarkSubset_.end(); ++it) {
    mean[0] += (*it)[0];
    mean[1] += (*it)[1];
    mean[2] += (*it)[2];
  }
  mean = mean / landmarkSubset_.size();
  okvis::kinematics::Transformation T_WS(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]));

  // calculating the distance between the T_WS and mean of landmarkSubset
  range_corrected = (T_WS.r() - mean).norm();
  error = range_ - range_corrected;

  // LOG(INFO)<<hp.toHomogeneous().transpose() << " : " << measurement.transpose();

  // weight:
  double weighted_error = _squareRootInformation * error;

  // assign:
  residuals[0] = weighted_error;
  Eigen::Vector3d sonar_landmark;

  okvis::kinematics::Transformation T_WSo = T_WS * params_.sonar.T_SSo;

  okvis::kinematics::Transformation sonar_point(Eigen::Vector3d(range_ * cos(heading_), range_ * sin(heading_), 0.0),
                                                Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  okvis::kinematics::Transformation T_WSo_point = T_WSo * sonar_point;

  sonar_landmark = T_WSo_point.r();

  // compute Jacobian...
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J0(jacobians[0]);
      Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J0_minimal;
      J0_minimal.setZero();
      Eigen::Vector3d temp = (T_WS.r() - sonar_landmark) / range_;
      J0_minimal(0, 0) = temp[0];
      J0_minimal(0, 1) = temp[1];
      J0_minimal(0, 2) = temp[2];
      // double J0_minimal;

      J0_minimal = (_squareRootInformation * J0_minimal).eval();

      // pseudo inverse of the local parametrization Jacobian:
      // Eigen::Matrix<double, 7, 1, Eigen::RowMajor> J_lift;
      // PoseManifold::liftJacobian(parameters[0], J_lift.data());

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
