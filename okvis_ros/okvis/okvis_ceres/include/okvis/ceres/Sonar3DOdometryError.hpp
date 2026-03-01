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
 *  Created on: Sep 10, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Sonar3DOdometryError.hpp
 * @brief Header file for the Sonar3DOdometryError class.
 * @author Chinmay Burgul
 */

#ifndef INCLUDE_OKVIS_CERES_SONAR3DODOMETRYERROR_HPP_
#define INCLUDE_OKVIS_CERES_SONAR3DODOMETRYERROR_HPP_

#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ErrorInterface.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <string>
#include <vector>
#include <iomanip>
#include "ceres/ceres.h"

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief Absolute error of a Sonar3DOdometry measurement.
class Sonar3DOdometryError : public ::ceres::SizedCostFunction<6 /* residuals: 3 position + 3 orientation */,
                                                               7 /* pose parameter block: position + quaternion */,
                                                               7 /* pose parameter block: position + quaternion */>,
                   public ErrorInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<6, 7, 7> base_t;

  /// \brief Number of residuals (6)
  static const int kNumResiduals = 6;

  // ToDo: Turn into matrix 
  /// \brief The information matrix type (6X6).
  typedef Eigen::Matrix<double, 6, 6> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 6, 6> covariance_t;

  /// \brief Default constructor.
  Sonar3DOdometryError();

  /// @param[in] T_L_old  Absolute sonar pose at reference time (lidar frame)
  /// @param[in] T_L_new  Absolute sonar pose at current time (lidar frame)
  /// @param[in] covariance_diag  6x1 diagonal [pos_x², pos_y², pos_z², ori_x², ori_y², ori_z²]
  /// @param[in] T_SL  Transformation from lidar to IMU sensor frame
  Sonar3DOdometryError(const okvis::kinematics::Transformation& T_L_old,
                      const okvis::kinematics::Transformation& T_L_new,
                      const Eigen::Matrix<double, 6, 1>& covariance_diag,
                      const okvis::kinematics::Transformation& T_SL);

  /// \brief Trivial destructor.
  virtual ~Sonar3DOdometryError() {}

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(Eigen::Vector3d position_m, Eigen::Quaterniond orientation_m) {
    position_m_ = position_m;
    orientation_m_ = orientation_m;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const Eigen::Matrix<double, 6, 1>& covariance_diag) {
    covariance_ = covariance_diag.asDiagonal();
    information_ = covariance_.inverse();
    _squareRootInformation = information_.llt().matrixL().transpose();
  }

  /// \brief Set the transformation from IMU to depth sensor frame.
  /// @param[in] T_SL The transformation.
  void setTransformation(const okvis::kinematics::Transformation& T_SL) {
    T_SL_ = T_SL;
  }

  /// \brief Get the measurement.
  /// \return The measurement vector.
  Eigen::Vector3d getPositionMeasurement() const { return position_m_; }

  /// \brief Get the orientation measurement.
  /// \return The orientation measurement as a quaternion.
  Eigen::Quaterniond getOrientationMeasurement() const { return orientation_m_; }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t& information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const covariance_t& covariance() const { return covariance_; }

  /**
   * @brief This evaluates the error term and additionally computes the Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

  /**
   * @brief EvaluateWithMinimalJacobians This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(double const* const* parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const { return this->parameter_block_sizes().size(); }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameterBlockId ID of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return this->parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "Sonar3DOdometryError"; }

 protected:
  // the measurement
  Eigen::Vector3d position_m_;  ///< The position measurement.
  Eigen::Quaterniond orientation_m_;  ///< The orientation measurement.
  okvis::kinematics::Transformation T_SL_;  ///< Transformation from Sonar3DOdometry to imu sensor frame.

  // weighting related
  information_t information_;            ///< The 6x6 information matrix.
  information_t _squareRootInformation;  ///< The 6x6 square root information matrix.
  covariance_t covariance_;              ///< The 6x6 covariance matrix.
};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_SONAR3DODOMETRYERROR_HPP_ */
