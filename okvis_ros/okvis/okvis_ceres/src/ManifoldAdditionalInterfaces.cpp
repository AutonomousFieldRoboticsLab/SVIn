/*
 * LocalParamizationAdditionalInterfaces.cpp
 *
 *  Created on: 27 Jul 2015
 *      Author: sleutene
 */

#include <okvis/ceres/ManifoldAdditionalInterfaces.hpp>

namespace okvis {
namespace ceres {

// Verifies the correctness of a inplementation.
bool ManifoldAdditionalInterfaces::verify(const double* x_raw, double purturbation_magnitude) const {
  const ::ceres::Manifold* casted = dynamic_cast<const ::ceres::Manifold*>(this);
  if (!casted) {
    return false;
  }
  // verify plus/minus
  Eigen::VectorXd x(casted->AmbientSize());
  memcpy(x.data(), x_raw, sizeof(double) * casted->AmbientSize());
  Eigen::VectorXd delta_x(casted->TangentSize());
  Eigen::VectorXd x_plus_delta(casted->AmbientSize());
  Eigen::VectorXd delta_x2(casted->TangentSize());
  delta_x.setRandom();
  delta_x *= purturbation_magnitude;
  casted->Plus(x.data(), delta_x.data(), x_plus_delta.data());
  this->Minus(x.data(), x_plus_delta.data(), delta_x2.data());
  if ((delta_x2 - delta_x).norm() > 1.0e-12) {
    return false;
  }

  // plusJacobian numDiff
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus_num_diff(casted->AmbientSize(), casted->TangentSize());
  const double dx = 1.0e-9;
  for (int i = 0; i < casted->TangentSize(); ++i) {
    Eigen::VectorXd delta_p(casted->TangentSize());
    delta_p.setZero();
    delta_p[i] = dx;
    Eigen::VectorXd delta_m(casted->TangentSize());
    delta_m.setZero();
    delta_m[i] = -dx;

    // reset
    Eigen::VectorXd x_p(casted->AmbientSize());
    Eigen::VectorXd x_m(casted->AmbientSize());
    memcpy(x_p.data(), x_raw, sizeof(double) * casted->AmbientSize());
    memcpy(x_m.data(), x_raw, sizeof(double) * casted->AmbientSize());
    casted->Plus(x.data(), delta_p.data(), x_p.data());
    casted->Plus(x.data(), delta_m.data(), x_m.data());
    J_plus_num_diff.col(i) = (x_p - x_m) / (2 * dx);
  }

  // verify lift
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus(casted->AmbientSize(), casted->TangentSize());
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_lift(casted->TangentSize(), casted->AmbientSize());
  casted->PlusJacobian(x_raw, J_plus.data());
  ComputeLiftJacobian(x_raw, J_lift.data());
  Eigen::MatrixXd identity(casted->TangentSize(), casted->TangentSize());
  identity.setIdentity();
  if (((J_lift * J_plus) - identity).norm() > 1.0e-6) {
    return false;
  }

  // verify numDiff jacobian
  if ((J_plus - J_plus_num_diff).norm() > 1.0e-6) {
    return false;
  }

  // everything fine...
  return true;
}

}  // namespace ceres
}  // namespace okvis
