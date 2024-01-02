#include "okvis/ceres/MagneticSyncPreintegrationError.hpp"

#include <glog/logging.h>

#include "okvis/Measurements.hpp"
#include "okvis/assert_macros.hpp"
#include "okvis/ceres/PoseManifold.hpp"
#include "okvis/ceres/ode/ode.hpp"

namespace okvis {
namespace ceres {

MagneticSyncPreintegrationError::MagneticSyncPreintegrationError(
    const okvis::MagnetometerMeasurementDeque& magnetometer_measurements,
    const okvis::MagnetometerParameters& magnetometer_params,
    const okvis::ImuMeasurementDeque& imu_measurements,
    const okvis::ImuParameters& imu_params,
    const okvis::Time& start_time,
    const okvis::MagnetometerMeasurement& magnetometer_1) {
  // Functions inherited from ceres::CostFunction.
  // https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/cost_function.h
  // Set int num_residuals_.

  int number_of_residuals = 3 * magnetometer_measurements.size();
  set_num_residuals(number_of_residuals);

  mutable_parameter_block_sizes()->clear();
  mutable_parameter_block_sizes()->push_back(7);  // T_WS_0
  mutable_parameter_block_sizes()->push_back(9);  // speed and biases
  mutable_parameter_block_sizes()->push_back(7);  // T_WS_1

  setImuMeasurements(imu_measurements);
  setMagnetometerMeasurements(magnetometer_measurements);
  setImuParameters(imu_params);
  setMagnetometerParameters(magnetometer_params);
  setStartTime(start_time);
  setMagnetometerMeasurementEnd(magnetometer_1);

  auto itr = imu_measurements.begin();
  while (itr->timeStamp <= start_time) {
    itr++;
  }

  for (auto mag_itr = magnetometer_measurements.begin(); mag_itr != magnetometer_measurements.end(); ++mag_itr) {
    assert(itr->timeStamp == mag_itr->timeStamp);
    itr++;
  }

  setEndTime(magnetometer_measurements.back().timeStamp);

  double variance =
      magnetometer_parameters_.sigma_m_c * magnetometer_parameters_.sigma_m_c * magnetometer_parameters_.rate;
  magnetic_covariance_ = Eigen::Matrix3d::Identity() * variance;
  Eigen::Matrix3d information = magnetic_covariance_.inverse();
  information = 0.5 * information + 0.5 * information.transpose().eval();
  Eigen::LLT<Eigen::Matrix3d> lltOfInformation(information);
  sqrt_information_ = lltOfInformation.matrixL().transpose();

  OKVIS_ASSERT_TRUE_DBG(Exception,
                        imu_measurements.back().timeStamp < magnetometer_measurements.back().timeStamp,
                        "Oldest IMU measurement is newer than oldest magnetometer measurement!");
  OKVIS_ASSERT_TRUE_DBG(Exception,
                        imu_measurements.front().timeStamp > magnetometer_measurements.front().timeStamp,
                        "Newest IMU measurement is older than newest magnetometer measurement!");
}

int MagneticSyncPreintegrationError::redoPreintegration(const Eigen::Quaterniond& /*T_WS*/,
                                                        const SpeedAndBias& speed_and_biases) const {
  std::unique_lock l(preintegration_mutex_);

  // perform propagation
  okvis::Time time = t_start_;
  assert(imu_measurements_.front().timeStamp <= time);

  // Oldest imu meas older than oldest gp meas.
  if (!(imu_measurements_.front().timeStamp < magnetometer_measurements_.front().timeStamp)) {
    return -1;  // nothing to do...
  }

  // Newest imu meas newer than newest gp meas.
  if (!(imu_measurements_.back().timeStamp > magnetometer_measurements_.back().timeStamp)) {
    return -1;  // nothing to do...
  }

  // increments initial condition
  delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  dalpha_db_g_ = Eigen::Matrix3d::Zero();
  // sub-Jacobians
  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 6, 6>::Zero();

  // Vectors for preintegrated values at magnetometer residual time.
  delta_qs_vec_.clear();
  dalpha_db_g_vec_.clear();
  P_delta_vec_.clear();
  square_root_information_vec_.clear();

  bool has_started = false;
  uint32_t imu_measumentes_used = 0;

  for (okvis::ImuMeasurementDeque::const_iterator it = imu_measurements_.begin(); it != imu_measurements_.end(); ++it) {
    Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
    Eigen::Vector3d acc_S_0 = it->measurement.accelerometers;
    Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
    Eigen::Vector3d acc_S_1 = (it + 1)->measurement.accelerometers;

    // Meaning the magnetometer is between the current and next IMU measurement.
    okvis::Time next_time = (it + 1)->timeStamp;

    if (next_time > t_end_) break;
    // time delta
    double dt = (next_time - time).toSec();

    // Sanity check
    if (dt <= 0.0) {
      continue;
    }

    if (!has_started) {
      has_started = true;
      const double r = dt / (next_time - imu_measurements_.front().timeStamp).toSec();
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_parameters_.sigma_g_c;
    double sigma_a_c = imu_parameters_.sigma_a_c;

    if (std::abs(omega_S_0[0]) > imu_parameters_.g_max || std::abs(omega_S_0[1]) > imu_parameters_.g_max ||
        std::abs(omega_S_0[2]) > imu_parameters_.g_max || std::abs(omega_S_1[0]) > imu_parameters_.g_max ||
        std::abs(omega_S_1[1]) > imu_parameters_.g_max || std::abs(omega_S_1[2]) > imu_parameters_.g_max) {
      sigma_g_c *= 100;
      LOG(WARNING) << "gyr saturation";
    }

    if (std::abs(acc_S_0[0]) > imu_parameters_.a_max || std::abs(acc_S_0[1]) > imu_parameters_.a_max ||
        std::abs(acc_S_0[2]) > imu_parameters_.a_max || std::abs(acc_S_1[0]) > imu_parameters_.a_max ||
        std::abs(acc_S_1[1]) > imu_parameters_.a_max || std::abs(acc_S_1[2]) > imu_parameters_.a_max) {
      sigma_a_c *= 100;
      LOG(WARNING) << "acc saturation";
    }

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = ode::sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond delta_q_1 = delta_q_ * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C_1 = delta_q_1.toRotationMatrix();

    // For Jacobian
    dalpha_db_g_ += dt * C_1;

    // covariance propagation
    Eigen::Matrix<double, 6, 6> F_delta = Eigen::Matrix<double, 6, 6>::Identity();
    F_delta.block<3, 3>(0, 3) = -dt * C_1;
    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // add noise. Note that transformations with rotation matrices can be
    // ignored, since the noise is isotropic.
    // F_tot = F_delta*F_tot;
    const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
    P_delta_(0, 0) += sigma2_dalpha;
    P_delta_(1, 1) += sigma2_dalpha;
    P_delta_(2, 2) += sigma2_dalpha;
    const double sigma2_b_g = dt * imu_parameters_.sigma_gw_c * imu_parameters_.sigma_gw_c;
    P_delta_(3, 3) += sigma2_b_g;
    P_delta_(4, 4) += sigma2_b_g;
    P_delta_(5, 5) += sigma2_b_g;

    P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();
    // calculate inverse
    Eigen::Matrix<double, 6, 6> information = P_delta_.inverse();
    information = 0.5 * information + 0.5 * information.transpose().eval();

    // square root
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> lltOfInformation(information);
    Eigen::Matrix<double, 6, 6> squareRootInformation = lltOfInformation.matrixL().transpose();

    P_delta_vec_.push_back(P_delta_);
    delta_qs_vec_.push_back(delta_q_1);
    dalpha_db_g_vec_.push_back(dalpha_db_g_);
    square_root_information_vec_.push_back(squareRootInformation);

    // memory shift
    delta_q_ = delta_q_1;
    time = next_time;
    imu_measumentes_used++;
  }

  // store the reference (linearisation) point
  speed_and_biases_ref_ = speed_and_biases;
  return imu_measumentes_used;
}

int MagneticSyncPreintegrationError::propagation(const okvis::ImuMeasurementDeque& imu_measurements,
                                                 const okvis::ImuParameters& imu_parameters,
                                                 const okvis::MagnetometerMeasurementDeque& magnetometer_measurements,
                                                 okvis::kinematics::Transformation& T_WS0,
                                                 okvis::SpeedAndBias& speed_and_biases0,
                                                 Quaternions& propragated_quaternions,
                                                 const okvis::Time& t_start,
                                                 Covariances* covariances,
                                                 Jacobians* jacobians) {
  okvis::Time time = t_start;
  assert(imu_measurements.front().timeStamp <= time);
  okvis::Time t_end = magnetometer_measurements.back().timeStamp;
  if (!(imu_measurements.back().timeStamp >= t_end)) return -1;  // nothing to do...

  // Oldest imu meas older than oldest gp meas.
  if (!(imu_measurements.front().timeStamp < magnetometer_measurements.front().timeStamp)) {
    return -1;  // nothing to do...
  }

  // Newest imu meas newer than newest gp meas.
  if (!(imu_measurements.back().timeStamp > magnetometer_measurements.back().timeStamp)) {
    return -1;  // nothing to do...
  }

  // initial condition
  Eigen::Quaterniond q_WS_0 = T_WS0.q();
  Eigen::Matrix3d C_WS_0 = T_WS0.C();

  // increments
  Eigen::Quaterniond delta_q = Eigen::Quaterniond(1, 0, 0, 0);

  // sub-Jacobians
  Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double, 6, 6> P_delta = Eigen::Matrix<double, 6, 6>::Zero();

  // Vectors for preintegrated values at rts residual time.
  std::vector<Eigen::Quaterniond> delta_qs;
  std::vector<Eigen::Matrix3d> dalpha_db_gs;
  std::vector<Eigen::Matrix<double, 6, 6>> P_deltas;

  bool has_started = false;
  uint32_t imu_measumentes_used = 0;

  for (okvis::ImuMeasurementDeque::const_iterator it = imu_measurements.begin(); it != imu_measurements.end(); ++it) {
    Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
    Eigen::Vector3d acc_S_0 = it->measurement.accelerometers;
    Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
    Eigen::Vector3d acc_S_1 = (it + 1)->measurement.accelerometers;

    // time delta

    okvis::Time next_time;
    if ((it + 1) == imu_measurements.end()) {
      next_time = t_end;
    } else
      next_time = (it + 1)->timeStamp;

    // time delta
    double dt = (next_time - time).toSec();

    if (t_end < next_time) {
      double interval = (next_time - it->timeStamp).toSec();
      next_time = t_end;
      dt = (next_time - time).toSec();
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    // Sanity check
    if (dt <= 0.0) {
      continue;
    }

    if (!has_started) {
      has_started = true;
      const double r = dt / (next_time - imu_measurements.front().timeStamp).toSec();
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_parameters.sigma_g_c;
    double sigma_a_c = imu_parameters.sigma_a_c;

    if (std::abs(omega_S_0[0]) > imu_parameters.g_max || std::abs(omega_S_0[1]) > imu_parameters.g_max ||
        std::abs(omega_S_0[2]) > imu_parameters.g_max || std::abs(omega_S_1[0]) > imu_parameters.g_max ||
        std::abs(omega_S_1[1]) > imu_parameters.g_max || std::abs(omega_S_1[2]) > imu_parameters.g_max) {
      sigma_g_c *= 100;
      LOG(WARNING) << "gyr saturation";
    }

    if (std::abs(acc_S_0[0]) > imu_parameters.a_max || std::abs(acc_S_0[1]) > imu_parameters.a_max ||
        std::abs(acc_S_0[2]) > imu_parameters.a_max || std::abs(acc_S_1[0]) > imu_parameters.a_max ||
        std::abs(acc_S_1[1]) > imu_parameters.a_max || std::abs(acc_S_1[2]) > imu_parameters.a_max) {
      sigma_a_c *= 100;
      LOG(WARNING) << "acc saturation";
    }

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases0.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = ode::sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond delta_q_1 = delta_q * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C_1 = delta_q_1.toRotationMatrix();

    // For Jacobian
    dalpha_db_g += dt * C_1;

    if (covariances) {
      Eigen::Matrix<double, 6, 6> F_delta = Eigen::Matrix<double, 6, 6>::Identity();
      F_delta.block<3, 3>(0, 3) = -dt * C_1;
      P_delta = F_delta * P_delta * F_delta.transpose();

      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta(0, 0) += sigma2_dalpha;
      P_delta(1, 1) += sigma2_dalpha;
      P_delta(2, 2) += sigma2_dalpha;

      const double sigma2_b_g = dt * imu_parameters.sigma_gw_c * imu_parameters.sigma_gw_c;
      P_delta(3, 3) += sigma2_b_g;
      P_delta(4, 4) += sigma2_b_g;
      P_delta(5, 5) += sigma2_b_g;

      P_delta = 0.5 * P_delta + 0.5 * P_delta.transpose().eval();
      // store quantity
      P_deltas.push_back(P_delta);
    }

    delta_qs.push_back(delta_q_1);
    dalpha_db_gs.push_back(dalpha_db_g);

    // memory shift
    delta_q = delta_q_1;

    time = next_time;
    imu_measumentes_used++;

    if (next_time == t_end) break;
  }

  // actual propagation output:
  // Only rotation

  for (uint32_t i = 0; i < magnetometer_measurements.size(); i++) {
    propragated_quaternions.push_back(q_WS_0 * delta_qs[i]);
  }

  // assign Jacobian, if requested (Only at ;ast rts measurement at the moment)
  if (jacobians) {
    for (uint32_t i = 0; i < magnetometer_measurements.size(); ++i) {
      Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
      F.block<3, 3>(0, 3) = -C_WS_0 * dalpha_db_gs[i];
      jacobians->push_back(F);
    }
  }

  if (covariances) {
    for (size_t i = 0; i < magnetometer_measurements.size(); ++i) {
      Eigen::Matrix<double, 6, 6> P;
      // transform from local increments to actual states
      Eigen::Matrix<double, 6, 6> T = Eigen::Matrix<double, 6, 6>::Identity();
      T.topLeftCorner<3, 3>() = C_WS_0;
      P = T * P_deltas[i] * T.transpose();

      covariances->push_back(P);
    }
  }

  return imu_measumentes_used;
}

// This evaluates the error term and additionally computes the Jacobians.
bool MagneticSyncPreintegrationError::Evaluate(double const* const* parameters,
                                               double* residuals,
                                               double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

bool MagneticSyncPreintegrationError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                                                   double* residuals,
                                                                   double** jacobians,
                                                                   double** jacobians_minimal) const {
  // get poses
  const Eigen::Quaterniond q_WS_0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  // get speed and bias
  SpeedAndBias speed_and_biases_0;
  for (size_t i = 0; i < 9; ++i) {
    speed_and_biases_0[i] = parameters[1][i];
  }

  const Eigen::Quaterniond q_WS_1(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  // call propagation
  const double dt = (t_end_ - t_start_).toSec();
  Eigen::Vector3d delta_b;

  // ensure unique access
  preintegration_mutex_.lock();
  delta_b = speed_and_biases_0.segment<3>(3) - speed_and_biases_ref_.segment<3>(3);
  preintegration_mutex_.unlock();

  redo_ = redo_ || (delta_b.norm() * dt > 0.0001);
  if (redo_) {
    redoPreintegration(q_WS_0, speed_and_biases_0);
    redoCounter_++;
    delta_b.setZero();
    redo_ = false;
  }

  const size_t n_residuals_terms = static_cast<size_t>(num_residuals());

  const size_t n_residuals = static_cast<size_t>(n_residuals_terms / 3);
  // Map residuals for Ceres.
  Eigen::Matrix<double, Eigen::Dynamic, 1> weighted_error = Eigen::MatrixXd::Zero(n_residuals_terms, 1);

  // Jacobian with respect to orientation
  std::vector<Eigen::Matrix<double, 3, 6>> J0_minimal_vec;
  std::vector<Eigen::Matrix<double, 3, 9>> J1_vec;
  std::vector<Eigen::Matrix<double, 3, 6>> J2_minimal_vec;

  // read only lock
  const Eigen::Matrix3d C_WS_0 = q_WS_0.toRotationMatrix();
  const Eigen::Matrix3d C_SW_0 = C_WS_0.transpose();
  const Eigen::Matrix3d C_WS_1 = q_WS_1.toRotationMatrix();
  const Eigen::Matrix3d C_SW_1 = C_WS_1.transpose();

  preintegration_mutex_.lock_shared();

  assert(delta_qs_vec_.size() == n_residuals);

  for (size_t i = 0; i < n_residuals; ++i) {
    const Eigen::Vector3d magnetic_measurement = magnetometer_measurements_[i].measurement.flux_density_;

    const Eigen::Quaterniond dq = okvis::kinematics::deltaQ(-dalpha_db_g_vec_[i] * delta_b) * delta_qs_vec_[i];
    const Eigen::Vector3d mag_measurement_0 = dq * magnetic_measurement;
    const Eigen::Vector3d mag_measurement_1 = magnetometer_measurement_1_.measurement.flux_density_;
    Eigen::Vector3d error = C_SW_0 * C_WS_1 * mag_measurement_1 - mag_measurement_0;

    // Eigen::Matrix3d rot_variance = P_delta_vec_[i].topLeftCorner<3, 3>();
    // Eigen::Matrix3d R = dq.toRotationMatrix();
    // Eigen::Matrix3d cross_m_x = R * okvis::kinematics::crossMx(magnetic_measurement);
    // Eigen::Matrix3d total_covariance =
    //     cross_m_x * rot_variance * cross_m_x.transpose() + R * magnetic_covariance_ * R.transpose();
    // LOG(INFO) << "Error:  " << error.transpose() << std::endl;
    // total_covariance = 0.5 * total_covariance + 0.5 * total_covariance.transpose().eval();

    // Eigen::Matrix3d information = total_covariance.inverse();
    // information = 0.5 * information + 0.5 * information.transpose().eval();
    // Eigen::LLT<Eigen::Matrix3d> lltOfInformation(information);
    // Eigen::Matrix3d sqrt_information = lltOfInformation.matrixL().transpose();

    weighted_error.segment<3>(3 * i) = sqrt_information_ * error;

    Eigen::Matrix<double, 3, 6> J0_minimal = Eigen::Matrix<double, 3, 6>::Zero();
    J0_minimal.block<3, 3>(0, 3) = sqrt_information_ * C_SW_0 * okvis::kinematics::crossMx(C_WS_1 * mag_measurement_1);

    Eigen::Matrix<double, 3, 9> J1;
    J1.setZero();
    // J1.block<3, 3>(0, 3) = sqrt_information_ * -okvis::kinematics::crossMx(mag_measurement_0) * dalpha_db_g_vec_[i];
    Eigen::Matrix<double, 3, 6> J2_minimal = Eigen::Matrix<double, 3, 6>::Zero();
    J2_minimal.block<3, 3>(0, 3) = sqrt_information_ * C_SW_0 * -okvis::kinematics::crossMx(C_WS_1 * mag_measurement_1);

    J0_minimal_vec.push_back(J0_minimal);
    J1_vec.push_back(J1);
    J2_minimal_vec.push_back(J2_minimal);
  }

  preintegration_mutex_.unlock_shared();

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(residuals, n_residuals_terms, 1) = weighted_error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::MatrixXd J0 = Eigen::MatrixXd::Zero(n_residuals_terms, 7);
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J0_lift;
      PoseManifold::liftJacobian(parameters[0], J0_lift.data());

      for (size_t k = 0; k < n_residuals; ++k) {
        J0.block<3, 7>(3 * k, 0) = J0_minimal_vec[k] * J0_lift;
      }

      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::RowMajor>>(jacobians[0], n_residuals_terms, 7) = J0;

      if (jacobians_minimal != nullptr) {
        if (jacobians_minimal[0] != nullptr) {
          Eigen::MatrixXd J0_minimal = Eigen::MatrixXd::Zero(n_residuals_terms, 6);

          for (size_t k = 0; k < n_residuals; ++k) {
            J0_minimal.block<3, 6>(3 * k, 0) = J0_minimal_vec[k];
          }

          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>>(
              jacobians_minimal[0], n_residuals_terms, 6) = J0_minimal;
        }
      }
    }

    if (jacobians[1] != nullptr) {
      Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(n_residuals_terms, 9);

      for (size_t k = 0; k < n_residuals; ++k) {
        J1.block<3, 9>(3 * k, 0) = J1_vec[k];  //.block<3,9>(0,0);
      }

      // Map jacobian
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 9, Eigen::RowMajor>>(jacobians[1], n_residuals_terms, 9) = J1;

      // if requested, provide minimal Jacobians
      if (jacobians_minimal != nullptr) {
        if (jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 9, Eigen::RowMajor>>(
              jacobians_minimal[1], n_residuals_terms, 9) = J1;
        }
      }
    }

    if (jacobians[2] != nullptr) {
      Eigen::MatrixXd J2 = Eigen::MatrixXd::Zero(n_residuals_terms, 7);
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J2_lift;
      PoseManifold::liftJacobian(parameters[2], J2_lift.data());

      for (size_t k = 0; k < n_residuals; ++k) {
        J2.block<3, 7>(3 * k, 0) = J2_minimal_vec[k] * J2_lift;
      }

      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::RowMajor>>(jacobians[2], n_residuals_terms, 7) = J2;

      if (jacobians_minimal != nullptr) {
        if (jacobians_minimal[2] != nullptr) {
          Eigen::MatrixXd J2_minimal = Eigen::MatrixXd::Zero(n_residuals_terms, 6);

          for (size_t k = 0; k < n_residuals; ++k) {
            J2_minimal.block<3, 6>(3 * k, 0) = J2_minimal_vec[k];
          }

          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>>(
              jacobians_minimal[2], n_residuals_terms, 6) = J2_minimal;
        }
      }
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis