#pragma once

#include <ceres/cost_function.h>

#include <memory>
#include <shared_mutex>

#include "okvis/Measurements.hpp"
#include "okvis/Parameters.hpp"
#include "okvis/ceres/ErrorInterface.hpp"
namespace okvis {
namespace ceres {

class MagneticSyncPreintegrationError : public ::ceres::CostFunction, public ErrorInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 3 for rotation, 3 for gyro bias
  typedef std::deque<Eigen::Matrix<double, 6, 6>> Covariances;

  typedef std::deque<Eigen::Matrix<double, 6, 6>> Jacobians;

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  // Default constructor
  MagneticSyncPreintegrationError() = default;

  // Default Destructor
  virtual ~MagneticSyncPreintegrationError() = default;

  MagneticSyncPreintegrationError(const okvis::MagnetometerMeasurementDeque& magnetometer_measurements,
                                  const okvis::MagnetometerParameters& magnetometer_params,
                                  const okvis::ImuMeasurementDeque& imu_measurements,
                                  const okvis::ImuParameters& imu_params,
                                  const okvis::Time& t_start,
                                  const okvis::MagnetometerMeasurement& magnetometer_1);

  static int propagation(const okvis::ImuMeasurementDeque& imu_measurements,
                         const okvis::ImuParameters& imu_parameters,
                         const okvis::MagnetometerMeasurementDeque& magnetometer_measurements,
                         okvis::kinematics::Transformation& T_WS0,
                         okvis::SpeedAndBias& speed_and_bias0,
                         Quaternions& propagated_quaternions,
                         const okvis::Time& t_start,
                         Covariances* covariances = nullptr,
                         Jacobians* jacobians = nullptr);

  int redoPreintegration(const Eigen::Quaterniond& q_WS, const SpeedAndBias& speed_and_biases) const;
  inline void setRedo(const bool redo = true) const { redo_ = redo; }

  inline void setImuParameters(const okvis::ImuParameters& imu_params) { imu_parameters_ = imu_params; }

  inline void setMagnetometerParameters(const okvis::MagnetometerParameters& magnetometer_params) {
    magnetometer_parameters_ = magnetometer_params;
  }

  void setImuMeasurements(const okvis::ImuMeasurementDeque& imu_measurements) { imu_measurements_ = imu_measurements; }

  void setMagnetometerMeasurements(const okvis::MagnetometerMeasurementDeque& magnetometer_measurements) {
    magnetometer_measurements_ = magnetometer_measurements;
  }

  void setMagnetometerMeasurementEnd(const okvis::MagnetometerMeasurement& mag_measurement_end) {
    magnetometer_measurement_1_ = mag_measurement_end;
  }

  void setStartTime(const okvis::Time& t_start) { t_start_ = t_start; }

  void setEndTime(const okvis::Time& t_end) { t_end_ = t_end; }

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

  bool EvaluateWithMinimalJacobians(double const* const* parameters,
                                    double* residuals,
                                    double** jacobians,
                                    double** jacobians_minimal) const;
  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return static_cast<size_t>(num_residuals()); }

  /// \brief Number of parameter blocks.
  virtual size_t parameterBlocks() const { return parameter_block_sizes().size(); }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameter_block_idx Index of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameter_block_idx) const {
    return ::ceres::CostFunction::parameter_block_sizes().at(parameter_block_idx);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const { return "MagneticSyncPreintegrationError"; }

  static bool symmetricMatrixJacobian(const double* parameters, double* jacobian);

 protected:
  // parameters
  okvis::ImuParameters imu_parameters_;
  okvis::MagnetometerParameters magnetometer_parameters_;

  // measurements
  okvis::ImuMeasurementDeque imu_measurements_;
  okvis::MagnetometerMeasurementDeque magnetometer_measurements_;

  okvis::MagnetometerMeasurement magnetometer_measurement_1_;
  // start and end time
  okvis::Time t_start_;
  okvis::Time t_end_;

  mutable std::shared_mutex preintegration_mutex_;

  // increments (initialise with identity)
  mutable Eigen::Quaterniond delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);

  // sub-Jacobians
  mutable Eigen::Matrix3d dalpha_db_g_ = Eigen::Matrix3d::Zero();

  /// \brief The Jacobian of the increment (w/o biases).
  mutable Eigen::Matrix<double, 6, 6> P_delta_ = Eigen::Matrix<double, 6, 6>::Zero();

  // increments vectors
  mutable std::vector<Eigen::Quaterniond> delta_qs_vec_;
  mutable std::vector<Eigen::Matrix3d> dalpha_db_g_vec_;
  mutable std::vector<Eigen::Matrix<double, 6, 6>> P_delta_vec_;
  mutable std::vector<Eigen::Matrix<double, 6, 6>> square_root_information_vec_;

  /// \brief Reference biases that are updated when called redoPreintegration.
  mutable SpeedAndBias speed_and_biases_ref_ = SpeedAndBias::Zero();

  mutable bool redo_ = true;
  ///< Keeps track of whether or not this redoPreintegration() needs to be done.
  mutable int redoCounter_ = 0;
  ///< Counts the number of preintegrations for statistics.

  // Gp covariance, information matrix and its square root
  mutable Eigen::Matrix3d magnetic_covariance_;
  Eigen::Matrix3d sqrt_information_;

  /// ID
  size_t id_ = 0;

  mutable int magnetic_measurement_start_index_ = 0;
};

}  // namespace ceres
}  // namespace okvis
