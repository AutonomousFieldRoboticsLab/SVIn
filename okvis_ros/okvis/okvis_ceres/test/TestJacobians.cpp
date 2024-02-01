#include <gtest/gtest.h>

#include <okvis/assert_macros.hpp>
#include <okvis/ceres/PoseManifold.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>

const double jacobianTolerance = 1.0e-3;

TEST(okvisTestSuite, JacobianTest) {
  // initialize random number generator
  srand((unsigned int)time(0));  // disabled: make unit tests deterministic...

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

  okvis::kinematics::Transformation T_WS;
  T_WS.setRandom();
  okvis::Time time(0.1);
  okvis::ceres::PoseParameterBlock poseParameterBlock(T_WS, 0, time);  // ground truth

  double* x = poseParameterBlock.parameters();

  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> Jplus;
  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> Jplus_numDiff;

  okvis::ceres::PoseManifold::plusJacobian(x, Jplus.data());

  double dx = 1e-9;

  for (size_t i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 7, 1> xp;
    Eigen::Matrix<double, 7, 1> xm;
    Eigen::Matrix<double, 6, 1> delta;
    delta.setZero();
    delta[i] = dx;
    okvis::ceres::PoseManifold::plus(x, delta.data(), xp.data());
    delta[i] = -dx;
    okvis::ceres::PoseManifold::plus(x, delta.data(), xm.data());
    Jplus_numDiff.col(i) = (xp - xm) / (2 * dx);
  }

  okvis::ceres::PoseManifold pose_manifold;
  OKVIS_ASSERT_TRUE(Exception, pose_manifold.verify(x, 1e-6), "Pose manifold verification failed.");

  // x = poseParameterBlock.parameters();
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> Jminus;
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> Jminus_numDiff;

  okvis::ceres::PoseManifold::minusJacobian(x, Jminus.data());

  Eigen::Matrix<double, 7, 1> xp, xm, x_vec(x);

  for (size_t i = 0; i < 7; ++i) {
    Eigen::Matrix<double, 6, 1> delta_p, delta_m;

    xp = x_vec;
    xm = x_vec;
    xp[i] = x[i] + dx;
    xm[i] = x[i] - dx;

    okvis::ceres::PoseManifold::minus(xp.data(), x, delta_p.data());
    okvis::ceres::PoseManifold::minus(xm.data(), x, delta_m.data());
    Jminus_numDiff.col(i) = (delta_p - delta_m) / (2 * dx);
  }

  OKVIS_ASSERT_TRUE(Exception,
                    (Jminus - Jminus_numDiff).norm() < jacobianTolerance,
                    "Jacobian  = \n"
                        << Jminus << std::endl
                        << "numDiff  Jacobian = \n"
                        << Jminus_numDiff);

  Eigen::Matrix<double, 3, 7, Eigen::RowMajor> Jminus_pose3d;
  Eigen::Matrix<double, 3, 7, Eigen::RowMajor> Jminus_numDiff_pose3d;

  okvis::ceres::PoseManifold3d::minusJacobian(x, Jminus_pose3d.data());

  for (size_t i = 0; i < 7; ++i) {
    Eigen::Matrix<double, 3, 1> delta_p, delta_m;

    xp = x_vec;
    xm = x_vec;
    xp[i] = x[i] + dx;
    xm[i] = x[i] - dx;

    okvis::ceres::PoseManifold3d::minus(xp.data(), x, delta_p.data());
    okvis::ceres::PoseManifold3d::minus(xm.data(), x, delta_m.data());
    Jminus_numDiff_pose3d.col(i) = (delta_p - delta_m) / (2 * dx);
  }

  OKVIS_ASSERT_TRUE(Exception,
                    (Jminus_pose3d - Jminus_numDiff_pose3d).norm() < jacobianTolerance,
                    "Jacobian  = \n"
                        << Jminus_pose3d << std::endl
                        << "numDiff  Jacobian = \n"
                        << Jminus_numDiff_pose3d);

  // LOG(INFO) << "Jminus\n" << Jminus << std::endl;
  // LOG(INFO) << "Jminus_pose3d\n" << Jminus_pose3d << std::endl;

  Eigen::Matrix<double, 4, 7, Eigen::RowMajor> Jminus_pose4d;
  Eigen::Matrix<double, 4, 7, Eigen::RowMajor> Jminus_numDiff_pose4d;

  okvis::ceres::PoseManifold4d::minusJacobian(x, Jminus_pose4d.data());

  for (size_t i = 0; i < 7; ++i) {
    Eigen::Matrix<double, 4, 1> delta_p, delta_m;

    xp = x_vec;
    xm = x_vec;
    xp[i] = x[i] + dx;
    xm[i] = x[i] - dx;

    okvis::ceres::PoseManifold4d::minus(xp.data(), x, delta_p.data());
    okvis::ceres::PoseManifold4d::minus(xm.data(), x, delta_m.data());
    Jminus_numDiff_pose4d.col(i) = (delta_p - delta_m) / (2 * dx);
  }

  OKVIS_ASSERT_TRUE(Exception,
                    (Jminus_pose4d - Jminus_numDiff_pose4d).norm() < jacobianTolerance,
                    "Jacobian  = \n"
                        << Jminus_pose4d << std::endl
                        << "numDiff  Jacobian = \n"
                        << Jminus_numDiff_pose4d);

  // LOG(INFO) << "Jminus\n" << Jminus << std::endl;
  // LOG(INFO) << "Jminus_pose4d\n" << Jminus_pose4d << std::endl;

  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> Jminus_pose2d;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> Jminus_numDiff_pose2d;

  okvis::ceres::PoseManifold2d::minusJacobian(x, Jminus_pose2d.data());

  for (size_t i = 0; i < 7; ++i) {
    Eigen::Matrix<double, 2, 1> delta_p, delta_m;

    xp = x_vec;
    xm = x_vec;
    xp[i] = x[i] + dx;
    xm[i] = x[i] - dx;

    okvis::ceres::PoseManifold2d::minus(xp.data(), x, delta_p.data());
    okvis::ceres::PoseManifold2d::minus(xm.data(), x, delta_m.data());
    Jminus_numDiff_pose2d.col(i) = (delta_p - delta_m) / (2 * dx);
  }

  OKVIS_ASSERT_TRUE(Exception,
                    (Jminus_pose2d - Jminus_numDiff_pose2d).norm() < jacobianTolerance,
                    "Jacobian  = \n"
                        << Jminus_pose2d << std::endl
                        << "numDiff  Jacobian = \n"
                        << Jminus_numDiff_pose2d);

  // LOG(INFO) << "Jminus\n" << Jminus << std::endl;
  // LOG(INFO) << "Jminus_pose2d\n" << Jminus_pose2d << std::endl;
}