#include "utils/Utils.h"

#include <string>

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d& g) {
  Eigen::Matrix3d R0;
  Eigen::Vector3d ng1 = g.normalized();
  Eigen::Vector3d ng2{0, 0, 1.0};
  R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
  return R0;
}

std::string Utility::getTimeStr() {
  std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  char s[100];
  std::strftime(s, sizeof(s), "%Y_%m_%d_%H_%M_%S", std::localtime(&now));
  return s;
}

Eigen::Matrix4d Utility::rosPoseToMatrix(const geometry_msgs::Pose& pose) {
  Eigen::Matrix4d m;
  m.setIdentity();
  m.block<3, 3>(0, 0) =
      Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
          .toRotationMatrix();
  m.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return m;
}

geometry_msgs::Pose Utility::matrixToRosPose(const Eigen::Matrix4d& trasform) {
  geometry_msgs::Pose pose;
  pose.position.x = trasform(0, 3);
  pose.position.y = trasform(1, 3);
  pose.position.z = trasform(2, 3);

  Eigen::Quaterniond q(trasform.block<3, 3>(0, 0));
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  return pose;
}

void Utility::printPoseAsEulerAngles(const Eigen::Matrix4d& pose) {
  Eigen::Vector3d trans = pose.block<3, 1>(0, 3);
  Eigen::Matrix3d rotm = pose.block<3, 3>(0, 0);
  std::cout << "trans: " << trans.transpose() << "\teul: " << Utility::R2ypr(rotm).transpose() << std::endl;
}

// Converts doulbe to sting with desired number of digits (total number of
// digits)
std::string Utility::To_string_with_precision(const double a_value, const int n) {
  std::ostringstream out;
  out << std::setprecision(n) << a_value;
  return out.str();
}

std::string Utility::healthMsgToString(const okvis_ros::SvinHealthConstPtr& health) {
  std::stringstream ss;
  std::setprecision(5);
  ss << "#keypoints: " << health->numTrackedKps << ","
     << "#newkps: " << health->newKps << "\n";
  ss << "keyframes_per_quartile: " << health->kpsPerQuadrant[0] << "," << health->kpsPerQuadrant[1] << ","
     << health->kpsPerQuadrant[2] << "," << health->kpsPerQuadrant[3] << ",";

  return ss.str();
}
