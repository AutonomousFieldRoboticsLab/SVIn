#ifndef UTILS_LOOPCLOSUREUTILS_H_
#define UTILS_LOOPCLOSUREUTILS_H_

#include <Eigen/Core>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <vector>

#include "utils/UtilsOpenCV.h"
using BearingVectors = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

class LoopClosureUtils {
 private:
  // Lcd typedefs
  using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
  using SacProblemMono = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using AdapterStereo = opengv::point_cloud::PointCloudAdapter;
  using SacProblemStereo = opengv::sac_problems::point_cloud::PointCloudSacProblem;

 public:
  static bool geometricVerificationNister(const std::vector<cv::KeyPoint>& matched_2d_cur,
                                          const std::vector<cv::Point2f>& matched_2d_old,
                                          std::vector<uchar>& status,  // NOLINT
                                          int min_correspondences,
                                          opengv::transformation_t* camMatch_T_camQuery_mono);
};

#endif  // UTILS_LOOPCLOSUREUTILS_H_
