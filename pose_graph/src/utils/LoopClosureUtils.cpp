#include "utils/LoopClosureUtils.h"

#include <ros/ros.h>

#include <memory>
#include <vector>

bool LoopClosureUtils::geometricVerificationNister(const std::vector<cv::KeyPoint>& matched_2d_cur,
                                                   const std::vector<cv::Point2f>& matched_2d_old,
                                                   std::vector<uchar>& status,
                                                   int min_correspondences,
                                                   opengv::transformation_t* camMatch_T_camQuery_mono) {
  BearingVectors query_versors, match_versors;

  query_versors.resize(matched_2d_cur.size());
  match_versors.resize(matched_2d_old.size());
  status.resize(matched_2d_cur.size());

  for (size_t i = 0; i < match_versors.size(); i++) {
    Eigen::Vector3d p_cur(matched_2d_cur[i].pt.x, matched_2d_cur[i].pt.y, 1.0);
    Eigen::Vector3d p_old(matched_2d_old[i].x, matched_2d_old[i].y, 1.0);
    query_versors[i] = p_cur.normalized();
    match_versors[i] = p_old.normalized();
    status[i] = 0;
  }

  // Recover relative pose between frames, with translation up to a scalar.
  if (static_cast<int>(match_versors.size()) >= min_correspondences) {
    AdapterMono adapter(match_versors, query_versors);

    // Use RANSAC to solve the central-relative-pose problem.
    opengv::sac::Ransac<SacProblemMono> ransac;

    ransac.sac_model_ = std::make_shared<SacProblemMono>(adapter, SacProblemMono::Algorithm::NISTER, true);
    ransac.max_iterations_ = 500;
    ransac.probability_ = 0.995;
    ransac.threshold_ = 1e-5;

    // Compute transformation via RANSAC.
    bool ransac_success = ransac.computeModel();
    // ROS_INFO_STREAM("ransac 5pt size of input: " << query_versors.size()
    //  << "\nransac 5pt inliers: " << ransac.inliers_.size()
    //  << "\nransac 5pt iterations: " << ransac.iterations_);
    if (!ransac_success) {
      // ROS_WARN_STREAM("LoopClosureDetector Failure: RANSAC 5pt could not solve.");
    } else {
      double inlier_percentage = static_cast<double>(ransac.inliers_.size()) / query_versors.size();

      if (inlier_percentage >= 0.20 && ransac.iterations_ < 500) {
        *camMatch_T_camQuery_mono = ransac.model_coefficients_;
        // Remove the outliers based on ransac.inliers (modify i_query and
        // i_match AND pass the result out)
        for (const auto i : ransac.inliers_) {
          status[i] = 1;
        }
        return true;
      }
    }
  }

  return false;
}
