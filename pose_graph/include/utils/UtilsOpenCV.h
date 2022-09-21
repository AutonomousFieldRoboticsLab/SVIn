#pragma once

#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using KeypointCV = cv::Point2f;
using KeypointsCV = std::vector<KeypointCV>;
using DMatchVec = std::vector<cv::DMatch>;

class UtilsOpenCV {
 public:
  /* ------------------------------------------------------------------------ */
  // add circles in the image at desired position/size/color
  static void DrawCirclesInPlace(cv::Mat& img,  // NOLINT
                                 const KeypointsCV& image_points,
                                 const cv::Scalar& color = cv::Scalar(0, 255, 0),
                                 const double& msize = 3,
                                 const std::vector<int>& point_ids = std::vector<int>(),
                                 const int& rem_id = 1e9);

  /* ------------------------------------------------------------------------ */
  // add squares in the image at desired position/size/color
  static void DrawSquaresInPlace(cv::Mat& img,  // NOLINT
                                 const std::vector<cv::Point2f>& imagePoints,
                                 const cv::Scalar& color = cv::Scalar(0, 255, 0),
                                 const double msize = 10,
                                 const std::vector<int>& pointIds = std::vector<int>(),
                                 const int remId = 1e9);

  /* ------------------------------------------------------------------------ */
  // add x in the image at desired position/size/color
  static void DrawCrossesInPlace(cv::Mat& img,  // NOLINT
                                 const std::vector<cv::Point2f>& imagePoints,
                                 const cv::Scalar& color = cv::Scalar(0, 255, 0),
                                 const double msize = 0.2,
                                 const std::vector<int>& pointIds = std::vector<int>(),
                                 const int remId = 1e9);

  /* ------------------------------------------------------------------------ */
  // add text (vector of doubles) in the image at desired position/size/color
  static void DrawTextInPlace(cv::Mat& img,  // NOLINT
                              const std::vector<cv::Point2f>& imagePoints,
                              const cv::Scalar& color = cv::Scalar(0, 255, 0),
                              const double msize = 0.4,
                              const std::vector<double>& textDoubles = std::vector<double>());

  /* ------------------------------------------------------------------------ */
  // Concatenate two images and return results as a new mat.
  // Clones the two images.
  static cv::Mat concatenateTwoImages(const cv::Mat& imL_in, const cv::Mat& imR_in);

  /* ------------------------------------------------------------------------ */
  // Draw corner matches and return results as a new mat.
  static cv::Mat DrawCornersMatches(const cv::Mat& img1,
                                    const KeypointsCV& corners1,
                                    const cv::Mat& img2,
                                    const KeypointsCV& corners2,
                                    const DMatchVec& matches,
                                    const bool& randomColor = false);

  /* ------------------------------------------------------------------------ */
  static cv::Mat DrawCircles(const cv::Mat img,
                             const KeypointsCV& imagePoints,
                             const std::vector<cv::Scalar>& circle_colors = std::vector<cv::Scalar>(),
                             const std::vector<double>& circle_sizes = std::vector<double>());

  /* -------------------------------------------------------------------------- */
  static cv::Mat DrawCircles(const cv::Mat img,
                             const std::vector<cv::KeyPoint>& image_points,
                             const std::vector<cv::Scalar>& circle_colors = std::vector<cv::Scalar>(),
                             const std::vector<double>& circle_sizes = std::vector<double>());

  /* ------------------------------------------------------------------------ */
  static void DrawCornersMatchesOneByOne(const cv::Mat img1,
                                         const std::vector<cv::Point2f>& corners1,
                                         const cv::Mat img2,
                                         const std::vector<cv::Point2f>& corners2,
                                         const DMatchVec& matches);

  /* ------------------------------------------------------------------------ */
  static void showImagesSideBySide(const cv::Mat& img_left,
                                   const cv::Mat& img_right,
                                   const std::string& title,
                                   const bool& show_images,
                                   const bool& save_images,
                                   const std::string& folder = "");

  static cv::Mat DrawCornersMatches(const cv::Mat& img1,
                                    const std::vector<cv::KeyPoint>& corners_1,
                                    const cv::Mat& img2,
                                    const KeypointsCV& corners_2,
                                    const bool& random_color);

  static cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg);
};
