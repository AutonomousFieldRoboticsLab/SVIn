#include "utils/UtilsOpenCV.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <algorithm>
#include <string>
#include <vector>

#include "utils/Utils.h"

/* -------------------------------------------------------------------------- */
// add circles in the image at desired position/size/color
void UtilsOpenCV::DrawCirclesInPlace(cv::Mat& img,
                                     const KeypointsCV& image_points,
                                     const cv::Scalar& color,
                                     const double& msize,
                                     const std::vector<int>& point_ids,
                                     const int& rem_id) {
  // text offset
  cv::Point2f text_offset(-10, -5);
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0u; i < image_points.size(); i++) {
    cv::circle(img, image_points[i], msize, color, 2);
    if (point_ids.size() == image_points.size()) {
      // We also have text
      cv::putText(img,
                  std::to_string(point_ids[i] % rem_id),
                  image_points[i] + text_offset,
                  cv::FONT_HERSHEY_COMPLEX,
                  0.5,
                  color);
    }
  }
}
/* -------------------------------------------------------------------------- */
// add squares in the image at desired position/size/color
void UtilsOpenCV::DrawSquaresInPlace(cv::Mat& img,
                                     const std::vector<cv::Point2f>& imagePoints,
                                     const cv::Scalar& color,
                                     const double msize,
                                     const std::vector<int>& pointIds,
                                     const int remId) {
  cv::Point2f textOffset = cv::Point2f(-10, -5);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    cv::Rect square = cv::Rect(imagePoints[i].x - msize / 2, imagePoints[i].y - msize / 2, msize, msize);
    rectangle(img, square, color, 2);
    if (pointIds.size() == imagePoints.size())  // we also have text
      cv::putText(
          img, std::to_string(pointIds[i] % remId), imagePoints[i] + textOffset, cv::FONT_HERSHEY_COMPLEX, 0.5, color);
  }
}
/* -------------------------------------------------------------------------- */
// add x in the image at desired position/size/color
void UtilsOpenCV::DrawCrossesInPlace(cv::Mat& img,
                                     const std::vector<cv::Point2f>& imagePoints,
                                     const cv::Scalar& color,
                                     const double msize,
                                     const std::vector<int>& pointIds,
                                     const int remId) {
  cv::Point2f textOffset = cv::Point2f(-10, -5);         // text offset
  cv::Point2f textOffsetToCenter = cv::Point2f(-3, +3);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    cv::putText(img, "X", imagePoints[i] + textOffsetToCenter, cv::FONT_HERSHEY_COMPLEX, msize, color, 2);
    if (pointIds.size() == imagePoints.size())  // we also have text
      cv::putText(
          img, std::to_string(pointIds[i] % remId), imagePoints[i] + textOffset, cv::FONT_HERSHEY_COMPLEX, 0.5, color);
  }
}
/* -------------------------------------------------------------------------- */
// add text (vector of doubles) in the image at desired position/size/color
void UtilsOpenCV::DrawTextInPlace(cv::Mat& img,
                                  const std::vector<cv::Point2f>& imagePoints,
                                  const cv::Scalar& color,
                                  const double msize,
                                  const std::vector<double>& textDoubles) {
  cv::Point2f textOffset = cv::Point2f(-12, -5);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    if (imagePoints.size() == textDoubles.size())  // write text
      cv::putText(img,
                  Utility::To_string_with_precision(textDoubles.at(i), 3),
                  imagePoints[i] + textOffset,
                  cv::FONT_HERSHEY_COMPLEX,
                  msize,
                  color);
  }
}

/* -------------------------------------------------------------------------- */
// Concatenate two images and return results as a new mat.
// Clones the two images.
cv::Mat UtilsOpenCV::concatenateTwoImages(const cv::Mat& left_img, const cv::Mat& right_img) {
  cv::Mat left_img_tmp = left_img.clone();
  if (left_img_tmp.channels() == 1) {
    cv::cvtColor(left_img_tmp, left_img_tmp, cv::COLOR_GRAY2BGR);
  }
  cv::Mat right_img_tmp = right_img.clone();
  if (right_img_tmp.channels() == 1) {
    cv::cvtColor(right_img_tmp, right_img_tmp, cv::COLOR_GRAY2BGR);
  }
  cv::Size left_img_size = left_img_tmp.size();
  cv::Size right_img_size = right_img_tmp.size();
  cv::Mat dual_img(left_img_size.height, left_img_size.width + right_img_size.width, CV_8UC3);
  cv::Mat left(dual_img, cv::Rect(0, 0, left_img_size.width, left_img_size.height));
  left_img_tmp.copyTo(left);
  cv::Mat right(dual_img, cv::Rect(left_img_size.width, 0, right_img_size.width, right_img_size.height));
  right_img_tmp.copyTo(right);
  return dual_img;
}

/* -------------------------------------------------------------------------- */
// Draw corner matches and return results as a new mat.
cv::Mat UtilsOpenCV::DrawCornersMatches(const cv::Mat& img1,
                                        const KeypointsCV& corners_1,
                                        const cv::Mat& img2,
                                        const KeypointsCV& corners_2,
                                        const DMatchVec& matches,
                                        const bool& random_color) {
  cv::Mat canvas = UtilsOpenCV::concatenateTwoImages(img1, img2);
  KeypointCV pt_offset(img1.cols, 0);
  cv::RNG rng(12345);
  for (const cv::DMatch& match : matches) {
    cv::Scalar color;
    if (random_color) {
      // Random color is useful to disambiguate between matches
      color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    } else {
      // Green color
      color = cv::Scalar(0, 255, 0);
    }

    // TODO TONI REUSE THE OTHER FUNCTIONS!!!!
    const KeypointCV& corner_1 = corners_1[match.queryIdx];
    const KeypointCV& corner_2 = corners_2[match.trainIdx];

    // Trace a line from the image on the left to the one image on the right
    cv::line(canvas, corner_1, corner_2 + pt_offset, color);
    cv::circle(canvas, corners_1[match.queryIdx], 3, color, 2);
    cv::circle(canvas, corners_2[match.trainIdx] + pt_offset, 3, color, 2);
  }

  return canvas;
}

/* -------------------------------------------------------------------------- */
cv::Mat UtilsOpenCV::DrawCircles(const cv::Mat img,
                                 const KeypointsCV& image_points,
                                 const std::vector<cv::Scalar>& circle_colors,
                                 const std::vector<double>& circle_sizes) {
  cv::Mat img_color = img.clone();
  if (img_color.channels() < 3u) {
    cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2BGR);
  }

  for (size_t i = 0u; i < image_points.size(); i++) {
    double circle_size = 3.0;
    cv::Scalar circle_color(0u, 0u, 255u);
    if (circle_sizes.size() == image_points.size()) {
      circle_size = 5.0 * std::max(circle_sizes[i], 0.5);
    }

    if (circle_colors.size() == image_points.size()) {
      circle_color = circle_colors[i];
    }

    cv::circle(img_color, image_points[i], circle_size, circle_color, 2u);
  }
  return img_color;
}

/* -------------------------------------------------------------------------- */
cv::Mat UtilsOpenCV::DrawCircles(const cv::Mat img,
                                 const std::vector<cv::KeyPoint>& image_points,
                                 const std::vector<cv::Scalar>& circle_colors,
                                 const std::vector<double>& circle_sizes) {
  cv::Mat img_color = img.clone();
  if (img_color.channels() < 3u) {
    cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2BGR);
  }

  for (size_t i = 0u; i < image_points.size(); i++) {
    double circle_size = 4.0;
    cv::Scalar circle_color(0u, 0u, 255u);
    if (circle_sizes.size() == image_points.size()) {
      circle_size = 5.0 * std::max(circle_sizes[i], 0.5);
    }

    if (circle_colors.size() == image_points.size()) {
      circle_color = circle_colors[i];
    }

    cv::circle(img_color, image_points[i].pt, circle_size, circle_color, 1u);
  }
  return img_color;
}
/* -------------------------------------------------------------------------- */
void UtilsOpenCV::DrawCornersMatchesOneByOne(const cv::Mat img1,
                                             const std::vector<cv::Point2f>& corners1,
                                             const cv::Mat img2,
                                             const std::vector<cv::Point2f>& corners2,
                                             const DMatchVec& matches) {
  cv::Mat canvas = UtilsOpenCV::concatenateTwoImages(img1, img2);
  cv::Point2f ptOffset = cv::Point2f(img1.cols, 0);

  for (int i = 0; i < matches.size(); i++) {
    cv::Mat baseCanvas = canvas.clone();
    printf("Match %d\n", i);
    cv::line(
        baseCanvas, corners1[matches[i].queryIdx], corners2[matches[i].trainIdx] + ptOffset, cv::Scalar(0, 255, 0));
    cv::imshow("Match one by one", baseCanvas);
    cv::waitKey(1);
  }
}

void UtilsOpenCV::showImagesSideBySide(const cv::Mat& img_left,
                                       const cv::Mat& img_right,
                                       const std::string& title,
                                       const bool& show_images,
                                       const bool& save_images,
                                       const std::string& filename) {
  cv::Mat original_left_right = UtilsOpenCV::concatenateTwoImages(img_left, img_right);

  if (show_images) {
    // Moved in here to allow saving images
    cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
    cv::imshow(title, original_left_right);
    cv::waitKey(1);
  }

  if (save_images) {
    cv::imwrite(filename, original_left_right);
  }
}

/* -------------------------------------------------------------------------- */
// Draw corner matches and return results as a new mat.
cv::Mat UtilsOpenCV::DrawCornersMatches(const cv::Mat& img1,
                                        const std::vector<cv::KeyPoint>& corners_1,
                                        const cv::Mat& img2,
                                        const KeypointsCV& corners_2,
                                        const bool& random_color) {
  cv::Mat canvas = UtilsOpenCV::concatenateTwoImages(img1, img2);
  KeypointCV pt_offset(img1.cols, 0);
  cv::RNG rng(12345);

  assert(corners_1.size() == corners_2.size());
  for (size_t i = 0u; i < corners_1.size(); ++i) {
    cv::Scalar color;
    if (random_color) {
      // Random color is useful to disambiguate between matches
      color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    } else {
      // Green color
      color = cv::Scalar(0, 255, 0);
    }

    // Trace a line from the image on the left to the one image on the right
    cv::line(canvas, corners_1[i].pt, corners_2[i] + pt_offset, color);
    cv::circle(canvas, corners_1[i].pt, 3, color, 2);
    cv::circle(canvas, corners_2[i] + pt_offset, 3, color, 2);
  }

  return canvas;
}

cv::Mat UtilsOpenCV::readRosImage(const sensor_msgs::ImageConstPtr& img_msg, bool grayscale) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  const cv::Mat img_const = cv_ptr->image;  // Don't modify shared image in ROS.
  cv::Mat converted_img(img_const.size(), CV_8U);
  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    if (grayscale) {
      cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    } else {
      converted_img = img_const;
    }
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    if (grayscale) {
      cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    } else {
      cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2BGR);
    }
    return converted_img;
  } else {
    ROS_ERROR_STREAM_COND(cv_ptr->encoding != sensor_msgs::image_encodings::MONO8,
                          "Expected image with MONO8, BGR8, or RGB8 encoding."
                          "Add in here more conversions if you wish.");
    return img_const;
  }
}
