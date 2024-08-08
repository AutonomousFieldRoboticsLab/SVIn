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
 *  Created on: Apr 17, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Publisher.hpp
 * @brief Header file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_PUBLISHER_HPP_
#define INCLUDE_OKVIS_PUBLISHER_HPP_

#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.hpp>  // Sharmin
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
// #include <pcl/filters/statistical_outlier_removal.h> // Sharmin: Statisitcal outlier removal
#include <pcl/point_cloud.h>

#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/Time.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "okvis_ros/msg/svin_health.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class handles the publishing to either ROS topics or files.
 */
class Publisher {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**
   * @brief Constructor. Calls setNodeHandle().
   * @param nh The ROS node handle for publishing.
   */
  Publisher(std::shared_ptr<rclcpp::Node> node);  // NOLINT
  ~Publisher();
  /// \name Setters
  /// \{

  /// \brief Set an odometry output CSV file.
  /// \param csvFile The file
  bool setCsvFile(std::fstream& csvFile);
  /// \brief Set an odometry output CSV file.
  /// \param csvFileName The filename of a new file
  bool setCsvFile(std::string& csvFileName);  // NOLINT
  /// \brief Set an odometry output CSV file.
  /// \param csvFileName The filename of a new file
  bool setCsvFile(std::string csvFileName);

  /// \brief              Set a CVS file where the landmarks will be saved to.
  /// \param csvFile      The file
  bool setLandmarksCsvFile(std::fstream& csvFile);  // NOLINT
  /// \brief              Set a CVS file where the landmarks will be saved to.
  /// \param csvFileName  The filename of a new file
  bool setLandmarksCsvFile(std::string& csvFileName);  // NOLINT
  /// \brief              Set a CVS file where the landmarks will be saved to.
  /// \param csvFileName  The filename of a new file
  bool setLandmarksCsvFile(std::string csvFileName);  // NOLINT

  /**
   * @brief Set the pose message that is published next.
   * @param T_WS The pose.
   */
  void setPose(const okvis::kinematics::Transformation& T_WS);

  /**
   * @brief Set the odometry message that is published next.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and biases.
   * @param omega_S Rotational speed of Sensor frame (w.r.t. to inertial frame W)
   */
  void setOdometry(const okvis::kinematics::Transformation& T_WS,
                   const okvis::SpeedAndBiases& speedAndBiases,
                   const Eigen::Vector3d& omega_S);

  /// \brief Set the parameters
  /// @param parameters The parameters.
  void setParameters(const okvis::VioParameters& parameters) { parameters_ = parameters; }

  /// \brief Set just T_Wc_W
  /// @param parameters The parameters.
  void setT_Wc_W(const okvis::kinematics::Transformation& T_Wc_W) { parameters_.publishing.T_Wc_W = T_Wc_W; }

  /**
   * @brief Set the points that are published next.
   * @param pointsMatched Vector of 3D points that have been matched with existing landmarks.
   * @param pointsUnmatched Vector of 3D points that were not matched with existing landmarks.
   * @param pointsTransferred Vector of landmarks that have been marginalised out.
   */
  void setPoints(const okvis::MapPointVector& pointsMatched,
                 const okvis::MapPointVector& pointsUnmatched,
                 const okvis::MapPointVector& pointsTransferred);

  /// @brief Set the time for the next message to be published.
  void setTime(const okvis::Time& t) { _t = rclcpp::Time(t.sec, t.nsec); }

  /// @brief Set the images to be published next.
  void setImages(const std::vector<cv::Mat>& images);

  /// @brief Add a pose to the path that is published next. The path contains a maximum of
  ///        \e pathLength_ (change with setPathLength)poses that are published. Once the
  ///        maximum is reached, the last pose is copied in a new path message. The rest are deleted.
  void setPath(const okvis::kinematics::Transformation& T_WS);

  /// \}
  /// \name Publish
  /// \{

  /// \brief Publish the pose.
  void publishPose();
  /// \brief Publish the T_WS transform.
  void publishTransform();

  /// \brief Publish the static tf between camera i  and body frame
  void publishStaticTfCamera(size_t camera_index);
  /// \brief Publish the static tf between sonar and body frame
  void publishStaticTfSonar();

  /**
   * @brief publish static transform between parent and child frame
   * @param pose staticPose between parent and child frame
   * @param parent_frame_id parent frame id
   * @param child_frame_id child frame id
   * @remark enough to publish once at the beginning
   */
  void publishStaticTf(const geometry_msgs::msg::Transform& static_transform,
                       const std::string& parent_frame_id,
                       const std::string& child_frame_id);

  /**
   * @brief publish the static tf between different sensors
   */
  void publishSensorStaticTf();

  /**
   * @brief Set and publish pose.
   * @remark This can be registered with the VioInterface.
   * @param t     Timestamp of pose.
   * @param T_WS  The pose.
   */
  void publishStateAsCallback(const okvis::Time& t, const okvis::kinematics::Transformation& T_WS);

  /**
   * // Modified by SHarmin
   * @brief Set and publish full state.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp of state.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and IMU biases.
   * @param omega_S Rotational speed of the sensor frame.
   */
  void publishFullStateAsCallback(const okvis::Time& t,
                                  const okvis::kinematics::Transformation& T_WS,
                                  const Eigen::Matrix<double, 9, 1>& speedAndBiases,
                                  const Eigen::Matrix<double, 3, 1>& omega_S);

  /**
   * @brief Set and publish landmarks.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp.
   * @param actualLandmarks Landmarks.
   * @param transferredLandmarks Landmarks that were marginalised out.
   */
  void publishLandmarksAsCallback(const okvis::Time& t,
                                  const okvis::MapPointVector& actualLandmarks,
                                  const okvis::MapPointVector& transferredLandmarks);

  // Added by Sharmin
  // void publishSteroPointCloudAsCallback(const okvis::Time & t, const std::vector<Eigen::Vector3d> & stereoMatched);
  void publishKeyframeAsCallback(const okvis::Time& t,
                                 const cv::Mat& imageL,
                                 const okvis::kinematics::Transformation& T_WCa,
                                 std::vector<std::list<std::vector<double> > >& keyframePoints);  // NOLINT
  void publishRelocRelativePoseAsCallback(const okvis::Time& t,
                                          const Eigen::Vector3d& relative_t,
                                          const Eigen::Quaterniond& relative_q,
                                          const double& relative_yaw,
                                          const double& frame_index);

  /**
   * @brief Set and write full state to CSV file.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp of state.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and IMU biases.
   * @param omega_S Rotational speed of the sensor frame.
   */
  void csvSaveFullStateAsCallback(const okvis::Time& t,
                                  const okvis::kinematics::Transformation& T_WS,
                                  const Eigen::Matrix<double, 9, 1>& speedAndBiases,
                                  const Eigen::Matrix<double, 3, 1>& omega_S);

  /**
   * @brief Set and write full state including camera extrinsics to file.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp of state.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and IMU biases.
   * @param omega_S Rotation speed of the sensor frame.
   * @param extrinsics Camera extrinsics.
   */
  void csvSaveFullStateWithExtrinsicsAsCallback(
      const okvis::Time& t,
      const okvis::kinematics::Transformation& T_WS,
      const Eigen::Matrix<double, 9, 1>& speedAndBiases,
      const Eigen::Matrix<double, 3, 1>& omega_S,
      const std::vector<okvis::kinematics::Transformation,
                        Eigen::aligned_allocator<okvis::kinematics::Transformation> >& extrinsics);

  /**
   * @brief Set and write landmarks to file.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp.
   * @param actualLandmarks Landmarks.
   * @param transferredLandmarks Landmarks that were marginalised out.
   */
  void csvSaveLandmarksAsCallback(const okvis::Time& t,
                                  const okvis::MapPointVector& actualLandmarks,
                                  const okvis::MapPointVector& transferredLandmarks);

  /// @brief Publish the last set odometry.
  void publishOdometry();
  /// @brief Publish the last set points.
  void publishPoints();
  /// @brief Publish the last set images.
  void publishImages();
  /// @brief Publish the last set path.
  void publishPath();

  // Added by Sharmin
  void publishSteroPointCloud(std::vector<Eigen::Vector3d> stereoMatched);

  // Hunter
  void publishDebugImageAsCallback(const okvis::Time& t, int i, const cv::Mat& image);

  /// @}

 private:
  /// @brief Write CSV header.
  bool writeCsvDescription();
  /// @brief Write CSV header for landmarks file.
  bool writeLandmarksCsvDescription();

  /// @name Node and subscriber related
  /// @{

  std::shared_ptr<rclcpp::Node> node_;                                            ///< The node handle.
  tf2_ros::TransformBroadcaster pubTf_;                                           ///< The transform broadcaster.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPointsMatched_;  ///< The publisher for matched points.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubPointsUnmatched_;  ///< The publisher for unmatched points.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubPointsTransferred_;  ///< The publisher for transferred/marginalised points.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubObometry_;  ///< The publisher for the odometry.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;          ///< The publisher for the path.
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
      pubTransform_;                                                       ///< The publisher for the transform.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMesh_;  ///< The publisher for a robot / camera mesh.
  std::vector<image_transport::Publisher> pubImagesVector_;                ///< The publisher for the images.
  image_transport::ImageTransport image_transport_;                        ///< The image transporters.

  // ros::Publisher pubStereoMatched_; ///< Sharmin: The publisher for stereo matched points.
  // image_transport::ImageTransport imageTransportKeyframeImageL_; ///< Sharmin: The image transporter for keyframe
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      pubKeyframeImageL_;  ///< Sharmin: The publisher for Keyframe image left.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pubKeyframePose_;  ///< Sharmin: The publisher for Keyframe pose.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
      pubKeyframePoints_;  ///< Sharmin: The publisher for Keyframe 3d-2d points.

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      keyframePointsMatched_;  ///< Sharmin Point cloud for matched points in a keyframe.

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sorfilter;

  /// @}
  /// @name To be published
  /// @{

  rclcpp::Time _t;                                       ///< Header timestamp.
  geometry_msgs::msg::TransformStamped poseMsg_;         ///< Pose message.
  nav_msgs::msg::Odometry odometryMsg_;                  ///< Odometry message.
  okvis::MapPointVector pointsMatched2_;                 ///< Matched points vector.
  pcl::PointCloud<pcl::PointXYZRGB> pointsMatched_;      ///< Point cloud for matched points.
  pcl::PointCloud<pcl::PointXYZRGB> pointsUnmatched_;    ///< Point cloud for unmatched points.
  pcl::PointCloud<pcl::PointXYZRGB> pointsTransferred_;  ///< Point cloud for transferred/marginalised points.
  std::vector<cv::Mat> images_;                          ///< The images.
  nav_msgs::msg::Path path_;                             ///< The path message.
  visualization_msgs::msg::Marker meshMsg_;              ///< Mesh message.

  /// @}

  rclcpp::Time lastOdometryTime_;   ///< Timestamp of the last broadcasted transform. (publishPose())
  rclcpp::Time lastOdometryTime2_;  ///< Timestamp of the last published odometry message. (publishOdometry())
  rclcpp::Time lastTransfromTime_;  ///< Timestamp of the last published transform. (publishTransform())

  okvis::VioParameters parameters_;  ///< All the parameters including publishing options.

  uint32_t ctr2_;  ///< The counter for the amount of transferred points. Used for the seq parameter in the header.

  std::shared_ptr<std::fstream> csvFile_;           ///< CSV file to save state in.
  std::shared_ptr<std::fstream> csvLandmarksFile_;  ///< CSV file to save landmarks in.

  // FIXME Sharmin: This is an easy hack to use this publisher as an extern
  rclcpp::Publisher<okvis_ros::msg::SvinHealth>::SharedPtr pubSvinHealth;  // Sharmin: To publish SVIn2 health

  // Hunter
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubDebugImage_;

  bool static_tf_published_;  ///< Whether the static transform has been published.
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_PUBLISHER_HPP_ */
