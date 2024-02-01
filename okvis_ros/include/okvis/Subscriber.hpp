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
 *  Created on: Mar 23, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Subscriber.hpp
 * @brief Header file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_SUBSCRIBER_HPP_
#define INCLUDE_OKVIS_SUBSCRIBER_HPP_

/// @Sharmin
#include <boost/shared_ptr.hpp>
#include <deque>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <vector>
// @Sharmin
// #include <imagenex831l/ProcessedRange.h>
// #include <ros/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <depth_node_py/Depth.h> // for stereo rig depth
// #include <aquacore/StateMsg.h> // Aqua depth
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <okvis/Publisher.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <okvis/Time.hpp>
#include <okvis/VioInterface.hpp>
#include <okvis/VioParametersReader.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>  // for subscribing /pose_graph/match_points
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class handles all the buffering of incoming data.
 */
class Subscriber {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// TODO: @Sharmin: Should it be virtual?
  ~Subscriber();
  /**
   * @brief Constructor. This will either subscribe to the relevant ROS topics or
   *        start up the sensor and register the callbacks directly there.
   * @param nh The ROS node handle.
   * @param vioInterfacePtr Pointer to the VioInterface.
   * @param param_reader  Parameter reader.
   */
  Subscriber(std::shared_ptr<rclcpp::Node> node,  // NOLINT
             okvis::VioInterface* vioInterfacePtr,
             const okvis::VioParametersReader& param_reader);

  /// @brief Set custom world transformation for reloc callback @Hunter
  void setT_Wc_W(okvis::kinematics::Transformation T_Wc_W);

 protected:
  const cv::Mat readRosImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) const;

  /// @name ROS callbacks
  /// @{

  /// @brief The image callback.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, unsigned int cameraIndex);
  /// @brief The depth image callback.
  /// @warning Not implemented.
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr, unsigned int) {
    OKVIS_THROW(Exception, "Subscriber::depthImageCallback() is not implemented.");
  }

  /// @brief The IMU callback.
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// @brief The Depth callback. @Sharmin
  // void depthCallback(const depth_node_py::Depth::ConstPtr& msg);
  // void depthCallback(const aquacore::StateMsg::ConstPtr& msg); // Aqua Depth

  /// @brief The Relocalization callback. @Sharmin
  void relocCallback(const sensor_msgs::msg::PointCloud::SharedPtr points_msg);

  /// @brief The Sonar Range callback. @Sharmin
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  // void sonarCallback(const imagenex831l::msg::ProcessedRange::ConstPtr& msg);

  std::shared_ptr<rclcpp::Node> node_;                             ///< The node handle.
  std::unique_ptr<image_transport::ImageTransport> imgTransport_;  ///< The image transporter.
  std::vector<image_transport::Subscriber> imageSubscribers_;      ///< The image message subscriber.
  unsigned int imgLeftCounter;                                     // @Sharmin
  unsigned int imgRightCounter;                                    // @Sharmin
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;  ///< The IMU message subscriber.
  // ros::Subscriber subSonarRange_;                                ///< The Sonar Range Subscriber @Sharmin
  // ros::Subscriber subDepth_;                                     ///< The Depth Subscriber @Sharmin
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr
      subReloPoints_;        ///< The Relocalization Points Subscriber from pose_graph @Sharmin
  cv::Ptr<cv::CLAHE> clahe;  /// Sharmin
  /// @}

  okvis::VioInterface* vioInterface_;   ///< The VioInterface. (E.g. ThreadedKFVio)
  okvis::VioParameters vioParameters_;  ///< The parameters and settings.

  /// @Sharmin
  // std::mutex lastState_mutex_;            ///< Lock when accessing any of the 'lastOptimized*' variables.
  /// TODO: @Sharmin: Parameter
  /// TODO: Check this transformation, q(w,x,y,z)
  // const static okvis::kinematics::Transformation T_SSo(Eigen::Vector3d(0.365, 0.095, 0.070), Eigen::Quaterniond(0.0,
  // 0.707, 0.000, 0.707));
};
}  // namespace okvis

#endif /* INCLUDE_OKVIS_SUBSCRIBER_HPP_ */
