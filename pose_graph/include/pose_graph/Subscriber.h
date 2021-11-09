#pragma once

#include <image_transport/image_transport.h>

#include "pose_graph/Parameters.h"

class Subscriber {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Subscriber(ros::NodeHandle& nh, const Parameters& params);
  ~Subscriber() = default;

  void setNodeHandle(ros::NodeHandle& nh);

 private:
  std::mutex measurementMutex_;

  ros::NodeHandle* nh_;                  // The node handle of the node that will subscribe to the topic.
  image_transport::ImageTransport* it_;  // The image transport.

  Parameters params_;  // The parameters of the node.

  // List of  subscribers.
  image_transport::Subscriber sub_kf_image_;  // Subscriber to the keyframe image.
  ros::Subscriber sub_kf_pose_;               // Subscriber to the keyframe pose.
  ros::Subscriber sub_kf_points_;             // Subscriber to the keyframe points.
  ros::Subscriber sub_svin_relocalization_;   // Subscriber to the relocalization odometry.
  ros::Subscriber sub_svin_health_;           // Subscriber to the health of the SVIn.
};