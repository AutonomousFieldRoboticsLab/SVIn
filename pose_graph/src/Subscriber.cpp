#include "pose_graph/Subscriber.h"

Subscriber::Subscriber(ros::NodeHandle& nh, const Parameters& params) : params_(params) { setNodeHandle(nh); }

void Subscriber::setNodeHandle(ros::NodeHandle& nh) {
  nh_ = &nh;
  if (it_) delete it_;
  it_ = new image_transport::ImageTransport(*nh_);
}
