#include "pose_graph/Publisher.h"

Publisher::Publisher() : nh_private_("~") {}

void Publisher::setParameters(const Parameters& params) { params_ = params; }

void Publisher::setPublishers() {
  // Publishers
  pub_matched_points_ = nh_private_.advertise<sensor_msgs::PointCloud>("match_points", 100);
  pub_gloal_map_ = nh_private_.advertise<sensor_msgs::PointCloud2>("global_map", 2);
}

void Publisher::kfMatchedPointCloudCallback(const sensor_msgs::PointCloud& msg) { pub_matched_points_.publish(msg); }

void Publisher::publishGlobalMap(const sensor_msgs::PointCloud2& cloud) { pub_gloal_map_.publish(cloud); }
