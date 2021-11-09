#include "pose_graph/PoseGraphOptimization.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle nh("~");

  // read parameters
  PoseGraphOptimization pose_graph_optimizer;
  //   pose_graph_optimizer.setup();

  // Subscribers
  // ros::Subscriber subSVIN = nh.subscribe("/okvis_node/relocalization_odometry", 500, svinCallback);
  // ros::Subscriber subKF = nh.subscribe("/okvis_node/keyframe_imageL", 500, kfCallback);
  // ros::Subscriber subKFPose = nh.subscribe("/okvis_node/keyframe_pose", 500, kfPoseCallback);
  // ros::Subscriber subPCL = nh.subscribe("/okvis_node/keyframe_points", 500, pclCallback);

  // ros::Subscriber subSVINHealth;
  // if (params.use_health_) subSVINHealth = nh.subscribe("/okvis_node/svin_health", 500, healthCallback);

  // Publishers
  //   pubCamPoseVisual = nh.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
  //   pubKfOdom = nh.advertise<visualization_msgs::Marker>("key_odometrys", 1000);

  std::thread processLC;

  processLC = std::thread(&PoseGraphOptimization::run, &pose_graph_optimizer);

  ros::spin();

  return 0;
}
