#include "pose_graph/PoseGraphOptimization.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle nh("~");

  // read parameters
  PoseGraphOptimization pose_graph_optimizer;

  std::thread processLC;

  processLC = std::thread(&PoseGraphOptimization::run, &pose_graph_optimizer);

  ros::spin();

  return 0;
}
