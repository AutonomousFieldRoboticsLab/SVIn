#include <boost/thread.hpp>

#include "pose_graph/Parameters.h"
#include "pose_graph/PoseGraphOptimization.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle nh("~");

  Parameters params;
  params.loadParameters(nh);

  // read parameters
  PoseGraphOptimization pose_graph_optimizer;

  // std::shared_ptr<boost::thread> process_thread(
  // new boost::thread(boost::bind(&PoseGraphOptimization::run, &pose_graph_optimizer)));

  ros::spin();

  // process_thread->join();

  return 0;
}
