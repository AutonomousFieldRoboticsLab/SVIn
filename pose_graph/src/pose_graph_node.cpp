#include <boost/thread.hpp>

#include "pose_graph/Parameters.h"
#include "pose_graph/PoseGraphOptimization.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle nh("~");

  // read parameters
  Parameters params;
  params.loadParameters(nh);

  Subscriber subscriber(nh, params);

  PoseGraphOptimization pose_graph_optimizer(params);

  // std::shared_ptr<boost::thread> process_thread(
  // new boost::thread(boost::bind(&PoseGraphOptimization::run, &pose_graph_optimizer)));

  ros::spin();

  // process_thread->join();

  return 0;
}
