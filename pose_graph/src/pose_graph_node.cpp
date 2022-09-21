#include <future>

#include "pose_graph/Parameters.h"
#include "pose_graph/PoseGraphOptimization.h"
#include "pose_graph/Subscriber.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle nh("~");

  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = false;

  // read parameters
  Parameters params;
  params.loadParameters(nh);

  auto subscriber = std::make_shared<Subscriber>(nh, params);
  auto pose_graph = std::make_shared<PoseGraphOptimization>(params);

  subscriber->registerKeyframeCallback(
      std::bind(&PoseGraphOptimization::fillKeyframeTrackingQueue, pose_graph, std::placeholders::_1));
  auto process_thread = std::make_unique<std::thread>(&PoseGraphOptimization::run, pose_graph);

  ros::Time last_print_time = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();
    if (ros::Time::now() - last_print_time > ros::Duration(10.0)) {
      last_print_time = ros::Time::now();
      LOG(INFO) << utils::Statistics::Print();
    }
  }

  LOG(INFO) << "Shutting down threads...";
  pose_graph->shutdown();

  return EXIT_SUCCESS;
}
