#include <future>

#include "pose_graph/LoopClosure.h"
#include "pose_graph/Parameters.h"
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
  std::string config_file;
  nh.getParam("config_file", config_file);

  Parameters params;
  params.loadParameters(config_file);

  auto subscriber = std::make_shared<Subscriber>(nh, params);
  auto loop_closure = std::make_shared<LoopClosure>(params);

  subscriber->registerKeyframeCallback(
      std::bind(&LoopClosure::fillKeyframeTrackingQueue, loop_closure, std::placeholders::_1));
  auto process_thread = std::make_unique<std::thread>(&LoopClosure::run, loop_closure);

  ros::Time last_print_time = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();
    if (ros::Time::now() - last_print_time > ros::Duration(10.0)) {
      last_print_time = ros::Time::now();
      LOG(INFO) << utils::Statistics::Print();
    }
  }

  LOG(INFO) << "Shutting down threads...";
  loop_closure->shutdown();

  return EXIT_SUCCESS;
}
