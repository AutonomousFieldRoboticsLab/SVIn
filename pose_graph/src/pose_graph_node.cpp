#include <glog/logging.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>

#include "pose_graph/LoopClosure.h"
#include "pose_graph/Parameters.h"
#include "pose_graph/Publisher.h"
#include "pose_graph/Subscriber.h"

void setupOutputLogDirectories(const std::string base_path) {
  std::string output_dir = base_path + "/loop_candidates/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/descriptor_matched/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/pnp_verified/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/loop_closure/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/geometric_verification/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  std::string loop_closure_file = base_path + "/loop_closure.txt";
  if (boost::filesystem::exists(loop_closure_file)) {
    boost::filesystem::remove(loop_closure_file);
  }
  std::ofstream loop_path_file(loop_closure_file, std::ios::out);
  loop_path_file << "cur_kf_id"
                 << " "
                 << "cur_kf_ts"
                 << " "
                 << "matched_kf_id"
                 << " "
                 << "matched_kf_ts"
                 << " "
                 << "relative_tx"
                 << " "
                 << "relative_ty"
                 << " "
                 << "relative_tz"
                 << " "
                 << "relative_yaw"
                 << " "
                 << "relative_pitch"
                 << " "
                 << "relative_roll" << std::endl;
  loop_path_file.close();

  std::string switch_info_file = base_path + "/switch_info.txt";
  if (boost::filesystem::exists(switch_info_file)) {
    boost::filesystem::remove(switch_info_file);
  }
  std::ofstream switch_info_file_stream(switch_info_file, std::ios::out);
  switch_info_file_stream << "type"
                          << " "
                          << "vio_stamp"
                          << " "
                          << "prim_stamp"
                          << " "
                          << "uber_stamp" << std::endl;
  switch_info_file_stream.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle nh("~");

  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  // FLAGS_log_prefix = true;

  // read parameters
  std::string config_file;
  nh.getParam("config_file", config_file);

  Parameters params;
  params.loadParameters(config_file);

  if (params.debug_mode_) {
    setupOutputLogDirectories(params.debug_output_path_);
  }

  auto subscriber = std::make_unique<Subscriber>(nh, params);
  auto loop_closure = std::make_unique<LoopClosure>(params);
  auto publisher = std::make_unique<Publisher>(nh, params.debug_mode_);

  loop_closure->setKeyframePoseCallback(
      std::bind(&Publisher::publishKeyframePath, publisher.get(), std::placeholders::_1, std::placeholders::_2));
  loop_closure->setLoopClosureCallback(
      std::bind(&Publisher::publishLoopClosurePath, publisher.get(), std::placeholders::_1, std::placeholders::_2));

  if (params.debug_mode_) {
    loop_closure->setPrimitivePublishCallback(
        std::bind(&Publisher::publishPrimitiveEstimator, publisher.get(), std::placeholders::_1 ));
  }

  subscriber->registerKeyframeCallback(
      std::bind(&LoopClosure::fillKeyframeTrackingQueue, loop_closure.get(), std::placeholders::_1));

  if (params.health_params_.enabled) {
    subscriber->registerPrimitiveEstimatorCallback(
        std::bind(&LoopClosure::fillPrimitiveEstimatorBuffer, loop_closure.get(), std::placeholders::_1));
  }

  ros::Timer timer;
  ros::ServiceServer pointcloud_service;

  if (params.global_mapping_params_.enabled) {
    publisher->setGlobalPointCloudFunction(
        std::bind(&LoopClosure::getGlobalMap, loop_closure.get(), std::placeholders::_1));
    subscriber->registerImageCallback(
        std::bind(&LoopClosure::fillImageQueue, loop_closure.get(), std::placeholders::_1));
    timer = nh.createTimer(ros::Duration(5), &Publisher::updatePublishGlobalMap, publisher.get());
    pointcloud_service = nh.advertiseService("save_pointcloud", &Publisher::savePointCloud, publisher.get());
  }

  auto process_thread = std::thread(&LoopClosure::run, loop_closure.get());

  ros::Time last_print_time = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();
    if (ros::Time::now() - last_print_time > ros::Duration(10.0)) {
      last_print_time = ros::Time::now();
      LOG(INFO) << utils::Statistics::Print();
    }
  }

  publisher->saveTrajectory(params.svin_w_loop_path_);
  LOG(INFO) << "Shutting down threads...";
  loop_closure->shutdown();

  return EXIT_SUCCESS;
}
