
#include <glog/logging.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>

#include "pose_graph/LoopClosure.h"
#include "pose_graph/Parameters.h"
#include "pose_graph/Publisher.h"
#include "pose_graph/Subscriber.h"

void setupOutputLogDirectories(const std::string base_path) {
  std::string output_dir = base_path + "/loop_candidates/";
  if (!std::filesystem::is_directory(output_dir) || !std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    std::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/descriptor_matched/";
  if (!std::filesystem::is_directory(output_dir) || !std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    std::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/pnp_verified/";
  if (!std::filesystem::is_directory(output_dir) || !std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    std::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/loop_closure/";
  if (!std::filesystem::is_directory(output_dir) || !std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    std::filesystem::remove_all(entry.path());
  }

  output_dir = base_path + "/geometric_verification/";
  if (!std::filesystem::is_directory(output_dir) || !std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
  }
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    std::filesystem::remove_all(entry.path());
  }

  std::string loop_closure_file = base_path + "/loop_closure.txt";
  if (std::filesystem::exists(loop_closure_file)) {
    std::filesystem::remove(loop_closure_file);
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
  if (std::filesystem::exists(switch_info_file)) {
    std::filesystem::remove(switch_info_file);
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
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("pose_graph_node");

  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // read parameters
  std::string config_file;

  node->declare_parameter<std::string>("config_file", "");
  node->get_parameter<std::string>("config_file", config_file);

  if (config_file.empty()) {
    LOG(ERROR) << "Config file not provided";
    return EXIT_FAILURE;
  }

  Parameters params;
  params.loadParameters(config_file);

  if (params.debug_mode_) {
    setupOutputLogDirectories(params.debug_output_path_);
    FLAGS_v = 0;
  }

  auto subscriber = std::make_unique<Subscriber>(node, params);
  auto loop_closure = std::make_unique<LoopClosure>(params);
  auto publisher = std::make_unique<Publisher>(node, params.debug_mode_);

  loop_closure->setKeyframePoseCallback(
      std::bind(&Publisher::publishKeyframePath, publisher.get(), std::placeholders::_1, std::placeholders::_2));
  loop_closure->setLoopClosureCallback(
      std::bind(&Publisher::publishLoopClosurePath, publisher.get(), std::placeholders::_1, std::placeholders::_2));

  if (params.debug_mode_) {
    loop_closure->setPrimitivePublishCallback(
        std::bind(&Publisher::publishPrimitiveEstimator, publisher.get(), std::placeholders::_1));
  }

  subscriber->registerKeyframeCallback(
      std::bind(&LoopClosure::fillKeyframeTrackingQueue, loop_closure.get(), std::placeholders::_1));

  if (params.health_params_.enabled) {
    subscriber->registerPrimitiveEstimatorCallback(
        std::bind(&LoopClosure::fillPrimitiveEstimatorBuffer, loop_closure.get(), std::placeholders::_1));
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pointcloud_service;

  if (params.global_mapping_params_.enabled) {
    publisher->setGlobalPointCloudFunction(
        std::bind(&LoopClosure::getGlobalMap, loop_closure.get(), std::placeholders::_1));
    subscriber->registerImageCallback(
        std::bind(&LoopClosure::fillImageQueue, loop_closure.get(), std::placeholders::_1));
    timer = node->create_wall_timer(std::chrono::seconds(5),
                                    std::bind(&Publisher::updatePublishGlobalMap, publisher.get()));
    pointcloud_service = node->create_service<std_srvs::srv::Trigger>("save_pointcloud",
                                                                      std::bind(&Publisher::savePointCloud,
                                                                                publisher.get(),
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2,
                                                                                std::placeholders::_3));
  }

  auto process_thread = std::thread(&LoopClosure::run, loop_closure.get());

  rclcpp::Time last_print_time = node->now();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok()) {
    executor.spin_once();
    if ((node->now() - last_print_time).seconds() > 10.0) {
      last_print_time = node->now();
      LOG(INFO) << utils::Statistics::Print();
    }
  }

  publisher->saveTrajectory(params.svin_w_loop_path_);
  LOG(INFO) << "Shutting down threads...";
  loop_closure->shutdown();

  return EXIT_SUCCESS;
}
