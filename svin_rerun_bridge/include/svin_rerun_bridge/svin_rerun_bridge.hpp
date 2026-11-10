#pragma once

#include "svin_rerun_bridge/compressed_image_bundle.hpp"
#include "svin_rerun_bridge/conversions.hpp"
#include "svin_rerun_bridge/pinhole.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace svin_rerun_bridge {

class SvinRerunBridge final : public rclcpp::Node {
public:
  explicit SvinRerunBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SvinRerunBridge() override;

private:
  void on_point_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
    const std::string & entity_path);
  void on_path(
    const nav_msgs::msg::Path::ConstSharedPtr & message,
    const std::string & entity_path,
    const Rgba8 & color,
    float radius);
  void on_image(
    std::size_t camera_index,
    const sensor_msgs::msg::Image::ConstSharedPtr & message,
    const std::string & topic);
  void on_pose(
    const nav_msgs::msg::Odometry::ConstSharedPtr & message,
    const std::string & topic);
  void on_odometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr & message,
    const std::string & topic);
  void on_image_bundle(const KeyframeImageBundle & bundle);
  void on_raw_image(
    std::size_t camera_index,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message,
    const std::string & topic);
  void on_raw_image_bundle(const CompressedImageBundle & bundle);
  void on_markers(const visualization_msgs::msg::MarkerArray::ConstSharedPtr & message);

  void log_static_pinhole_scene();
  void log_world_rig_pose(const geometry_msgs::msg::Pose & pose);
  void log_world_camera_poses(const geometry_msgs::msg::Pose & pose);
  void remember_trusted_sensor_stamp(const builtin_interfaces::msg::Time & stamp);

  void begin_update(const builtin_interfaces::msg::Time & sensor_stamp);
  void clear_entity(const std::string & entity_path, bool recursive = false);
  [[nodiscard]] bool should_log() const noexcept { return logging_enabled_; }

  std::unique_ptr<rerun::RecordingStream> recording_;
  std::mutex recording_mutex_;
  std::int64_t processing_step_{0};
  int jpeg_quality_{85};
  std::size_t keyframe_subscription_depth_{32U};
  float point_radius_ui_points_{2.0F};
  bool logging_enabled_{false};
  std::string camera_model_{"pinhole"};
  std::string camera_pose_source_{"pose_graph"};
  float pinhole_image_plane_distance_{0.35F};
  float pinhole_line_width_ui_points_{1.5F};

  std::string okvis_config_;
  std::string points_matched_topic_;
  std::string global_map_topic_;
  std::string okvis_path_topic_;
  std::string loop_closure_path_topic_;
  std::string odometry_topic_;
  std::string keyframe_pose_topic_;
  std::string visualization_topic_;
  std::vector<std::string> keyframe_image_topics_;
  std::vector<std::string> raw_compressed_image_topics_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_matched_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr okvis_path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr loop_closure_path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_pose_subscription_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
    keyframe_image_subscriptions_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>
    raw_compressed_image_subscriptions_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_subscription_;

  std::unique_ptr<KeyframeImageBundleAssembler> keyframe_image_assembler_;
  std::mutex keyframe_image_mutex_;
  std::unique_ptr<CompressedImageBundleAssembler> raw_image_assembler_;
  std::mutex raw_image_mutex_;
  std::size_t raw_sync_pending_capacity_{32U};
  std::optional<builtin_interfaces::msg::Time> latest_trusted_sensor_stamp_;
  std::vector<PinholeCamera> pinhole_cameras_;
};

}  // namespace svin_rerun_bridge
