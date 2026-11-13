#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace svin_rerun_bridge {

struct Rgba8 {
  std::uint8_t r{255};
  std::uint8_t g{255};
  std::uint8_t b{255};
  std::uint8_t a{255};
};

struct PointCloudSnapshot {
  std::vector<std::array<float, 3>> positions;
  std::vector<Rgba8> colors;
  std::string frame_id;

  [[nodiscard]] bool has_colors() const noexcept {
    return !colors.empty() && colors.size() == positions.size();
  }
};

struct PathSnapshot {
  std::vector<std::array<float, 3>> positions;
  std::string frame_id;
};

enum class MarkerLineType {
  kLineStrip,
  kLineList,
};

struct MarkerLine {
  std::int32_t id{0};
  std::string ns;
  MarkerLineType type{MarkerLineType::kLineStrip};
  std::vector<std::vector<std::array<float, 3>>> strips;
  Rgba8 color{};
  float radius{0.01F};
  std::string frame_id;
};

struct JpegImage {
  std::vector<std::uint8_t> bytes;
  std::uint32_t width{0};
  std::uint32_t height{0};
};

/// A complete set of keyframe images for one ROS header timestamp.
///
/// The image vector is ordered by camera index, not by callback arrival order.
struct KeyframeImageBundle {
  builtin_interfaces::msg::Time stamp;
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;
  nav_msgs::msg::Odometry::ConstSharedPtr pose;
};

struct KeyframeImageBundleAddResult {
  std::optional<KeyframeImageBundle> complete;
  bool replaced_duplicate{false};
  std::size_t dropped_pending{0};
};

/// Collect N camera image streams by exact (sec, nanosec) header timestamp.
///
/// This helper is deliberately independent of ROS executors and logging. The
/// owner supplies synchronization when calling add_image()/add_pose().
/// Incomplete bundles are bounded by max_pending_bundles; the oldest timestamp
/// is dropped first.
class KeyframeImageBundleAssembler final {
public:
  KeyframeImageBundleAssembler(
    std::size_t camera_count, std::size_t max_pending_bundles, bool require_pose = true);

  [[nodiscard]] KeyframeImageBundleAddResult add_image(
    std::size_t camera_index,
    const sensor_msgs::msg::Image::ConstSharedPtr & image);

  [[nodiscard]] KeyframeImageBundleAddResult add_pose(
    const nav_msgs::msg::Odometry::ConstSharedPtr & pose);

  [[nodiscard]] std::size_t camera_count() const noexcept { return camera_count_; }
  [[nodiscard]] std::size_t max_pending_bundles() const noexcept { return max_pending_bundles_; }
  [[nodiscard]] bool require_pose() const noexcept { return require_pose_; }
  [[nodiscard]] std::size_t pending_count() const noexcept { return pending_.size(); }

private:
  using StampKey = std::pair<std::int32_t, std::uint32_t>;

  struct PendingBundle {
    builtin_interfaces::msg::Time stamp;
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;
    nav_msgs::msg::Odometry::ConstSharedPtr pose;
    std::size_t received_count{0};
  };

  [[nodiscard]] static StampKey stamp_key(const builtin_interfaces::msg::Time & stamp) noexcept;
  [[nodiscard]] bool is_complete(const PendingBundle & pending) const noexcept;
  void finalize_or_trim(const StampKey & key, KeyframeImageBundleAddResult * result);

  std::size_t camera_count_;
  std::size_t max_pending_bundles_;
  bool require_pose_;
  std::map<StampKey, PendingBundle> pending_;
};

/// Resolve the configured camera image topics.
///
/// An explicit non-empty list wins. Otherwise count topics are generated from
/// pattern. The pattern accepts {index}, {}, or %d; if no placeholder is
/// present, the decimal camera index is appended.
[[nodiscard]] std::vector<std::string> keyframe_image_topics(
  const std::vector<std::string> & explicit_topics,
  std::size_t count,
  const std::string & pattern);

/// Convert a ROS timestamp to nanoseconds since the Unix epoch.
[[nodiscard]] std::int64_t stamp_to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept;

/// Keep the newest non-zero timestamp from a trusted bag-synchronised source.
/// Derived snapshots can carry a wall-clock stamp during bag replay, so image,
/// path, and keyframe timestamps are retained separately for those snapshots.
[[nodiscard]] std::optional<builtin_interfaces::msg::Time> update_latest_trusted_sensor_stamp(
  const std::optional<builtin_interfaces::msg::Time> & current,
  const builtin_interfaces::msg::Time & candidate);

/// Select the sensor timeline stamp for a derived PointCloud2 snapshot.
/// A trusted bag stamp takes precedence over the snapshot's own header stamp.
[[nodiscard]] builtin_interfaces::msg::Time select_point_cloud_sensor_stamp(
  const builtin_interfaces::msg::Time & message_stamp,
  const std::optional<builtin_interfaces::msg::Time> & trusted_stamp);

/// Clamp a JPEG quality parameter to the range accepted by OpenCV.
[[nodiscard]] int clamp_jpeg_quality(int quality) noexcept;

/// Normalize the supported moving-rig pose source name. Unknown values use
/// pose_graph, which is the source shared with the pose-graph frusta by
/// default.
[[nodiscard]] std::string normalize_camera_pose_source(const std::string & source);

/// Decode the common PointCloud2 XYZ/RGB layouts used by SVIn.
///
/// Invalid or non-finite XYZ samples are omitted. RGB may be a packed
/// float32/uint32 field named rgb/rgba, or separate r/g/b fields.
[[nodiscard]] std::optional<PointCloudSnapshot> point_cloud_snapshot(
  const sensor_msgs::msg::PointCloud2 & message,
  std::string * error = nullptr);

/// Extract the XYZ positions from a nav_msgs/Path.
[[nodiscard]] PathSnapshot path_snapshot(const nav_msgs::msg::Path & message);

/// Convert a LINE_STRIP or LINE_LIST Marker into one or more Rerun line
/// strips. Other marker types and delete actions return std::nullopt.
[[nodiscard]] std::optional<MarkerLine> marker_line(
  const visualization_msgs::msg::Marker & message,
  std::string * error = nullptr);

/// Convert a ROS Image to a JPEG byte payload. The conversion intentionally
/// accepts the camera encodings used by the current SVIn launch (including
/// mono8) and normalizes them through cv_bridge to BGR before encoding.
[[nodiscard]] std::optional<JpegImage> encode_jpeg(
  const sensor_msgs::msg::Image & message,
  int quality,
  std::string * error = nullptr);

/// Build a stable entity path for one visualization marker.
[[nodiscard]] std::string marker_entity_path(const MarkerLine & marker);

}  // namespace svin_rerun_bridge
