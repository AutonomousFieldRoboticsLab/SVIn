#include "svin_rerun_bridge/svin_rerun_bridge.hpp"

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <iterator>
#include <utility>
#include <vector>

namespace svin_rerun_bridge {
namespace {

std::vector<rerun::Position3D> to_positions(
  const std::vector<std::array<float, 3>> & points)
{
  std::vector<rerun::Position3D> result;
  result.reserve(points.size());
  for (const auto & point : points) {
    result.emplace_back(point[0], point[1], point[2]);
  }
  return result;
}

rerun::Color to_color(const Rgba8 & color)
{
  return rerun::Color(color.r, color.g, color.b, color.a);
}

constexpr char kDefaultKeyframeTopicPattern[] = "/keyframe_image_{index}";

std::string normalize_camera_model(std::string camera_model)
{
  std::transform(
    camera_model.begin(), camera_model.end(), camera_model.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (camera_model == "ds") {
    return "double_sphere";
  }
  return camera_model;
}

std::string default_keyframe_topic(std::size_t index)
{
  return std::string("/keyframe_image_") + std::to_string(index);
}

using Matrix4 = std::array<double, 16>;

Matrix4 pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  double x = pose.orientation.x;
  double y = pose.orientation.y;
  double z = pose.orientation.z;
  double w = pose.orientation.w;
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (std::isfinite(norm) && norm > std::numeric_limits<double>::epsilon()) {
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  } else {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 1.0;
  }

  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
    pose.position.x,
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    pose.position.y,
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y),
    pose.position.z,
    0.0, 0.0, 0.0, 1.0};
}

std::array<float, 4> rotation_quaternion(const Matrix4 & transform)
{
  const double r00 = transform[0U];
  const double r01 = transform[1U];
  const double r02 = transform[2U];
  const double r10 = transform[4U];
  const double r11 = transform[5U];
  const double r12 = transform[6U];
  const double r20 = transform[8U];
  const double r21 = transform[9U];
  const double r22 = transform[10U];
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  const double trace = r00 + r11 + r22;
  if (trace > 0.0) {
    const double scale = 0.5 / std::sqrt(trace + 1.0);
    w = 0.25 / scale;
    x = (r21 - r12) * scale;
    y = (r02 - r20) * scale;
    z = (r10 - r01) * scale;
  } else if (r00 > r11 && r00 > r22) {
    const double scale = 2.0 * std::sqrt(std::max(0.0, 1.0 + r00 - r11 - r22));
    if (scale > 0.0) {
      w = (r21 - r12) / scale;
      x = 0.25 * scale;
      y = (r01 + r10) / scale;
      z = (r02 + r20) / scale;
    }
  } else if (r11 > r22) {
    const double scale = 2.0 * std::sqrt(std::max(0.0, 1.0 + r11 - r00 - r22));
    if (scale > 0.0) {
      w = (r02 - r20) / scale;
      x = (r01 + r10) / scale;
      y = 0.25 * scale;
      z = (r12 + r21) / scale;
    }
  } else {
    const double scale = 2.0 * std::sqrt(std::max(0.0, 1.0 + r22 - r00 - r11));
    if (scale > 0.0) {
      w = (r10 - r01) / scale;
      x = (r02 + r20) / scale;
      y = (r12 + r21) / scale;
      z = 0.25 * scale;
    }
  }
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!(norm > std::numeric_limits<double>::epsilon()) || !std::isfinite(norm)) {
    return {0.0F, 0.0F, 0.0F, 1.0F};
  }
  return {
    static_cast<float>(x / norm), static_cast<float>(y / norm),
    static_cast<float>(z / norm), static_cast<float>(w / norm)};
}

rerun::Transform3D transform_from_matrix(const Matrix4 & transform)
{
  const auto quaternion = rotation_quaternion(transform);
  const auto translation = rerun::components::Translation3D(
    static_cast<float>(transform[3U]), static_cast<float>(transform[7U]),
    static_cast<float>(transform[11U]));
  return rerun::Transform3D(
    translation,
    rerun::Rotation3D(rerun::encodings::Quaternion::from_xyzw(
      quaternion[0], quaternion[1], quaternion[2], quaternion[3])));
}

Matrix4 multiply_rigid(const Matrix4 & lhs, const Matrix4 & rhs)
{
  Matrix4 result{};
  for (std::size_t row = 0U; row < 4U; ++row) {
    for (std::size_t column = 0U; column < 4U; ++column) {
      double value = 0.0;
      for (std::size_t index = 0U; index < 4U; ++index) {
        value += lhs[row * 4U + index] * rhs[index * 4U + column];
      }
      result[row * 4U + column] = value;
    }
  }
  return result;
}

Matrix4 inverse_rigid(const Matrix4 & transform)
{
  Matrix4 result{};
  // The calibration stores a rigid row-major transform. Its inverse is the
  // transposed rotation followed by the negated, rotated translation.
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      result[row * 4U + column] = transform[column * 4U + row];
    }
  }
  for (std::size_t row = 0U; row < 3U; ++row) {
    result[row * 4U + 3U] = -(
      result[row * 4U + 0U] * transform[3U] +
      result[row * 4U + 1U] * transform[7U] +
      result[row * 4U + 2U] * transform[11U]);
  }
  result[15U] = 1.0;
  return result;
}

rerun::Mesh3D mesh_archetype(const DoubleSphereMesh & mesh)
{
  std::vector<rerun::Position3D> positions;
  positions.reserve(mesh.vertices.size());
  for (const auto & vertex : mesh.vertices) {
    positions.emplace_back(vertex[0], vertex[1], vertex[2]);
  }
  std::vector<rerun::TriangleIndices> triangles;
  triangles.reserve(mesh.triangles.size());
  for (const auto & triangle : mesh.triangles) {
    triangles.emplace_back(triangle[0], triangle[1], triangle[2]);
  }
  std::vector<rerun::components::Texcoord2D> texcoords;
  texcoords.reserve(mesh.texcoords.size());
  for (const auto & texcoord : mesh.texcoords) {
    texcoords.emplace_back(texcoord[0], texcoord[1]);
  }
  return rerun::Mesh3D(
    rerun::Collection<rerun::Position3D>::take_ownership(std::move(positions)))
    .with_triangle_indices(
      rerun::Collection<rerun::TriangleIndices>::take_ownership(std::move(triangles)))
    .with_vertex_texcoords(
      rerun::Collection<rerun::components::Texcoord2D>::take_ownership(std::move(texcoords)));
}

std::optional<rerun::MediaType> media_type_for_format(const std::string & format)
{
  std::string lower;
  lower.reserve(format.size());
  for (const auto character : format) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
  if (lower.find("jpeg") != std::string::npos || lower.find("jpg") != std::string::npos) {
    return rerun::MediaType::jpeg();
  }
  if (lower.find("png") != std::string::npos) {
    return rerun::MediaType::png();
  }
  return std::nullopt;
}

}  // namespace

SvinRerunBridge::SvinRerunBridge(const rclcpp::NodeOptions & options)
: Node("svin_rerun_bridge", options)
{
  declare_parameter<std::string>("application_id", "svin_rerun");
  declare_parameter<bool>("enable_live", true);
  declare_parameter<bool>("enable_recording", false);
  declare_parameter<std::string>("rrd_path", "");
  declare_parameter<std::string>("viewer_url", "rerun+http://127.0.0.1:9876/proxy");
  declare_parameter<int>("jpeg_quality", 75);
  declare_parameter<int>("keyframe_subscription_depth", 32);
  declare_parameter<std::string>("okvis_config", "");
  declare_parameter<std::string>("camera_model", "pinhole");
  declare_parameter<std::string>("camera_rendering.pose_source", "pose_graph");
  declare_parameter<bool>("camera_rendering.double_sphere.enable_mesh", true);
  declare_parameter<bool>("camera_rendering.double_sphere.enable_texture_updates", true);
  declare_parameter<int>("camera_rendering.double_sphere.texture_stride", 10);
  declare_parameter<double>("camera_rendering.double_sphere.depth", 0.35);
  declare_parameter<int>("camera_rendering.double_sphere.grid", 31);
  declare_parameter<int>("camera_rendering.double_sphere.texture_max_width", 128);
  declare_parameter<double>("camera_rendering.pinhole.image_plane_distance", 0.35);
  declare_parameter<double>("camera_rendering.pinhole.line_width_ui_points", 1.5);
  declare_parameter<double>("point_radius_ui_points", 1.5);
  declare_parameter<std::vector<std::string>>(
    "raw_compressed_image_topics", std::vector<std::string>{});
  declare_parameter<int>("raw_sync_pending_capacity", 32);

  declare_parameter<std::string>("points_matched_topic", "/okvis_points_matched");
  declare_parameter<std::string>("global_map_topic", "/global_map");
  declare_parameter<std::string>("okvis_path_topic", "/okvis_path");
  declare_parameter<std::string>("loop_closure_path_topic", "/loop_closure_path");
  declare_parameter<std::string>("odometry_topic", "/okvis_odometry");
  declare_parameter<std::string>("keyframe_pose_topic", "/keyframe_pose");
  declare_parameter<bool>("keyframe_pose_required", true);
  declare_parameter<std::vector<std::string>>("keyframe_image_topics", std::vector<std::string>{});
  declare_parameter<int>("keyframe_image_count", 2);
  declare_parameter<std::string>("keyframe_image_topic_pattern", kDefaultKeyframeTopicPattern);
  declare_parameter<int>("keyframe_sync_pending_capacity", 32);
  declare_parameter<bool>("enable_visualization_markers", true);
  declare_parameter<std::string>("visualization_topic", "/visualization");

  const auto application_id = get_parameter("application_id").as_string();
  const bool enable_live = get_parameter("enable_live").as_bool();
  const bool enable_recording = get_parameter("enable_recording").as_bool();
  const auto rrd_path = get_parameter("rrd_path").as_string();
  const auto viewer_url = get_parameter("viewer_url").as_string();
  jpeg_quality_ = clamp_jpeg_quality(get_parameter("jpeg_quality").as_int());
  const auto configured_keyframe_subscription_depth =
    get_parameter("keyframe_subscription_depth").as_int();
  keyframe_subscription_depth_ = static_cast<std::size_t>(
    std::max<std::int64_t>(configured_keyframe_subscription_depth, 1));
  okvis_config_ = get_parameter("okvis_config").as_string();
  camera_model_ = normalize_camera_model(get_parameter("camera_model").as_string());
  if (camera_model_ != "double_sphere" && camera_model_ != "pinhole") {
    RCLCPP_WARN(
      get_logger(), "Unsupported camera_model '%s'; using 'pinhole'",
      camera_model_.c_str());
    camera_model_ = "pinhole";
  }
  enable_double_sphere_mesh_ =
    get_parameter("camera_rendering.double_sphere.enable_mesh").as_bool();
  const auto configured_mesh_pose_source =
    get_parameter("camera_rendering.pose_source").as_string();
  mesh_pose_source_ = normalize_mesh_pose_source(configured_mesh_pose_source);
  if (mesh_pose_source_ != configured_mesh_pose_source) {
    RCLCPP_WARN(
      get_logger(), "Unsupported camera_rendering.pose_source '%s'; using '%s'",
      configured_mesh_pose_source.c_str(), mesh_pose_source_.c_str());
  }
  enable_mesh_texture_updates_ =
    get_parameter("camera_rendering.double_sphere.enable_texture_updates").as_bool();
  const auto configured_texture_stride =
    get_parameter("camera_rendering.double_sphere.texture_stride").as_int();
  mesh_texture_stride_ = static_cast<std::size_t>(
    std::max<std::int64_t>(configured_texture_stride, 1));
  const auto configured_depth =
    get_parameter("camera_rendering.double_sphere.depth").as_double();
  if (std::isfinite(configured_depth) && configured_depth > 0.0) {
    double_sphere_depth_ = static_cast<float>(configured_depth);
  } else {
    RCLCPP_WARN(
      get_logger(), "camera_rendering.double_sphere.depth must be finite and positive; using %.3f",
      double_sphere_depth_);
  }
  const auto configured_grid =
    get_parameter("camera_rendering.double_sphere.grid").as_int();
  double_sphere_grid_ = std::max<std::int64_t>(configured_grid, 2);
  const auto configured_pinhole_distance =
    get_parameter("camera_rendering.pinhole.image_plane_distance").as_double();
  if (std::isfinite(configured_pinhole_distance) && configured_pinhole_distance > 0.0) {
    pinhole_image_plane_distance_ = static_cast<float>(configured_pinhole_distance);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "camera_rendering.pinhole.image_plane_distance must be finite and positive; using %.3f",
      pinhole_image_plane_distance_);
  }
  const auto configured_pinhole_line_width =
    get_parameter("camera_rendering.pinhole.line_width_ui_points").as_double();
  if (std::isfinite(configured_pinhole_line_width) && configured_pinhole_line_width > 0.0) {
    pinhole_line_width_ui_points_ = static_cast<float>(configured_pinhole_line_width);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "camera_rendering.pinhole.line_width_ui_points must be finite and positive; using %.1f",
      pinhole_line_width_ui_points_);
  }
  texture_max_width_ = std::max<std::int64_t>(
    get_parameter("camera_rendering.double_sphere.texture_max_width").as_int(), 1);
  const auto configured_point_radius = get_parameter("point_radius_ui_points").as_double();
  if (std::isfinite(configured_point_radius) && configured_point_radius > 0.0) {
    point_radius_ui_points_ = static_cast<float>(configured_point_radius);
  } else {
    RCLCPP_WARN(
      get_logger(), "point_radius_ui_points must be finite and positive; using %.1f",
      point_radius_ui_points_);
  }
  const auto configured_raw_pending = get_parameter("raw_sync_pending_capacity").as_int();
  raw_sync_pending_capacity_ = static_cast<std::size_t>(
    std::max<std::int64_t>(configured_raw_pending, 1));

  points_matched_topic_ = get_parameter("points_matched_topic").as_string();
  global_map_topic_ = get_parameter("global_map_topic").as_string();
  okvis_path_topic_ = get_parameter("okvis_path_topic").as_string();
  loop_closure_path_topic_ = get_parameter("loop_closure_path_topic").as_string();
  odometry_topic_ = get_parameter("odometry_topic").as_string();
  keyframe_pose_topic_ = get_parameter("keyframe_pose_topic").as_string();
  const bool keyframe_pose_required = get_parameter("keyframe_pose_required").as_bool();
  const bool enable_visualization_markers =
    get_parameter("enable_visualization_markers").as_bool();
  visualization_topic_ = get_parameter("visualization_topic").as_string();
  raw_compressed_image_topics_ =
    get_parameter("raw_compressed_image_topics").as_string_array();

  const auto explicit_keyframe_topics =
    get_parameter("keyframe_image_topics").as_string_array();
  const auto configured_count = get_parameter("keyframe_image_count").as_int();
  const auto keyframe_count = static_cast<std::size_t>(std::max<std::int64_t>(configured_count, 1));
  const auto keyframe_topic_pattern =
    get_parameter("keyframe_image_topic_pattern").as_string();
  const auto configured_pending_capacity =
    get_parameter("keyframe_sync_pending_capacity").as_int();
  const auto keyframe_pending_capacity = static_cast<std::size_t>(
    std::max<std::int64_t>(configured_pending_capacity, 1));

  keyframe_image_topics_ = keyframe_image_topics(
    explicit_keyframe_topics, keyframe_count, keyframe_topic_pattern);
  for (std::size_t index = 0U; index < keyframe_image_topics_.size(); ++index) {
    if (keyframe_image_topics_[index].empty()) {
      RCLCPP_WARN(
        get_logger(), "keyframe_image_topics[%zu] is empty; using the default topic", index);
      keyframe_image_topics_[index] = default_keyframe_topic(index);
    }
  }
  keyframe_image_assembler_ = std::make_unique<KeyframeImageBundleAssembler>(
    keyframe_image_topics_.size(), keyframe_pending_capacity, keyframe_pose_required);

  if (!raw_compressed_image_topics_.empty()) {
    const auto first_empty = std::find(
      raw_compressed_image_topics_.begin(), raw_compressed_image_topics_.end(), std::string{});
    if (first_empty != raw_compressed_image_topics_.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "raw_compressed_image_topics contains an empty topic at index %zu; raw logging is disabled",
        static_cast<std::size_t>(std::distance(raw_compressed_image_topics_.begin(), first_empty)));
      raw_compressed_image_topics_.clear();
    } else {
      raw_image_assembler_ = std::make_unique<CompressedImageBundleAssembler>(
        raw_compressed_image_topics_.size(), raw_sync_pending_capacity_);
    }
  }

  recording_ = std::make_unique<rerun::RecordingStream>(application_id);

  // A non-empty output path is necessary but not sufficient: the explicit
  // enable_recording switch keeps accidental large recordings disabled.
  const bool use_file_sink = enable_recording && !rrd_path.empty();
  if (enable_recording && rrd_path.empty()) {
    RCLCPP_WARN(
      get_logger(), "enable_recording is true but rrd_path is empty; file recording is disabled");
  }

  rerun::Error sink_error = rerun::Error::ok();
  if (enable_live && use_file_sink) {
    sink_error = recording_->set_sinks(
      rerun::GrpcSink{viewer_url}, rerun::FileSink{rrd_path});
  } else if (enable_live) {
    sink_error = recording_->set_sinks(rerun::GrpcSink{viewer_url});
  } else if (use_file_sink) {
    sink_error = recording_->set_sinks(rerun::FileSink{rrd_path});
  }

  if (enable_live || use_file_sink) {
    if (sink_error.is_err()) {
      logging_enabled_ = false;
      RCLCPP_ERROR(
        get_logger(), "Failed to configure Rerun sink: %s", sink_error.description.c_str());
    } else {
      logging_enabled_ = true;
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Both Rerun sinks are disabled; the bridge will subscribe but will not buffer data");
  }

  if (logging_enabled_ && camera_model_ == "double_sphere" && enable_double_sphere_mesh_) {
    if (okvis_config_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Double-sphere mesh rendering is enabled but okvis_config is empty; camera meshes are disabled");
    } else {
      std::string error;
      const auto cameras = load_double_sphere_cameras(okvis_config_, &error);
      if (!cameras.has_value()) {
        RCLCPP_ERROR(
          get_logger(), "Cannot load double-sphere camera meshes from %s: %s",
          okvis_config_.c_str(), error.c_str());
      } else {
        double_sphere_cameras_ = *cameras;
        if (double_sphere_cameras_.size() != keyframe_image_topics_.size()) {
          RCLCPP_WARN(
            get_logger(),
            "Calibration contains %zu cameras but %zu keyframe image topics are configured",
            double_sphere_cameras_.size(), keyframe_image_topics_.size());
        }
        log_static_double_sphere_scene();
      }
    }
  }

  if (logging_enabled_ && camera_model_ == "pinhole") {
    if (okvis_config_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "camera_model is pinhole but okvis_config is empty; semantic cameras are disabled");
    } else {
      std::string error;
      const auto cameras = load_pinhole_cameras(okvis_config_, &error);
      if (!cameras.has_value()) {
        RCLCPP_ERROR(
          get_logger(), "Cannot load pinhole cameras from %s: %s",
          okvis_config_.c_str(), error.c_str());
      } else {
        pinhole_cameras_ = *cameras;
        if (pinhole_cameras_.size() != keyframe_image_topics_.size()) {
          RCLCPP_WARN(
            get_logger(),
            "Calibration contains %zu pinhole cameras but %zu keyframe image topics are configured",
            pinhole_cameras_.size(), keyframe_image_topics_.size());
        }
        log_static_pinhole_scene();
      }
    }
  }

  points_matched_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_matched_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
      on_point_cloud(message, "world/points/okvis_points_matched");
    });
  global_map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    global_map_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
      on_point_cloud(message, "world/points/global_map");
    });

  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  okvis_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
    okvis_path_topic_, state_qos,
    [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
      on_path(message, "world/paths/okvis", Rgba8{240U, 130U, 25U, 255U}, 0.015F);
    });
  loop_closure_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
    loop_closure_path_topic_, state_qos,
    [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
      on_path(message, "world/paths/loop_closure", Rgba8{25U, 255U, 0U, 255U}, 0.015F);
    });
  if (odometry_topic_.empty()) {
    if (mesh_pose_source_ == "odometry") {
      RCLCPP_WARN(
        get_logger(),
        "camera_rendering.pose_source is odometry but odometry_topic is empty; "
        "world/rig will not move");
    }
  } else {
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, state_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        on_odometry(message, odometry_topic_);
      });
  }
  auto keyframe_qos = rclcpp::QoS(rclcpp::KeepLast(keyframe_subscription_depth_));
  keyframe_qos.reliable();
  if (keyframe_pose_topic_.empty()) {
    if (keyframe_pose_required) {
      RCLCPP_ERROR(
        get_logger(), "keyframe_pose_required is true but keyframe_pose_topic is empty; "
        "no keyframe bundles can complete");
    } else {
      RCLCPP_WARN(get_logger(), "keyframe_pose_topic is empty; pose synchronization is disabled");
    }
  } else {
    keyframe_pose_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      keyframe_pose_topic_, keyframe_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        on_pose(message, keyframe_pose_topic_);
      });
  }
  if (enable_visualization_markers) {
    visualization_subscription_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, state_qos,
      [this](const visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
        on_markers(message);
      });
  }

  keyframe_image_subscriptions_.reserve(keyframe_image_topics_.size());
  for (std::size_t camera_index = 0U; camera_index < keyframe_image_topics_.size();
    ++camera_index)
  {
    const auto topic = keyframe_image_topics_[camera_index];
    keyframe_image_subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
      topic, keyframe_qos,
      [this, camera_index, topic](const sensor_msgs::msg::Image::ConstSharedPtr message) {
        on_image(camera_index, message, topic);
      }));
  }

  if (raw_image_assembler_ != nullptr) {
    raw_compressed_image_subscriptions_.reserve(raw_compressed_image_topics_.size());
    for (std::size_t camera_index = 0U; camera_index < raw_compressed_image_topics_.size();
      ++camera_index)
    {
      const auto topic = raw_compressed_image_topics_[camera_index];
      raw_compressed_image_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::CompressedImage>(
          topic, rclcpp::SensorDataQoS(),
          [this, camera_index, topic](
            const sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
            on_raw_image(camera_index, message, topic);
          }));
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Rerun bridge ready (live=%s, file=%s, JPEG quality=%d, keyframe cameras=%zu, "
    "pose=%s, pending capacity=%zu, raw cameras=%zu, raw pending capacity=%zu, "
    "keyframe QoS=reliable/%zu, mesh pose source=%s, odometry=%s, "
    "camera model=%s, double-sphere mesh=%s, pinhole=%s, visualization markers=%s, "
    "texture updates=%s/stride=%zu)",
    enable_live ? "enabled" : "disabled", use_file_sink ? rrd_path.c_str() : "disabled",
    jpeg_quality_, keyframe_image_topics_.size(), keyframe_pose_required ? "required" : "optional",
    keyframe_pending_capacity, raw_compressed_image_topics_.size(), raw_sync_pending_capacity_,
    keyframe_subscription_depth_,
    mesh_pose_source_.c_str(), odometry_topic_.empty() ? "disabled" : odometry_topic_.c_str(),
    camera_model_.c_str(),
    double_sphere_cameras_.empty() ? "disabled" : "enabled",
    pinhole_cameras_.empty() ? "disabled" : "enabled",
    enable_visualization_markers ? "enabled" : "disabled",
    enable_mesh_texture_updates_ ? "enabled" : "disabled", mesh_texture_stride_);
}

SvinRerunBridge::~SvinRerunBridge()
{
  if (recording_ != nullptr && logging_enabled_) {
    std::lock_guard<std::mutex> lock(recording_mutex_);
    recording_->flush_blocking(2.0F).handle();
  }
}

void SvinRerunBridge::begin_update(const builtin_interfaces::msg::Time & sensor_stamp)
{
  ++processing_step_;
  recording_->set_time_timestamp_nanos_since_epoch(
    "sensor_time", stamp_to_nanoseconds(sensor_stamp));
  recording_->set_time_sequence("processing_step", processing_step_);
}

void SvinRerunBridge::clear_entity(const std::string & entity_path, bool recursive)
{
  recording_->log(entity_path, recursive ? rerun::Clear::RECURSIVE : rerun::Clear::FLAT);
}

void SvinRerunBridge::log_static_double_sphere_scene()
{
  if (!should_log() || double_sphere_cameras_.empty()) {
    return;
  }

  // The bridge declares a single world convention for all entities. Camera
  // vertices remain in each camera's optical frame. Their moving world pose
  // is logged separately, so no static transform can shadow it.
  recording_->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  for (std::size_t camera_index = 0U; camera_index < double_sphere_cameras_.size();
    ++camera_index)
  {
    const auto & camera = double_sphere_cameras_[camera_index];
    const auto entity_path = "world/cameras/cam" + std::to_string(camera_index);

    const auto mesh = make_double_sphere_mesh(
      camera, double_sphere_depth_, double_sphere_grid_);
    if (mesh.vertices.empty() || mesh.triangles.empty() ||
      mesh.vertices.size() != mesh.texcoords.size())
    {
      RCLCPP_WARN(
        get_logger(), "Skipping empty or malformed double-sphere mesh for camera %zu",
        camera_index);
      continue;
    }
    recording_->log_static(entity_path, mesh_archetype(mesh));
  }
  RCLCPP_INFO(
    get_logger(),
    "Logged static double-sphere scene for %zu cameras (depth=%.3f m, grid=%d, "
    "world convention=RIGHT_HAND_Z_UP, root=%s)",
    double_sphere_cameras_.size(), double_sphere_depth_, double_sphere_grid_,
    mesh_pose_source_.c_str());
}

void SvinRerunBridge::log_static_pinhole_scene()
{
  if (!should_log() || pinhole_cameras_.empty()) {
    return;
  }

  recording_->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  for (std::size_t camera_index = 0U; camera_index < pinhole_cameras_.size(); ++camera_index) {
    const auto & camera = pinhole_cameras_[camera_index];
    // Rerun matrices are column-major. This is the calibrated K matrix:
    // [fx 0 cx; 0 fy cy; 0 0 1]. The default RDF convention matches a ROS
    // optical frame (+X right, +Y down, +Z forward).
    const auto projection = rerun::components::PinholeProjection(std::array<float, 9>{
      static_cast<float>(camera.fx), 0.0F, 0.0F,
      0.0F, static_cast<float>(camera.fy), 0.0F,
      static_cast<float>(camera.cx), static_cast<float>(camera.cy), 1.0F});
    const auto image_entity_path =
      "world/cameras/cam" + std::to_string(camera_index) + "/image";
    recording_->log_static(
      image_entity_path,
      rerun::Pinhole(projection)
        .with_resolution(camera.width, camera.height)
        .with_camera_xyz(rerun::components::ViewCoordinates::RDF)
        .with_image_plane_distance(pinhole_image_plane_distance_)
        .with_color(rerun::Color(80U, 200U, 255U))
        .with_line_width(rerun::Radius::ui_points(pinhole_line_width_ui_points_)));

    const bool has_nonzero_distortion = std::any_of(
      camera.distortion_coefficients.begin(), camera.distortion_coefficients.end(),
      [](double coefficient) {return std::abs(coefficient) > 1.0e-12;});
    if (has_nonzero_distortion) {
      RCLCPP_WARN(
        get_logger(),
        "Pinhole camera %zu uses %s distortion; Rerun's semantic Pinhole projects K only "
        "and displays the compressed image without rectification",
        camera_index, camera.distortion_type.c_str());
    }
  }
  RCLCPP_INFO(
    get_logger(), "Logged %zu semantic pinhole camera(s) (image plane distance=%.3f m)",
    pinhole_cameras_.size(), pinhole_image_plane_distance_);
}

void SvinRerunBridge::on_point_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
  const std::string & entity_path)
{
  if (!should_log()) {
    return;
  }
  std::lock_guard<std::mutex> lock(recording_mutex_);
  const auto sensor_stamp = select_point_cloud_sensor_stamp(
    message->header.stamp, latest_trusted_sensor_stamp_);
  begin_update(sensor_stamp);

  std::string error;
  const auto snapshot = point_cloud_snapshot(*message, &error);
  if (!snapshot.has_value()) {
    RCLCPP_WARN(get_logger(), "Skipping PointCloud2 on %s: %s", entity_path.c_str(), error.c_str());
    return;
  }

  // PointCloud2 is a full snapshot. Clearing before logging also handles a
  // later message with fewer points without retaining old component state.
  clear_entity(entity_path);
  auto positions = to_positions(snapshot->positions);
  if (snapshot->has_colors()) {
    std::vector<rerun::Color> colors;
    colors.reserve(snapshot->colors.size());
    for (const auto & color : snapshot->colors) {
      colors.push_back(to_color(color));
    }
    recording_->log(
      entity_path,
      rerun::Points3D(std::move(positions))
      .with_colors(std::move(colors))
      .with_radii(rerun::Radius::ui_points(point_radius_ui_points_)));
  } else if (!positions.empty()) {
    recording_->log(
      entity_path,
      rerun::Points3D(std::move(positions))
      .with_radii(rerun::Radius::ui_points(point_radius_ui_points_)));
  }
}

void SvinRerunBridge::on_path(
  const nav_msgs::msg::Path::ConstSharedPtr & message,
  const std::string & entity_path,
  const Rgba8 & color,
  float radius)
{
  if (!should_log()) {
    return;
  }
  std::lock_guard<std::mutex> lock(recording_mutex_);
  remember_trusted_sensor_stamp(message->header.stamp);
  begin_update(message->header.stamp);
  const auto snapshot = path_snapshot(*message);

  clear_entity(entity_path);
  if (mesh_pose_source_ == "pose_graph" && entity_path == "world/paths/loop_closure" &&
    !message->poses.empty())
  {
    // CameraPoseVisualization uses the last loop-closure pose as the current
    // frustum root. Keep this transform in the same processing update and
    // timestamp as the path snapshot so the two views cannot diverge.
    log_world_rig_pose(message->poses.back().pose);
  }
  if (snapshot.positions.size() < 2U) {
    return;
  }
  auto positions = to_positions(snapshot.positions);
  std::vector<rerun::LineStrip3D> strips;
  strips.emplace_back(std::move(positions));
  recording_->log(
    entity_path,
    rerun::LineStrips3D(std::move(strips))
      .with_colors(to_color(color))
      .with_radii(rerun::Radius(radius)));
}

void SvinRerunBridge::on_image(
  std::size_t camera_index,
  const sensor_msgs::msg::Image::ConstSharedPtr & message,
  const std::string & topic)
{
  if (!should_log() || message == nullptr) {
    return;
  }

  KeyframeImageBundleAddResult result;
  {
    std::lock_guard<std::mutex> lock(keyframe_image_mutex_);
    result = keyframe_image_assembler_->add_image(camera_index, message);
  }

  if (result.replaced_duplicate) {
    RCLCPP_WARN(
      get_logger(),
      "Replacing duplicate keyframe image for camera %zu at stamp %d.%u on %s",
      camera_index, message->header.stamp.sec, message->header.stamp.nanosec, topic.c_str());
  }
  if (result.dropped_pending != 0U) {
    RCLCPP_WARN(
      get_logger(),
      "Dropped %zu incomplete keyframe image bundle(s); pending capacity is %zu",
      result.dropped_pending, keyframe_image_assembler_->max_pending_bundles());
  }
  if (result.complete.has_value()) {
    on_image_bundle(*result.complete);
  }
}

void SvinRerunBridge::on_pose(
  const nav_msgs::msg::Odometry::ConstSharedPtr & message,
  const std::string & topic)
{
  if (!should_log() || message == nullptr) {
    return;
  }

  KeyframeImageBundleAddResult result;
  {
    std::lock_guard<std::mutex> lock(keyframe_image_mutex_);
    result = keyframe_image_assembler_->add_pose(message);
  }

  if (result.replaced_duplicate) {
    RCLCPP_WARN(
      get_logger(),
      "Replacing duplicate keyframe pose at stamp %d.%u on %s",
      message->header.stamp.sec, message->header.stamp.nanosec, topic.c_str());
  }
  if (result.dropped_pending != 0U) {
    RCLCPP_WARN(
      get_logger(),
      "Dropped %zu incomplete keyframe image/pose bundle(s); pending capacity is %zu",
      result.dropped_pending, keyframe_image_assembler_->max_pending_bundles());
  }
  if (result.complete.has_value()) {
    on_image_bundle(*result.complete);
  }
}

void SvinRerunBridge::on_odometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr & message,
  const std::string & topic)
{
  if (!should_log() || message == nullptr || mesh_pose_source_ != "odometry") {
    return;
  }

  // Live odometry is selected explicitly as the moving transform source. It
  // does not compete with path-graph or synchronized keyframe updates.
  std::lock_guard<std::mutex> lock(recording_mutex_);
  remember_trusted_sensor_stamp(message->header.stamp);
  begin_update(message->header.stamp);
  log_world_rig_pose(message->pose.pose);
  (void)topic;
}

void SvinRerunBridge::log_world_rig_pose(const geometry_msgs::msg::Pose & pose)
{
  clear_entity("world/rig");
  const auto translation = rerun::components::Translation3D(
    static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
    static_cast<float>(pose.position.z));
  const auto quaternion = rerun::encodings::Quaternion::from_xyzw(
    static_cast<float>(pose.orientation.x), static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z), static_cast<float>(pose.orientation.w));
  recording_->log(
    "world/rig", rerun::Transform3D(translation, rerun::Rotation3D(quaternion)));
  log_world_camera_poses(pose);
}

void SvinRerunBridge::log_world_camera_poses(const geometry_msgs::msg::Pose & pose)
{
  if (double_sphere_cameras_.empty() && pinhole_cameras_.empty()) {
    return;
  }

  const auto world_from_rig = pose_to_matrix(pose);
  const bool root_is_odometry = mesh_pose_source_ == "odometry";
  const auto log_camera_transforms = [this, &world_from_rig, root_is_odometry](
      const auto & cameras) {
    if (cameras.empty()) {
      return;
    }
    const auto inverse_camera0 = root_is_odometry ? Matrix4{} :
      inverse_rigid(cameras.front().T_SC);
    for (std::size_t camera_index = 0U; camera_index < cameras.size(); ++camera_index) {
      const auto & camera = cameras[camera_index];
    // Pose-graph/keyframe roots are C0:
    //   T_WCi = T_WC0 * inv(T_SC0) * T_SCi.
    // Odometry roots are the sensor/body frame (T_BS=I for this dataset):
    //   T_WCi = T_WR * T_SCi.
    const auto camera_from_rig = root_is_odometry ? camera.T_SC :
      multiply_rigid(inverse_camera0, camera.T_SC);
    const auto world_from_camera = multiply_rigid(world_from_rig, camera_from_rig);
    recording_->log(
      "world/cameras/cam" + std::to_string(camera_index),
      transform_from_matrix(world_from_camera));
    }
  };
  log_camera_transforms(double_sphere_cameras_);
  log_camera_transforms(pinhole_cameras_);
}

void SvinRerunBridge::remember_trusted_sensor_stamp(
  const builtin_interfaces::msg::Time & stamp)
{
  latest_trusted_sensor_stamp_ = update_latest_trusted_sensor_stamp(
    latest_trusted_sensor_stamp_, stamp);
}

void SvinRerunBridge::on_image_bundle(const KeyframeImageBundle & bundle)
{
  if (!should_log()) {
    return;
  }
  // A complete N-camera bundle is one state update: all image entities share
  // one lock and one processing_step, regardless of callback arrival order.
  std::lock_guard<std::mutex> lock(recording_mutex_);
  remember_trusted_sensor_stamp(bundle.stamp);
  begin_update(bundle.stamp);

  if (mesh_pose_source_ == "keyframe" && bundle.pose != nullptr) {
    log_world_rig_pose(bundle.pose->pose.pose);
  }

  const bool raw_images_configured = !raw_compressed_image_topics_.empty();
  bool update_mesh_textures = false;
  if (!raw_images_configured && enable_double_sphere_mesh_ &&
    enable_mesh_texture_updates_ && !double_sphere_cameras_.empty())
  {
    update_mesh_textures = mesh_texture_update_due(
      ++keyframe_texture_bundle_count_, mesh_texture_stride_);
  }

  for (std::size_t camera_index = 0U; camera_index < bundle.images.size(); ++camera_index) {
    const auto & image_message = bundle.images[camera_index];
    if (image_message == nullptr) {
      RCLCPP_WARN(get_logger(), "Skipping null image in complete keyframe bundle at camera %zu", camera_index);
      continue;
    }

    const auto image_entity_path = "images/keyframe_" + std::to_string(camera_index);
    clear_entity(image_entity_path);

    std::string error;
    const auto image = encode_jpeg(*image_message, jpeg_quality_, &error);
    if (image.has_value()) {
      recording_->log(
        image_entity_path,
        rerun::EncodedImage::from_bytes(std::move(image->bytes), rerun::MediaType::jpeg()));
    } else {
      RCLCPP_WARN(
        get_logger(), "Skipping JPEG image on %s: %s", image_entity_path.c_str(), error.c_str());
    }

    // Keep the static Mesh3D geometry/UV/triangles and replace only its
    // albedo texture fields. Mesh3D::update_fields is essential here: logging
    // a fresh mesh would make every keyframe resend the static geometry.
    if (!update_mesh_textures || camera_index >= double_sphere_cameras_.size()) {
      continue;
    }
    error.clear();
    auto texture = to_rgb8(*image_message, texture_max_width_, &error);
    if (!texture.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Skipping mesh texture for camera %zu: %s", camera_index, error.c_str());
      continue;
    }
    log_mesh_texture(camera_index, std::move(*texture));
  }
}

void SvinRerunBridge::log_mesh_texture(std::size_t camera_index, RgbImage texture)
{
  if (camera_index >= double_sphere_cameras_.size() || texture.width == 0U ||
    texture.height == 0U)
  {
    return;
  }
  auto texture_buffer = rerun::components::ImageBuffer(
    rerun::Collection<std::uint8_t>::take_ownership(std::move(texture.bytes)));
  const auto texture_format = rerun::components::ImageFormat(
    rerun::WidthHeight{texture.width, texture.height},
    rerun::ColorModel::RGB, rerun::ChannelDatatype::U8);
  const auto mesh_entity_path = "world/cameras/cam" + std::to_string(camera_index);
  recording_->log(
    mesh_entity_path,
    rerun::Mesh3D::update_fields()
      .with_albedo_texture_buffer(texture_buffer)
      .with_albedo_texture_format(texture_format));
}

void SvinRerunBridge::on_raw_image(
  std::size_t camera_index,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message,
  const std::string & topic)
{
  if (!should_log() || message == nullptr || raw_image_assembler_ == nullptr) {
    return;
  }

  CompressedImageBundleAddResult result;
  {
    std::lock_guard<std::mutex> lock(raw_image_mutex_);
    result = raw_image_assembler_->add(camera_index, message);
  }
  if (result.replaced_duplicate) {
    RCLCPP_WARN(
      get_logger(),
      "Replacing duplicate raw compressed image for camera %zu at stamp %d.%u on %s",
      camera_index, message->header.stamp.sec, message->header.stamp.nanosec, topic.c_str());
  }
  if (result.dropped_pending != 0U) {
    RCLCPP_WARN(
      get_logger(),
      "Dropped %zu incomplete raw compressed image bundle(s); pending capacity is %zu",
      result.dropped_pending, raw_image_assembler_->max_pending_bundles());
  }
  if (result.complete.has_value()) {
    on_raw_image_bundle(*result.complete);
  }
}

void SvinRerunBridge::on_raw_image_bundle(const CompressedImageBundle & bundle)
{
  if (!should_log()) {
    return;
  }
  // All raw camera payloads from one exact timestamp share one processing step.
  // The payload is copied without decoding or re-encoding so the original
  // compressed bytes remain available for downstream inspection.
  std::lock_guard<std::mutex> lock(recording_mutex_);
  remember_trusted_sensor_stamp(bundle.stamp);
  begin_update(bundle.stamp);
  const bool update_mesh_textures = enable_double_sphere_mesh_ &&
    enable_mesh_texture_updates_ && !double_sphere_cameras_.empty() &&
    mesh_texture_update_due(++raw_texture_bundle_count_, mesh_texture_stride_);
  for (std::size_t camera_index = 0U; camera_index < bundle.images.size(); ++camera_index) {
    const auto & image = bundle.images[camera_index];
    if (image == nullptr) {
      continue;
    }
    std::vector<std::uint8_t> bytes(image->data.begin(), image->data.end());
    const auto image_entity_path = camera_index < pinhole_cameras_.size() ?
      "world/cameras/cam" + std::to_string(camera_index) + "/image" :
      "images/raw/cam" + std::to_string(camera_index);
    recording_->log(
      image_entity_path,
      rerun::EncodedImage::from_bytes(
        rerun::Collection<std::uint8_t>::take_ownership(std::move(bytes)),
        media_type_for_format(image->format)));

    if (!update_mesh_textures || camera_index >= double_sphere_cameras_.size()) {
      continue;
    }
    std::string error;
    const auto texture = decode_compressed_rgb8(*image, texture_max_width_, &error);
    if (!texture.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Skipping raw mesh texture for camera %zu: %s",
        camera_index, error.c_str());
      continue;
    }
    log_mesh_texture(camera_index, std::move(*texture));
  }
}

void SvinRerunBridge::on_markers(
  const visualization_msgs::msg::MarkerArray::ConstSharedPtr & message)
{
  if (!should_log()) {
    return;
  }
  std::lock_guard<std::mutex> lock(recording_mutex_);
  builtin_interfaces::msg::Time stamp;
  if (!message->markers.empty()) {
    stamp = message->markers.front().header.stamp;
  }
  begin_update(stamp);

  // MarkerArray is treated as a full snapshot. A recursive clear makes a
  // smaller replacement array unable to leave old namespace/id entities.
  clear_entity("world/visualization", true);
  for (const auto & message_marker : message->markers) {
    std::string error;
    const auto marker = marker_line(message_marker, &error);
    if (!marker.has_value()) {
      if (!error.empty()) {
        RCLCPP_WARN(
          get_logger(), "Skipping marker %d/%s: %s", message_marker.id,
          message_marker.ns.c_str(), error.c_str());
      }
      continue;
    }

    std::vector<rerun::LineStrip3D> strips;
    strips.reserve(marker->strips.size());
    for (const auto & strip : marker->strips) {
      strips.emplace_back(to_positions(strip));
    }
    auto lines = rerun::LineStrips3D(std::move(strips))
      .with_colors(to_color(marker->color))
      .with_radii(rerun::Radius(marker->radius));
    recording_->log(marker_entity_path(*marker), std::move(lines));
  }
}

}  // namespace svin_rerun_bridge
