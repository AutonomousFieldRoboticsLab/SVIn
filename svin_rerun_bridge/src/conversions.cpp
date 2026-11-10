#include "svin_rerun_bridge/conversions.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace svin_rerun_bridge {
namespace {

using PointField = sensor_msgs::msg::PointField;

void set_error(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

const PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  const auto it = std::find_if(
    message.fields.begin(), message.fields.end(),
    [&name](const PointField & field) { return field.name == name; });
  return it == message.fields.end() ? nullptr : &(*it);
}

std::size_t datatype_size(std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

bool fits(
  const sensor_msgs::msg::PointCloud2 & message,
  std::size_t offset,
  std::size_t size)
{
  return offset <= message.data.size() && size <= message.data.size() - offset;
}

std::uint64_t read_unsigned(
  const sensor_msgs::msg::PointCloud2 & message,
  std::size_t offset,
  std::size_t size)
{
  std::uint64_t value = 0;
  if (message.is_bigendian) {
    for (std::size_t i = 0; i < size; ++i) {
      value = (value << 8U) | message.data[offset + i];
    }
  } else {
    for (std::size_t i = 0; i < size; ++i) {
      value |= static_cast<std::uint64_t>(message.data[offset + i]) << (8U * i);
    }
  }
  return value;
}

std::int64_t sign_extend(std::uint64_t value, std::size_t size)
{
  const auto bits = size * 8U;
  if (bits == 0U || bits >= 64U || ((value >> (bits - 1U)) & 1U) == 0U) {
    return static_cast<std::int64_t>(value);
  }
  const std::uint64_t mask = ~((std::uint64_t{1} << bits) - 1U);
  return static_cast<std::int64_t>(value | mask);
}

std::optional<double> read_scalar(
  const sensor_msgs::msg::PointCloud2 & message,
  const PointField & field,
  std::size_t base)
{
  const auto size = datatype_size(field.datatype);
  const auto offset = base + field.offset;
  if (size == 0U || !fits(message, offset, size)) {
    return std::nullopt;
  }

  const auto raw = read_unsigned(message, offset, size);
  switch (field.datatype) {
    case PointField::INT8:
    case PointField::INT16:
    case PointField::INT32:
      return static_cast<double>(sign_extend(raw, size));
    case PointField::UINT8:
    case PointField::UINT16:
    case PointField::UINT32:
      return static_cast<double>(raw);
    case PointField::FLOAT32: {
      const auto bits = static_cast<std::uint32_t>(raw);
      float value = 0.0F;
      std::memcpy(&value, &bits, sizeof(value));
      return static_cast<double>(value);
    }
    case PointField::FLOAT64: {
      const auto bits = raw;
      double value = 0.0;
      std::memcpy(&value, &bits, sizeof(value));
      return value;
    }
    default:
      return std::nullopt;
  }
}

std::optional<std::uint32_t> read_packed_color(
  const sensor_msgs::msg::PointCloud2 & message,
  const PointField & field,
  std::size_t base)
{
  if (field.datatype != PointField::FLOAT32 &&
    field.datatype != PointField::UINT32 &&
    field.datatype != PointField::INT32)
  {
    return std::nullopt;
  }
  const auto offset = base + field.offset;
  if (!fits(message, offset, sizeof(std::uint32_t))) {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(read_unsigned(message, offset, sizeof(std::uint32_t)));
}

std::uint8_t component_to_u8(double value, bool normalized)
{
  if (!std::isfinite(value)) {
    return 0U;
  }
  if (normalized) {
    value *= 255.0;
  }
  return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 255.0)));
}

std::uint8_t marker_channel(float value)
{
  if (!std::isfinite(value)) {
    return 0U;
  }
  return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0F, 1.0F) * 255.0F));
}

bool is_finite_point(const std::array<float, 3> & point)
{
  return std::all_of(point.begin(), point.end(), [](float value) { return std::isfinite(value); });
}

std::string safe_path_component(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const auto character : value) {
    const auto is_alphanumeric =
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9');
    result.push_back(is_alphanumeric || character == '_' || character == '-' || character == '.' ?
      character : '_');
  }
  return result.empty() ? "default" : result;
}

}  // namespace

std::int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(stamp.nanosec);
}

std::optional<builtin_interfaces::msg::Time> update_latest_trusted_sensor_stamp(
  const std::optional<builtin_interfaces::msg::Time> & current,
  const builtin_interfaces::msg::Time & candidate)
{
  if (candidate.sec == 0 && candidate.nanosec == 0U) {
    return current;
  }
  if (!current.has_value() || candidate.sec > current->sec ||
    (candidate.sec == current->sec && candidate.nanosec >= current->nanosec))
  {
    return candidate;
  }
  return current;
}

builtin_interfaces::msg::Time select_point_cloud_sensor_stamp(
  const builtin_interfaces::msg::Time & message_stamp,
  const std::optional<builtin_interfaces::msg::Time> & trusted_stamp)
{
  return trusted_stamp.has_value() ? *trusted_stamp : message_stamp;
}

KeyframeImageBundleAssembler::KeyframeImageBundleAssembler(
  std::size_t camera_count, std::size_t max_pending_bundles, bool require_pose)
: camera_count_(camera_count),
  max_pending_bundles_(std::max<std::size_t>(max_pending_bundles, 1U)),
  require_pose_(require_pose)
{
  if (camera_count_ == 0U) {
    throw std::invalid_argument("keyframe image bundle requires at least one camera");
  }
}

KeyframeImageBundleAssembler::StampKey KeyframeImageBundleAssembler::stamp_key(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  return {stamp.sec, stamp.nanosec};
}

bool KeyframeImageBundleAssembler::is_complete(const PendingBundle & pending) const noexcept
{
  return pending.received_count == camera_count_ && (!require_pose_ || pending.pose != nullptr);
}

void KeyframeImageBundleAssembler::finalize_or_trim(
  const StampKey & key, KeyframeImageBundleAddResult * result)
{
  auto it = pending_.find(key);
  if (it == pending_.end()) {
    return;
  }

  if (is_complete(it->second)) {
    result->complete = KeyframeImageBundle{
      it->second.stamp, std::move(it->second.images), std::move(it->second.pose)};
    pending_.erase(it);
    return;
  }

  while (pending_.size() > max_pending_bundles_) {
    pending_.erase(pending_.begin());
    ++result->dropped_pending;
  }
}

KeyframeImageBundleAddResult KeyframeImageBundleAssembler::add_image(
  std::size_t camera_index,
  const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  KeyframeImageBundleAddResult result;
  if (camera_index >= camera_count_ || image == nullptr) {
    return result;
  }

  const auto key = stamp_key(image->header.stamp);
  auto [it, inserted] = pending_.try_emplace(key);
  if (inserted) {
    it->second.stamp = image->header.stamp;
    it->second.images.resize(camera_count_);
  }

  auto & pending = it->second;
  if (pending.images[camera_index] != nullptr) {
    result.replaced_duplicate = true;
  } else {
    ++pending.received_count;
  }
  pending.images[camera_index] = image;

  finalize_or_trim(key, &result);
  return result;
}

KeyframeImageBundleAddResult KeyframeImageBundleAssembler::add_pose(
  const nav_msgs::msg::Odometry::ConstSharedPtr & pose)
{
  KeyframeImageBundleAddResult result;
  if (pose == nullptr) {
    return result;
  }

  const auto key = stamp_key(pose->header.stamp);
  auto [it, inserted] = pending_.try_emplace(key);
  if (inserted) {
    it->second.stamp = pose->header.stamp;
    it->second.images.resize(camera_count_);
  }

  if (it->second.pose != nullptr) {
    result.replaced_duplicate = true;
  }
  it->second.pose = pose;
  finalize_or_trim(key, &result);
  return result;
}

namespace {

void replace_all(std::string & value, const std::string & token, const std::string & replacement)
{
  if (token.empty()) {
    return;
  }
  std::size_t position = 0U;
  while ((position = value.find(token, position)) != std::string::npos) {
    value.replace(position, token.size(), replacement);
    position += replacement.size();
  }
}

}  // namespace

std::vector<std::string> keyframe_image_topics(
  const std::vector<std::string> & explicit_topics,
  std::size_t count,
  const std::string & pattern)
{
  if (!explicit_topics.empty()) {
    return explicit_topics;
  }

  std::vector<std::string> result;
  result.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    std::string topic = pattern;
    const auto value = std::to_string(index);
    const auto before = topic;
    replace_all(topic, "{index}", value);
    replace_all(topic, "{}", value);
    replace_all(topic, "%d", value);
    if (topic == before) {
      topic += value;
    }
    result.push_back(std::move(topic));
  }
  return result;
}

int clamp_jpeg_quality(int quality) noexcept
{
  return std::clamp(quality, 1, 100);
}

std::string normalize_camera_pose_source(const std::string & source)
{
  if (source == "odometry" || source == "keyframe" || source == "pose_graph") {
    return source;
  }
  return "pose_graph";
}

std::optional<PointCloudSnapshot> point_cloud_snapshot(
  const sensor_msgs::msg::PointCloud2 & message,
  std::string * error)
{
  const auto * x_field = find_field(message, "x");
  const auto * y_field = find_field(message, "y");
  const auto * z_field = find_field(message, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    set_error(error, "PointCloud2 must contain x, y, and z fields");
    return std::nullopt;
  }
  if (datatype_size(x_field->datatype) == 0U || datatype_size(y_field->datatype) == 0U ||
    datatype_size(z_field->datatype) == 0U)
  {
    set_error(error, "PointCloud2 contains an unsupported XYZ datatype");
    return std::nullopt;
  }

  const std::size_t width = message.width;
  const std::size_t height = message.height == 0U ? 1U : message.height;
  const std::size_t point_count = width * height;
  const std::size_t row_step = message.row_step == 0U ? message.point_step * width : message.row_step;
  if (point_count != 0U && message.point_step == 0U) {
    set_error(error, "PointCloud2 has points but point_step is zero");
    return std::nullopt;
  }

  const auto * packed_color_field = find_field(message, "rgba");
  const auto * rgb_field = packed_color_field != nullptr ? packed_color_field : find_field(message, "rgb");
  const auto * r_field = find_field(message, "r");
  const auto * g_field = find_field(message, "g");
  const auto * b_field = find_field(message, "b");
  const bool has_packed_color = rgb_field != nullptr;
  const bool has_separate_color = r_field != nullptr && g_field != nullptr && b_field != nullptr;
  const bool has_color = has_packed_color || has_separate_color;

  PointCloudSnapshot result;
  result.frame_id = message.header.frame_id;
  result.positions.reserve(point_count);
  if (has_color) {
    result.colors.reserve(point_count);
  }

  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t column = 0; column < width; ++column) {
      const std::size_t base = row * row_step + column * message.point_step;
      const auto x = read_scalar(message, *x_field, base);
      const auto y = read_scalar(message, *y_field, base);
      const auto z = read_scalar(message, *z_field, base);
      if (!x.has_value() || !y.has_value() || !z.has_value()) {
        continue;
      }
      const std::array<float, 3> point{
        static_cast<float>(*x), static_cast<float>(*y), static_cast<float>(*z)};
      if (!is_finite_point(point)) {
        continue;
      }
      result.positions.push_back(point);

      if (has_color) {
        Rgba8 color{};
        if (has_packed_color) {
          const auto packed = read_packed_color(message, *rgb_field, base);
          if (packed.has_value()) {
            color.r = static_cast<std::uint8_t>((*packed >> 16U) & 0xFFU);
            color.g = static_cast<std::uint8_t>((*packed >> 8U) & 0xFFU);
            color.b = static_cast<std::uint8_t>(*packed & 0xFFU);
            color.a = packed_color_field != nullptr ?
              static_cast<std::uint8_t>((*packed >> 24U) & 0xFFU) : 255U;
          }
        } else {
          const auto red = read_scalar(message, *r_field, base);
          const auto green = read_scalar(message, *g_field, base);
          const auto blue = read_scalar(message, *b_field, base);
          const bool normalized =
            r_field->datatype == PointField::FLOAT32 || r_field->datatype == PointField::FLOAT64 ||
            g_field->datatype == PointField::FLOAT32 || g_field->datatype == PointField::FLOAT64 ||
            b_field->datatype == PointField::FLOAT32 || b_field->datatype == PointField::FLOAT64;
          if (red.has_value() && green.has_value() && blue.has_value()) {
            color.r = component_to_u8(*red, normalized);
            color.g = component_to_u8(*green, normalized);
            color.b = component_to_u8(*blue, normalized);
          }
        }
        result.colors.push_back(color);
      }
    }
  }

  return result;
}

PathSnapshot path_snapshot(const nav_msgs::msg::Path & message)
{
  PathSnapshot result;
  result.frame_id = message.header.frame_id;
  result.positions.reserve(message.poses.size());
  for (const auto & pose : message.poses) {
    const std::array<float, 3> point{
      static_cast<float>(pose.pose.position.x),
      static_cast<float>(pose.pose.position.y),
      static_cast<float>(pose.pose.position.z)};
    if (is_finite_point(point)) {
      result.positions.push_back(point);
    }
  }
  return result;
}

std::optional<MarkerLine> marker_line(
  const visualization_msgs::msg::Marker & message,
  std::string * error)
{
  if (message.action == visualization_msgs::msg::Marker::DELETE ||
    message.action == visualization_msgs::msg::Marker::DELETEALL)
  {
    return std::nullopt;
  }
  if (message.type != visualization_msgs::msg::Marker::LINE_STRIP &&
    message.type != visualization_msgs::msg::Marker::LINE_LIST)
  {
    return std::nullopt;
  }
  if (message.action != visualization_msgs::msg::Marker::ADD &&
    message.action != visualization_msgs::msg::Marker::MODIFY)
  {
    set_error(error, "unsupported Marker action");
    return std::nullopt;
  }

  MarkerLine result;
  result.id = message.id;
  result.ns = message.ns;
  result.type = message.type == visualization_msgs::msg::Marker::LINE_STRIP ?
    MarkerLineType::kLineStrip : MarkerLineType::kLineList;
  result.frame_id = message.header.frame_id;
  result.color = {
    marker_channel(message.color.r), marker_channel(message.color.g),
    marker_channel(message.color.b), marker_channel(message.color.a)};
  // RViz permits colors to be supplied per vertex. SVIn camera-frustum
  // markers use that form and leave marker.color at its zero default. The
  // current frusta are uniformly colored, so one representative vertex color
  // preserves their intended red/cyan appearance in Rerun.
  if (!message.colors.empty()) {
    const auto & color = message.colors.front();
    result.color = {
      marker_channel(color.r), marker_channel(color.g),
      marker_channel(color.b), marker_channel(color.a)};
  }
  result.radius = std::isfinite(message.scale.x) && message.scale.x > 0.0F ?
    message.scale.x / 2 : 0.01F;

  std::vector<std::array<float, 3>> points;
  points.reserve(message.points.size());
  for (const auto & point : message.points) {
    const std::array<float, 3> xyz{
      static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)};
    if (is_finite_point(xyz)) {
      points.push_back(xyz);
    }
  }

  if (result.type == MarkerLineType::kLineStrip) {
    if (points.size() < 2U) {
      set_error(error, "LINE_STRIP marker has fewer than two finite points");
      return std::nullopt;
    }
    result.strips.push_back(std::move(points));
  } else {
    if (points.size() < 2U) {
      set_error(error, "LINE_LIST marker has fewer than two finite points");
      return std::nullopt;
    }
    for (std::size_t i = 0; i + 1U < points.size(); i += 2U) {
      result.strips.push_back({points[i], points[i + 1U]});
    }
    if (result.strips.empty()) {
      set_error(error, "LINE_LIST marker has no complete segment");
      return std::nullopt;
    }
  }

  return result;
}

std::optional<JpegImage> encode_jpeg(
  const sensor_msgs::msg::Image & message,
  int quality,
  std::string * error)
{
  if (message.width == 0U || message.height == 0U) {
    set_error(error, "image has zero width or height");
    return std::nullopt;
  }

  try {
    const auto cv_image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
    if (cv_image == nullptr || cv_image->image.empty()) {
      set_error(error, "cv_bridge returned an empty image");
      return std::nullopt;
    }
    JpegImage result;
    const std::vector<int> params{
      cv::IMWRITE_JPEG_QUALITY, clamp_jpeg_quality(quality)};
    if (!cv::imencode(".jpg", cv_image->image, result.bytes, params)) {
      set_error(error, "OpenCV failed to encode JPEG");
      return std::nullopt;
    }
    result.width = static_cast<std::uint32_t>(cv_image->image.cols);
    result.height = static_cast<std::uint32_t>(cv_image->image.rows);
    return result;
  } catch (const std::exception & exception) {
    set_error(error, std::string("image conversion failed: ") + exception.what());
    return std::nullopt;
  }
}

std::string marker_entity_path(const MarkerLine & marker)
{
  std::ostringstream path;
  path << "world/visualization/" << safe_path_component(marker.ns) << "/" << marker.id;
  return path.str();
}

}  // namespace svin_rerun_bridge
