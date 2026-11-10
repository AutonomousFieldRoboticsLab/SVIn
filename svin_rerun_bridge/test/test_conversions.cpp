#include "svin_rerun_bridge/conversions.hpp"

#include <gtest/gtest.h>
#include <sensor_msgs/image_encodings.hpp>

#include <cmath>
#include <cstring>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace {

void write_float(std::vector<std::uint8_t> & data, std::size_t offset, float value)
{
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void write_u32(std::vector<std::uint8_t> & data, std::size_t offset, std::uint32_t value)
{
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

sensor_msgs::msg::PointField make_field(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

sensor_msgs::msg::PointCloud2 make_point_cloud()
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = "world";
  message.width = 2;
  message.height = 1;
  message.point_step = 16;
  message.row_step = 32;
  message.fields = {
    make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    make_field("rgb", 12, sensor_msgs::msg::PointField::UINT32),
  };
  message.data.resize(message.row_step);
  write_float(message.data, 0, 1.0F);
  write_float(message.data, 4, 2.0F);
  write_float(message.data, 8, 3.0F);
  write_u32(message.data, 12, 0x112233U);
  write_float(message.data, 16, std::numeric_limits<float>::quiet_NaN());
  write_float(message.data, 20, 5.0F);
  write_float(message.data, 24, 6.0F);
  write_u32(message.data, 28, 0xAABBCCU);
  return message;
}

sensor_msgs::msg::Image::ConstSharedPtr make_keyframe_image(
  std::int32_t sec, std::uint32_t nanosec, std::uint8_t marker)
{
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->header.stamp.sec = sec;
  message->header.stamp.nanosec = nanosec;
  message->width = 1;
  message->height = 1;
  message->encoding = sensor_msgs::image_encodings::MONO8;
  message->step = 1;
  message->data = {marker};
  return message;
}

nav_msgs::msg::Odometry::ConstSharedPtr make_keyframe_pose(
  std::int32_t sec, std::uint32_t nanosec)
{
  auto message = std::make_shared<nav_msgs::msg::Odometry>();
  message->header.stamp.sec = sec;
  message->header.stamp.nanosec = nanosec;
  message->pose.pose.orientation.w = 1.0;
  return message;
}

}  // namespace

TEST(Conversions, PointCloudParsesPackedRgbAndDropsInvalidPoints)
{
  std::string error;
  const auto result = svin_rerun_bridge::point_cloud_snapshot(make_point_cloud(), &error);
  ASSERT_TRUE(result.has_value()) << error;
  ASSERT_EQ(result->positions.size(), 1U);
  ASSERT_TRUE(result->has_colors());
  EXPECT_EQ(result->frame_id, "world");
  EXPECT_FLOAT_EQ(result->positions[0][0], 1.0F);
  EXPECT_FLOAT_EQ(result->positions[0][2], 3.0F);
  ASSERT_EQ(result->colors.size(), 1U);
  EXPECT_EQ(result->colors[0].r, 0x11U);
  EXPECT_EQ(result->colors[0].g, 0x22U);
  EXPECT_EQ(result->colors[0].b, 0x33U);
  EXPECT_EQ(result->colors[0].a, 255U);
}

TEST(Conversions, PointCloudParsesPackedRgba)
{
  auto message = make_point_cloud();
  message.fields.back().name = "rgba";
  write_u32(message.data, 12, 0xAA112233U);

  std::string error;
  const auto result = svin_rerun_bridge::point_cloud_snapshot(message, &error);
  ASSERT_TRUE(result.has_value()) << error;
  ASSERT_EQ(result->colors.size(), 1U);
  EXPECT_EQ(result->colors[0].r, 0x11U);
  EXPECT_EQ(result->colors[0].g, 0x22U);
  EXPECT_EQ(result->colors[0].b, 0x33U);
  EXPECT_EQ(result->colors[0].a, 0xAAU);
}

TEST(Conversions, PathExtractsFinitePositions)
{
  nav_msgs::msg::Path message;
  message.header.frame_id = "world";
  geometry_msgs::msg::PoseStamped first;
  first.pose.position.x = 1.0;
  first.pose.position.y = 2.0;
  first.pose.position.z = 3.0;
  message.poses.push_back(first);
  geometry_msgs::msg::PoseStamped invalid;
  invalid.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  message.poses.push_back(invalid);

  const auto result = svin_rerun_bridge::path_snapshot(message);
  EXPECT_EQ(result.frame_id, "world");
  ASSERT_EQ(result.positions.size(), 1U);
  EXPECT_FLOAT_EQ(result.positions[0][1], 2.0F);
}

TEST(Conversions, MarkerLineStripAndLineListAreSupported)
{
  visualization_msgs::msg::Marker strip;
  strip.ns = "camera frusta";
  strip.id = 7;
  strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  strip.action = visualization_msgs::msg::Marker::ADD;
  strip.scale.x = 0.02;
  strip.color.r = 1.0F;
  strip.color.a = 0.5F;
  strip.points.resize(3);
  strip.points[1].x = 1.0;
  strip.points[2].x = 2.0;
  const auto strip_result = svin_rerun_bridge::marker_line(strip);
  ASSERT_TRUE(strip_result.has_value());
  EXPECT_EQ(strip_result->strips.size(), 1U);
  EXPECT_EQ(strip_result->strips[0].size(), 3U);
  EXPECT_EQ(strip_result->color.r, 255U);
  EXPECT_EQ(strip_result->color.a, 128U);
  EXPECT_EQ(
    svin_rerun_bridge::marker_entity_path(*strip_result),
    "world/visualization/camera_frusta/7");

  visualization_msgs::msg::Marker list = strip;
  list.type = visualization_msgs::msg::Marker::LINE_LIST;
  list.points.resize(4);
  list.points[2].y = 1.0;
  list.points[3].y = 1.0;
  const auto list_result = svin_rerun_bridge::marker_line(list);
  ASSERT_TRUE(list_result.has_value());
  EXPECT_EQ(list_result->strips.size(), 2U);
  EXPECT_EQ(list_result->strips[0].size(), 2U);
  EXPECT_EQ(list_result->strips[1].size(), 2U);
}

TEST(Conversions, MarkerUsesPerVertexColorWhenPresent)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.points.resize(2);
  marker.points[1].x = 1.0;
  marker.colors.resize(2);
  marker.colors[0].g = 0.7F;
  marker.colors[0].b = 1.0F;
  marker.colors[0].a = 1.0F;
  marker.colors[1] = marker.colors[0];

  const auto result = svin_rerun_bridge::marker_line(marker);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->color.r, 0U);
  EXPECT_EQ(result->color.g, 179U);
  EXPECT_EQ(result->color.b, 255U);
  EXPECT_EQ(result->color.a, 255U);
}

TEST(Conversions, MonoImageIsEncodedAsJpeg)
{
  sensor_msgs::msg::Image message;
  message.width = 2;
  message.height = 2;
  message.encoding = sensor_msgs::image_encodings::MONO8;
  message.step = 2;
  message.data = {0U, 64U, 128U, 255U};

  std::string error;
  const auto result = svin_rerun_bridge::encode_jpeg(message, 0, &error);
  ASSERT_TRUE(result.has_value()) << error;
  EXPECT_EQ(result->width, 2U);
  EXPECT_EQ(result->height, 2U);
  ASSERT_GE(result->bytes.size(), 4U);
  EXPECT_EQ(result->bytes[0], 0xFFU);
  EXPECT_EQ(result->bytes[1], 0xD8U);
  EXPECT_EQ(svin_rerun_bridge::clamp_jpeg_quality(-5), 1);
  EXPECT_EQ(svin_rerun_bridge::clamp_jpeg_quality(101), 100);
}

TEST(Conversions, StampUsesNanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 345;
  EXPECT_EQ(svin_rerun_bridge::stamp_to_nanoseconds(stamp), 12000000345LL);
}

TEST(Conversions, TrustedSensorStampUsesNewestNonZeroCandidate)
{
  builtin_interfaces::msg::Time current;
  current.sec = 10;
  current.nanosec = 20;
  builtin_interfaces::msg::Time older = current;
  older.nanosec = 19;
  builtin_interfaces::msg::Time newer = current;
  newer.nanosec = 21;
  builtin_interfaces::msg::Time empty;

  const auto unchanged = svin_rerun_bridge::update_latest_trusted_sensor_stamp(
    current, older);
  ASSERT_TRUE(unchanged.has_value());
  EXPECT_EQ(unchanged->nanosec, current.nanosec);

  const auto updated = svin_rerun_bridge::update_latest_trusted_sensor_stamp(
    unchanged, newer);
  ASSERT_TRUE(updated.has_value());
  EXPECT_EQ(updated->nanosec, newer.nanosec);

  const auto ignored_empty = svin_rerun_bridge::update_latest_trusted_sensor_stamp(
    updated, empty);
  ASSERT_TRUE(ignored_empty.has_value());
  EXPECT_EQ(ignored_empty->nanosec, newer.nanosec);
}

TEST(Conversions, PointCloudUsesTrustedSensorStampWhenAvailable)
{
  builtin_interfaces::msg::Time message_stamp;
  message_stamp.sec = 2026;
  builtin_interfaces::msg::Time trusted_stamp;
  trusted_stamp.sec = 42;

  const auto selected = svin_rerun_bridge::select_point_cloud_sensor_stamp(
    message_stamp, trusted_stamp);
  EXPECT_EQ(selected.sec, trusted_stamp.sec);

  const auto fallback = svin_rerun_bridge::select_point_cloud_sensor_stamp(
    message_stamp, std::nullopt);
  EXPECT_EQ(fallback.sec, message_stamp.sec);
}

TEST(Conversions, KeyframeBundleWaitsForAllImagesAndPose)
{
  svin_rerun_bridge::KeyframeImageBundleAssembler assembler(3, 4, true);
  const auto camera0 = make_keyframe_image(7, 11, 0U);
  const auto camera1 = make_keyframe_image(7, 11, 1U);
  const auto camera2 = make_keyframe_image(7, 11, 2U);
  const auto pose = make_keyframe_pose(7, 11);

  EXPECT_FALSE(assembler.add_image(2, camera2).complete.has_value());
  EXPECT_FALSE(assembler.add_pose(pose).complete.has_value());
  EXPECT_FALSE(assembler.add_image(0, camera0).complete.has_value());
  const auto result = assembler.add_image(1, camera1);

  ASSERT_TRUE(result.complete.has_value());
  ASSERT_EQ(result.complete->images.size(), 3U);
  EXPECT_EQ(result.complete->images[0], camera0);
  EXPECT_EQ(result.complete->images[1], camera1);
  EXPECT_EQ(result.complete->images[2], camera2);
  EXPECT_EQ(result.complete->pose, pose);
  EXPECT_EQ(assembler.pending_count(), 0U);
}

TEST(Conversions, KeyframeBundleDropsOldestIncompleteTimestamp)
{
  svin_rerun_bridge::KeyframeImageBundleAssembler assembler(2, 2, false);
  const auto first = make_keyframe_image(1, 0, 1U);
  const auto second = make_keyframe_image(2, 0, 2U);
  const auto third = make_keyframe_image(3, 0, 3U);

  EXPECT_EQ(assembler.add_image(0, first).dropped_pending, 0U);
  EXPECT_EQ(assembler.add_image(0, second).dropped_pending, 0U);
  const auto dropped = assembler.add_image(0, third);
  EXPECT_EQ(dropped.dropped_pending, 1U);
  EXPECT_EQ(assembler.pending_count(), 2U);

  EXPECT_FALSE(assembler.add_image(1, first).complete.has_value());
  const auto completed = assembler.add_image(1, second);
  ASSERT_TRUE(completed.complete.has_value());
  EXPECT_EQ(completed.complete->images[0], second);
  EXPECT_EQ(completed.complete->images[1], second);
}

TEST(Conversions, KeyframeTopicsSupportExplicitListAndPatterns)
{
  const auto explicit_topics = svin_rerun_bridge::keyframe_image_topics(
    {"/left", "/right", "/top"}, 2, "/unused_{}");
  EXPECT_EQ(explicit_topics, (std::vector<std::string>{"/left", "/right", "/top"}));

  const auto generated_topics = svin_rerun_bridge::keyframe_image_topics(
    {}, 3, "/keyframe_image_{index}");
  EXPECT_EQ(
    generated_topics,
    (std::vector<std::string>{"/keyframe_image_0", "/keyframe_image_1", "/keyframe_image_2"}));

  const auto percent_topics = svin_rerun_bridge::keyframe_image_topics({}, 2, "/camera_%d");
  EXPECT_EQ(percent_topics, (std::vector<std::string>{"/camera_0", "/camera_1"}));
}

TEST(Conversions, CameraPoseSourceDefaultsSafely)
{
  EXPECT_EQ(svin_rerun_bridge::normalize_camera_pose_source("pose_graph"), "pose_graph");
  EXPECT_EQ(svin_rerun_bridge::normalize_camera_pose_source("odometry"), "odometry");
  EXPECT_EQ(svin_rerun_bridge::normalize_camera_pose_source("keyframe"), "keyframe");
  EXPECT_EQ(svin_rerun_bridge::normalize_camera_pose_source("unsupported"), "pose_graph");
}
