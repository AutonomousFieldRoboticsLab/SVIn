#include "svin_rerun_bridge/compressed_image_bundle.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace svin_rerun_bridge {
namespace {

sensor_msgs::msg::CompressedImage::ConstSharedPtr image(int second, std::uint32_t nanosecond)
{
  auto message = std::make_shared<sensor_msgs::msg::CompressedImage>();
  message->header.stamp.sec = second;
  message->header.stamp.nanosec = nanosecond;
  message->format = "jpeg";
  message->data = {0xFF, 0xD8, 0xFF, 0xD9};
  return message;
}

TEST(CompressedImageBundle, EmitsNImagesInCameraOrder)
{
  CompressedImageBundleAssembler assembler(3U, 4U);
  EXPECT_FALSE(assembler.add(2U, image(1, 2U)).complete.has_value());
  EXPECT_FALSE(assembler.add(0U, image(1, 2U)).complete.has_value());
  const auto result = assembler.add(1U, image(1, 2U));
  ASSERT_TRUE(result.complete.has_value());
  ASSERT_EQ(result.complete->images.size(), 3U);
  EXPECT_EQ(result.complete->stamp.sec, 1);
}

TEST(CompressedImageBundle, BoundsIncompleteTimestamps)
{
  CompressedImageBundleAssembler assembler(2U, 2U);
  EXPECT_EQ(assembler.add(0U, image(1, 0U)).dropped_pending, 0U);
  EXPECT_EQ(assembler.add(0U, image(2, 0U)).dropped_pending, 0U);
  EXPECT_EQ(assembler.add(0U, image(3, 0U)).dropped_pending, 1U);
  EXPECT_EQ(assembler.pending_count(), 2U);
}

}  // namespace
}  // namespace svin_rerun_bridge
