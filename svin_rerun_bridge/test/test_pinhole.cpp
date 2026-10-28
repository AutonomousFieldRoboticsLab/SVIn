#include "svin_rerun_bridge/pinhole.hpp"

#include <gtest/gtest.h>

#include <string>

namespace svin_rerun_bridge {
namespace {

TEST(Pinhole, LoadsWorkspaceCalibration)
{
  std::string error;
  const auto cameras = load_pinhole_cameras(
    "/home/ws/src/SVIn2/config/config_gopro13_uw.yaml", &error);
  ASSERT_TRUE(cameras.has_value()) << error;
  ASSERT_EQ(cameras->size(), 1U);
  EXPECT_EQ((*cameras)[0].width, 960);
  EXPECT_EQ((*cameras)[0].height, 540);
  EXPECT_NEAR((*cameras)[0].fx, 585.1458698638675, 1.0e-12);
  EXPECT_NEAR((*cameras)[0].cy, 273.39643747563633, 1.0e-12);
  EXPECT_EQ((*cameras)[0].distortion_type, "radialtangential");
  EXPECT_EQ((*cameras)[0].distortion_coefficients.size(), 4U);
}

TEST(Pinhole, RejectsDoubleSphereCalibration)
{
  std::string error;
  const auto cameras = load_pinhole_cameras(
    "/home/ws/src/SVIn2/config/config_insta360X5_dual_ds_air_stav.yaml", &error);
  EXPECT_FALSE(cameras.has_value());
  EXPECT_NE(error.find("double_sphere"), std::string::npos);
}

}  // namespace
}  // namespace svin_rerun_bridge
