#include "svin_rerun_bridge/pinhole.hpp"

#include <gtest/gtest.h>

#include <string>

namespace svin_rerun_bridge {
namespace {

const std::string config_dir = SVIN_RERUN_TEST_CONFIG_DIR;

TEST(Pinhole, LoadsWorkspaceCalibration)
{
  std::string error;
  const auto cameras = load_pinhole_cameras(
    config_dir + "/config_gopro13_uw.yaml", &error);
  ASSERT_TRUE(cameras.has_value()) << error;
  ASSERT_EQ(cameras->size(), 1U);
  EXPECT_EQ((*cameras)[0].width, 960);
  EXPECT_EQ((*cameras)[0].height, 540);
  EXPECT_NEAR((*cameras)[0].fx, 585.1458698638675, 1.0e-12);
  EXPECT_NEAR((*cameras)[0].cy, 273.39643747563633, 1.0e-12);
  EXPECT_EQ((*cameras)[0].distortion_type, "radialtangential");
  EXPECT_EQ((*cameras)[0].distortion_coefficients.size(), 4U);
}

TEST(Pinhole, LoadsStereoWorkspaceCalibration)
{
  std::string error;
  const auto cameras = load_pinhole_cameras(
    config_dir + "/config_aqua2_A6_BBDOS26_1280_720.yaml", &error);
  ASSERT_TRUE(cameras.has_value()) << error;
  ASSERT_EQ(cameras->size(), 2U);
  EXPECT_EQ((*cameras)[0].width, 1280);
  EXPECT_EQ((*cameras)[0].height, 720);
  EXPECT_NEAR((*cameras)[0].fx, 975.3634442928654, 1.0e-12);
  EXPECT_NEAR((*cameras)[1].fx, 970.31110446077, 1.0e-12);
  EXPECT_EQ((*cameras)[0].distortion_type, "radialtangential");
  EXPECT_EQ((*cameras)[1].distortion_coefficients.size(), 4U);
}

TEST(Pinhole, LoadsOdometryFrameCalibration)
{
  std::string error;
  const auto calibration = load_pinhole_calibration(
    config_dir + "/config_aqua2_A6_BBDOS26_1280_720.yaml", &error);
  ASSERT_TRUE(calibration.has_value()) << error;
  EXPECT_EQ(calibration->tracked_body_frame, TrackedBodyFrame::body);
  EXPECT_NEAR(calibration->T_BS[3], -0.018, 1.0e-12);
  EXPECT_NEAR(calibration->T_BS[7], -0.016, 1.0e-12);
  EXPECT_NEAR(calibration->T_BS[2], 1.0, 1.0e-12);
}

}  // namespace
}  // namespace svin_rerun_bridge
