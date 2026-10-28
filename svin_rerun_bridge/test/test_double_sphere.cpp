#include "svin_rerun_bridge/double_sphere.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace svin_rerun_bridge {
namespace {

DoubleSphereCamera test_camera()
{
  DoubleSphereCamera camera;
  camera.width = 832;
  camera.height = 832;
  camera.fx = 244.1628281634585;
  camera.fy = 245.06462724291987;
  camera.cx = 421.27176788834316;
  camera.cy = 420.608742357202;
  camera.xi = 0.022851811680617545;
  camera.alpha = 0.6831017255310529;
  return camera;
}

TEST(DoubleSphere, PrincipalPointUnprojectsAlongOpticalAxis)
{
  const auto camera = test_camera();
  const auto bearing = double_sphere_unproject(camera.cx, camera.cy, camera);
  ASSERT_TRUE(bearing.has_value());
  EXPECT_NEAR((*bearing)[0], 0.0, 1.0e-12);
  EXPECT_NEAR((*bearing)[1], 0.0, 1.0e-12);
  EXPECT_NEAR((*bearing)[2], 1.0, 1.0e-12);
}

TEST(DoubleSphere, MeshHasMatchingVerticesAndTextureCoordinates)
{
  const auto mesh = make_double_sphere_mesh(test_camera(), 0.45F, 31);
  EXPECT_GT(mesh.vertices.size(), 100U);
  EXPECT_EQ(mesh.vertices.size(), mesh.texcoords.size());
  EXPECT_GT(mesh.triangles.size(), 100U);
  for (const auto & vertex : mesh.vertices) {
    const double radius = std::sqrt(
      vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]);
    EXPECT_NEAR(radius, 0.45, 1.0e-5);
  }
}

TEST(DoubleSphere, LoadsWorkspaceCalibration)
{
  std::string error;
  const auto cameras = load_double_sphere_cameras(
    "/home/ws/src/SVIn2/config/config_insta360X5_dual_ds_air_stav.yaml", &error);
  ASSERT_TRUE(cameras.has_value()) << error;
  ASSERT_EQ(cameras->size(), 2U);
  EXPECT_EQ((*cameras)[0].width, 832);
  EXPECT_NEAR((*cameras)[1].alpha, 0.6864870568393587, 1.0e-12);
}

}  // namespace
}  // namespace svin_rerun_bridge
