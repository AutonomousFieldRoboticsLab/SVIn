#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace svin_rerun_bridge {

struct DoubleSphereCamera {
  std::string name;
  std::array<double, 16> T_SC{};
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double xi{0.0};
  double alpha{0.0};
};

struct DoubleSphereMesh {
  std::vector<std::array<float, 3>> vertices;
  std::vector<std::array<std::uint32_t, 3>> triangles;
  std::vector<std::array<float, 2>> texcoords;
};

[[nodiscard]] std::optional<std::array<double, 3>> double_sphere_unproject(
  double x, double y, const DoubleSphereCamera & camera);

[[nodiscard]] DoubleSphereMesh make_double_sphere_mesh(
  const DoubleSphereCamera & camera, float depth, int grid_size);

[[nodiscard]] std::optional<std::vector<DoubleSphereCamera>> load_double_sphere_cameras(
  const std::string & config_path, std::string * error = nullptr);

}  // namespace svin_rerun_bridge
