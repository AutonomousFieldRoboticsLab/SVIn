#include "svin_rerun_bridge/double_sphere.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace svin_rerun_bridge {
namespace {

void set_error(std::string * error, std::string message)
{
  if (error != nullptr) {
    *error = std::move(message);
  }
}

template<typename T, std::size_t N>
bool read_array(const cv::FileNode & node, std::array<T, N> * values)
{
  if (!node.isSeq() || node.size() != N) {
    return false;
  }
  std::size_t index = 0;
  for (const auto & value : node) {
    (*values)[index++] = static_cast<T>(value.real());
  }
  return true;
}

}  // namespace

std::optional<std::array<double, 3>> double_sphere_unproject(
  double x, double y, const DoubleSphereCamera & camera)
{
  if (!(camera.fx > 0.0) || !(camera.fy > 0.0) ||
    !std::isfinite(x) || !std::isfinite(y))
  {
    return std::nullopt;
  }

  const double mx = (x - camera.cx) / camera.fx;
  const double my = (y - camera.cy) / camera.fy;
  const double r2 = mx * mx + my * my;
  const double root = 1.0 - (2.0 * camera.alpha - 1.0) * r2;
  if (!(root > 0.0)) {
    return std::nullopt;
  }

  const double denominator = camera.alpha * std::sqrt(root) + 1.0 - camera.alpha;
  if (std::abs(denominator) <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }
  const double mz = (1.0 - camera.alpha * camera.alpha * r2) / denominator;
  const double inner = mz * mz + (1.0 - camera.xi * camera.xi) * r2;
  const double scale_denominator = mz * mz + r2;
  if (!(inner > 0.0) || scale_denominator <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  const double scale = (mz * camera.xi + std::sqrt(inner)) / scale_denominator;
  std::array<double, 3> bearing{
    scale * mx,
    scale * my,
    scale * mz - camera.xi};
  const double norm = std::sqrt(
    bearing[0] * bearing[0] + bearing[1] * bearing[1] + bearing[2] * bearing[2]);
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return std::nullopt;
  }
  for (double & component : bearing) {
    component /= norm;
  }
  return bearing;
}

DoubleSphereMesh make_double_sphere_mesh(
  const DoubleSphereCamera & camera, float depth, int grid_size)
{
  DoubleSphereMesh mesh;
  if (!(depth > 0.0F) || grid_size < 2 || camera.width < 2 || camera.height < 2) {
    return mesh;
  }

  const auto grid_count = static_cast<std::size_t>(grid_size * grid_size);
  std::vector<std::int64_t> indices(grid_count, -1);
  mesh.vertices.reserve(grid_count);
  mesh.texcoords.reserve(grid_count);

  for (int row = 0; row < grid_size; ++row) {
    const double v = static_cast<double>(row) / static_cast<double>(grid_size - 1);
    const double y = v * static_cast<double>(camera.height - 1);
    for (int column = 0; column < grid_size; ++column) {
      const double u = static_cast<double>(column) / static_cast<double>(grid_size - 1);
      const double x = u * static_cast<double>(camera.width - 1);
      const auto bearing = double_sphere_unproject(x, y, camera);
      if (!bearing.has_value()) {
        continue;
      }
      const auto flat_index = static_cast<std::size_t>(row * grid_size + column);
      indices[flat_index] = static_cast<std::int64_t>(mesh.vertices.size());
      mesh.vertices.push_back({
        depth * static_cast<float>((*bearing)[0]),
        depth * static_cast<float>((*bearing)[1]),
        depth * static_cast<float>((*bearing)[2])});
      // Rerun textures use top-left image origin, matching the ROS image buffer.
      mesh.texcoords.push_back({static_cast<float>(u), static_cast<float>(v)});
    }
  }

  const auto vertex_at = [&indices, grid_size](int row, int column) {
      return indices[static_cast<std::size_t>(row * grid_size + column)];
    };
  for (int row = 0; row < grid_size - 1; ++row) {
    for (int column = 0; column < grid_size - 1; ++column) {
      const std::int64_t top_left = vertex_at(row, column);
      const std::int64_t top_right = vertex_at(row, column + 1);
      const std::int64_t bottom_left = vertex_at(row + 1, column);
      const std::int64_t bottom_right = vertex_at(row + 1, column + 1);
      if (top_left < 0 || top_right < 0 || bottom_left < 0 || bottom_right < 0) {
        continue;
      }
      mesh.triangles.push_back({
        static_cast<std::uint32_t>(top_left),
        static_cast<std::uint32_t>(bottom_right),
        static_cast<std::uint32_t>(top_right)});
      mesh.triangles.push_back({
        static_cast<std::uint32_t>(top_left),
        static_cast<std::uint32_t>(bottom_left),
        static_cast<std::uint32_t>(bottom_right)});
    }
  }
  return mesh;
}

std::optional<std::vector<DoubleSphereCamera>> load_double_sphere_cameras(
  const std::string & config_path, std::string * error)
{
  cv::FileStorage storage(config_path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    set_error(error, "cannot open camera configuration: " + config_path);
    return std::nullopt;
  }
  const cv::FileNode cameras = storage["cameras"];
  if (!cameras.isSeq() || cameras.empty()) {
    set_error(error, "configuration has no camera sequence");
    return std::nullopt;
  }

  std::vector<DoubleSphereCamera> result;
  result.reserve(cameras.size());
  std::size_t index = 0;
  for (const auto & node : cameras) {
    std::string projection_type;
    node["projection_type"] >> projection_type;
    if (projection_type != "double_sphere") {
      std::ostringstream stream;
      stream << "camera " << index << " uses unsupported projection_type '"
             << projection_type << "'";
      set_error(error, stream.str());
      return std::nullopt;
    }

    DoubleSphereCamera camera;
    camera.name = "cam" + std::to_string(index);
    std::array<double, 2> dimensions{};
    std::array<double, 2> focal{};
    std::array<double, 2> principal{};
    if (!read_array(node["T_SC"], &camera.T_SC) ||
      !read_array(node["image_dimension"], &dimensions) ||
      !read_array(node["focal_length"], &focal) ||
      !read_array(node["principal_point"], &principal))
    {
      set_error(error, "camera " + std::to_string(index) + " has malformed calibration arrays");
      return std::nullopt;
    }
    camera.width = static_cast<int>(dimensions[0]);
    camera.height = static_cast<int>(dimensions[1]);
    camera.fx = focal[0];
    camera.fy = focal[1];
    camera.cx = principal[0];
    camera.cy = principal[1];
    camera.xi = static_cast<double>(node["xi"].real());
    camera.alpha = static_cast<double>(node["alpha"].real());
    if (camera.width <= 1 || camera.height <= 1 || camera.fx <= 0.0 || camera.fy <= 0.0) {
      set_error(error, "camera " + std::to_string(index) + " has invalid dimensions/intrinsics");
      return std::nullopt;
    }
    result.push_back(std::move(camera));
    ++index;
  }
  return result;
}

}  // namespace svin_rerun_bridge
