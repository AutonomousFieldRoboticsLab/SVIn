#include "svin_rerun_bridge/pinhole.hpp"

#include <opencv2/core.hpp>

#include <cmath>
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
  std::size_t index = 0U;
  for (const auto & value : node) {
    (*values)[index++] = static_cast<T>(value.real());
  }
  return true;
}

bool read_vector(const cv::FileNode & node, std::vector<double> * values)
{
  if (node.empty()) {
    values->clear();
    return true;
  }
  if (!node.isSeq()) {
    return false;
  }
  values->clear();
  values->reserve(node.size());
  for (const auto & value : node) {
    values->push_back(value.real());
  }
  return true;
}

}  // namespace

std::optional<std::vector<PinholeCamera>> load_pinhole_cameras(
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

  std::vector<PinholeCamera> result;
  result.reserve(cameras.size());
  std::size_t index = 0U;
  for (const auto & node : cameras) {
    std::string projection_type;
    node["projection_type"] >> projection_type;
    if (projection_type != "pinhole") {
      std::ostringstream stream;
      stream << "camera " << index << " uses unsupported projection_type '"
             << projection_type << "'";
      set_error(error, stream.str());
      return std::nullopt;
    }

    PinholeCamera camera;
    camera.name = "cam" + std::to_string(index);
    std::array<double, 2> dimensions{};
    std::array<double, 2> focal{};
    std::array<double, 2> principal{};
    node["distortion_type"] >> camera.distortion_type;
    if (!read_array(node["T_SC"], &camera.T_SC) ||
      !read_array(node["image_dimension"], &dimensions) ||
      !read_array(node["focal_length"], &focal) ||
      !read_array(node["principal_point"], &principal) ||
      !read_vector(node["distortion_coefficients"], &camera.distortion_coefficients))
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
    if (camera.width <= 1 || camera.height <= 1 || camera.fx <= 0.0 || camera.fy <= 0.0 ||
      !std::isfinite(camera.cx) || !std::isfinite(camera.cy))
    {
      set_error(error, "camera " + std::to_string(index) + " has invalid dimensions/intrinsics");
      return std::nullopt;
    }
    result.push_back(std::move(camera));
    ++index;
  }
  return result;
}

}  // namespace svin_rerun_bridge
