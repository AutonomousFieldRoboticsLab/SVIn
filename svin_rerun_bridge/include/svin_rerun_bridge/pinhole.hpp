#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace svin_rerun_bridge {

struct PinholeCamera {
  std::string name;
  std::array<double, 16> T_SC{};
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string distortion_type;
  std::vector<double> distortion_coefficients;
};

[[nodiscard]] std::optional<std::vector<PinholeCamera>> load_pinhole_cameras(
  const std::string & config_path, std::string * error = nullptr);

}  // namespace svin_rerun_bridge
