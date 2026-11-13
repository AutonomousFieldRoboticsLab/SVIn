#pragma once

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace svin_rerun_bridge {

struct CompressedImageBundle {
  builtin_interfaces::msg::Time stamp;
  std::vector<sensor_msgs::msg::CompressedImage::ConstSharedPtr> images;
};

struct CompressedImageBundleAddResult {
  std::optional<CompressedImageBundle> complete;
  bool replaced_duplicate{false};
  std::size_t dropped_pending{0};
};

/// Exact-timestamp N-camera bundler for raw compressed sensor streams.
class CompressedImageBundleAssembler final {
public:
  CompressedImageBundleAssembler(std::size_t camera_count, std::size_t max_pending_bundles);

  [[nodiscard]] CompressedImageBundleAddResult add(
    std::size_t camera_index,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image);

  [[nodiscard]] std::size_t camera_count() const noexcept { return camera_count_; }
  [[nodiscard]] std::size_t max_pending_bundles() const noexcept {
    return max_pending_bundles_;
  }
  [[nodiscard]] std::size_t pending_count() const noexcept { return pending_.size(); }

private:
  using StampKey = std::pair<std::int32_t, std::uint32_t>;
  struct PendingBundle {
    builtin_interfaces::msg::Time stamp;
    std::vector<sensor_msgs::msg::CompressedImage::ConstSharedPtr> images;
    std::size_t received_count{0};
  };

  std::size_t camera_count_;
  std::size_t max_pending_bundles_;
  std::map<StampKey, PendingBundle> pending_;
};

}  // namespace svin_rerun_bridge
