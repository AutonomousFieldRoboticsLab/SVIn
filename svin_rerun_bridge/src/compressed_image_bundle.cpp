#include "svin_rerun_bridge/compressed_image_bundle.hpp"

#include <algorithm>
#include <stdexcept>

namespace svin_rerun_bridge {

CompressedImageBundleAssembler::CompressedImageBundleAssembler(
  std::size_t camera_count, std::size_t max_pending_bundles)
: camera_count_(camera_count),
  max_pending_bundles_(std::max<std::size_t>(max_pending_bundles, 1U))
{
  if (camera_count_ == 0U) {
    throw std::invalid_argument("compressed image bundle requires at least one camera");
  }
}

CompressedImageBundleAddResult CompressedImageBundleAssembler::add(
  std::size_t camera_index,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image)
{
  CompressedImageBundleAddResult result;
  if (camera_index >= camera_count_ || image == nullptr) {
    return result;
  }

  const StampKey key{image->header.stamp.sec, image->header.stamp.nanosec};
  auto [iterator, inserted] = pending_.try_emplace(key);
  if (inserted) {
    iterator->second.stamp = image->header.stamp;
    iterator->second.images.resize(camera_count_);
  }

  auto & pending = iterator->second;
  if (pending.images[camera_index] != nullptr) {
    result.replaced_duplicate = true;
  } else {
    ++pending.received_count;
  }
  pending.images[camera_index] = image;

  if (pending.received_count == camera_count_) {
    result.complete = CompressedImageBundle{pending.stamp, std::move(pending.images)};
    pending_.erase(iterator);
    return result;
  }

  while (pending_.size() > max_pending_bundles_) {
    pending_.erase(pending_.begin());
    ++result.dropped_pending;
  }
  return result;
}

}  // namespace svin_rerun_bridge
