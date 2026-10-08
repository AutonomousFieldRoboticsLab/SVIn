/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the conditions in the project
 *  license are met.
 *********************************************************************************/

#include <okvis/SynchronizedCompressedImageSubscriber.hpp>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <utility>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace okvis {

SynchronizedCompressedImageSubscriber::SynchronizedCompressedImageSubscriber(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& cameraTopics,
    std::size_t queueSize,
    TupleCallback tupleCallback,
    bool logCounters)
    : node_(node),
      queueSize_(std::max<std::size_t>(queueSize, 1)),
      tupleCallback_(std::move(tupleCallback)),
      logCounters_(logCounters),
      cameraQueues_(cameraTopics.size()) {
  if (cameraTopics.empty()) {
    throw std::invalid_argument(
        "Synchronized camera input requires at least one camera topic");
  }
  if (!tupleCallback_) {
    throw std::invalid_argument(
        "Synchronized camera input requires a tuple callback");
  }

  counters_.receivedPerCamera.assign(cameraTopics.size(), 0);
  counters_.expiredPerCamera.assign(cameraTopics.size(), 0);
  subscriptions_.reserve(cameraTopics.size());

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(queueSize_)).reliable();
  for (std::size_t cameraIndex = 0; cameraIndex < cameraTopics.size();
       ++cameraIndex) {
    subscriptions_.push_back(node_->create_subscription<CompressedImage>(
        cameraTopics[cameraIndex], qos,
        [this, cameraIndex](const CompressedImage::ConstSharedPtr message) {
          imageReceived(message, cameraIndex);
        }));
  }

  if (logCounters_) {
    diagnosticsTimer_ = node_->create_wall_timer(
        std::chrono::seconds(10),
        [this]() { this->logCounters("periodic"); });
  }

  std::ostringstream topics;
  for (std::size_t cameraIndex = 0; cameraIndex < cameraTopics.size();
       ++cameraIndex) {
    if (cameraIndex != 0) {
      topics << ", ";
    }
    topics << cameraTopics[cameraIndex];
  }
  RCLCPP_INFO(node_->get_logger(),
              "Using exact-time synchronized compressed camera input for %zu "
              "cameras: [%s] (queue size %zu)",
              cameraTopics.size(), topics.str().c_str(), queueSize_);
}

SynchronizedCompressedImageSubscriber::~SynchronizedCompressedImageSubscriber() {
  if (diagnosticsTimer_) {
    diagnosticsTimer_->cancel();
  }
  if (logCounters_) {
    logCounters("final");
  }
}

SynchronizedCompressedImageSubscriber::Counters
SynchronizedCompressedImageSubscriber::counters() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return counters_;
}

void SynchronizedCompressedImageSubscriber::imageReceived(
    const CompressedImage::ConstSharedPtr& message,
    std::size_t cameraIndex) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.receivedPerCamera[cameraIndex];

  const Timestamp timestamp{message->header.stamp.sec,
                            message->header.stamp.nanosec};
  CameraQueue& cameraQueue = cameraQueues_[cameraIndex];
  cameraQueue[timestamp] = message;
  while (cameraQueue.size() > queueSize_) {
    cameraQueue.erase(cameraQueue.begin());
    ++counters_.expiredPerCamera[cameraIndex];
  }

  std::vector<CompressedImage::ConstSharedPtr> tuple;
  tuple.reserve(cameraQueues_.size());
  for (const CameraQueue& queue : cameraQueues_) {
    const auto iterator = queue.find(timestamp);
    if (iterator == queue.end()) {
      return;
    }
    tuple.push_back(iterator->second);
  }

  for (CameraQueue& queue : cameraQueues_) {
    queue.erase(timestamp);
  }
  ++counters_.matchedTuples;
  processTuple(tuple);
}

void SynchronizedCompressedImageSubscriber::processTuple(
    const std::vector<CompressedImage::ConstSharedPtr>& messages) {
  ImageTuple images;
  images.reserve(messages.size());
  try {
    for (const auto& message : messages) {
      const auto cvImage = cv_bridge::toCvCopy(message);
      Image::SharedPtr image = cvImage->toImageMsg();
      image->header = message->header;
      images.push_back(std::move(image));
    }
  } catch (const cv_bridge::Exception& exception) {
    ++counters_.decodeFailedTuples;
    RCLCPP_ERROR(node_->get_logger(),
                 "Dropping synchronized image tuple: decode failed: %s",
                 exception.what());
    return;
  } catch (const cv::Exception& exception) {
    ++counters_.decodeFailedTuples;
    RCLCPP_ERROR(node_->get_logger(),
                 "Dropping synchronized image tuple: OpenCV decode failed: %s",
                 exception.what());
    return;
  }

  ++counters_.decodeSuccessTuples;
  const bool tupleAddedWithoutInputLoss = tupleCallback_(images);
  if (tupleAddedWithoutInputLoss) {
    ++counters_.inputLossFreeTuples;
  } else {
    ++counters_.inputLossTuples;
  }
}

void SynchronizedCompressedImageSubscriber::logCounters(
    const char* phase) const {
  const Counters values = counters();
  std::ostringstream received;
  std::ostringstream expired;
  for (std::size_t cameraIndex = 0;
       cameraIndex < values.receivedPerCamera.size(); ++cameraIndex) {
    if (cameraIndex != 0) {
      received << ',';
      expired << ',';
    }
    received << values.receivedPerCamera[cameraIndex];
    expired << values.expiredPerCamera[cameraIndex];
  }
  RCLCPP_INFO(
      node_->get_logger(),
      "[SYNCHRONIZED_COMPRESSED_INPUT] phase=%s received_per_camera=[%s] "
      "expired_per_camera=[%s] matched_tuples=%llu "
      "decode_success_tuples=%llu decode_failed_tuples=%llu "
      "input_loss_free_tuples=%llu input_loss_tuples=%llu",
      phase, received.str().c_str(), expired.str().c_str(),
      static_cast<unsigned long long>(values.matchedTuples),
      static_cast<unsigned long long>(values.decodeSuccessTuples),
      static_cast<unsigned long long>(values.decodeFailedTuples),
      static_cast<unsigned long long>(values.inputLossFreeTuples),
      static_cast<unsigned long long>(values.inputLossTuples));
}

}  // namespace okvis
