/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the conditions in the project
 *  license are met.
 *********************************************************************************/

#ifndef INCLUDE_OKVIS_SYNCHRONIZEDCOMPRESSEDIMAGESUBSCRIBER_HPP_
#define INCLUDE_OKVIS_SYNCHRONIZEDCOMPRESSEDIMAGESUBSCRIBER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace okvis {

/// Exact-time compressed-image ingress for a synchronized multicamera rig.
///
/// A complete timestamp tuple is decoded before it is passed atomically to the
/// estimator callback. The implementation is independent of camera count and
/// camera model.
class SynchronizedCompressedImageSubscriber {
 public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using Image = sensor_msgs::msg::Image;
  using ImageTuple = std::vector<Image::ConstSharedPtr>;
  using TupleCallback = std::function<bool(const ImageTuple&)>;

  struct Counters {
    std::vector<uint64_t> receivedPerCamera;
    std::vector<uint64_t> expiredPerCamera;
    uint64_t matchedTuples = 0;
    uint64_t decodeSuccessTuples = 0;
    uint64_t decodeFailedTuples = 0;
    uint64_t inputLossFreeTuples = 0;
    uint64_t inputLossTuples = 0;
  };

  SynchronizedCompressedImageSubscriber(
      const std::shared_ptr<rclcpp::Node>& node,
      const std::vector<std::string>& cameraTopics,
      std::size_t queueSize,
      TupleCallback tupleCallback,
      bool logCounters = false);

  ~SynchronizedCompressedImageSubscriber();

  /// Return a thread-safe snapshot of observable ingress state.
  Counters counters() const;

 private:
  struct Timestamp {
    int32_t second = 0;
    uint32_t nanosecond = 0;

    bool operator<(const Timestamp& other) const {
      if (second != other.second) {
        return second < other.second;
      }
      return nanosecond < other.nanosecond;
    }
  };

  using CameraQueue =
      std::map<Timestamp, CompressedImage::ConstSharedPtr>;

  void imageReceived(const CompressedImage::ConstSharedPtr& message,
                     std::size_t cameraIndex);
  void processTuple(
      const std::vector<CompressedImage::ConstSharedPtr>& messages);
  void logCounters(const char* phase) const;

  std::shared_ptr<rclcpp::Node> node_;
  std::size_t queueSize_;
  TupleCallback tupleCallback_;
  bool logCounters_ = false;
  std::vector<rclcpp::Subscription<CompressedImage>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr diagnosticsTimer_;

  // This mutex intentionally covers tuple decoding and estimator input as
  // well as queue access. It prevents two complete tuples from interleaving
  // under a multithreaded executor.
  mutable std::mutex mutex_;
  std::vector<CameraQueue> cameraQueues_;
  Counters counters_;
};

}  // namespace okvis

#endif  // INCLUDE_OKVIS_SYNCHRONIZEDCOMPRESSEDIMAGESUBSCRIBER_HPP_
