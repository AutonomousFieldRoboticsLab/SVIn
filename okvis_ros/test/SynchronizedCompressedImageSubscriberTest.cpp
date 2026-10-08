#include <okvis/SynchronizedCompressedImageSubscriber.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace {

using namespace std::chrono_literals;

sensor_msgs::msg::CompressedImage makeCompressedImage(int32_t second,
                                                       uint32_t nanosecond,
                                                       uint8_t value) {
  cv::Mat image(8, 8, CV_8UC1, cv::Scalar(value));
  sensor_msgs::msg::CompressedImage message;
  message.header.stamp.sec = second;
  message.header.stamp.nanosec = nanosecond;
  message.format = "jpeg";
  EXPECT_TRUE(cv::imencode(".jpg", image, message.data));
  return message;
}

sensor_msgs::msg::CompressedImage makeInvalidCompressedImage(int32_t second) {
  sensor_msgs::msg::CompressedImage message;
  message.header.stamp.sec = second;
  message.format = "jpeg";
  return message;
}

TEST(SynchronizedCompressedImageSubscriber,
     DeliversOnlyCompleteExactTuplesInCameraOrder) {
  rclcpp::init(0, nullptr);
  auto node =
      std::make_shared<rclcpp::Node>("synchronized_compressed_input_test");
  const std::vector<std::string> topics{
      "/test/camera0/compressed", "/test/camera1/compressed",
      "/test/camera2/compressed"};

  std::mutex mutex;
  std::condition_variable callbackCondition;
  std::vector<std::pair<unsigned int, builtin_interfaces::msg::Time>> callbacks;
  std::size_t callbackInvocations = 0;
  okvis::SynchronizedCompressedImageSubscriber subscriber(
      node, topics, 2,
      [&](const okvis::SynchronizedCompressedImageSubscriber::ImageTuple& images) {
        std::lock_guard<std::mutex> lock(mutex);
        ++callbackInvocations;
        for (std::size_t cameraIndex = 0; cameraIndex < images.size(); ++cameraIndex) {
          callbacks.emplace_back(cameraIndex, images[cameraIndex]->header.stamp);
        }
        callbackCondition.notify_all();
        return images.front()->header.stamp.sec != 50;
      });

  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr>
      publishers;
  publishers.reserve(topics.size());
  for (const std::string& topic : topics) {
    publishers.push_back(
        node->create_publisher<sensor_msgs::msg::CompressedImage>(topic, 10));
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinThread([&executor]() { executor.spin(); });

  const auto discoveryDeadline = std::chrono::steady_clock::now() + 2s;
  while ((publishers[0]->get_subscription_count() == 0 ||
          publishers[1]->get_subscription_count() == 0 ||
          publishers[2]->get_subscription_count() == 0) &&
         std::chrono::steady_clock::now() < discoveryDeadline) {
    std::this_thread::sleep_for(10ms);
  }
  const bool discovered =
      publishers[0]->get_subscription_count() == 1u &&
      publishers[1]->get_subscription_count() == 1u &&
      publishers[2]->get_subscription_count() == 1u;
  EXPECT_TRUE(discovered);

  if (discovered) {
    publishers[1]->publish(makeCompressedImage(10, 123u, 80u));
    publishers[0]->publish(makeCompressedImage(10, 123u, 40u));
    publishers[2]->publish(makeCompressedImage(10, 123u, 120u));
  }

  bool receivedTuple = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    receivedTuple = callbackCondition.wait_for(
        lock, 2s, [&callbacks]() { return callbacks.size() == 3; });
    EXPECT_TRUE(receivedTuple);
    if (receivedTuple) {
      EXPECT_EQ(callbackInvocations, 1u);
      for (std::size_t cameraIndex = 0; cameraIndex < 3; ++cameraIndex) {
        EXPECT_EQ(callbacks[cameraIndex].first, cameraIndex);
        EXPECT_EQ(callbacks[cameraIndex].second.sec, 10);
        EXPECT_EQ(callbacks[cameraIndex].second.nanosec, 123u);
      }
    }
  }

  if (discovered && receivedTuple) {
    // Different timestamps must not form a tuple.
    publishers[0]->publish(makeCompressedImage(20, 0u, 40u));
    publishers[1]->publish(makeCompressedImage(21, 0u, 80u));
    publishers[2]->publish(makeCompressedImage(22, 0u, 120u));
    std::this_thread::sleep_for(100ms);
    {
      std::lock_guard<std::mutex> lock(mutex);
      EXPECT_EQ(callbacks.size(), 3u);
    }

    // A second complete tuple must also be delivered in index order.
    for (std::size_t cameraIndex = 0; cameraIndex < 3; ++cameraIndex) {
      publishers[cameraIndex]->publish(makeCompressedImage(
          30, 0u, static_cast<uint8_t>(40u * (cameraIndex + 1))));
    }
    const auto secondTupleDeadline = std::chrono::steady_clock::now() + 2s;
    while (subscriber.counters().inputLossFreeTuples != 2u &&
           std::chrono::steady_clock::now() < secondTupleDeadline) {
      std::this_thread::sleep_for(10ms);
    }

    // Add another unmatched timestamp to each camera. Adding timestamp 50 then
    // expires the oldest unmatched message because the queue size is two.
    publishers[0]->publish(makeCompressedImage(40, 0u, 40u));
    publishers[1]->publish(makeCompressedImage(41, 0u, 80u));
    publishers[2]->publish(makeCompressedImage(42, 0u, 120u));
    for (std::size_t cameraIndex = 0; cameraIndex < 3; ++cameraIndex) {
      publishers[cameraIndex]->publish(makeCompressedImage(
          50, 0u, static_cast<uint8_t>(40u * (cameraIndex + 1))));
    }
    const auto inputLossDeadline = std::chrono::steady_clock::now() + 2s;
    while (subscriber.counters().inputLossTuples != 1u &&
           std::chrono::steady_clock::now() < inputLossDeadline) {
      std::this_thread::sleep_for(10ms);
    }

    // A complete corrupt tuple is matched but no image is admitted.
    for (const auto& publisher : publishers) {
      publisher->publish(makeInvalidCompressedImage(60));
    }
    const auto decodeDeadline = std::chrono::steady_clock::now() + 2s;
    while (subscriber.counters().decodeFailedTuples != 1u &&
           std::chrono::steady_clock::now() < decodeDeadline) {
      std::this_thread::sleep_for(10ms);
    }

    const auto counters = subscriber.counters();
    ASSERT_EQ(counters.receivedPerCamera.size(), 3u);
    ASSERT_EQ(counters.expiredPerCamera.size(), 3u);
    for (std::size_t cameraIndex = 0; cameraIndex < 3; ++cameraIndex) {
      EXPECT_EQ(counters.receivedPerCamera[cameraIndex], 6u);
      EXPECT_EQ(counters.expiredPerCamera[cameraIndex], 1u);
    }
    EXPECT_EQ(counters.matchedTuples, 4u);
    EXPECT_EQ(counters.decodeSuccessTuples, 3u);
    EXPECT_EQ(counters.decodeFailedTuples, 1u);
    EXPECT_EQ(counters.inputLossFreeTuples, 2u);
    EXPECT_EQ(counters.inputLossTuples, 1u);
  }

  executor.cancel();
  spinThread.join();
  rclcpp::shutdown();
}

}  // namespace
