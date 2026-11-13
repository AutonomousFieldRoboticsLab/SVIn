#include "svin_rerun_bridge/svin_rerun_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<svin_rerun_bridge::SvinRerunBridge>());
  rclcpp::shutdown();
  return 0;
}
