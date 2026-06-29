#ifndef ARCH_NAV_OAS_RANGEFINDER_SENSOR_HUB_HPP_
#define ARCH_NAV_OAS_RANGEFINDER_SENSOR_HUB_HPP_

#include <atomic>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "oas_config.hpp"
#include "oas_types.hpp"

namespace arch_nav_oas {

class RangefinderSensorHub {
 public:
  explicit RangefinderSensorHub(const OasConfig& config);

  void start();
  void stop();

  SensorSnapshot get_snapshot() const;

 private:
  void on_range_received(const sensor_msgs::msg::Range::SharedPtr msg,
                         SensorPosition position);

  OasConfig config_;

  rclcpp::Node::SharedPtr node_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
             kNumSensors> subscriptions_;

  mutable std::mutex snapshot_mutex_;
  SensorSnapshot snapshot_;

  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_RANGEFINDER_SENSOR_HUB_HPP_
