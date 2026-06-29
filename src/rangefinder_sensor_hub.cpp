#include "rangefinder_sensor_hub.hpp"

namespace arch_nav_oas {

RangefinderSensorHub::RangefinderSensorHub(const OasConfig& config)
    : config_(config) {}

void RangefinderSensorHub::start() {
  if (running_) return;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("oas_sensor_hub");

  auto make_callback = [this](SensorPosition pos) {
    return [this, pos](const sensor_msgs::msg::Range::SharedPtr msg) {
      on_range_received(msg, pos);
    };
  };

  subscriptions_[0] = node_->create_subscription<sensor_msgs::msg::Range>(
      config_.topics.front, 10, make_callback(SensorPosition::FRONT));
  subscriptions_[1] = node_->create_subscription<sensor_msgs::msg::Range>(
      config_.topics.left, 10, make_callback(SensorPosition::LEFT));
  subscriptions_[2] = node_->create_subscription<sensor_msgs::msg::Range>(
      config_.topics.right, 10, make_callback(SensorPosition::RIGHT));
  subscriptions_[3] = node_->create_subscription<sensor_msgs::msg::Range>(
      config_.topics.rear, 10, make_callback(SensorPosition::REAR));

  running_ = true;
  spin_thread_ = std::thread([this] {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    while (running_ && rclcpp::ok()) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
  });
}

void RangefinderSensorHub::stop() {
  running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  subscriptions_.fill(nullptr);
  node_.reset();
}

void RangefinderSensorHub::on_range_received(
    const sensor_msgs::msg::Range::SharedPtr msg,
    SensorPosition position) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  auto& reading = snapshot_.at(position);
  reading.range = msg->range;
  reading.timestamp = std::chrono::steady_clock::now();
  reading.valid = (msg->range >= msg->min_range && msg->range <= msg->max_range);
}

SensorSnapshot RangefinderSensorHub::get_snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

}  // namespace arch_nav_oas
