#include "threat_assessor.hpp"

#include <cmath>

namespace arch_nav_oas {

ThreatAssessor::ThreatAssessor(const OasConfig& config)
    : config_(config) {}

void ThreatAssessor::apply_staleness(SensorSnapshot& snapshot) const {
  auto now = std::chrono::steady_clock::now();
  auto stale_limit = std::chrono::milliseconds(config_.assessment.sensor_stale_ms);

  for (auto& reading : snapshot.readings) {
    if (reading.valid && (now - reading.timestamp) > stale_limit) {
      reading.valid = false;
    }
  }
}

ThreatClassification ThreatAssessor::classify(
    const SensorSnapshot& snapshot,
    double vehicle_x, double vehicle_y,
    double heading_rad,
    double waypoint_x, double waypoint_y,
    double& out_obs_dist,
    double& out_obs_x, double& out_obs_y) const {

  double dx = waypoint_x - vehicle_x;
  double dy = waypoint_y - vehicle_y;
  double dist_to_wp = std::sqrt(dx * dx + dy * dy);
  if (dist_to_wp < 1e-6) return ThreatClassification::LATERAL;

  double mline_angle_ned = std::atan2(dy, dx);

  double best_alignment = config_.assessment.path_alignment_threshold_rad;
  int best_sensor = -1;

  for (std::size_t i = 0; i < kNumSensors; ++i) {
    if (!snapshot.readings[i].valid) continue;
    if (snapshot.readings[i].range >= config_.apf.d0) continue;

    double sensor_angle_ned = heading_rad + config_.sensor_angles[i];
    double diff = std::atan2(
        std::sin(sensor_angle_ned - mline_angle_ned),
        std::cos(sensor_angle_ned - mline_angle_ned));

    if (std::abs(diff) < best_alignment) {
      best_alignment = std::abs(diff);
      best_sensor = static_cast<int>(i);
    }
  }

  if (best_sensor < 0) return ThreatClassification::LATERAL;

  double d_obs = snapshot.readings[best_sensor].range;
  double sensor_angle_ned =
      heading_rad + config_.sensor_angles[best_sensor];
  out_obs_dist = d_obs;
  out_obs_x = vehicle_x + d_obs * std::cos(sensor_angle_ned);
  out_obs_y = vehicle_y + d_obs * std::sin(sensor_angle_ned);

  if (std::abs(d_obs - dist_to_wp) < config_.assessment.on_waypoint_tolerance) {
    return ThreatClassification::ON_WAYPOINT;
  }

  if (d_obs < dist_to_wp) {
    return ThreatClassification::ON_PATH;
  }

  return ThreatClassification::LATERAL;
}

ThreatAssessment ThreatAssessor::assess(
    const SensorSnapshot& snapshot,
    double vehicle_x, double vehicle_y,
    double heading_rad,
    double waypoint_x, double waypoint_y) {

  ThreatAssessment result;
  result.snapshot = snapshot;

  apply_staleness(result.snapshot);

  if (is_in_cooldown()) return result;

  bool any_active = false;
  for (std::size_t i = 0; i < kNumSensors; ++i) {
    const auto& reading = result.snapshot.readings[i];
    if (!reading.valid) {
      activate_counters_[i] = 0;
      continue;
    }

    if (reading.range < config_.assessment.d_activate) {
      activate_counters_[i]++;
      deactivate_counters_[i] = 0;
    } else if (reading.range > config_.assessment.d_deactivate) {
      deactivate_counters_[i]++;
      activate_counters_[i] = 0;
    } else {
      activate_counters_[i] = 0;
      deactivate_counters_[i] = 0;
    }

    if (sensor_active_[i]) {
      if (deactivate_counters_[i] >= config_.assessment.n_deactivate) {
        sensor_active_[i] = false;
      }
    } else {
      if (activate_counters_[i] >= config_.assessment.n_activate) {
        sensor_active_[i] = true;
      }
    }

    if (sensor_active_[i]) any_active = true;
  }

  if (!any_active) {
    threat_active_ = false;
    return result;
  }

  threat_active_ = true;
  result.has_threat = true;

  result.classification = classify(
      result.snapshot, vehicle_x, vehicle_y, heading_rad,
      waypoint_x, waypoint_y,
      result.obstacle_distance, result.obstacle_x, result.obstacle_y);

  return result;
}

bool ThreatAssessor::is_in_cooldown() const {
  if (last_avoidance_complete_ == std::chrono::steady_clock::time_point{})
    return false;

  auto elapsed = std::chrono::steady_clock::now() - last_avoidance_complete_;
  return elapsed < std::chrono::milliseconds(config_.cooldown_ms);
}

void ThreatAssessor::notify_avoidance_complete() {
  last_avoidance_complete_ = std::chrono::steady_clock::now();
  threat_active_ = false;
  activate_counters_.fill(0);
  deactivate_counters_.fill(0);
  sensor_active_.fill(false);
}

void ThreatAssessor::reset() {
  threat_active_ = false;
  activate_counters_.fill(0);
  deactivate_counters_.fill(0);
  sensor_active_.fill(false);
  last_avoidance_complete_ = {};
}

}  // namespace arch_nav_oas
