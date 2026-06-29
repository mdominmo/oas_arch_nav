#include "avoidance_computer.hpp"

#include <algorithm>
#include <cmath>

namespace arch_nav_oas {

AvoidanceComputer::AvoidanceComputer(const OasConfig& config)
    : config_(config) {}

AvoidanceComputer::Force2D AvoidanceComputer::compute_repulsive_force(
    double distance, double sensor_angle, double heading) const {
  double d0 = config_.apf.d0;
  if (distance >= d0 || distance <= 0.0) return {0.0, 0.0};

  double d = std::max(distance, config_.apf.d_min_clamp);

  double magnitude = config_.apf.eta * (1.0 / d - 1.0 / d0) / (d * d);

  double repulsive_body_x = -std::cos(sensor_angle);
  double repulsive_body_y = -std::sin(sensor_angle);

  double cos_h = std::cos(heading);
  double sin_h = std::sin(heading);
  double ned_x = cos_h * repulsive_body_x - sin_h * repulsive_body_y;
  double ned_y = sin_h * repulsive_body_x + cos_h * repulsive_body_y;

  return {magnitude * ned_x, magnitude * ned_y};
}

AvoidanceResult AvoidanceComputer::compute_escape(
    const ThreatVector& threat,
    double current_x, double current_y, double current_z,
    double heading_rad) const {
  return compute_escape_with_escalation(
      threat, current_x, current_y, current_z, heading_rad, 1.0);
}

AvoidanceResult AvoidanceComputer::compute_escape_with_escalation(
    const ThreatVector& threat,
    double current_x, double current_y, double current_z,
    double heading_rad, double escalation_multiplier) const {
  Force2D total{0.0, 0.0};

  for (std::size_t i = 0; i < kNumSensors; ++i) {
    const auto& reading = threat.snapshot.readings[i];
    if (!reading.valid) continue;

    auto force = compute_repulsive_force(
        reading.range, config_.sensor_angles[i], heading_rad);
    total.fx += force.fx;
    total.fy += force.fy;
  }

  double magnitude = std::sqrt(total.fx * total.fx + total.fy * total.fy);

  if (magnitude < config_.avoidance.f_min_threshold) {
    return {true, current_x, current_y, current_z, true};
  }

  double dir_x = total.fx / magnitude;
  double dir_y = total.fy / magnitude;

  double d_escape = config_.avoidance.d_escape * escalation_multiplier;
  d_escape = std::clamp(d_escape,
                        config_.avoidance.d_escape_min,
                        config_.avoidance.d_escape_max);

  return {
      true,
      current_x + d_escape * dir_x,
      current_y + d_escape * dir_y,
      current_z,
      false
  };
}

}  // namespace arch_nav_oas
