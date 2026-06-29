#include "mission_restructurer.hpp"

#include <cmath>
#include <algorithm>

namespace arch_nav_oas {

MissionRestructurer::MissionRestructurer(const OasConfig& config)
    : config_(config) {}

int MissionRestructurer::choose_bypass_side(
    const SensorSnapshot& snapshot,
    double heading_rad,
    double mline_angle_ned) const {

  double perp_left_ned = mline_angle_ned + M_PI_2;
  double perp_right_ned = mline_angle_ned - M_PI_2;

  double best_left_clearance = 0.0;
  double best_right_clearance = 0.0;

  for (std::size_t i = 0; i < kNumSensors; ++i) {
    const auto& reading = snapshot.readings[i];
    double sensor_ned = heading_rad + config_.sensor_angles[i];

    double diff_left = std::abs(std::atan2(
        std::sin(sensor_ned - perp_left_ned),
        std::cos(sensor_ned - perp_left_ned)));
    double diff_right = std::abs(std::atan2(
        std::sin(sensor_ned - perp_right_ned),
        std::cos(sensor_ned - perp_right_ned)));

    double clearance = reading.valid ? reading.range : config_.apf.d0;

    if (diff_left < M_PI_2) {
      best_left_clearance = std::max(best_left_clearance, clearance * std::cos(diff_left));
    }
    if (diff_right < M_PI_2) {
      best_right_clearance = std::max(best_right_clearance, clearance * std::cos(diff_right));
    }
  }

  return (best_left_clearance >= best_right_clearance) ? 1 : -1;
}

std::vector<arch_nav::vehicle::Waypoint>
MissionRestructurer::compute_bypass_waypoints(
    double obs_x, double obs_y,
    double vehicle_x, double vehicle_y,
    double waypoint_x, double waypoint_y,
    const SensorSnapshot& snapshot,
    double heading_rad) const {

  double dx = waypoint_x - vehicle_x;
  double dy = waypoint_y - vehicle_y;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 1e-6) return {};

  double mline_angle = std::atan2(dy, dx);
  double mline_dir_x = dx / dist;
  double mline_dir_y = dy / dist;

  int side = choose_bypass_side(snapshot, heading_rad, mline_angle);

  double perp_x = -mline_dir_y * side;
  double perp_y = mline_dir_x * side;

  double d_safety = std::max(config_.bypass.d_safety, config_.bypass.d_safety_min);

  double bypass_x = obs_x + d_safety * perp_x;
  double bypass_y = obs_y + d_safety * perp_y;

  std::vector<arch_nav::vehicle::Waypoint> bypass_wps;
  bypass_wps.emplace_back(bypass_x, bypass_y, 0.0);

  return bypass_wps;
}

MissionRestructurer::RestructureResult MissionRestructurer::restructure(
    const ThreatAssessment& assessment,
    const AvoidanceResult& escape,
    const arch_nav::descriptor::WaypointOperationDescriptor& current_desc,
    const SensorSnapshot& snapshot,
    double vehicle_x, double vehicle_y, double heading_rad) const {

  RestructureResult result;

  if (assessment.classification == ThreatClassification::LATERAL ||
      assessment.classification == ThreatClassification::NONE) {
    result.modified = false;
    if (escape.valid && !escape.fallback_stop) {
      result.supervisor_waypoints.emplace_back(
          escape.escape_x, escape.escape_y, escape.escape_z);
    }
    return result;
  }

  const auto& waypoints = current_desc.waypoints();
  auto frame = current_desc.frame();
  int current_wp = current_desc.progress().current_waypoint.load();

  if (current_wp < 0 || current_wp >= static_cast<int>(waypoints.size())) {
    result.mission_blocked = true;
    return result;
  }

  double wp_x = waypoints[current_wp].x;
  double wp_y = waypoints[current_wp].y;
  double wp_z = waypoints[current_wp].z;

  auto bypass_wps = compute_bypass_waypoints(
      assessment.obstacle_x, assessment.obstacle_y,
      vehicle_x, vehicle_y, wp_x, wp_y,
      snapshot, heading_rad);

  if (bypass_wps.empty()) {
    if (current_wp == static_cast<int>(waypoints.size()) - 1) {
      result.mission_blocked = true;
      return result;
    }
  }

  for (auto& bp : bypass_wps) {
    bp.z = wp_z;
  }

  std::vector<arch_nav::vehicle::Waypoint> new_waypoints;
  for (int i = 0; i < current_wp; ++i) {
    new_waypoints.push_back(waypoints[i]);
  }

  if (assessment.classification == ThreatClassification::ON_PATH) {
    for (const auto& bp : bypass_wps) {
      new_waypoints.push_back(bp);
    }
    for (int i = current_wp; i < static_cast<int>(waypoints.size()); ++i) {
      new_waypoints.push_back(waypoints[i]);
    }
  } else {
    if (!bypass_wps.empty()) {
      new_waypoints.push_back(bypass_wps.front());
    }
    for (int i = current_wp + 1; i < static_cast<int>(waypoints.size()); ++i) {
      new_waypoints.push_back(waypoints[i]);
    }
  }

  auto new_desc = std::make_shared<
      arch_nav::descriptor::WaypointOperationDescriptor>(
      new_waypoints, frame);
  new_desc->progress().current_waypoint.store(current_wp);

  result.modified = true;
  result.new_descriptor = new_desc;

  if (escape.valid && !escape.fallback_stop) {
    result.supervisor_waypoints.emplace_back(
        escape.escape_x, escape.escape_y, escape.escape_z);
  }
  for (const auto& bp : bypass_wps) {
    result.supervisor_waypoints.push_back(bp);
  }

  return result;
}

}  // namespace arch_nav_oas
