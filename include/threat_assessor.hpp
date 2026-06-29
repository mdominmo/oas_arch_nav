#ifndef ARCH_NAV_OAS_THREAT_ASSESSOR_HPP_
#define ARCH_NAV_OAS_THREAT_ASSESSOR_HPP_

#include <array>
#include <chrono>

#include "oas_config.hpp"
#include "oas_types.hpp"

namespace arch_nav_oas {

class ThreatAssessor {
 public:
  explicit ThreatAssessor(const OasConfig& config);

  ThreatAssessment assess(const SensorSnapshot& snapshot,
                          double vehicle_x, double vehicle_y,
                          double heading_rad,
                          double waypoint_x, double waypoint_y);

  void notify_avoidance_complete();

  void reset();

 private:
  bool is_in_cooldown() const;
  void apply_staleness(SensorSnapshot& snapshot) const;
  ThreatClassification classify(const SensorSnapshot& snapshot,
                                double vehicle_x, double vehicle_y,
                                double heading_rad,
                                double waypoint_x, double waypoint_y,
                                double& out_obs_dist,
                                double& out_obs_x, double& out_obs_y) const;

  OasConfig config_;

  std::array<int, kNumSensors> activate_counters_{};
  std::array<int, kNumSensors> deactivate_counters_{};
  std::array<bool, kNumSensors> sensor_active_{};

  bool threat_active_{false};
  std::chrono::steady_clock::time_point last_avoidance_complete_{};
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_THREAT_ASSESSOR_HPP_
