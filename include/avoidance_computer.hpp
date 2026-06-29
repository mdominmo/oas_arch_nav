#ifndef ARCH_NAV_OAS_AVOIDANCE_COMPUTER_HPP_
#define ARCH_NAV_OAS_AVOIDANCE_COMPUTER_HPP_

#include "oas_config.hpp"
#include "oas_types.hpp"

namespace arch_nav_oas {

class AvoidanceComputer {
 public:
  explicit AvoidanceComputer(const OasConfig& config);

  AvoidanceResult compute_escape(
      const ThreatVector& threat,
      double current_x, double current_y, double current_z,
      double heading_rad) const;

  AvoidanceResult compute_escape_with_escalation(
      const ThreatVector& threat,
      double current_x, double current_y, double current_z,
      double heading_rad,
      double escalation_multiplier) const;

 private:
  struct Force2D {
    double fx{0.0};
    double fy{0.0};
  };

  Force2D compute_repulsive_force(double distance, double sensor_angle,
                                  double heading) const;

  OasConfig config_;
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_AVOIDANCE_COMPUTER_HPP_
