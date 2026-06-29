#ifndef ARCH_NAV_OAS_MISSION_RESTRUCTURER_HPP_
#define ARCH_NAV_OAS_MISSION_RESTRUCTURER_HPP_

#include <memory>
#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/descriptor/waypoint_operation_descriptor.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"

#include "oas_config.hpp"
#include "oas_types.hpp"

namespace arch_nav_oas {

class MissionRestructurer {
 public:
  explicit MissionRestructurer(const OasConfig& config);

  struct RestructureResult {
    bool modified{false};
    bool mission_blocked{false};
    std::shared_ptr<arch_nav::descriptor::WaypointOperationDescriptor>
        new_descriptor;
    std::vector<arch_nav::vehicle::Waypoint> supervisor_waypoints;
  };

  RestructureResult restructure(
      const ThreatAssessment& assessment,
      const AvoidanceResult& escape,
      const arch_nav::descriptor::WaypointOperationDescriptor& current_desc,
      const SensorSnapshot& snapshot,
      double vehicle_x, double vehicle_y, double heading_rad) const;

 private:
  std::vector<arch_nav::vehicle::Waypoint> compute_bypass_waypoints(
      double obs_x, double obs_y,
      double vehicle_x, double vehicle_y,
      double waypoint_x, double waypoint_y,
      const SensorSnapshot& snapshot,
      double heading_rad) const;

  int choose_bypass_side(const SensorSnapshot& snapshot,
                         double heading_rad,
                         double mline_angle_ned) const;

  OasConfig config_;
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_MISSION_RESTRUCTURER_HPP_
