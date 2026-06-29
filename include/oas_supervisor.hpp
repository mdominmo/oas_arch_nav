#ifndef ARCH_NAV_OAS_SUPERVISOR_HPP_
#define ARCH_NAV_OAS_SUPERVISOR_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "arch_nav/supervisor/i_supervisor.hpp"

#include "avoidance_computer.hpp"
#include "mission_restructurer.hpp"
#include "oas_config.hpp"
#include "oas_types.hpp"
#include "rangefinder_sensor_hub.hpp"
#include "threat_assessor.hpp"

namespace arch_nav_oas {

class OasSupervisor : public arch_nav::supervisor::ISupervisor {
 public:
  explicit OasSupervisor(const std::string& config_path);

  void start(
      arch_nav::context::IVehicleContextReader& vehicle_reader,
      arch_nav::context::IOperationContextReader& operation_reader,
      arch_nav::supervisor::SupervisorChain& chain) override;

  void stop() override;

  void execute(
      arch_nav::controller::IOperationalController& controller,
      arch_nav::context::IOperationContextWriter& operation_writer) override;

 private:
  void monitor_loop();
  bool is_waypoint_following_active() const;

  OasConfig config_;
  AvoidanceComputer avoidance_computer_;
  ThreatAssessor threat_assessor_;
  MissionRestructurer mission_restructurer_;
  RangefinderSensorHub sensor_hub_;

  arch_nav::context::IVehicleContextReader* vehicle_reader_{nullptr};
  arch_nav::context::IOperationContextReader* operation_reader_{nullptr};
  arch_nav::supervisor::SupervisorChain* chain_{nullptr};

  std::atomic<OasState> state_{OasState::STOPPED};
  std::atomic<bool> running_{false};
  std::thread monitor_thread_;

  ThreatAssessment current_assessment_;
  std::mutex assessment_mutex_;
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_SUPERVISOR_HPP_
