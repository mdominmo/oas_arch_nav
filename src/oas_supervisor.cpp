#include "oas_supervisor.hpp"

#include <chrono>

#include "arch_nav/constants/operation_type.hpp"
#include "arch_nav/controller/preemption_info.hpp"
#include "arch_nav/descriptor/waypoint_operation_descriptor.hpp"
#include "arch_nav/supervisor/supervisor_chain.hpp"

namespace arch_nav_oas {

OasSupervisor::OasSupervisor(const std::string& config_path)
    : config_(OasConfig::load(config_path)),
      avoidance_computer_(config_),
      threat_assessor_(config_),
      mission_restructurer_(config_),
      sensor_hub_(config_) {}

void OasSupervisor::start(
    arch_nav::context::IVehicleContextReader& vehicle_reader,
    arch_nav::context::IOperationContextReader& operation_reader,
    arch_nav::supervisor::SupervisorChain& chain) {
  vehicle_reader_ = &vehicle_reader;
  operation_reader_ = &operation_reader;
  chain_ = &chain;

  chain.register_supervisor(
      *this, config_.priority,
      arch_nav::controller::PreemptionType::TRANSIENT);

  sensor_hub_.start();

  state_.store(OasState::MONITORING);
  running_ = true;
  monitor_thread_ = std::thread([this] { monitor_loop(); });
}

void OasSupervisor::stop() {
  running_ = false;
  state_.store(OasState::STOPPED);
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
  sensor_hub_.stop();
}

bool OasSupervisor::is_waypoint_following_active() const {
  auto desc = operation_reader_->current_descriptor();
  return desc &&
         desc->operation_type() ==
             arch_nav::constants::OperationType::WAYPOINT_FOLLOWING;
}

void OasSupervisor::monitor_loop() {
  while (running_) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.assessment.poll_interval_ms));
    if (!running_) break;

    if (state_.load() != OasState::MONITORING) continue;

    if (!is_waypoint_following_active()) continue;

    auto desc = operation_reader_->current_descriptor();
    if (!desc) continue;

    auto* wp_desc = dynamic_cast<
        const arch_nav::descriptor::WaypointOperationDescriptor*>(desc.get());
    if (!wp_desc) continue;

    int current_wp = wp_desc->progress().current_waypoint.load();
    const auto& waypoints = wp_desc->waypoints();
    if (current_wp < 0 || current_wp >= static_cast<int>(waypoints.size()))
      continue;

    auto kinematics = vehicle_reader_->get_kinematic();
    if (!kinematics.is_valid()) continue;

    auto snapshot = sensor_hub_.get_snapshot();

    auto assessment = threat_assessor_.assess(
        snapshot,
        kinematics.x, kinematics.y,
        kinematics.heading,
        waypoints[current_wp].x, waypoints[current_wp].y);

    if (!assessment.has_threat) continue;

    assessment.blocked_waypoint_index = current_wp;

    {
      std::lock_guard<std::mutex> lock(assessment_mutex_);
      current_assessment_ = assessment;
    }

    state_.store(OasState::TRIGGERED);

    chain_->request_control(
        *this,
        arch_nav::controller::PreemptionInfo{
            "oas",
            "Obstacle detected",
            ""});
  }
}

void OasSupervisor::execute(
    arch_nav::controller::IOperationalController& controller,
    arch_nav::context::IOperationContextWriter& operation_writer) {

  state_.store(OasState::AVOIDING);

  ThreatAssessment assessment;
  {
    std::lock_guard<std::mutex> lock(assessment_mutex_);
    assessment = current_assessment_;
  }

  auto kinematics = vehicle_reader_->get_kinematic();

  ThreatVector threat;
  threat.snapshot = assessment.snapshot;
  threat.has_threat = true;

  auto escape = avoidance_computer_.compute_escape(
      threat, kinematics.x, kinematics.y, kinematics.z,
      kinematics.heading);

  if (escape.fallback_stop) {
    controller.stop();
    threat_assessor_.notify_avoidance_complete();
    state_.store(OasState::MONITORING);
    chain_->release_control(*this);
    return;
  }

  if (assessment.classification == ThreatClassification::ON_PATH ||
      assessment.classification == ThreatClassification::ON_WAYPOINT) {

    auto desc = operation_reader_->current_descriptor();
    auto* wp_desc = dynamic_cast<
        const arch_nav::descriptor::WaypointOperationDescriptor*>(desc.get());

    if (wp_desc) {
      auto restructure_result = mission_restructurer_.restructure(
          assessment, escape, *wp_desc, assessment.snapshot,
          kinematics.x, kinematics.y, kinematics.heading);

      if (restructure_result.mission_blocked) {
        controller.stop();
        threat_assessor_.notify_avoidance_complete();
        state_.store(OasState::MONITORING);
        chain_->release_control(*this);
        return;
      }

      if (restructure_result.modified && restructure_result.new_descriptor) {
        operation_writer.set_current_descriptor(
            restructure_result.new_descriptor);
      }

      if (!restructure_result.supervisor_waypoints.empty()) {
        controller.waypoint_following(
            restructure_result.supervisor_waypoints,
            arch_nav::constants::ReferenceFrame::LOCAL_NED);
      }

      threat_assessor_.notify_avoidance_complete();
      state_.store(OasState::MONITORING);
      chain_->release_control(*this);
      return;
    }
  }

  controller.waypoint_following(
      {arch_nav::vehicle::Waypoint{escape.escape_x, escape.escape_y, escape.escape_z}},
      arch_nav::constants::ReferenceFrame::LOCAL_NED);

  threat_assessor_.notify_avoidance_complete();
  state_.store(OasState::MONITORING);
  chain_->release_control(*this);
}

}  // namespace arch_nav_oas
