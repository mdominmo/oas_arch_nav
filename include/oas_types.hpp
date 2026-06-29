#ifndef ARCH_NAV_OAS_TYPES_HPP_
#define ARCH_NAV_OAS_TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>

namespace arch_nav_oas {

enum class SensorPosition : uint8_t { FRONT = 0, LEFT = 1, RIGHT = 2, REAR = 3 };

static constexpr std::size_t kNumSensors = 4;

struct RangeReading {
  float range{0.0f};
  std::chrono::steady_clock::time_point timestamp{};
  bool valid{false};
};

struct SensorSnapshot {
  std::array<RangeReading, kNumSensors> readings{};

  const RangeReading& at(SensorPosition pos) const {
    return readings[static_cast<uint8_t>(pos)];
  }
  RangeReading& at(SensorPosition pos) {
    return readings[static_cast<uint8_t>(pos)];
  }
};

enum class ThreatClassification : uint8_t {
  NONE,
  LATERAL,
  ON_PATH,
  ON_WAYPOINT
};

struct ThreatAssessment {
  SensorSnapshot snapshot;
  ThreatClassification classification{ThreatClassification::NONE};
  bool has_threat{false};
  int blocked_waypoint_index{-1};
  double obstacle_distance{0.0};
  double obstacle_x{0.0};
  double obstacle_y{0.0};
};

enum class OasState : uint8_t {
  MONITORING,
  TRIGGERED,
  AVOIDING,
  STOPPED
};

struct AvoidanceResult {
  bool valid{false};
  double escape_x{0.0};
  double escape_y{0.0};
  double escape_z{0.0};
  bool fallback_stop{false};
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_TYPES_HPP_
