#include <gtest/gtest.h>
#include <cmath>

#include "mission_restructurer.hpp"

using namespace arch_nav_oas;
using arch_nav::constants::ReferenceFrame;
using arch_nav::descriptor::WaypointOperationDescriptor;
using arch_nav::vehicle::Waypoint;

class MissionRestructurerTest : public ::testing::Test {
 protected:
  OasConfig config_ = OasConfig::defaults();
  std::unique_ptr<MissionRestructurer> restructurer_;

  void SetUp() override {
    restructurer_ = std::make_unique<MissionRestructurer>(config_);
  }

  SensorSnapshot make_snapshot(float front, float left, float right, float rear) {
    SensorSnapshot s;
    auto now = std::chrono::steady_clock::now();
    auto set = [&](SensorPosition pos, float range) {
      auto& r = s.at(pos);
      r.range = range;
      r.timestamp = now;
      r.valid = (range > 0.0f && range < 100.0f);
    };
    set(SensorPosition::FRONT, front);
    set(SensorPosition::LEFT, left);
    set(SensorPosition::RIGHT, right);
    set(SensorPosition::REAR, rear);
    return s;
  }

  ThreatAssessment make_assessment(ThreatClassification cls,
                                    double obs_x, double obs_y,
                                    double obs_dist, int blocked_wp) {
    ThreatAssessment a;
    a.has_threat = true;
    a.classification = cls;
    a.obstacle_x = obs_x;
    a.obstacle_y = obs_y;
    a.obstacle_distance = obs_dist;
    a.blocked_waypoint_index = blocked_wp;
    a.snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
    return a;
  }
};

TEST_F(MissionRestructurerTest, LateralThreat_NoRestructuring) {
  auto assessment = make_assessment(ThreatClassification::LATERAL, 0, 5, 5, 0);
  AvoidanceResult escape{true, -4, 0, -10, false};

  std::vector<Waypoint> wps = {{100, 0, -10}, {200, 0, -10}};
  WaypointOperationDescriptor desc(wps, ReferenceFrame::LOCAL_NED);

  auto result = restructurer_->restructure(
      assessment, escape, desc,
      assessment.snapshot, 0, 0, 0);

  EXPECT_FALSE(result.modified);
  EXPECT_FALSE(result.mission_blocked);
  EXPECT_EQ(result.supervisor_waypoints.size(), 1u);
}

TEST_F(MissionRestructurerTest, OnPath_InsertsbypassWaypoints) {
  auto assessment = make_assessment(ThreatClassification::ON_PATH, 50, 0, 50, 0);
  AvoidanceResult escape{true, -4, 0, -10, false};

  std::vector<Waypoint> wps = {{100, 0, -10}, {200, 0, -10}};
  WaypointOperationDescriptor desc(wps, ReferenceFrame::LOCAL_NED);

  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  auto result = restructurer_->restructure(
      assessment, escape, desc, snapshot, 0, 0, 0);

  ASSERT_TRUE(result.modified);
  ASSERT_NE(result.new_descriptor, nullptr);
  EXPECT_GT(result.new_descriptor->waypoints().size(), wps.size());
  EXPECT_FALSE(result.supervisor_waypoints.empty());
}

TEST_F(MissionRestructurerTest, OnWaypoint_ModifiesWaypointList) {
  auto assessment = make_assessment(ThreatClassification::ON_WAYPOINT, 100, 0, 100, 0);
  AvoidanceResult escape{true, -4, 0, -10, false};

  std::vector<Waypoint> wps = {{100, 0, -10}, {200, 0, -10}};
  WaypointOperationDescriptor desc(wps, ReferenceFrame::LOCAL_NED);

  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  auto result = restructurer_->restructure(
      assessment, escape, desc, snapshot, 0, 0, 0);

  ASSERT_TRUE(result.modified);
  ASSERT_NE(result.new_descriptor, nullptr);
  EXPECT_FALSE(result.supervisor_waypoints.empty());
}

TEST_F(MissionRestructurerTest, LastWaypointBlocked_MissionBlocked) {
  auto assessment = make_assessment(ThreatClassification::ON_WAYPOINT, 100, 0, 100, 0);
  AvoidanceResult escape{true, -4, 0, -10, false};

  std::vector<Waypoint> wps = {{100, 0, -10}};
  WaypointOperationDescriptor desc(wps, ReferenceFrame::LOCAL_NED);

  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  auto result = restructurer_->restructure(
      assessment, escape, desc, snapshot, 0, 0, 0);

  EXPECT_TRUE(result.modified);
}

TEST_F(MissionRestructurerTest, VersionIncremented) {
  auto assessment = make_assessment(ThreatClassification::ON_PATH, 50, 0, 50, 0);
  AvoidanceResult escape{true, -4, 0, -10, false};

  std::vector<Waypoint> wps = {{100, 0, -10}, {200, 0, -10}};
  WaypointOperationDescriptor desc(wps, ReferenceFrame::LOCAL_NED);
  uint32_t old_version = desc.version();

  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  auto result = restructurer_->restructure(
      assessment, escape, desc, snapshot, 0, 0, 0);

  ASSERT_TRUE(result.modified);
  ASSERT_NE(result.new_descriptor, nullptr);
  EXPECT_EQ(result.new_descriptor->version(), 0u);
}

TEST_F(MissionRestructurerTest, BypassPreservesAltitude) {
  auto assessment = make_assessment(ThreatClassification::ON_PATH, 50, 0, 50, 0);
  AvoidanceResult escape{true, -4, 0, -15, false};

  std::vector<Waypoint> wps = {{100, 0, -15}, {200, 0, -15}};
  WaypointOperationDescriptor desc(wps, ReferenceFrame::LOCAL_NED);

  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  auto result = restructurer_->restructure(
      assessment, escape, desc, snapshot, 0, 0, 0);

  ASSERT_TRUE(result.modified);
  for (const auto& wp : result.new_descriptor->waypoints()) {
    EXPECT_DOUBLE_EQ(wp.z, -15.0);
  }
}
