#include <gtest/gtest.h>
#include <cmath>

#include "avoidance_computer.hpp"

using namespace arch_nav_oas;

class AvoidanceComputerTest : public ::testing::Test {
 protected:
  OasConfig config_ = OasConfig::defaults();
  std::unique_ptr<AvoidanceComputer> computer_;

  void SetUp() override {
    computer_ = std::make_unique<AvoidanceComputer>(config_);
  }

  ThreatVector make_threat(float front, float left, float right, float rear) {
    ThreatVector t;
    t.has_threat = true;
    auto now = std::chrono::steady_clock::now();
    auto set = [&](SensorPosition pos, float range) {
      auto& r = t.snapshot.at(pos);
      r.range = range;
      r.timestamp = now;
      r.valid = (range > 0.0f && range < 100.0f);
    };
    set(SensorPosition::FRONT, front);
    set(SensorPosition::LEFT, left);
    set(SensorPosition::RIGHT, right);
    set(SensorPosition::REAR, rear);
    return t;
  }
};

TEST_F(AvoidanceComputerTest, FrontalObstacle_EscapesBackward) {
  auto threat = make_threat(1.0f, 100.0f, 100.0f, 100.0f);
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, 0.0);

  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.fallback_stop);
  EXPECT_LT(result.escape_x, 0.0);
  EXPECT_NEAR(result.escape_y, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(result.escape_z, -10.0);
}

TEST_F(AvoidanceComputerTest, LeftObstacle_EscapesRight) {
  auto threat = make_threat(100.0f, 1.0f, 100.0f, 100.0f);
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, 0.0);

  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.fallback_stop);
  EXPECT_LT(result.escape_y, 0.0);
}

TEST_F(AvoidanceComputerTest, RightObstacle_EscapesLeft) {
  auto threat = make_threat(100.0f, 100.0f, 1.0f, 100.0f);
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, 0.0);

  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.fallback_stop);
  EXPECT_GT(result.escape_y, 0.0);
}

TEST_F(AvoidanceComputerTest, OpposingForces_FallbackStop) {
  auto threat = make_threat(1.0f, 100.0f, 100.0f, 1.0f);
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fallback_stop);
}

TEST_F(AvoidanceComputerTest, NoValidReadings_FallbackStop) {
  ThreatVector threat;
  threat.has_threat = true;
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fallback_stop);
}

TEST_F(AvoidanceComputerTest, AllSensorsOutOfRange_FallbackStop) {
  auto threat = make_threat(100.0f, 100.0f, 100.0f, 100.0f);
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fallback_stop);
}

TEST_F(AvoidanceComputerTest, EscapeWithHeading_RotatesForces) {
  auto threat = make_threat(1.0f, 100.0f, 100.0f, 100.0f);
  double heading = M_PI_2;
  auto result = computer_->compute_escape(threat, 0.0, 0.0, -10.0, heading);

  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.fallback_stop);
  EXPECT_NEAR(result.escape_x, 0.0, 0.1);
  EXPECT_LT(result.escape_y, 0.0);
}

TEST_F(AvoidanceComputerTest, EscapeDistanceClamped) {
  auto threat = make_threat(1.0f, 100.0f, 100.0f, 100.0f);
  auto result = computer_->compute_escape_with_escalation(
      threat, 0.0, 0.0, -10.0, 0.0, 100.0);

  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.fallback_stop);
  double dist = std::sqrt(result.escape_x * result.escape_x +
                           result.escape_y * result.escape_y);
  EXPECT_LE(dist, config_.avoidance.d_escape_max + 0.01);
}

TEST_F(AvoidanceComputerTest, PreservesAltitude) {
  auto threat = make_threat(2.0f, 100.0f, 100.0f, 100.0f);
  auto result = computer_->compute_escape(threat, 10.0, 20.0, -15.0, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.escape_z, -15.0);
}
