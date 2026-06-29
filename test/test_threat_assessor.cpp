#include <gtest/gtest.h>
#include <cmath>

#include "threat_assessor.hpp"

using namespace arch_nav_oas;

class ThreatAssessorTest : public ::testing::Test {
 protected:
  OasConfig config_ = OasConfig::defaults();
  std::unique_ptr<ThreatAssessor> assessor_;

  void SetUp() override {
    assessor_ = std::make_unique<ThreatAssessor>(config_);
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
};

TEST_F(ThreatAssessorTest, NoThreat_AllSensorsFar) {
  auto snapshot = make_snapshot(10.0f, 10.0f, 10.0f, 10.0f);
  auto result = assessor_->assess(snapshot, 0, 0, 0, 100, 0);
  EXPECT_FALSE(result.has_threat);
}

TEST_F(ThreatAssessorTest, HysteresisRequiresConsecutiveSamples) {
  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);

  for (int i = 0; i < config_.assessment.n_activate - 1; ++i) {
    auto result = assessor_->assess(snapshot, 0, 0, 0, 100, 0);
    EXPECT_FALSE(result.has_threat)
        << "Should not trigger before n_activate samples, iteration=" << i;
  }

  auto result = assessor_->assess(snapshot, 0, 0, 0, 100, 0);
  EXPECT_TRUE(result.has_threat);
}

TEST_F(ThreatAssessorTest, DeactivationRequiresConsecutiveSamples) {
  auto close = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  for (int i = 0; i < config_.assessment.n_activate; ++i) {
    assessor_->assess(close, 0, 0, 0, 100, 0);
  }

  auto far = make_snapshot(10.0f, 10.0f, 10.0f, 10.0f);
  for (int i = 0; i < config_.assessment.n_deactivate - 1; ++i) {
    auto result = assessor_->assess(far, 0, 0, 0, 100, 0);
    EXPECT_TRUE(result.has_threat)
        << "Should remain active before n_deactivate, iteration=" << i;
  }

  auto result = assessor_->assess(far, 0, 0, 0, 100, 0);
  EXPECT_FALSE(result.has_threat);
}

TEST_F(ThreatAssessorTest, ClassifiesLateralObstacle) {
  auto snapshot = make_snapshot(10.0f, 2.0f, 10.0f, 10.0f);

  ThreatAssessment result;
  for (int i = 0; i < config_.assessment.n_activate; ++i) {
    result = assessor_->assess(snapshot, 0, 0, 0, 100, 0);
  }

  ASSERT_TRUE(result.has_threat);
  EXPECT_EQ(result.classification, ThreatClassification::LATERAL);
}

TEST_F(ThreatAssessorTest, ClassifiesOnPathObstacle) {
  auto snapshot = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);

  ThreatAssessment result;
  for (int i = 0; i < config_.assessment.n_activate; ++i) {
    result = assessor_->assess(snapshot, 0, 0, 0, 100, 0);
  }

  ASSERT_TRUE(result.has_threat);
  EXPECT_EQ(result.classification, ThreatClassification::ON_PATH);
}

TEST_F(ThreatAssessorTest, ClassifiesOnWaypointObstacle) {
  auto snapshot = make_snapshot(5.0f, 10.0f, 10.0f, 10.0f);

  ThreatAssessment result;
  for (int i = 0; i < config_.assessment.n_activate; ++i) {
    result = assessor_->assess(snapshot, 0, 0, 0, 5.0, 0);
  }

  ASSERT_TRUE(result.has_threat);
  EXPECT_EQ(result.classification, ThreatClassification::ON_WAYPOINT);
}

TEST_F(ThreatAssessorTest, CooldownPreventsImmediateRetrigger) {
  auto close = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);

  for (int i = 0; i < config_.assessment.n_activate; ++i) {
    assessor_->assess(close, 0, 0, 0, 100, 0);
  }

  assessor_->notify_avoidance_complete();

  assessor_->reset();
  auto result = assessor_->assess(close, 0, 0, 0, 100, 0);
  EXPECT_FALSE(result.has_threat);
}

TEST_F(ThreatAssessorTest, ResetClearsState) {
  auto close = make_snapshot(2.0f, 10.0f, 10.0f, 10.0f);
  for (int i = 0; i < config_.assessment.n_activate; ++i) {
    assessor_->assess(close, 0, 0, 0, 100, 0);
  }

  assessor_->reset();

  auto far = make_snapshot(10.0f, 10.0f, 10.0f, 10.0f);
  auto result = assessor_->assess(far, 0, 0, 0, 100, 0);
  EXPECT_FALSE(result.has_threat);
}
