#ifndef ARCH_NAV_OAS_CONFIG_HPP_
#define ARCH_NAV_OAS_CONFIG_HPP_

#include <array>
#include <cmath>
#include <string>

#include "oas_types.hpp"

namespace arch_nav_oas {

struct OasConfig {
  struct Topics {
    std::string front = "/rangefinder/front";
    std::string left = "/rangefinder/left";
    std::string right = "/rangefinder/right";
    std::string rear = "/rangefinder/rear";
  } topics;

  std::array<double, kNumSensors> sensor_angles = {
      0.0,            // front
      M_PI_2,         // left
      -M_PI_2,        // right
      M_PI            // rear
  };

  struct Apf {
    double eta = 1.0;
    double d0 = 5.0;
    double d_min_clamp = 0.3;
  } apf;

  struct Assessment {
    double d_activate = 3.0;
    double d_deactivate = 5.0;
    int n_activate = 3;
    int n_deactivate = 5;
    int poll_interval_ms = 50;
    int sensor_stale_ms = 500;
    double path_alignment_threshold_rad = M_PI / 4.0;
    double on_waypoint_tolerance = 1.0;
  } assessment;

  struct Avoidance {
    double d_escape = 4.0;
    double d_escape_min = 2.0;
    double d_escape_max = 10.0;
    double f_min_threshold = 0.01;
  } avoidance;

  struct Bypass {
    double d_safety = 5.0;
    double d_safety_min = 3.0;
  } bypass;

  int cooldown_ms = 2000;
  int priority = 50;

  static OasConfig load(const std::string& config_path);
  static OasConfig defaults();
};

}  // namespace arch_nav_oas

#endif  // ARCH_NAV_OAS_CONFIG_HPP_
