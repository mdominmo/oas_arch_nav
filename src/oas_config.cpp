#include "oas_config.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace arch_nav_oas {

OasConfig OasConfig::defaults() { return OasConfig{}; }

OasConfig OasConfig::load(const std::string& config_path) {
  OasConfig cfg;
  if (config_path.empty()) return cfg;

  std::ifstream file(config_path);
  if (!file.is_open()) return cfg;

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (...) {
    return cfg;
  }

  auto oas = root["oas"];
  if (!oas) return cfg;

  if (auto topics = oas["topics"]) {
    if (topics["front"]) cfg.topics.front = topics["front"].as<std::string>();
    if (topics["left"]) cfg.topics.left = topics["left"].as<std::string>();
    if (topics["right"]) cfg.topics.right = topics["right"].as<std::string>();
    if (topics["rear"]) cfg.topics.rear = topics["rear"].as<std::string>();
  }

  if (auto angles = oas["sensor_angles"]) {
    if (angles["front"]) cfg.sensor_angles[0] = angles["front"].as<double>();
    if (angles["left"]) cfg.sensor_angles[1] = angles["left"].as<double>();
    if (angles["right"]) cfg.sensor_angles[2] = angles["right"].as<double>();
    if (angles["rear"]) cfg.sensor_angles[3] = angles["rear"].as<double>();
  }

  if (auto apf = oas["apf"]) {
    if (apf["eta"]) cfg.apf.eta = apf["eta"].as<double>();
    if (apf["d0"]) cfg.apf.d0 = apf["d0"].as<double>();
    if (apf["d_min_clamp"]) cfg.apf.d_min_clamp = apf["d_min_clamp"].as<double>();
  }

  if (auto a = oas["assessment"]) {
    if (a["d_activate"]) cfg.assessment.d_activate = a["d_activate"].as<double>();
    if (a["d_deactivate"]) cfg.assessment.d_deactivate = a["d_deactivate"].as<double>();
    if (a["n_activate"]) cfg.assessment.n_activate = a["n_activate"].as<int>();
    if (a["n_deactivate"]) cfg.assessment.n_deactivate = a["n_deactivate"].as<int>();
    if (a["poll_interval_ms"]) cfg.assessment.poll_interval_ms = a["poll_interval_ms"].as<int>();
    if (a["sensor_stale_ms"]) cfg.assessment.sensor_stale_ms = a["sensor_stale_ms"].as<int>();
    if (a["path_alignment_threshold_rad"])
      cfg.assessment.path_alignment_threshold_rad = a["path_alignment_threshold_rad"].as<double>();
    if (a["on_waypoint_tolerance"])
      cfg.assessment.on_waypoint_tolerance = a["on_waypoint_tolerance"].as<double>();
  }

  if (auto av = oas["avoidance"]) {
    if (av["d_escape"]) cfg.avoidance.d_escape = av["d_escape"].as<double>();
    if (av["d_escape_min"]) cfg.avoidance.d_escape_min = av["d_escape_min"].as<double>();
    if (av["d_escape_max"]) cfg.avoidance.d_escape_max = av["d_escape_max"].as<double>();
    if (av["f_min_threshold"]) cfg.avoidance.f_min_threshold = av["f_min_threshold"].as<double>();
  }

  if (auto bp = oas["bypass"]) {
    if (bp["d_safety"]) cfg.bypass.d_safety = bp["d_safety"].as<double>();
    if (bp["d_safety_min"]) cfg.bypass.d_safety_min = bp["d_safety_min"].as<double>();
  }

  if (oas["cooldown_ms"]) cfg.cooldown_ms = oas["cooldown_ms"].as<int>();
  if (oas["priority"]) cfg.priority = oas["priority"].as<int>();

  return cfg;
}

}  // namespace arch_nav_oas
