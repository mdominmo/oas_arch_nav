#include "arch_nav/supervisor/supervisor_registry.hpp"
#include "oas_supervisor.hpp"

namespace {

struct OasSupervisorRegistration {
  OasSupervisorRegistration() {
    arch_nav::supervisor::SupervisorRegistry::instance().register_supervisor(
        "oas",
        [](const std::string& config_path) {
          return std::make_unique<arch_nav_oas::OasSupervisor>(config_path);
        });
  }
};

static OasSupervisorRegistration registration;

}  // namespace
