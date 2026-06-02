#pragma once

#include <optional>

#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/app/app_config.hpp"
#include "agentic_et1_tracker/app/app_runner.hpp"
#include "agentic_et1_tracker/core/status.hpp"

namespace agentic_et1_tracker {

struct AppRuntimeFactoryResult {
  std::optional<AppRuntimeDeps> deps;
  StatusSnapshot snapshot;
  HealthSnapshot health;
};

AppRuntimeFactoryResult createAppRuntimeDeps(const AppConfig& config);
bool shouldReleaseMotionModeOnStartup(const AppConfig& config);

}  // namespace agentic_et1_tracker
