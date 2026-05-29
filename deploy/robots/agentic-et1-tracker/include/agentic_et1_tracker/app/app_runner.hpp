#pragma once

#include <functional>
#include <memory>

#include "agentic_et1_tracker/app/app_config.hpp"
#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"

namespace agentic_et1_tracker {

struct AppRuntimeDeps {
  std::unique_ptr<RobotIO> robot_io;
  std::unique_ptr<PolicyInference> policy;
  DeployConfig deploy_config;
  RuntimeMode mode{RuntimeMode::Sim};
};

struct AppRuntimeFactoryResult;
using AppRuntimeFactory = std::function<AppRuntimeFactoryResult(const AppConfig&)>;

class AppRunner {
 public:
  explicit AppRunner(AppConfig config);
  AppRunner(AppConfig config, AppRuntimeFactory runtime_factory);
  AppRunner(AppConfig config, AppRuntimeDeps deps);
  ~AppRunner();

  AppRunner(const AppRunner&) = delete;
  AppRunner& operator=(const AppRunner&) = delete;

  bool start();
  void stop();
  bool isRunning() const;
  int boundPort() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace agentic_et1_tracker
