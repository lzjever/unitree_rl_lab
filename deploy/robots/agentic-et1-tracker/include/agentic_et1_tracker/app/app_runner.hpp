#pragma once

#include <functional>
#include <memory>

#include "agentic_et1_tracker/app/app_config.hpp"
#include "agentic_et1_tracker/control/fixstand.hpp"
#include "agentic_et1_tracker/control/passive.hpp"
#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

struct AppRuntimeDeps {
  std::unique_ptr<RobotIO> robot_io;
  std::unique_ptr<PolicyInference> policy;
  std::unique_ptr<VelocityPolicyInference> velocity_policy;
  DeployConfig deploy_config;
  VelocityDeployConfig velocity_deploy_config;
  FixStandConfig fixstand_config;
  PassiveConfig passive_config;
  ControlMode startup_control{ControlMode::FixStand};
  RuntimeMode mode{RuntimeMode::Sim};
  std::shared_ptr<const TrkTrack> standby_track;
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
