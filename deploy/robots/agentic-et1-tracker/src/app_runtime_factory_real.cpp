#include "agentic_et1_tracker/app/app_runtime_factory.hpp"

#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/policy/onnx_policy_runtime.hpp"
#include "agentic_et1_tracker/robot/unitree_sdk_robot_io.hpp"

#include "app_policy_path_guard.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr const char* kPolicyNotLoadedBlock = "policy_not_loaded";
constexpr const char* kRobotDisconnectedBlock = "robot_disconnected";

enum class FactoryPhase {
  Deploy,
  Policy,
  Robot,
};

RuntimeMode runtimeMode(const AppConfig& config) {
  return config.mode_machine == 0 ? RuntimeMode::Sim : RuntimeMode::Real;
}

AppRuntimeFactoryResult notReady(const AppConfig& config,
                                 ErrorCode err,
                                 RobotState robot,
                                 const std::string& block,
                                 RuntimeMode mode) {
  AppRuntimeFactoryResult result;
  result.snapshot.ready = false;
  result.snapshot.mode = mode;
  result.snapshot.robot = robot;
  result.snapshot.ctrl = ControllerState::Starting;
  result.snapshot.hz = config.runtime.hz;
  result.snapshot.queue.limit = config.runtime.queue_limit;
  result.snapshot.block = block;
  result.snapshot.err = err;
  result.health = {ServiceHealth::Starting, mode, err, block};
  return result;
}

AppRuntimeFactoryResult modelNotReady(const AppConfig& config) {
  return notReady(config, ErrorCode::ModelNotReady, RobotState::NotReady,
                  kPolicyNotLoadedBlock, runtimeMode(config));
}

AppRuntimeFactoryResult robotDisconnected(const AppConfig& config) {
  return notReady(config, ErrorCode::RobotDisconnected, RobotState::Disconnected,
                  kRobotDisconnectedBlock, runtimeMode(config));
}

std::filesystem::path modelPath(const PolicyConfig& config) {
  return std::filesystem::path(config.policy_dir) / "exported" / config.policy_file;
}

}  // namespace

AppRuntimeFactoryResult createAppRuntimeDeps(const AppConfig& config) {
  FactoryPhase phase = FactoryPhase::Deploy;
  try {
    DeployConfig deploy_config = loadDeployConfig(config.policy.deploy);

    phase = FactoryPhase::Policy;
    const std::filesystem::path policy_model_path = modelPath(config.policy);
    if (app_internal::referencesEt1PolicyTree(policy_model_path)) {
      return modelNotReady(config);
    }
    auto policy = std::make_unique<OnnxPolicyRuntime>(
        OnnxPolicyRuntimeConfig{policy_model_path, deploy_config});

    phase = FactoryPhase::Robot;
    UnitreeSdkRobotIOConfig robot_config;
    robot_config.network = config.network;
    robot_config.domain_id = config.domain_id;
    auto robot_io = std::make_unique<UnitreeSdkRobotIO>(std::move(robot_config));

    AppRuntimeDeps deps;
    deps.robot_io = std::move(robot_io);
    deps.policy = std::move(policy);
    deps.deploy_config = std::move(deploy_config);
    deps.mode = runtimeMode(config);

    AppRuntimeFactoryResult result;
    result.deps.emplace(std::move(deps));
    return result;
  } catch (const DeployConfigError&) {
    return modelNotReady(config);
  } catch (const PolicyRuntimeError&) {
    return modelNotReady(config);
  } catch (const RobotIOError&) {
    return robotDisconnected(config);
  } catch (const std::exception&) {
    return phase == FactoryPhase::Robot ? robotDisconnected(config) : modelNotReady(config);
  } catch (...) {
    return phase == FactoryPhase::Robot ? robotDisconnected(config) : modelNotReady(config);
  }
}

}  // namespace agentic_et1_tracker
