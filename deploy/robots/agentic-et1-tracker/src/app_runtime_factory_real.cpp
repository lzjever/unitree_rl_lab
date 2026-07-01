#include "agentic_et1_tracker/app/app_runtime_factory.hpp"

#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "agentic_et1_tracker/control/fixstand.hpp"
#include "agentic_et1_tracker/control/passive.hpp"
#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"
#include "agentic_et1_tracker/loco_upper/lowcmd_composer.hpp"
#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/policy/onnx_policy_runtime.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"
#include "agentic_et1_tracker/robot/unitree_sdk_robot_io.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

#include "app_policy_path_guard.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr const char* kPolicyNotLoadedBlock = "policy_not_loaded";
constexpr const char* kRobotDisconnectedBlock = "robot_disconnected";
constexpr double kInternalStandbyMaxTrackDurationS = 5.0;

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

AppRuntimeFactoryResult lowCmdOccupied(const AppConfig& config) {
  return notReady(config, ErrorCode::RobotNotReady, RobotState::NotReady,
                  "lowcmd_occupied", runtimeMode(config));
}

AppRuntimeFactoryResult motionModeReleaseFailed(const AppConfig& config) {
  return notReady(config, ErrorCode::RobotNotReady, RobotState::NotReady,
                  "motion_mode_release_failed", runtimeMode(config));
}

std::filesystem::path modelPath(const PolicyConfig& config) {
  return std::filesystem::path(config.policy_dir) / "exported" / config.policy_file;
}

std::filesystem::path velocityModelPath(const ControlConfig& config) {
  return std::filesystem::path(config.velocity_policy_dir) / "exported" /
         config.velocity_policy_file;
}

std::filesystem::path locoLowerModelPath(const LocoUpperConfig& config) {
  return std::filesystem::path(config.policy_dir) / "exported" / config.policy_file;
}

bool hasLocoUpperRuntimeDeps(const AppConfig& config, const AppRuntimeDeps& deps) {
  if (!config.loco_upper.enabled) {
    return true;
  }
  return deps.loco_lower_policy != nullptr &&
         deps.loco_lower_deploy_config.has_value() &&
         deps.loco_upper_composer_config.has_value();
}

}  // namespace

namespace app_internal {

TrkValidationConfig internalStandbyTrkValidationConfig(
    const AppConfig& config,
    const std::filesystem::path& standby_reference) {
  TrkValidationConfig trk_config = config.trk;
  trk_config.allowlist_dirs = {standby_reference.parent_path()};
  trk_config.max_duration_s = kInternalStandbyMaxTrackDurationS;
  return trk_config;
}

void tryAttachLocoUpperDeps(const AppConfig& config, AppRuntimeDeps& deps) {
  if (!config.loco_upper.enabled) {
    return;
  }

  try {
    const LocoLowerDeployConfig loco_lower_deploy_config =
        loadLocoLowerDeployConfig(config.loco_upper.deploy);
    LocoUpperLowCmdComposerConfig composer_config =
        loadLocoUpperLowCmdComposerConfig(config.loco_upper.limits,
                                          config.loco_upper.joint_map,
                                          deps.deploy_config,
                                          loco_lower_deploy_config);
    composer_config.expected_mode_machine =
        static_cast<std::uint8_t>(config.mode_machine);

    const std::filesystem::path model_path = locoLowerModelPath(config.loco_upper);
    if (referencesEt1RuntimeDependency(model_path)) {
      return;
    }

    auto loco_lower_policy = std::make_unique<OnnxLocoLowerPolicyRuntime>(
        OnnxLocoLowerPolicyRuntimeConfig{model_path, loco_lower_deploy_config});

    deps.loco_lower_policy = std::move(loco_lower_policy);
    deps.loco_lower_deploy_config = std::move(loco_lower_deploy_config);
    deps.loco_upper_composer_config = std::move(composer_config);
  } catch (const LocoLowerDeployConfigError&) {
  } catch (const LocoUpperLowCmdComposerError&) {
  } catch (const PolicyRuntimeError&) {
  } catch (const std::exception&) {
  }
}

}  // namespace app_internal

namespace {

std::shared_ptr<const TrkTrack> loadInternalStandbyTrack(const AppConfig& config) {
  const std::filesystem::path path = config.control.standby_reference;
  if (app_internal::referencesEt1RuntimeDependency(path)) {
    throw std::runtime_error("standby reference points at ET1 app");
  }

  TrkValidationConfig trk_config =
      app_internal::internalStandbyTrkValidationConfig(config, path);
  TrkLoadResult loaded = TrkLoader(std::move(trk_config)).load(path);
  if (!loaded.ok()) {
    throw std::runtime_error("standby reference failed to load");
  }
  return std::make_shared<TrkTrack>(std::move(*loaded.track));
}

ControlMode startupControl(const ControlConfig& config) {
  if (config.startup_control == "FixStand") {
    return ControlMode::FixStand;
  }
  if (config.startup_control == "StandbyVelocity") {
    return ControlMode::StandbyVelocity;
  }
  throw PolicyRuntimeError("unsupported startup control");
}

bool policyProfileMatchesDeployContract(const PolicyConfig& policy,
                                        ObservationContract contract) {
  std::optional<ObservationContract> expected_contract;
  if (policy.profile == "GeneralTrackerCLNFootstate") {
    expected_contract = ObservationContract::GeneralTrackerCLNFootstate;
  } else if (policy.profile == "GeneralTrackerCLN") {
    expected_contract = ObservationContract::GeneralTrackerCLN;
  } else if (policy.profile == "GeneralTracker") {
    expected_contract = ObservationContract::GeneralTracker;
  } else if (policy.profile == "GeneralTrackerDR3") {
    expected_contract = ObservationContract::GeneralTrackerDR3;
  }
  return expected_contract && contract == *expected_contract;
}

}  // namespace

bool shouldReleaseMotionModeOnStartup(const AppConfig& config) {
  return config.mode_machine != 0 && config.release_motion_mode_on_startup;
}

AppRuntimeFactoryResult createAppRuntimeDeps(const AppConfig& config) {
  FactoryPhase phase = FactoryPhase::Deploy;
  try {
    DeployConfig deploy_config = loadDeployConfig(config.policy.deploy);
    if (!policyProfileMatchesDeployContract(config.policy,
                                            deploy_config.observation_contract)) {
      return modelNotReady(config);
    }
    VelocityDeployConfig velocity_deploy_config =
        loadVelocityDeployConfig(config.control.velocity_deploy);
    FixStandConfig fixstand_config = loadFixStandConfig(config.control.fixstand_config);
    PassiveConfig passive_config = loadPassiveConfig(config.control.passive_config);
    std::shared_ptr<const TrkTrack> standby_track = loadInternalStandbyTrack(config);

    phase = FactoryPhase::Policy;
    const std::filesystem::path policy_model_path = modelPath(config.policy);
    if (app_internal::referencesEt1RuntimeDependency(policy_model_path)) {
      return modelNotReady(config);
    }
    auto policy = std::make_unique<OnnxPolicyRuntime>(
        OnnxPolicyRuntimeConfig{policy_model_path, deploy_config});

    const std::filesystem::path velocity_model_path = velocityModelPath(config.control);
    if (app_internal::referencesEt1RuntimeDependency(velocity_model_path)) {
      return modelNotReady(config);
    }
    auto velocity_policy = std::make_unique<OnnxVelocityPolicyRuntime>(
        OnnxVelocityPolicyRuntimeConfig{velocity_model_path, velocity_deploy_config});

    phase = FactoryPhase::Robot;
    UnitreeSdkRobotIOConfig robot_config;
    robot_config.network = config.network;
    robot_config.domain_id = config.domain_id;
    robot_config.lowcmd_startup_preflight_ms = config.lowcmd_startup_preflight_ms;
    robot_config.release_motion_mode_on_startup =
        shouldReleaseMotionModeOnStartup(config);
    robot_config.release_motion_mode_timeout_s = config.release_motion_mode_timeout_s;
    robot_config.release_motion_mode_max_attempts =
        config.release_motion_mode_max_attempts;
    robot_config.release_motion_mode_retry_interval_ms =
        config.release_motion_mode_retry_interval_ms;
    auto robot_io = std::make_unique<UnitreeSdkRobotIO>(std::move(robot_config));

    AppRuntimeDeps deps;
    deps.robot_io = std::move(robot_io);
    deps.policy = std::move(policy);
    deps.velocity_policy = std::move(velocity_policy);
    deps.deploy_config = std::move(deploy_config);
    deps.velocity_deploy_config = std::move(velocity_deploy_config);
    deps.fixstand_config = std::move(fixstand_config);
    deps.passive_config = std::move(passive_config);
    deps.startup_control = startupControl(config.control);
    deps.mode = runtimeMode(config);
    deps.standby_track = std::move(standby_track);
    app_internal::tryAttachLocoUpperDeps(config, deps);
    if (!hasLocoUpperRuntimeDeps(config, deps)) {
      return modelNotReady(config);
    }

    AppRuntimeFactoryResult result;
    result.deps.emplace(std::move(deps));
    return result;
  } catch (const DeployConfigError&) {
    return modelNotReady(config);
  } catch (const VelocityDeployConfigError&) {
    return modelNotReady(config);
  } catch (const FixStandConfigError&) {
    return modelNotReady(config);
  } catch (const PassiveConfigError&) {
    return modelNotReady(config);
  } catch (const PolicyRuntimeError&) {
    return modelNotReady(config);
  } catch (const MotionModeReleaseError&) {
    return motionModeReleaseFailed(config);
  } catch (const LowCmdStartupPreflightError&) {
    return lowCmdOccupied(config);
  } catch (const RobotIOError&) {
    return robotDisconnected(config);
  } catch (const std::exception&) {
    return phase == FactoryPhase::Robot ? robotDisconnected(config) : modelNotReady(config);
  } catch (...) {
    return phase == FactoryPhase::Robot ? robotDisconnected(config) : modelNotReady(config);
  }
}

}  // namespace agentic_et1_tracker
