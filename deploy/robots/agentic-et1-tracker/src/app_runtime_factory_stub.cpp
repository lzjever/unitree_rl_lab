#include "agentic_et1_tracker/app/app_runtime_factory.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr const char* kPolicyNotLoadedBlock = "policy_not_loaded";

AppRuntimeFactoryResult notReady(const AppConfig& config) {
  AppRuntimeFactoryResult result;
  result.snapshot.ready = false;
  result.snapshot.mode = RuntimeMode::Sim;
  result.snapshot.robot = RobotState::NotReady;
  result.snapshot.ctrl = ControllerState::Starting;
  result.snapshot.hz = config.runtime.hz;
  result.snapshot.queue.limit = config.runtime.queue_limit;
  result.snapshot.block = kPolicyNotLoadedBlock;
  result.snapshot.err = ErrorCode::ModelNotReady;
  result.health = {ServiceHealth::Starting, RuntimeMode::Sim, ErrorCode::ModelNotReady,
                   kPolicyNotLoadedBlock};
  return result;
}

}  // namespace

AppRuntimeFactoryResult createAppRuntimeDeps(const AppConfig& config) {
  return notReady(config);
}

}  // namespace agentic_et1_tracker
