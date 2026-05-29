#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/command_mailbox.hpp"
#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/control/fixstand.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

class RuntimeControlLoop final {
 public:
  RuntimeControlLoop(RuntimeConfig config,
                     RuntimeBridge& bridge,
                     RuntimeStatusStore& status,
                     TrkLoader loader);
  RuntimeControlLoop(RuntimeConfig config,
                     RuntimeBridge& bridge,
                     RuntimeStatusStore& status,
                     TrkLoader loader,
                     RobotIO& robot_io,
                     PolicyInference& policy,
                     DeployConfig deploy_config,
                     std::uint8_t expected_mode_machine,
                     RuntimeMode mode = RuntimeMode::Real);
  RuntimeControlLoop(RuntimeConfig config,
                     RuntimeBridge& bridge,
                     RuntimeStatusStore& status,
                     TrkLoader loader,
                     RobotIO& robot_io,
                     PolicyInference& policy,
                     DeployConfig deploy_config,
                     VelocityPolicyInference& velocity_policy,
                     VelocityDeployConfig velocity_deploy_config,
                     FixStandConfig fixstand_config,
                     ControlMode startup_control,
                     std::uint8_t expected_mode_machine,
                     RuntimeMode mode = RuntimeMode::Real);

  void tick();

 private:
  struct SnapshotRuntimeState {
    bool ready{true};
    RobotState robot{RobotState::Idle};
    std::size_t low_ms{0};
    std::string block;
    ErrorCode err{ErrorCode::Ok};
  };

  bool consumePendingCommands();
  void consumeStoppingCommands();
  void handleStop(std::uint64_t sequence);
  void handleControl(ControlMode mode);
  void handleInterrupt(MotionRequest request);
  void cancelWaiting(StopReason reason);
  void cancelWaiting(StopReason reason, std::uint64_t sequence);
  void startNext();
  void completePreparing();
  void advanceActive();
  void advanceActiveWithPolicy();
  void publishIdleHoldIfReady();
  void publishControlIfReady();
  bool writeFixStand();
  bool writeStandbyVelocity();
  bool writeStoppingHold();
  void markActiveStopping(StopReason reason);
  void completeStoppingActive(MotionState state, ErrorCode error);
  std::optional<MotionRequest> finishActive(MotionState state,
                                            StopReason reason,
                                            ErrorCode error);
  void enterStopping(StopReason reason);
  std::size_t stopHoldTicks() const;
  void publishActive();
  void publishSnapshot();
  MotionStatus toStatus(const MotionRequest& request) const;
  std::vector<std::string> waitingIds() const;
  RobotState robotState() const;
  bool hasPolicyRuntime() const;
  bool hasControlRuntime() const;
  void refreshReadinessForPolicyRuntime();
  void applyReadiness(const RobotReadinessStatus& readiness);
  bool readinessRequiresFault(const RobotReadinessStatus& readiness) const;
  void enterFault(ErrorCode error,
                  RobotState robot,
                  std::string block,
                  std::optional<std::size_t> low_ms = std::nullopt);
  void failActiveWithFault(ErrorCode error,
                           RobotState robot,
                           std::string block,
                           std::optional<std::size_t> low_ms = std::nullopt);
  void failStoppingActiveWithFault(ErrorCode error);

  RuntimeConfig config_;
  RuntimeBridge& bridge_;
  RuntimeStatusStore& status_;
  TrkLoader loader_;
  RobotIO* robot_io_{nullptr};
  PolicyInference* policy_{nullptr};
  std::optional<DeployConfig> deploy_config_;
  VelocityPolicyInference* velocity_policy_{nullptr};
  std::optional<VelocityDeployConfig> velocity_deploy_config_;
  std::optional<FixStandConfig> fixstand_config_;
  std::optional<FixStandRunner> fixstand_runner_;
  std::optional<VelocityStepRunner> velocity_runner_;
  std::uint8_t expected_mode_machine_{0};
  RuntimeMode mode_{RuntimeMode::Sim};
  std::deque<MotionRequest> waiting_;
  std::optional<MotionRequest> active_;
  std::optional<PolicyStepRunner> policy_runner_;
  SnapshotRuntimeState runtime_state_;
  ControllerState ctrl_{ControllerState::Idle};
  StopReason stop_reason_{StopReason::None};
  bool stop_to_idle_pending_{false};
  ControlMode post_stop_control_{ControlMode::StandbyVelocity};
  bool active_first_advance_{false};
  std::size_t stopping_hold_ticks_remaining_{0};
};

}  // namespace agentic_et1_tracker
