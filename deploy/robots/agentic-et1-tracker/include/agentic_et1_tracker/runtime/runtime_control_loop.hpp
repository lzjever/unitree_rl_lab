#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/command_mailbox.hpp"
#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/control/fixstand.hpp"
#include "agentic_et1_tracker/control/passive.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/reference/reference_frame.hpp"
#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

enum class RuntimeInternalState {
  Passive,
  FixStand,
  Velocity,
  GeneralTrackerIdle,
  GeneralTrackerActive,
  GeneralTrackerTransition,
  Stopping,
  Fault,
};

enum class RuntimeInternalEvent {
  FixStand,
  Velocity,
  MotionRequest,
  SafetyPassive,
  Fault,
};

class RuntimeControlLoop final {
 public:
  RuntimeControlLoop(RuntimeConfig config,
                     RuntimeBridge& bridge,
                     RuntimeStatusStore& status,
                     TrkLoader loader,
                     ReferenceFrameSink* reference_sink = nullptr);
  RuntimeControlLoop(RuntimeConfig config,
                     RuntimeBridge& bridge,
                     RuntimeStatusStore& status,
                     TrkLoader loader,
                     RobotIO& robot_io,
                     PolicyInference& policy,
                     DeployConfig deploy_config,
                     PassiveConfig passive_config,
                     std::uint8_t expected_mode_machine,
                     RuntimeMode mode = RuntimeMode::Real,
                     ReferenceFrameSink* reference_sink = nullptr);
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
                     PassiveConfig passive_config,
                     ControlMode startup_control,
                     std::uint8_t expected_mode_machine,
                     RuntimeMode mode = RuntimeMode::Real,
                     ReferenceFrameSink* reference_sink = nullptr);

  void tick();
  RuntimeInternalState internalStateForTest() const;
  void failNextTransitionStartForTest();

 private:
  struct SnapshotRuntimeState {
    bool ready{true};
    RobotState robot{RobotState::Idle};
    std::size_t low_ms{0};
    std::string block;
    ErrorCode err{ErrorCode::Ok};
  };

  enum class TransitionTargetKind {
    User,
    Idle,
    Standby,
  };

  struct PendingTransition {
    TransitionTargetKind target_kind{TransitionTargetKind::User};
    std::string target_id;
    std::optional<MotionState> target_state;
    std::optional<MotionRequest> target_request;
    std::shared_ptr<const TrkTrack> target_track;
    std::optional<std::size_t> idle_index;
  };

  bool consumePendingCommands();
  void consumeStoppingCommands();
  void handleStop(std::uint64_t sequence, bool requires_stopping);
  void handleControl(ControlMode mode);
  void handleIdleConfig(std::vector<IdleMotion> motions);
  void handleInterrupt(MotionRequest request);
  void cancelWaiting(StopReason reason);
  void cancelWaiting(StopReason reason, std::uint64_t sequence);
  ControllerState controllerStateForInternal(RuntimeInternalState state) const;
  void enterInternalState(RuntimeInternalState state);
  void handleInternalEvent(RuntimeInternalEvent event);
  void enterPassiveState(const RobotReadinessStatus& readiness);
  void enterFixStandState();
  void enterVelocityState();
  void enterGeneralTrackerIdleState();
  void enterTrackPreparingState();
  void enterTrackActiveState();
  bool isMotionAcceptingState() const;
  bool isControlPublishingState() const;
  void runPassiveState();
  void startNext();
  bool hasIdleStartCandidate() const;
  bool canStartIdle() const;
  void startIdle();
  void completePreparing();
  void advanceActive();
  void advanceActiveWithPolicy();
  void advanceHolding();
  void advanceHoldingWithPolicy();
  bool startTransitionFromHoldingToNextUser();
  bool startTransitionFromCurrentReferenceToUser(MotionRequest target_request,
                                                StopReason replaced_reason);
  bool startTransitionFromCompletedUserToIdle();
  bool startSyntheticTransitionFromActiveFrame(PendingTransition target,
                                               const TrkFrameView& target_frame,
                                               double target_fps);
  bool startInternalTransition(std::shared_ptr<const TrkTrack> track,
                               PendingTransition target,
                               std::optional<LowStateSample> entry_low_state);
  void advanceTransition();
  void advanceTransitionWithPolicy();
  void completeTransition();
  void finishTransitionTarget(MotionState state,
                              StopReason reason,
                              ErrorCode error);
  void abortTransition(MotionState target_state = MotionState::Canceled,
                       StopReason reason = StopReason::Stop,
                       ErrorCode error = ErrorCode::Ok);
  bool failActiveReadiness(const RobotReadinessStatus& readiness);
  void publishIdleHoldIfReady();
  void publishControlIfReady();
  bool writePassiveDamping();
  bool writeFixStand();
  bool writeStandbyVelocity();
  bool writeStoppingHold();
  bool republishLowCmdBuffer();
  void applyGeneralTrackerIdleHold(LowCmdFrame& frame) const;
  const LowCmdFrame* lowCmdBaseFrame() const;
  void writeLowCmdFrame(const LowCmdFrame& frame);
  void markActiveStopping(StopReason reason);
  void completeStoppingActive(MotionState state, ErrorCode error);
  std::optional<MotionRequest> finishActive(MotionState state,
                                            StopReason reason,
                                            ErrorCode error);
  void stopIdleActive();
  void enterStopping(StopReason reason);
  std::size_t ticksForPeriod(double seconds) const;
  std::size_t ticksForRate(double rate_hz) const;
  bool consumeStepDue(std::size_t& ticks_until_next, std::size_t interval_ticks);
  std::size_t velocityPolicyIntervalTicks() const;
  std::size_t activePolicyIntervalTicks() const;
  std::size_t stopHoldTicks() const;
  void publishActive();
  void publishReferenceActive();
  void publishReferenceTransition();
  void clearReference();
  void publishSnapshot();
  std::optional<LowStateSample> readLowStateForStatus();
  std::optional<HighStateSample> readHighStateForStatus();
  void fillSnapshotPose(StatusSnapshot& snapshot);
  MotionStatus toStatus(const MotionRequest& request) const;
  std::vector<std::string> waitingIds() const;
  IdleStatus idleStatus() const;
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
  ReferenceFrameSink* reference_sink_{nullptr};
  RobotIO* robot_io_{nullptr};
  PolicyInference* policy_{nullptr};
  std::optional<DeployConfig> deploy_config_;
  VelocityPolicyInference* velocity_policy_{nullptr};
  std::optional<VelocityDeployConfig> velocity_deploy_config_;
  std::optional<FixStandConfig> fixstand_config_;
  std::optional<PassiveConfig> passive_config_;
  std::optional<FixStandRunner> fixstand_runner_;
  std::optional<VelocityStepRunner> velocity_runner_;
  std::uint8_t expected_mode_machine_{0};
  RuntimeMode mode_{RuntimeMode::Sim};
  std::deque<MotionRequest> waiting_;
  std::optional<MotionRequest> active_;
  ActiveKind active_kind_{ActiveKind::None};
  std::shared_ptr<const TrkTrack> active_track_;
  std::optional<PendingTransition> transition_;
  std::optional<PolicyStepRunner> policy_runner_;
  std::vector<IdleMotion> idle_config_;
  std::size_t idle_next_index_{0};
  std::optional<std::size_t> idle_current_index_;
  SnapshotRuntimeState runtime_state_;
  RuntimeInternalState fsm_state_{RuntimeInternalState::GeneralTrackerIdle};
  ControllerState ctrl_{ControllerState::Idle};
  StopReason stop_reason_{StopReason::None};
  bool stop_to_idle_pending_{false};
  ControlMode post_stop_control_{ControlMode::StandbyVelocity};
  bool active_first_advance_{false};
  std::size_t stopping_hold_ticks_remaining_{0};
  std::size_t active_policy_ticks_until_next_{0};
  std::size_t velocity_policy_ticks_until_next_{0};
  bool fail_next_transition_start_for_test_{false};
  std::optional<LowCmdFrame> lowcmd_buffer_;
  std::optional<LowStateSample> latest_low_state_;
  std::optional<HighStateSample> latest_high_state_;
};

}  // namespace agentic_et1_tracker
