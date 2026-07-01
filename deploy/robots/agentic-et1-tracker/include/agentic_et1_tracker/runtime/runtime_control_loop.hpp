#pragma once

#include <array>
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
#include "agentic_et1_tracker/loco_upper/compiler.hpp"
#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"
#include "agentic_et1_tracker/loco_upper/lowcmd_composer.hpp"
#include "agentic_et1_tracker/loco_upper/planner.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/reference/reference_frame.hpp"
#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"
#include "agentic_et1_tracker/trk/synthetic_transition.hpp"

namespace agentic_et1_tracker {

enum class RuntimeInternalState {
  Passive,
  FixStand,
  Velocity,
  GeneralTrackerIdle,
  GeneralTrackerActive,
  GeneralTrackerTransition,
  LocoUpperActive,
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
                     ReferenceFrameSink* reference_sink = nullptr,
                     std::shared_ptr<const TrkTrack> standby_track = nullptr);
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
                     ReferenceFrameSink* reference_sink = nullptr,
                     std::shared_ptr<const TrkTrack> standby_track = nullptr);
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
                     VelocityPolicyInference& loco_lower_policy,
                     LocoLowerDeployConfig loco_lower_deploy_config,
                     LocoUpperLowCmdComposerConfig loco_upper_composer_config,
                     RuntimeMode mode = RuntimeMode::Real,
                     ReferenceFrameSink* reference_sink = nullptr,
                     std::shared_ptr<const TrkTrack> standby_track = nullptr);
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
                     ReferenceFrameSink* reference_sink = nullptr,
                     std::shared_ptr<const TrkTrack> standby_track = nullptr);

  void tick();
  RuntimeInternalState internalStateForTest() const;
  void failNextTransitionStartForTest();
  void faultNextTransitionStartForTest();
  void forceNextRootYawResidualForTest(double residual_rad);

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

  enum class StandbyTransitionResult {
    Started,
    Fallback,
    SafetyTerminal,
  };

  enum class StartupHoldMode {
    Run,
    Reduced,
  };

  struct PendingTransition {
    TransitionTargetKind target_kind{TransitionTargetKind::User};
    std::string target_id;
    std::optional<MotionState> target_state;
    std::optional<MotionRequest> target_request;
    std::shared_ptr<const TrkTrack> target_track;
    std::optional<std::size_t> idle_index;
    MotionState source_completion_state{MotionState::Done};
    StopReason source_completion_reason{StopReason::None};
    ErrorCode source_completion_error{ErrorCode::Ok};
    bool reduced_target_startup_hold{false};
  };

  struct LocoUpperRuntimeState {
    LocoUpperRootPlan root_plan;
    std::vector<LocoUpperVelocityCommand> commands_body;
    std::vector<LocoUpperLogicalJointFrame> upper_frames;
    std::optional<LocoLowerStepRunner> lower_runner;
    std::vector<float> entry_start_upper;
    std::vector<float> lower_entry_start_q;
    std::vector<float> first_upper;
    std::vector<float> final_upper;
    std::vector<float> last_upper;
    VelocityCommand current_command;
    std::vector<float> current_upper;
    std::vector<float> transition_start_upper;
    std::vector<float> transition_target_upper;
    std::size_t phase_ticks_remaining{0};
    std::size_t phase_total_ticks{0};
    std::size_t hold_ticks_remaining{0};
    bool radius_clamped{false};
    bool radius_limit_reached{false};
    bool envelope_clamped{false};
    bool upper_clamped{false};
    bool upper_rate_limited{false};
    bool raw_action_clamped{false};
    bool lower_q_limited{false};
    bool lower_action_clamped{false};
    std::array<double, 2> integrated_xy{{0.0, 0.0}};
    std::array<double, 2> highstate_start_xy{{0.0, 0.0}};
    std::array<double, 2> last_highstate_xy{{0.0, 0.0}};
    bool has_highstate_start{false};
    bool has_last_highstate_xy{false};
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
  void enterTrackActiveState(
      const std::optional<LowStateSample>& entry_low_state,
      StartupHoldMode startup_hold_mode = StartupHoldMode::Run);
  void resetPolicyStartupHoldForActiveUser(
      const std::optional<LowStateSample>& entry_low_state,
      StartupHoldMode startup_hold_mode = StartupHoldMode::Run);
  void clearPolicyStartupHold();
  void applyPolicyStartupUpperBodyInterpolation(LowCmdFrame& frame) const;
  bool isMotionAcceptingState() const;
  bool isControlPublishingState() const;
  bool isUserOwnedTransition() const;
  bool isBackgroundOwnedTransitionOrPlayback() const;
  void runPassiveState();
  void startNext();
  void startLocoUpper(MotionRequest request);
  bool prepareLocoUpperTrack(MotionRequest& request);
  bool hasIdleStartCandidate() const;
  bool canStartIdle() const;
  void startIdle();
  bool startTransitionFromIdleToUser(MotionRequest target_request);
  bool shouldStartTransitionFromStandbyToUser() const;
  void startTransitionFromStandbyToUser(
      MotionRequest target_request,
      std::optional<LowStateSample> entry_low_state);
  void completePreparing();
  void advanceActive();
  void advanceLocoUpperActive();
  void advanceLocoUpperEntry();
  void advanceLocoUpperMotion();
  void advanceLocoUpperExit();
  void advanceLocoUpperHolding();
  void advanceLocoUpperStopping();
  void beginLocoUpperExit(LocoReason reason);
  bool requireStrictPoseForLocoUpper(std::array<double, 2>& estimate_xy);
  void failLocoUpperSafety(LocoReason reason, const char* block);
  bool updateLocoUpperRadiusState(std::array<double, 2>& estimate_xy);
  bool enforceLocoUpperRadiusLimit(const std::array<double, 2>& estimate_xy);
  void failLocoUpperPolicy(LocoReason reason,
                           const RobotReadinessStatus& readiness);
  bool writeLocoUpperStep(VelocityCommand command,
                          const std::vector<float>& upper_targets,
                          std::optional<float> lower_entry_alpha = std::nullopt);
  bool prepareLocoUpperFrameStep(std::size_t frame, bool zero_lower_command);
  bool writeLocoUpperFrame(std::size_t frame, bool zero_lower_command);
  bool writeLocoUpperFinalHold(bool zero_lower_command);
  bool writeLocoUpperUpperTransitionStep();
  std::vector<float> fixstandUpperTargets() const;
  std::vector<float> locoUpperStandbyTargets() const;
  std::vector<float> currentLocoUpperTargets() const;
  void beginLocoUpperStopping(StopReason reason);
  void completeLocoUpperStopped();
  void finishLocoUpperDone();
  void setLocoPhase(LocoPhase phase);
  void advanceActiveWithPolicy();
  bool advanceUserPolicyStartupHold();
  void advanceHolding();
  void advanceHoldingWithPolicy();
  bool startTransitionFromHoldingToNextUser();
  bool startTransitionFromHoldingToStandby();
  bool startTransitionFromCurrentReferenceToUser(MotionRequest target_request,
                                                StopReason replaced_reason);
  bool startTransitionFromCompletedIdleToIdle();
  bool startTransitionFromCompletedUserToNextUser();
  bool startTransitionFromCompletedUserToIdle();
  bool startTransitionFromCompletedUserToStandby();
  StandbyTransitionResult startTransitionFromActiveToStandbyCancellation();
  bool startSyntheticTransitionFromActiveFrame(PendingTransition target);
  bool startInternalTransition(std::shared_ptr<const TrkTrack> track,
                               PendingTransition target,
                               std::optional<LowStateSample> entry_low_state,
                               bool publish_target_failure = true);
  bool startStandbyPlayback(PendingTransition target);
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
  void clearIdleConfig();
  void enterStopping(StopReason reason);
  std::size_t ticksForPeriod(double seconds) const;
  std::size_t ticksForRate(double rate_hz) const;
  bool consumeStepDue(std::size_t& ticks_until_next, std::size_t interval_ticks);
  double generalTrackerPolicyStepDt() const;
  void resetGeneralTrackerPolicyTiming(double playback_time_s = 0.0);
  bool consumeGeneralTrackerPolicyDue();
  std::size_t activePlaybackFrame() const;
  double activePlaybackEndTime() const;
  void advanceActivePlaybackTime();
  std::size_t velocityPolicyIntervalTicks() const;
  std::size_t activePolicyIntervalTicks() const;
  std::size_t policyStartupHoldPolicySteps(double duration_s) const;
  std::size_t stopHoldTicks() const;
  std::optional<double> transitionDurationForUse() const;
  double transitionSampleFpsForTarget(const TrkTrack& target) const;
  SyntheticTransitionOptions transitionOptionsForTarget(const TrkTrack& target) const;
  std::optional<bool> rootYawResidualAllowsBridge(const TrkFrameView& source,
                                                  const TrkFrameView& target);
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
  bool hasLocoUpperRuntime() const;
  void refreshReadinessForPolicyRuntime();
  void applyReadiness(const RobotReadinessStatus& readiness);
  bool readinessRequiresFault(const RobotReadinessStatus& readiness) const;
  bool isSafetyTerminalState() const;
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
  std::shared_ptr<const TrkTrack> standby_track_;
  ReferenceFrameSink* reference_sink_{nullptr};
  RobotIO* robot_io_{nullptr};
  PolicyInference* policy_{nullptr};
  std::optional<DeployConfig> deploy_config_;
  VelocityPolicyInference* velocity_policy_{nullptr};
  std::optional<VelocityDeployConfig> velocity_deploy_config_;
  VelocityPolicyInference* loco_lower_policy_{nullptr};
  std::optional<LocoLowerDeployConfig> loco_lower_deploy_config_;
  std::optional<LocoUpperLowCmdComposerConfig> loco_upper_composer_config_;
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
  std::optional<LocoUpperRuntimeState> loco_upper_;
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
  double general_tracker_policy_phase_s_{0.0};
  double general_tracker_next_policy_time_s_{0.0};
  double active_playback_time_s_{0.0};
  std::size_t policy_startup_hold_total_steps_{0};
  std::size_t policy_startup_hold_steps_remaining_{0};
  std::vector<float> policy_startup_upper_body_start_q_;
  std::vector<float> policy_startup_upper_body_target_q_;
  std::size_t velocity_policy_ticks_until_next_{0};
  bool fail_next_transition_start_for_test_{false};
  bool fault_next_transition_start_for_test_{false};
  std::optional<double> forced_next_root_yaw_residual_for_test_;
  std::optional<LowCmdFrame> lowcmd_buffer_;
  std::optional<LowStateSample> latest_low_state_;
  std::optional<HighStateSample> latest_high_state_;
};

}  // namespace agentic_et1_tracker
