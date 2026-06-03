#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "agentic_et1_tracker/policy/observation_builder.hpp"
#include "agentic_et1_tracker/trk/synthetic_transition.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr double kSyntheticTransitionDurationS = 0.30;

bool isStartupOrWaitingError(ErrorCode error) {
  switch (error) {
    case ErrorCode::Ok:
    case ErrorCode::ServiceNotReady:
    case ErrorCode::RobotNotReady:
    case ErrorCode::RobotDisconnected:
    case ErrorCode::ModelNotReady:
      return true;
    default:
      return false;
  }
}

ServiceHealth healthStateForSnapshot(const StatusSnapshot& snapshot) {
  if (snapshot.ready && snapshot.err == ErrorCode::Ok && snapshot.block.empty()) {
    return ServiceHealth::Ready;
  }
  if (snapshot.ctrl == ControllerState::Fault || snapshot.robot == RobotState::Fault) {
    return ServiceHealth::Error;
  }
  if (!snapshot.ready && isStartupOrWaitingError(snapshot.err)) {
    return ServiceHealth::Starting;
  }
  return ServiceHealth::Error;
}

HealthSnapshot healthFromSnapshot(const StatusSnapshot& snapshot) {
  HealthSnapshot health;
  health.state = healthStateForSnapshot(snapshot);
  health.mode = snapshot.mode;
  health.err = snapshot.err;
  health.block = snapshot.block;
  return health;
}

}  // namespace

RuntimeControlLoop::RuntimeControlLoop(RuntimeConfig config,
                                       RuntimeBridge& bridge,
                                       RuntimeStatusStore& status,
                                       TrkLoader loader,
                                       ReferenceFrameSink* reference_sink,
                                       std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink) {
  clearReference();
  publishSnapshot();
}

RuntimeControlLoop::RuntimeControlLoop(RuntimeConfig config,
                                       RuntimeBridge& bridge,
                                       RuntimeStatusStore& status,
                                       TrkLoader loader,
                                       RobotIO& robot_io,
                                       PolicyInference& policy,
                                       DeployConfig deploy_config,
                                       PassiveConfig passive_config,
                                       std::uint8_t expected_mode_machine,
                                       RuntimeMode mode,
                                       ReferenceFrameSink* reference_sink,
                                       std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink),
      robot_io_(&robot_io),
      policy_(&policy),
      deploy_config_(std::move(deploy_config)),
      passive_config_(std::move(passive_config)),
      expected_mode_machine_(expected_mode_machine),
      mode_(mode) {
  runtime_state_.ready = false;
  runtime_state_.robot = RobotState::Disconnected;
  runtime_state_.err = ErrorCode::ServiceNotReady;
  runtime_state_.block = "runtime_not_started";
  clearReference();
  publishSnapshot();
}

RuntimeControlLoop::RuntimeControlLoop(RuntimeConfig config,
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
                                       RuntimeMode mode,
                                       ReferenceFrameSink* reference_sink,
                                       std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink),
      robot_io_(&robot_io),
      policy_(&policy),
      deploy_config_(std::move(deploy_config)),
      velocity_policy_(&velocity_policy),
      velocity_deploy_config_(std::move(velocity_deploy_config)),
      fixstand_config_(std::move(fixstand_config)),
      passive_config_(std::move(passive_config)),
      expected_mode_machine_(expected_mode_machine),
      mode_(mode),
      post_stop_control_(startup_control) {
  runtime_state_.ready = false;
  runtime_state_.robot = RobotState::Disconnected;
  runtime_state_.err = ErrorCode::ServiceNotReady;
  runtime_state_.block = "runtime_not_started";
  fixstand_runner_.emplace(*fixstand_config_, expected_mode_machine_, config_.hz);
  velocity_runner_.emplace(*velocity_deploy_config_, expected_mode_machine_);
  if (startup_control == ControlMode::FixStand) {
    enterFixStandState();
  } else {
    enterVelocityState();
  }
  clearReference();
  publishSnapshot();
}

RuntimeInternalState RuntimeControlLoop::internalStateForTest() const {
  return fsm_state_;
}

void RuntimeControlLoop::failNextTransitionStartForTest() {
  fail_next_transition_start_for_test_ = true;
}

void RuntimeControlLoop::tick() {
  if (fsm_state_ == RuntimeInternalState::Stopping) {
    consumeStoppingCommands();
    if (hasPolicyRuntime() && stopping_hold_ticks_remaining_ > 0) {
      if (!writeStoppingHold()) {
        publishSnapshot();
        return;
      }
      --stopping_hold_ticks_remaining_;
    }
    if (stop_to_idle_pending_ && stopping_hold_ticks_remaining_ == 0) {
      completeStoppingActive(MotionState::Stopped, ErrorCode::Ok);
      stop_to_idle_pending_ = false;
      stop_reason_ = StopReason::None;
      if (!hasControlRuntime()) {
        enterGeneralTrackerIdleState();
      } else if (post_stop_control_ == ControlMode::FixStand) {
        enterFixStandState();
      } else {
        enterGeneralTrackerIdleState();
      }
    }
    publishSnapshot();
    return;
  }

  if (consumePendingCommands()) {
    publishSnapshot();
    return;
  }

  if (fsm_state_ == RuntimeInternalState::GeneralTrackerActive ||
      fsm_state_ == RuntimeInternalState::GeneralTrackerTransition) {
    if (ctrl_ == ControllerState::Preparing) {
      completePreparing();
    } else {
      advanceActive();
    }
    publishSnapshot();
    return;
  }

  if (fsm_state_ == RuntimeInternalState::Passive) {
    runPassiveState();
  } else if (isMotionAcceptingState() && !waiting_.empty()) {
    startNext();
    publishSnapshot();
    return;
  } else if (hasIdleStartCandidate()) {
    refreshReadinessForPolicyRuntime();
    if (canStartIdle()) {
      startIdle();
    }
    publishSnapshot();
    return;
  } else if (hasControlRuntime() && isControlPublishingState() && waiting_.empty()) {
    publishControlIfReady();
  } else if (fsm_state_ == RuntimeInternalState::GeneralTrackerIdle && waiting_.empty()) {
    refreshReadinessForPolicyRuntime();
    publishIdleHoldIfReady();
  }
  publishSnapshot();
}

bool RuntimeControlLoop::consumePendingCommands() {
  while (auto command = bridge_.consumeNextCommand()) {
    switch (command->kind) {
      case CommandKind::Stop:
        handleStop(command->sequence, command->stop_requires_stopping);
        return true;
      case CommandKind::Passive:
        handleControl(ControlMode::Passive);
        return true;
      case CommandKind::FixStand:
        handleControl(ControlMode::FixStand);
        if (fsm_state_ == RuntimeInternalState::Stopping) {
          return true;
        }
        break;
      case CommandKind::StandbyVelocity:
        handleControl(ControlMode::StandbyVelocity);
        if (fsm_state_ == RuntimeInternalState::Stopping) {
          return true;
        }
        break;
      case CommandKind::IdleConfig:
        handleIdleConfig(std::move(command->idle_motions));
        return true;
      case CommandKind::Interrupt:
        handleInterrupt(std::move(command->request));
        return fsm_state_ == RuntimeInternalState::Stopping;
      case CommandKind::Queue:
        if (active_kind_ == ActiveKind::Idle) {
          stopIdleActive();
        }
        waiting_.push_back(std::move(command->request));
        break;
    }
  }
  return false;
}

void RuntimeControlLoop::consumeStoppingCommands() {
  while (auto command = bridge_.consumeNextCommand()) {
    switch (command->kind) {
      case CommandKind::Stop:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::StandbyVelocity;
        break;
      case CommandKind::Passive:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::Passive;
        handleControl(ControlMode::Passive);
        break;
      case CommandKind::FixStand:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::FixStand;
        break;
      case CommandKind::StandbyVelocity:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::StandbyVelocity;
        break;
      case CommandKind::IdleConfig:
        handleIdleConfig(std::move(command->idle_motions));
        break;
      case CommandKind::Interrupt:
        cancelWaiting(StopReason::Interrupt, command->sequence);
        waiting_.push_back(std::move(command->request));
        break;
      case CommandKind::Queue:
        waiting_.push_back(std::move(command->request));
        break;
    }
  }
}

void RuntimeControlLoop::handleStop(std::uint64_t sequence, bool requires_stopping) {
  cancelWaiting(StopReason::Stop, sequence);
  idle_config_.clear();
  idle_next_index_ = 0;
  if (fsm_state_ == RuntimeInternalState::Fault) {
    return;
  }
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    if (hasControlRuntime()) {
      enterGeneralTrackerIdleState();
    }
    return;
  }
  if (active_kind_ == ActiveKind::Transition) {
    abortTransition();
    post_stop_control_ = ControlMode::StandbyVelocity;
    stop_reason_ = StopReason::Stop;
    enterStopping(StopReason::Stop);
    return;
  }
  if (fsm_state_ == RuntimeInternalState::Passive) {
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }
  post_stop_control_ = ControlMode::StandbyVelocity;
  if (active_) {
    markActiveStopping(StopReason::Stop);
  } else if (fsm_state_ == RuntimeInternalState::FixStand && waiting_.empty()) {
    policy_runner_.reset();
    handleInternalEvent(RuntimeInternalEvent::Velocity);
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  } else if (!requires_stopping && waiting_.empty() &&
             (fsm_state_ == RuntimeInternalState::Velocity ||
              fsm_state_ == RuntimeInternalState::GeneralTrackerIdle)) {
    policy_runner_.reset();
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }
  stop_reason_ = StopReason::Stop;
  enterStopping(StopReason::Stop);
}

void RuntimeControlLoop::handleControl(ControlMode mode) {
  if (mode == ControlMode::Passive) {
    cancelWaiting(StopReason::Stop);
    idle_config_.clear();
    idle_next_index_ = 0;
    idle_current_index_.reset();
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    } else if (active_kind_ == ActiveKind::Transition) {
      abortTransition(MotionState::Canceled, StopReason::Stop, ErrorCode::Ok);
    } else if (active_) {
      finishActive(MotionState::Stopped, StopReason::Stop, ErrorCode::Ok);
    }
    active_.reset();
    active_track_.reset();
    policy_runner_.reset();
    clearReference();
    post_stop_control_ = ControlMode::Passive;
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    handleInternalEvent(RuntimeInternalEvent::SafetyPassive);
    return;
  }
  if (mode == ControlMode::StandbyVelocity && active_kind_ == ActiveKind::User &&
      active_ && active_->state == MotionState::Holding) {
    if (startTransitionFromHoldingToStandby()) {
      return;
    }
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    enterGeneralTrackerIdleState();
    return;
  }
  if (fsm_state_ == RuntimeInternalState::Fault && mode != ControlMode::FixStand) {
    return;
  }
  if (fsm_state_ == RuntimeInternalState::Passive &&
      mode != ControlMode::FixStand) {
    return;
  }

  if (mode == ControlMode::FixStand) {
    cancelWaiting(StopReason::Stop);
  }
  post_stop_control_ = mode;
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
  } else if (active_kind_ == ActiveKind::Transition) {
    abortTransition();
  }
  if (fsm_state_ == RuntimeInternalState::Fault) {
    active_.reset();
    active_track_.reset();
    policy_runner_.reset();
    clearReference();
    handleInternalEvent(RuntimeInternalEvent::FixStand);
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }

  if (active_) {
    markActiveStopping(StopReason::Stop);
    enterStopping(StopReason::Stop);
    return;
  }

  policy_runner_.reset();
  if (mode == ControlMode::FixStand) {
    handleInternalEvent(RuntimeInternalEvent::FixStand);
  } else {
    handleInternalEvent(RuntimeInternalEvent::Velocity);
  }
  stop_reason_ = StopReason::None;
  stop_to_idle_pending_ = false;
}

void RuntimeControlLoop::handleIdleConfig(std::vector<IdleMotion> motions) {
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
  }
  idle_config_ = std::move(motions);
  idle_next_index_ = 0;
  if (idle_config_.empty()) {
    idle_current_index_.reset();
    return;
  }
  if (isMotionAcceptingState()) {
    refreshReadinessForPolicyRuntime();
  }
}

void RuntimeControlLoop::handleInterrupt(MotionRequest request) {
  cancelWaiting(StopReason::Interrupt);
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
  }
  if (active_kind_ == ActiveKind::Transition) {
    startTransitionFromCurrentReferenceToUser(std::move(request),
                                              StopReason::Interrupt);
    return;
  }
  waiting_.push_back(std::move(request));
  if (active_kind_ == ActiveKind::User && active_ &&
      active_->state == MotionState::Holding) {
    return;
  }
  if (active_) {
    markActiveStopping(StopReason::Interrupt);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterStopping(StopReason::Interrupt);
  }
}

void RuntimeControlLoop::cancelWaiting(StopReason reason) {
  for (auto& request : waiting_) {
    request.state = MotionState::Canceled;
    request.stop_reason = reason;
    request.err = ErrorCode::Ok;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
  }
  waiting_.clear();
}

void RuntimeControlLoop::cancelWaiting(StopReason reason, std::uint64_t sequence) {
  std::deque<MotionRequest> remaining;
  for (auto& request : waiting_) {
    if (request.sequence > sequence) {
      remaining.push_back(std::move(request));
      continue;
    }

    request.state = MotionState::Canceled;
    request.stop_reason = reason;
    request.err = ErrorCode::Ok;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
  }
  waiting_ = std::move(remaining);
}

ControllerState RuntimeControlLoop::controllerStateForInternal(
    RuntimeInternalState state) const {
  switch (state) {
    case RuntimeInternalState::Passive:
      return ControllerState::Passive;
    case RuntimeInternalState::FixStand:
      return ControllerState::FixStand;
    case RuntimeInternalState::Velocity:
      return ControllerState::StandbyVelocity;
    case RuntimeInternalState::GeneralTrackerIdle:
      return hasControlRuntime() ? ControllerState::StandbyVelocity : ControllerState::Idle;
    case RuntimeInternalState::GeneralTrackerActive:
    case RuntimeInternalState::GeneralTrackerTransition:
      return ControllerState::Running;
    case RuntimeInternalState::Stopping:
      return ControllerState::Stopping;
    case RuntimeInternalState::Fault:
      return ControllerState::Fault;
  }
  return ControllerState::Fault;
}

void RuntimeControlLoop::enterInternalState(RuntimeInternalState state) {
  fsm_state_ = state;
  ctrl_ = controllerStateForInternal(state);
  switch (state) {
    case RuntimeInternalState::Passive:
      active_policy_ticks_until_next_ = 0;
      velocity_policy_ticks_until_next_ = 0;
      break;
    case RuntimeInternalState::FixStand:
      active_policy_ticks_until_next_ = 0;
      velocity_policy_ticks_until_next_ = 0;
      if (fixstand_runner_) {
        fixstand_runner_->reset();
      }
      break;
    case RuntimeInternalState::Velocity:
    case RuntimeInternalState::GeneralTrackerIdle:
      active_policy_ticks_until_next_ = 0;
      velocity_policy_ticks_until_next_ = 0;
      if (velocity_runner_) {
        velocity_runner_->reset();
      }
      break;
    case RuntimeInternalState::GeneralTrackerActive:
    case RuntimeInternalState::GeneralTrackerTransition:
      active_policy_ticks_until_next_ = 0;
      active_first_advance_ = true;
      break;
    case RuntimeInternalState::Stopping:
      break;
    case RuntimeInternalState::Fault:
      active_policy_ticks_until_next_ = 0;
      velocity_policy_ticks_until_next_ = 0;
      break;
  }
}

void RuntimeControlLoop::handleInternalEvent(RuntimeInternalEvent event) {
  switch (event) {
    case RuntimeInternalEvent::FixStand:
      enterFixStandState();
      break;
    case RuntimeInternalEvent::Velocity:
      enterVelocityState();
      break;
    case RuntimeInternalEvent::MotionRequest:
      enterTrackPreparingState();
      break;
    case RuntimeInternalEvent::SafetyPassive:
      enterInternalState(RuntimeInternalState::Passive);
      break;
    case RuntimeInternalEvent::Fault:
      enterInternalState(RuntimeInternalState::Fault);
      break;
  }
}

void RuntimeControlLoop::enterPassiveState(const RobotReadinessStatus& readiness) {
  handleInternalEvent(RuntimeInternalEvent::SafetyPassive);
  applyReadiness(readiness);
}

void RuntimeControlLoop::enterFixStandState() {
  enterInternalState(RuntimeInternalState::FixStand);
}

void RuntimeControlLoop::enterVelocityState() {
  enterInternalState(RuntimeInternalState::Velocity);
}

void RuntimeControlLoop::enterGeneralTrackerIdleState() {
  enterInternalState(RuntimeInternalState::GeneralTrackerIdle);
}

void RuntimeControlLoop::enterTrackPreparingState() {
  enterInternalState(RuntimeInternalState::GeneralTrackerActive);
  ctrl_ = ControllerState::Preparing;
}

void RuntimeControlLoop::enterTrackActiveState() {
  enterInternalState(RuntimeInternalState::GeneralTrackerActive);
}

bool RuntimeControlLoop::isMotionAcceptingState() const {
  return fsm_state_ == RuntimeInternalState::Velocity ||
         fsm_state_ == RuntimeInternalState::GeneralTrackerIdle;
}

bool RuntimeControlLoop::isControlPublishingState() const {
  return fsm_state_ == RuntimeInternalState::FixStand ||
         fsm_state_ == RuntimeInternalState::Velocity ||
         fsm_state_ == RuntimeInternalState::GeneralTrackerIdle;
}

void RuntimeControlLoop::runPassiveState() {
  if (!hasPolicyRuntime()) {
    return;
  }

  writePassiveDamping();
}

bool RuntimeControlLoop::writePassiveDamping() {
  if (!passive_config_) {
    return false;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const LowCmdOccupancy occupancy = robot_io_->lowCmdOccupancy();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, occupancy, expected_mode_machine_);
  if (readinessRequiresFault(readiness)) {
    enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    return false;
  }
  applyReadiness(readiness);

  if (!low_state.has_value()) {
    return false;
  }

  try {
    const LowCmdFrame frame =
        makePassiveLowCmdFrame(*passive_config_, *low_state, expected_mode_machine_);
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }
  return true;
}

void RuntimeControlLoop::startNext() {
  std::optional<LowStateSample> entry_low_state;
  std::optional<RobotReadinessStatus> readiness;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    applyReadiness(*readiness);
    if (readiness->err != ErrorCode::Ok) {
      if (readinessRequiresFault(*readiness)) {
        enterFault(readiness->err,
                   readiness->robot,
                   readiness->block,
                   readiness->low_ms);
      } else {
        enterPassiveState(*readiness);
      }
      return;
    }
  }

  MotionRequest request = std::move(waiting_.front());
  waiting_.pop_front();
  request.state = MotionState::Queued;
  request.frame = 0;
  request.err = ErrorCode::Ok;
  request.stop_reason = StopReason::None;
  active_ = std::move(request);
  active_kind_ = ActiveKind::User;
  idle_current_index_.reset();
  handleInternalEvent(RuntimeInternalEvent::MotionRequest);
  publishActive();
}

bool RuntimeControlLoop::canStartIdle() const {
  if (idle_config_.empty() || active_ || !waiting_.empty()) {
    return false;
  }
  if (!isMotionAcceptingState()) {
    return false;
  }
  if (hasPolicyRuntime()) {
    return runtime_state_.ready && runtime_state_.err == ErrorCode::Ok;
  }
  return true;
}

bool RuntimeControlLoop::hasIdleStartCandidate() const {
  return !idle_config_.empty() && !active_ && waiting_.empty() &&
         isMotionAcceptingState();
}

void RuntimeControlLoop::startIdle() {
  if (!canStartIdle()) {
    return;
  }

  if (idle_next_index_ >= idle_config_.size()) {
    idle_next_index_ = 0;
  }
  const std::size_t index = idle_next_index_;
  idle_next_index_ = (idle_next_index_ + 1) % idle_config_.size();
  const IdleMotion& motion = idle_config_.at(index);

  MotionRequest request;
  request.id.clear();
  request.path = motion.path;
  request.state = MotionState::Queued;
  request.frame = 0;
  request.frames = motion.track.frames;
  request.fps = motion.track.fps;
  request.duration_s = motion.track.duration_s;
  request.err = ErrorCode::Ok;
  request.stop_reason = StopReason::None;
  active_ = std::move(request);
  active_kind_ = ActiveKind::Idle;
  idle_current_index_ = index;
  handleInternalEvent(RuntimeInternalEvent::MotionRequest);
}

void RuntimeControlLoop::completePreparing() {
  if (!active_) {
    active_track_.reset();
    clearReference();
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  std::optional<LowStateSample> entry_low_state;
  std::optional<RobotReadinessStatus> readiness;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    applyReadiness(*readiness);
    if (readiness->err != ErrorCode::Ok) {
      if (active_kind_ == ActiveKind::Idle) {
        stopIdleActive();
        if (readinessRequiresFault(*readiness)) {
          enterFault(readiness->err,
                     readiness->robot,
                     readiness->block,
                     readiness->low_ms);
        } else {
          enterPassiveState(*readiness);
        }
        return;
      }
      if (readinessRequiresFault(*readiness)) {
        failActiveWithFault(readiness->err,
                            readiness->robot,
                            readiness->block,
                            readiness->low_ms);
        return;
      }

      MotionRequest request = std::move(*active_);
      request.state = MotionState::Queued;
      request.frame = 0;
      request.err = ErrorCode::Ok;
      request.stop_reason = StopReason::None;
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      waiting_.push_front(std::move(request));
      status_.publishRunStatus(toStatus(waiting_.front()));
      enterPassiveState(*readiness);
      return;
    }
  }

  TrkLoadResult loaded = loader_.load(active_->path);
  if (!loaded.ok()) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
      return;
    }
    active_->state = MotionState::Failed;
    active_->frame = 0;
    active_->err = toCoreErrorCode(loaded.code);
    active_->stop_reason = StopReason::None;
    active_->ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(*active_));
    active_.reset();
    active_track_.reset();
    clearReference();
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  const std::size_t loaded_frames = loaded.track->metadata.frames;
  const double loaded_duration_s = loaded.track->metadata.duration_s;
  active_->frames = loaded_frames;
  active_->fps = loaded.track->metadata.fps;
  active_->duration_s = loaded_duration_s;
  auto track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  active_track_ = track;
  if (hasPolicyRuntime()) {
    try {
      policy_runner_.emplace(*deploy_config_,
                             track,
                             *entry_low_state,
                             expected_mode_machine_);
    } catch (const std::exception&) {
      if (active_kind_ == ActiveKind::Idle) {
        stopIdleActive();
        post_stop_control_ = ControlMode::StandbyVelocity;
        enterFault(ErrorCode::ModelInferenceFailed,
                   RobotState::Fault,
                   "policy_inference_failed",
                   readiness ? readiness->low_ms : std::nullopt);
        return;
      }
      active_->state = MotionState::Failed;
      active_->frame = 0;
      active_->err = ErrorCode::ModelInferenceFailed;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 readiness ? readiness->low_ms : std::nullopt);
      return;
    }
  }

  active_->state = MotionState::Running;
  active_->frame = 0;
  active_->err = ErrorCode::Ok;
  active_->stop_reason = StopReason::None;
  active_->started_at = std::chrono::steady_clock::now();
  enterTrackActiveState();
  if (active_kind_ == ActiveKind::User) {
    publishReferenceActive();
  }
  publishActive();
}

void RuntimeControlLoop::advanceActive() {
  if (active_kind_ == ActiveKind::Transition) {
    if (hasPolicyRuntime()) {
      advanceTransitionWithPolicy();
    } else {
      advanceTransition();
    }
    return;
  }

  if (hasPolicyRuntime()) {
    advanceActiveWithPolicy();
    return;
  }

  if (!active_) {
    active_track_.reset();
    clearReference();
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    transition_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  if (active_kind_ == ActiveKind::User &&
      active_->state == MotionState::Holding) {
    advanceHolding();
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_first_advance_ = false;
    active_->frame = 0;
  } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
    ++active_->frame;
  }

  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    const std::size_t last_frame = active_->frames == 0 ? 0 : active_->frames - 1;
    if (active_->frame < last_frame) {
      active_->frame = last_frame;
      if (active_kind_ == ActiveKind::User) {
        publishReferenceActive();
      }
      publishActive();
      return;
    }
    active_->frame = last_frame;
    if (active_kind_ == ActiveKind::User) {
      publishReferenceActive();
    }
    if (active_kind_ == ActiveKind::User && active_->hold) {
      active_->state = MotionState::Holding;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      publishActive();
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToIdle()) {
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToStandby()) {
      return;
    }
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  publishActive();
  if (active_kind_ == ActiveKind::User) {
    publishReferenceActive();
  }
}

void RuntimeControlLoop::advanceActiveWithPolicy() {
  if (!active_) {
    active_track_.reset();
    policy_runner_.reset();
    clearReference();
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    transition_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  if (active_kind_ == ActiveKind::User &&
      active_->state == MotionState::Holding) {
    advanceHoldingWithPolicy();
    return;
  }

  if (!policy_runner_) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed");
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return;
    }
    if (readinessRequiresFault(readiness)) {
      failActiveWithFault(readiness.err,
                          readiness.robot,
                          readiness.block,
                          readiness.low_ms);
    } else {
      finishActive(MotionState::Failed, StopReason::None, readiness.err);
      enterPassiveState(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_->frame = 0;
    active_first_advance_ = false;
  } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
    ++active_->frame;
  }
  publishActive();
  if (active_kind_ == ActiveKind::User) {
    publishReferenceActive();
  }

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 readiness.low_ms);
    } else {
      failActiveWithFault(ErrorCode::ModelInferenceFailed,
                          RobotState::Fault,
                          "policy_inference_failed",
                          readiness.low_ms);
    }
    return;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      enterFault(ErrorCode::InternalError,
                 RobotState::Fault,
                 "lowcmd_write_failed",
                 readiness.low_ms);
    } else {
      failActiveWithFault(ErrorCode::InternalError,
                          RobotState::Fault,
                          "lowcmd_write_failed",
                          readiness.low_ms);
    }
    return;
  }

  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    const std::size_t last_frame = active_->frames == 0 ? 0 : active_->frames - 1;
    active_->frame = last_frame;
    if (active_kind_ == ActiveKind::User) {
      publishReferenceActive();
    }
    if (active_kind_ == ActiveKind::User && active_->hold) {
      active_->state = MotionState::Holding;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      publishActive();
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToIdle()) {
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToStandby()) {
      return;
    }
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  publishActive();
}

void RuntimeControlLoop::advanceHolding() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding) {
    return;
  }

  if (!waiting_.empty() && startTransitionFromHoldingToNextUser()) {
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
  publishReferenceActive();
  publishActive();
}

void RuntimeControlLoop::advanceHoldingWithPolicy() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding) {
    return;
  }

  if (!waiting_.empty() && startTransitionFromHoldingToNextUser()) {
    return;
  }

  if (!policy_runner_) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed");
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    failActiveReadiness(readiness);
    return;
  }
  applyReadiness(readiness);

  active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
  publishActive();
  publishReferenceActive();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed",
                        readiness.low_ms);
    return;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "lowcmd_write_failed",
                        readiness.low_ms);
    return;
  }

  publishActive();
}

bool RuntimeControlLoop::startTransitionFromHoldingToNextUser() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding || !active_track_ ||
      waiting_.empty()) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      failActiveReadiness(readiness);
      return true;
    }
    applyReadiness(readiness);
  }

  MotionRequest target_request = std::move(waiting_.front());
  waiting_.pop_front();
  target_request.state = MotionState::Queued;
  target_request.frame = 0;
  target_request.err = ErrorCode::Ok;
  target_request.stop_reason = StopReason::None;

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    target_request.state = MotionState::Failed;
    target_request.frame = 0;
    target_request.err = toCoreErrorCode(loaded.code);
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    return true;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!source_frame || !target_frame) {
    target_request.state = MotionState::Failed;
    target_request.err = ErrorCode::InternalError;
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(target_track);
  target.target_request = std::move(target_request);
  const double transition_fps = target.target_track->metadata.fps;
  return startSyntheticTransitionFromActiveFrame(std::move(target),
                                                 *target_frame,
                                                 transition_fps);
}

bool RuntimeControlLoop::startTransitionFromHoldingToStandby() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding || !active_track_ ||
      !waiting_.empty() || !standby_track_) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      failActiveReadiness(readiness);
      return true;
    }
    applyReadiness(readiness);
  }

  const std::optional<TrkFrameView> target_frame = standby_track_->frame(0);
  if (!target_frame) {
    return false;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Standby;
  target.target_track = standby_track_;
  return startSyntheticTransitionFromActiveFrame(std::move(target),
                                                 *target_frame,
                                                 standby_track_->metadata.fps);
}

bool RuntimeControlLoop::startTransitionFromCurrentReferenceToUser(
    MotionRequest target_request,
    StopReason replaced_reason) {
  if (active_kind_ != ActiveKind::Transition || !active_ || !active_track_) {
    return false;
  }

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    target_request.state = MotionState::Failed;
    target_request.frame = 0;
    target_request.err = ErrorCode::InternalError;
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
    enterGeneralTrackerIdleState();
    return true;
  }

  target_request.state = MotionState::Queued;
  target_request.frame = 0;
  target_request.err = ErrorCode::Ok;
  target_request.stop_reason = StopReason::None;

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    target_request.state = MotionState::Failed;
    target_request.frame = 0;
    target_request.err = toCoreErrorCode(loaded.code);
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
    enterGeneralTrackerIdleState();
    return true;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!target_frame) {
    target_request.state = MotionState::Failed;
    target_request.err = ErrorCode::InternalError;
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
    enterGeneralTrackerIdleState();
    return true;
  }

  std::optional<TrkTrack> transition_track =
      makeSyntheticTransitionTrk(*source_frame,
                                 *target_frame,
                                 target_track->metadata.fps,
                                 kSyntheticTransitionDurationS);
  if (!transition_track) {
    target_request.state = MotionState::Failed;
    target_request.err = ErrorCode::InternalError;
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
    enterGeneralTrackerIdleState();
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(target_track);
  target.target_request = std::move(target_request);

  auto transition_track_ptr =
      std::make_shared<TrkTrack>(std::move(*transition_track));
  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = latest_low_state_;
    if (!entry_low_state) {
      entry_low_state = readLowStateForStatus();
    }
  }
  finishTransitionTarget(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
  const bool started = startInternalTransition(std::move(transition_track_ptr),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (!started) {
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
  }
  return true;
}

bool RuntimeControlLoop::startTransitionFromCompletedUserToIdle() {
  if (!active_ || active_kind_ != ActiveKind::User || active_->hold ||
      !active_track_ || !waiting_.empty() || idle_config_.empty()) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      active_->state = MotionState::Done;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  if (idle_next_index_ >= idle_config_.size()) {
    idle_next_index_ = 0;
  }
  const std::size_t index = idle_next_index_;
  idle_next_index_ = (idle_next_index_ + 1) % idle_config_.size();
  const IdleMotion& motion = idle_config_.at(index);

  TrkLoadResult loaded = loader_.load(motion.path);
  if (!loaded.ok()) {
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return true;
  }

  MotionRequest idle_request;
  idle_request.id.clear();
  idle_request.path = motion.path;
  idle_request.state = MotionState::Queued;
  idle_request.frame = 0;
  idle_request.frames = loaded.track->metadata.frames;
  idle_request.fps = loaded.track->metadata.fps;
  idle_request.duration_s = loaded.track->metadata.duration_s;
  idle_request.err = ErrorCode::Ok;
  idle_request.stop_reason = StopReason::None;

  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!target_frame) {
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Idle;
  target.target_id.clear();
  target.target_state = MotionState::Running;
  target.target_track = std::move(target_track);
  target.target_request = std::move(idle_request);
  target.idle_index = index;
  const double transition_fps = target.target_track->metadata.fps;
  return startSyntheticTransitionFromActiveFrame(std::move(target),
                                                 *target_frame,
                                                 transition_fps);
}

bool RuntimeControlLoop::startTransitionFromCompletedUserToStandby() {
  if (!active_ || active_kind_ != ActiveKind::User || active_->hold ||
      !active_track_ || !waiting_.empty() || !idle_config_.empty() ||
      !standby_track_) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      active_->state = MotionState::Done;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  const std::optional<TrkFrameView> target_frame = standby_track_->frame(0);
  if (!target_frame) {
    return false;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Standby;
  target.target_track = standby_track_;
  return startSyntheticTransitionFromActiveFrame(std::move(target),
                                                 *target_frame,
                                                 standby_track_->metadata.fps);
}

bool RuntimeControlLoop::startSyntheticTransitionFromActiveFrame(
    PendingTransition target,
    const TrkFrameView& target_frame,
    double target_fps) {
  if (!active_ || !active_track_) {
    return false;
  }

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    return false;
  }

  std::optional<TrkTrack> transition_track =
      makeSyntheticTransitionTrk(*source_frame,
                                 target_frame,
                                 target_fps,
                                 kSyntheticTransitionDurationS);
  if (!transition_track) {
    if (target.target_request && !target.target_request->id.empty()) {
      target.target_request->state = MotionState::Failed;
      target.target_request->err = ErrorCode::InternalError;
      target.target_request->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*target.target_request));
    } else if (target.target_kind == TransitionTargetKind::Idle &&
               active_kind_ == ActiveKind::User) {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
    } else if (target.target_kind == TransitionTargetKind::Standby) {
      return false;
    }
    return true;
  }

  std::optional<MotionRequest> source_to_complete;
  if (active_kind_ == ActiveKind::User) {
    source_to_complete = active_;
  }

  auto transition_track_ptr =
      std::make_shared<TrkTrack>(std::move(*transition_track));
  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = latest_low_state_;
    if (!entry_low_state) {
      entry_low_state = readLowStateForStatus();
    }
  }
  const bool started = startInternalTransition(std::move(transition_track_ptr),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (started && source_to_complete) {
    source_to_complete->state = MotionState::Done;
    source_to_complete->err = ErrorCode::Ok;
    source_to_complete->stop_reason = StopReason::None;
    source_to_complete->ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(*source_to_complete));
  }
  return true;
}

bool RuntimeControlLoop::startInternalTransition(
    std::shared_ptr<const TrkTrack> track,
    PendingTransition target,
    std::optional<LowStateSample> entry_low_state) {
  if (fail_next_transition_start_for_test_) {
    fail_next_transition_start_for_test_ = false;
    if (target.target_request && !target.target_request->id.empty()) {
      target.target_request->state = MotionState::Failed;
      target.target_request->err = ErrorCode::InternalError;
      target.target_request->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*target.target_request));
    }
    return false;
  }

  std::optional<PolicyStepRunner> runner;
  if (hasPolicyRuntime()) {
    if (!entry_low_state) {
      if (target.target_request && !target.target_request->id.empty()) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = ErrorCode::RobotNotReady;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      failActiveWithFault(ErrorCode::RobotNotReady,
                          RobotState::NotReady,
                          "low_state_missing");
      return false;
    }
    try {
      runner.emplace(*deploy_config_, track, *entry_low_state, expected_mode_machine_);
    } catch (const std::exception&) {
      if (target.target_request && !target.target_request->id.empty()) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = ErrorCode::ModelInferenceFailed;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 runtime_state_.low_ms);
      return false;
    }
  }

  MotionRequest transition_request;
  transition_request.state = MotionState::Running;
  transition_request.frame = 0;
  transition_request.frames = track->metadata.frames;
  transition_request.fps = track->metadata.fps;
  transition_request.duration_s = track->metadata.duration_s;
  transition_request.err = ErrorCode::Ok;
  transition_request.stop_reason = StopReason::None;
  transition_request.started_at = std::chrono::steady_clock::now();

  active_ = std::move(transition_request);
  active_kind_ = ActiveKind::Transition;
  active_track_ = std::move(track);
  transition_ = std::move(target);
  idle_current_index_.reset();
  policy_runner_ = std::move(runner);
  enterInternalState(RuntimeInternalState::GeneralTrackerTransition);
  publishReferenceTransition();
  return true;
}

bool RuntimeControlLoop::startStandbyPlayback(PendingTransition target) {
  if (target.target_kind != TransitionTargetKind::Standby ||
      !target.target_track) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  std::optional<PolicyStepRunner> runner;
  if (hasPolicyRuntime()) {
    try {
      runner.emplace(*deploy_config_,
                     target.target_track,
                     *entry_low_state,
                     expected_mode_machine_);
    } catch (const std::exception&) {
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 runtime_state_.low_ms);
      return true;
    }
  }

  MotionRequest playback_request;
  playback_request.state = MotionState::Running;
  playback_request.frame = 0;
  playback_request.frames = target.target_track->metadata.frames;
  playback_request.fps = target.target_track->metadata.fps;
  playback_request.duration_s = target.target_track->metadata.duration_s;
  playback_request.err = ErrorCode::Ok;
  playback_request.stop_reason = StopReason::None;
  playback_request.started_at = std::chrono::steady_clock::now();

  PendingTransition playback_target;
  playback_target.target_kind = TransitionTargetKind::Standby;

  active_ = std::move(playback_request);
  active_kind_ = ActiveKind::Transition;
  active_track_ = std::move(target.target_track);
  transition_ = std::move(playback_target);
  idle_current_index_.reset();
  policy_runner_ = std::move(runner);
  enterInternalState(RuntimeInternalState::GeneralTrackerTransition);
  publishReferenceTransition();
  return true;
}

void RuntimeControlLoop::advanceTransition() {
  if (!active_ || active_kind_ != ActiveKind::Transition || !active_track_) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::InternalError);
    enterGeneralTrackerIdleState();
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_->frame = 0;
    active_first_advance_ = false;
  } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
    ++active_->frame;
  }

  publishReferenceTransition();
  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
    publishReferenceTransition();
    completeTransition();
  }
}

void RuntimeControlLoop::advanceTransitionWithPolicy() {
  if (!active_ || active_kind_ != ActiveKind::Transition || !active_track_) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::InternalError);
    enterGeneralTrackerIdleState();
    return;
  }

  if (!policy_runner_) {
    enterFault(ErrorCode::ModelInferenceFailed,
               RobotState::Fault,
               "policy_inference_failed");
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::ModelInferenceFailed);
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    abortTransition(MotionState::Failed, StopReason::None, readiness.err);
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_->frame = 0;
    active_first_advance_ = false;
  } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
    ++active_->frame;
  }
  publishReferenceTransition();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::ModelInferenceFailed);
    enterFault(ErrorCode::ModelInferenceFailed,
               RobotState::Fault,
               "policy_inference_failed",
               readiness.low_ms);
    return;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::InternalError);
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return;
  }

  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
    publishReferenceTransition();
    completeTransition();
  }
}

void RuntimeControlLoop::completeTransition() {
  if (!transition_) {
    abortTransition();
    enterGeneralTrackerIdleState();
    return;
  }

  PendingTransition target = std::move(*transition_);
  transition_.reset();
  active_.reset();
  active_track_.reset();
  policy_runner_.reset();
  clearReference();

  if (target.target_kind == TransitionTargetKind::Standby) {
    if (target.target_track && startStandbyPlayback(std::move(target))) {
      return;
    }
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  if (!target.target_request || !target.target_track) {
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      if (target.target_kind == TransitionTargetKind::User) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = readiness.err;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return;
    }
    applyReadiness(readiness);
  }

  std::optional<PolicyStepRunner> runner;
  if (hasPolicyRuntime()) {
    try {
      runner.emplace(*deploy_config_,
                     target.target_track,
                     *entry_low_state,
                     expected_mode_machine_);
    } catch (const std::exception&) {
      if (target.target_kind == TransitionTargetKind::User) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = ErrorCode::ModelInferenceFailed;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 runtime_state_.low_ms);
      return;
    }
  }

  target.target_request->state = MotionState::Running;
  target.target_request->frame = 0;
  target.target_request->err = ErrorCode::Ok;
  target.target_request->stop_reason = StopReason::None;
  target.target_request->started_at = std::chrono::steady_clock::now();
  active_ = std::move(*target.target_request);
  active_track_ = std::move(target.target_track);
  policy_runner_ = std::move(runner);
  active_kind_ = target.target_kind == TransitionTargetKind::User ? ActiveKind::User
                                                                  : ActiveKind::Idle;
  idle_current_index_ = target.target_kind == TransitionTargetKind::Idle
                            ? target.idle_index
                            : std::optional<std::size_t>{};
  enterTrackActiveState();
  if (active_kind_ == ActiveKind::User) {
    publishReferenceActive();
    publishActive();
  }
}

void RuntimeControlLoop::finishTransitionTarget(MotionState state,
                                                StopReason reason,
                                                ErrorCode error) {
  if (!transition_ || transition_->target_kind != TransitionTargetKind::User ||
      !transition_->target_request || transition_->target_request->id.empty()) {
    return;
  }

  MotionRequest target = *transition_->target_request;
  target.state = state;
  target.stop_reason = reason;
  target.err = error;
  target.ended_at = std::chrono::steady_clock::now();
  status_.publishRunStatus(toStatus(target));
  transition_->target_request = std::move(target);
}

void RuntimeControlLoop::abortTransition(MotionState target_state,
                                         StopReason reason,
                                         ErrorCode error) {
  if (active_kind_ != ActiveKind::Transition) {
    return;
  }
  finishTransitionTarget(target_state, reason, error);
  active_.reset();
  active_kind_ = ActiveKind::None;
  active_track_.reset();
  transition_.reset();
  policy_runner_.reset();
  idle_current_index_.reset();
  clearReference();
}

bool RuntimeControlLoop::failActiveReadiness(
    const RobotReadinessStatus& readiness) {
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return true;
  }
  if (readinessRequiresFault(readiness)) {
    failActiveWithFault(readiness.err,
                        readiness.robot,
                        readiness.block,
                        readiness.low_ms);
  } else {
    finishActive(MotionState::Failed, StopReason::None, readiness.err);
    enterPassiveState(readiness);
  }
  return true;
}

void RuntimeControlLoop::publishControlIfReady() {
  if (!hasControlRuntime() || active_ || !waiting_.empty()) {
    return;
  }

  if (ctrl_ == ControllerState::FixStand) {
    writeFixStand();
  } else if (ctrl_ == ControllerState::StandbyVelocity) {
    writeStandbyVelocity();
  }
}

bool RuntimeControlLoop::writeFixStand() {
  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const LowCmdOccupancy occupancy = robot_io_->lowCmdOccupancy();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, occupancy, expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    const bool can_write_bad_orientation =
        readiness.err == ErrorCode::RobotBadOrientation && low_state.has_value() &&
        low_state->fresh &&
        (low_state->mode_machine == 0 ||
         low_state->mode_machine == expected_mode_machine_) &&
        !occupancy.occupied;
    if (!can_write_bad_orientation) {
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return false;
    }
    applyReadiness(readiness);
  } else {
    applyReadiness(readiness);
  }

  LowCmdFrame frame;
  try {
    frame = fixstand_runner_->step(*low_state, lowCmdBaseFrame());
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }

  try {
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }
  return true;
}

bool RuntimeControlLoop::writeStandbyVelocity() {
  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return false;
  }
  applyReadiness(readiness);

  if (!consumeStepDue(velocity_policy_ticks_until_next_, velocityPolicyIntervalTicks())) {
    return republishLowCmdBuffer();
  }

  VelocityStepResult step;
  try {
    step = velocity_runner_->step(*low_state, *velocity_policy_, lowCmdBaseFrame());
    if (fsm_state_ == RuntimeInternalState::GeneralTrackerIdle) {
      applyGeneralTrackerIdleHold(step.low_cmd);
    }
  } catch (const std::exception&) {
    enterFault(ErrorCode::ModelInferenceFailed,
               RobotState::Fault,
               "policy_inference_failed",
               readiness.low_ms);
    return false;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }
  return true;
}

void RuntimeControlLoop::publishIdleHoldIfReady() {
  if (!hasPolicyRuntime() || ctrl_ != ControllerState::Idle || active_ ||
      !waiting_.empty() || !runtime_state_.ready || runtime_state_.err != ErrorCode::Ok) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  try {
    const LowCmdFrame frame =
        makeHoldLowCmdFrame(*deploy_config_, *low_state, expected_mode_machine_);
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
  }
}

bool RuntimeControlLoop::writeStoppingHold() {
  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    stopping_hold_ticks_remaining_ = 0;
    stop_to_idle_pending_ = false;
    failStoppingActiveWithFault(readiness.err);
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return false;
  }
  applyReadiness(readiness);

  try {
    const LowCmdFrame frame =
        makeHoldLowCmdFrame(*deploy_config_, *low_state, expected_mode_machine_);
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    stopping_hold_ticks_remaining_ = 0;
    stop_to_idle_pending_ = false;
    failStoppingActiveWithFault(ErrorCode::InternalError);
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }

  return true;
}

void RuntimeControlLoop::applyGeneralTrackerIdleHold(LowCmdFrame& frame) const {
  if (!fixstand_config_ || !velocity_deploy_config_) {
    return;
  }

  std::array<bool, kSdkMotorCount> velocity_controlled{};
  for (std::size_t i = 0; i < velocity_deploy_config_->joint_ids_map.size(); ++i) {
    const int logical = velocity_deploy_config_->joint_ids_map.at(i);
    if (logical < 0) {
      continue;
    }
    int slot = logical;
    if (!velocity_deploy_config_->sdk_joint_ids_map.empty()) {
      const auto logical_index = static_cast<std::size_t>(logical);
      if (logical_index >= velocity_deploy_config_->sdk_joint_ids_map.size()) {
        continue;
      }
      slot = velocity_deploy_config_->sdk_joint_ids_map.at(logical_index);
    }
    if (slot >= 0 && slot < static_cast<int>(kSdkMotorCount)) {
      velocity_controlled.at(static_cast<std::size_t>(slot)) = true;
    }
  }

  const std::size_t count =
      std::min<std::size_t>({kFixStandMotorCount,
                             kSdkMotorCount,
                             fixstand_config_->target_q.size(),
                             fixstand_config_->kp.size(),
                             fixstand_config_->kd.size()});
  for (std::size_t slot = 0; slot < count; ++slot) {
    if (velocity_controlled.at(slot)) {
      continue;
    }
    MotorCommand& motor = frame.motors.at(slot);
    motor.mode = 1;
    motor.q = static_cast<float>(fixstand_config_->target_q.at(slot));
    motor.dq = 0.0F;
    motor.kp = static_cast<float>(fixstand_config_->kp.at(slot));
    motor.kd = static_cast<float>(fixstand_config_->kd.at(slot));
    motor.tau = 0.0F;
  }
}

bool RuntimeControlLoop::republishLowCmdBuffer() {
  if (!lowcmd_buffer_) {
    return false;
  }

  try {
    writeLowCmdFrame(*lowcmd_buffer_);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               runtime_state_.low_ms);
    return false;
  }
  return true;
}

const LowCmdFrame* RuntimeControlLoop::lowCmdBaseFrame() const {
  return lowcmd_buffer_ ? &*lowcmd_buffer_ : nullptr;
}

void RuntimeControlLoop::writeLowCmdFrame(const LowCmdFrame& frame) {
  robot_io_->writeLowCmd(frame);
  lowcmd_buffer_ = frame;
}

void RuntimeControlLoop::markActiveStopping(StopReason reason) {
  if (!active_) {
    return;
  }
  if (active_kind_ == ActiveKind::Transition) {
    abortTransition(MotionState::Canceled, reason, ErrorCode::Ok);
    return;
  }

  active_->state = MotionState::Stopping;
  active_->stop_reason = reason;
  active_->err = ErrorCode::Ok;
  if (active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(*active_));
  }
  active_track_.reset();
  policy_runner_.reset();
  clearReference();
}

void RuntimeControlLoop::completeStoppingActive(MotionState state, ErrorCode error) {
  if (!active_ || active_->state != MotionState::Stopping) {
    return;
  }

  active_->state = state;
  active_->err = error;
  active_->ended_at = std::chrono::steady_clock::now();
  if (active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(*active_));
  }
  active_.reset();
  active_kind_ = ActiveKind::None;
  idle_current_index_.reset();
  active_track_.reset();
  transition_.reset();
  clearReference();
}

std::optional<MotionRequest> RuntimeControlLoop::finishActive(MotionState state,
                                                              StopReason reason,
                                                              ErrorCode error) {
  if (!active_) {
    return std::nullopt;
  }

  active_->state = state;
  active_->stop_reason = reason;
  active_->err = error;
  active_->ended_at = std::chrono::steady_clock::now();
  MotionRequest completed = *active_;
  if (active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(completed));
  }
  active_.reset();
  active_kind_ = ActiveKind::None;
  idle_current_index_.reset();
  active_track_.reset();
  policy_runner_.reset();
  transition_.reset();
  clearReference();
  return completed;
}

void RuntimeControlLoop::stopIdleActive() {
  if (active_kind_ != ActiveKind::Idle) {
    return;
  }
  active_.reset();
  active_kind_ = ActiveKind::None;
  idle_current_index_.reset();
  active_track_.reset();
  policy_runner_.reset();
  transition_.reset();
  clearReference();
  if (fsm_state_ == RuntimeInternalState::GeneralTrackerActive ||
      fsm_state_ == RuntimeInternalState::GeneralTrackerTransition ||
      fsm_state_ == RuntimeInternalState::Stopping) {
    enterGeneralTrackerIdleState();
  }
}

void RuntimeControlLoop::enterStopping(StopReason reason) {
  if (fsm_state_ == RuntimeInternalState::Stopping) {
    return;
  }

  enterInternalState(RuntimeInternalState::Stopping);
  stop_reason_ = reason;
  stop_to_idle_pending_ = true;
  stopping_hold_ticks_remaining_ = hasPolicyRuntime() ? stopHoldTicks() : 0;
}

std::size_t RuntimeControlLoop::ticksForPeriod(double seconds) const {
  if (!std::isfinite(seconds) || !std::isfinite(config_.hz) || seconds <= 0.0 ||
      config_.hz <= 0.0) {
    return 1;
  }

  const double max_ticks =
      static_cast<double>(std::numeric_limits<std::size_t>::max());
  const double ticks = std::ceil(seconds * config_.hz);
  if (!std::isfinite(ticks) || ticks <= 1.0) {
    return 1;
  }
  if (ticks >= max_ticks) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(ticks);
}

std::size_t RuntimeControlLoop::ticksForRate(double rate_hz) const {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    return 1;
  }
  return ticksForPeriod(1.0 / rate_hz);
}

bool RuntimeControlLoop::consumeStepDue(std::size_t& ticks_until_next,
                                        std::size_t interval_ticks) {
  if (interval_ticks <= 1) {
    ticks_until_next = 0;
    return true;
  }
  if (ticks_until_next > 0) {
    --ticks_until_next;
    return false;
  }
  ticks_until_next = interval_ticks - 1;
  return true;
}

std::size_t RuntimeControlLoop::velocityPolicyIntervalTicks() const {
  if (!velocity_deploy_config_) {
    return 1;
  }
  return ticksForPeriod(velocity_deploy_config_->step_dt);
}

std::size_t RuntimeControlLoop::activePolicyIntervalTicks() const {
  if (!active_) {
    return 1;
  }
  return ticksForRate(active_->fps);
}

std::size_t RuntimeControlLoop::stopHoldTicks() const {
  return 0;
}

void RuntimeControlLoop::publishActive() {
  if (active_ && active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(*active_));
  }
}

void RuntimeControlLoop::publishReferenceActive() {
  if (reference_sink_ == nullptr || !active_ || active_kind_ != ActiveKind::User ||
      !active_track_) {
    return;
  }
  try {
    const auto snapshot =
        makeReferenceFrameSnapshot(active_->id, *active_track_, active_->frame);
    if (snapshot) {
      reference_sink_->publish(*snapshot);
    } else {
      reference_sink_->clear();
    }
  } catch (...) {
  }
}

void RuntimeControlLoop::publishReferenceTransition() {
  if (reference_sink_ == nullptr || !active_ ||
      active_kind_ != ActiveKind::Transition || !active_track_) {
    return;
  }
  try {
    const auto snapshot = makeReferenceFrameSnapshot("", *active_track_, active_->frame);
    if (snapshot) {
      reference_sink_->publish(*snapshot);
    } else {
      reference_sink_->clear();
    }
  } catch (...) {
  }
}

void RuntimeControlLoop::clearReference() {
  if (reference_sink_ == nullptr) {
    return;
  }
  try {
    reference_sink_->clear();
  } catch (...) {
  }
}

void RuntimeControlLoop::publishSnapshot() {
  StatusSnapshot snapshot;
  snapshot.ready = hasPolicyRuntime() ? runtime_state_.ready : true;
  snapshot.mode = mode_;
  snapshot.robot =
      hasPolicyRuntime() && !runtime_state_.ready ? runtime_state_.robot : robotState();
  snapshot.ctrl = ctrl_;
  snapshot.stop_reason =
      ctrl_ == ControllerState::Stopping ? stop_reason_ : StopReason::None;
  snapshot.hz = config_.hz;
  if (active_kind_ == ActiveKind::User && active_) {
    snapshot.active = {ActiveKind::User, active_->id};
  } else if (active_kind_ == ActiveKind::Idle && active_) {
    snapshot.active = {ActiveKind::Idle, ""};
  } else if (active_kind_ == ActiveKind::Transition && active_) {
    snapshot.active = {ActiveKind::Transition, ""};
  } else {
    snapshot.active = {ActiveKind::None, ""};
  }
  snapshot.queue.limit = config_.queue_limit;
  snapshot.queue.ids = waitingIds();
  snapshot.queue.n = snapshot.queue.ids.size();
  snapshot.idle = idleStatus();
  if (hasPolicyRuntime()) {
    snapshot.low_ms = runtime_state_.low_ms;
    snapshot.block = runtime_state_.block;
    snapshot.err = runtime_state_.err;
  } else {
    snapshot.err = ErrorCode::Ok;
  }
  if (active_ && active_kind_ == ActiveKind::User) {
    snapshot.exec = toStatus(*active_);
  }
  if (active_ && active_kind_ == ActiveKind::Transition && transition_) {
    snapshot.transition.active = true;
    switch (transition_->target_kind) {
      case TransitionTargetKind::User:
        snapshot.transition.target = "user";
        break;
      case TransitionTargetKind::Idle:
        snapshot.transition.target = "idle";
        break;
      case TransitionTargetKind::Standby:
        snapshot.transition.target = "standby";
        break;
    }
    snapshot.transition.target_id = transition_->target_id;
    snapshot.transition.target_state = transition_->target_state;
    snapshot.transition.frame = active_->frame;
    snapshot.transition.frames = active_->frames;
    snapshot.transition.progress =
        computeProgress(active_->frame, active_->frames, active_->state);
  }
  fillSnapshotPose(snapshot);
  const HealthSnapshot health = healthFromSnapshot(snapshot);
  status_.publishSnapshot(std::move(snapshot));
  status_.publishHealthSnapshot(health);
}

std::optional<LowStateSample> RuntimeControlLoop::readLowStateForStatus() {
  if (robot_io_ == nullptr) {
    latest_low_state_.reset();
    return std::nullopt;
  }
  latest_low_state_ = robot_io_->readLowState();
  return latest_low_state_;
}

std::optional<HighStateSample> RuntimeControlLoop::readHighStateForStatus() {
  if (robot_io_ == nullptr) {
    latest_high_state_.reset();
    return std::nullopt;
  }
  latest_high_state_ = robot_io_->readHighState();
  return latest_high_state_;
}

void RuntimeControlLoop::fillSnapshotPose(StatusSnapshot& snapshot) {
  if (!hasPolicyRuntime()) {
    return;
  }

  readHighStateForStatus();
  if (latest_low_state_) {
    snapshot.pose.q_wxyz = latest_low_state_->quat_wxyz;
    snapshot.pose.gyro_xyz = latest_low_state_->gyro;
  }
  if (latest_high_state_ && latest_high_state_->fresh) {
    snapshot.pose.position_xyz = latest_high_state_->position;
    snapshot.pose.velocity_xyz = latest_high_state_->linear_velocity;
  }
}

MotionStatus RuntimeControlLoop::toStatus(const MotionRequest& request) const {
  return makeMotionStatus(request);
}

std::vector<std::string> RuntimeControlLoop::waitingIds() const {
  std::vector<std::string> ids;
  ids.reserve(waiting_.size());
  for (const auto& request : waiting_) {
    ids.push_back(request.id);
  }
  return ids;
}

IdleStatus RuntimeControlLoop::idleStatus() const {
  IdleStatus status;
  status.enabled = !idle_config_.empty();
  status.n = idle_config_.size();
  status.active = active_kind_ == ActiveKind::Idle && active_.has_value();
  if (!status.active || !active_) {
    return status;
  }

  status.current = idle_current_index_;
  status.frame = active_->frame;
  status.frames = active_->frames;
  status.time_s = active_->fps > 0.0 ? static_cast<double>(active_->frame) / active_->fps
                                     : 0.0;
  status.duration_s = active_->duration_s;
  status.progress = computeProgress(active_->frame, active_->frames, active_->state);
  return status;
}

RobotState RuntimeControlLoop::robotState() const {
  if (ctrl_ == ControllerState::Fault) {
    return RobotState::Fault;
  }
  if (ctrl_ == ControllerState::Running || ctrl_ == ControllerState::Preparing) {
    return RobotState::Running;
  }
  if (ctrl_ == ControllerState::Stopping) {
    return RobotState::Holding;
  }
  return RobotState::Idle;
}

bool RuntimeControlLoop::hasPolicyRuntime() const {
  return robot_io_ != nullptr && policy_ != nullptr && deploy_config_.has_value();
}

bool RuntimeControlLoop::hasControlRuntime() const {
  return hasPolicyRuntime() && velocity_policy_ != nullptr &&
         velocity_deploy_config_.has_value() && fixstand_config_.has_value() &&
         passive_config_.has_value() &&
         fixstand_runner_.has_value() && velocity_runner_.has_value();
}

void RuntimeControlLoop::refreshReadinessForPolicyRuntime() {
  if (!hasPolicyRuntime()) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readinessRequiresFault(readiness)) {
    enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    return;
  }
  if (readiness.err != ErrorCode::Ok) {
    enterPassiveState(readiness);
    return;
  }
  applyReadiness(readiness);
}

void RuntimeControlLoop::applyReadiness(const RobotReadinessStatus& readiness) {
  runtime_state_.ready = readiness.err == ErrorCode::Ok;
  runtime_state_.robot = readiness.robot;
  runtime_state_.low_ms = readiness.low_ms.value_or(0);
  runtime_state_.block = readiness.block;
  runtime_state_.err = readiness.err;
}

bool RuntimeControlLoop::readinessRequiresFault(
    const RobotReadinessStatus& readiness) const {
  return readiness.block == "lowcmd_occupied";
}

void RuntimeControlLoop::enterFault(ErrorCode error,
                                    RobotState robot,
                                    std::string block,
                                    std::optional<std::size_t> low_ms) {
  enterInternalState(RuntimeInternalState::Fault);
  runtime_state_.ready = false;
  runtime_state_.robot = robot;
  runtime_state_.low_ms = low_ms.value_or(runtime_state_.low_ms);
  runtime_state_.block = std::move(block);
  runtime_state_.err = error;
}

void RuntimeControlLoop::failActiveWithFault(ErrorCode error,
                                             RobotState robot,
                                             std::string block,
                                             std::optional<std::size_t> low_ms) {
  finishActive(MotionState::Failed, StopReason::None, error);
  enterFault(error, robot, std::move(block), low_ms);
}

void RuntimeControlLoop::failStoppingActiveWithFault(ErrorCode error) {
  if (!active_ || active_->state != MotionState::Stopping) {
    return;
  }

  completeStoppingActive(MotionState::Failed, error);
}

}  // namespace agentic_et1_tracker
