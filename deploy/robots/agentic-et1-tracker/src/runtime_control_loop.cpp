#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include "agentic_et1_tracker/policy/observation_builder.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

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
                                       TrkLoader loader)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)) {
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
                                       RuntimeMode mode)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
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
                                       RuntimeMode mode)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
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
  publishSnapshot();
}

RuntimeInternalState RuntimeControlLoop::internalStateForTest() const {
  return fsm_state_;
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

  if (fsm_state_ == RuntimeInternalState::GeneralTrackerActive) {
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
        handleStop(command->sequence);
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
      case CommandKind::Interrupt:
        handleInterrupt(std::move(command->request));
        return fsm_state_ == RuntimeInternalState::Stopping;
      case CommandKind::Queue:
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
      case CommandKind::FixStand:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::FixStand;
        break;
      case CommandKind::StandbyVelocity:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::StandbyVelocity;
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

void RuntimeControlLoop::handleStop(std::uint64_t sequence) {
  cancelWaiting(StopReason::Stop, sequence);
  if (fsm_state_ == RuntimeInternalState::Fault) {
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
  }
  stop_reason_ = StopReason::Stop;
  enterStopping(StopReason::Stop);
}

void RuntimeControlLoop::handleControl(ControlMode mode) {
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
  if (fsm_state_ == RuntimeInternalState::Fault) {
    active_.reset();
    policy_runner_.reset();
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

void RuntimeControlLoop::handleInterrupt(MotionRequest request) {
  cancelWaiting(StopReason::Interrupt);
  waiting_.push_back(std::move(request));
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

  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
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
    entry_low_state = robot_io_->readLowState();
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
  handleInternalEvent(RuntimeInternalEvent::MotionRequest);
  publishActive();
}

void RuntimeControlLoop::completePreparing() {
  if (!active_) {
    enterGeneralTrackerIdleState();
    return;
  }

  std::optional<LowStateSample> entry_low_state;
  std::optional<RobotReadinessStatus> readiness;
  if (hasPolicyRuntime()) {
    entry_low_state = robot_io_->readLowState();
    readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    applyReadiness(*readiness);
    if (readiness->err != ErrorCode::Ok) {
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
      policy_runner_.reset();
      waiting_.push_front(std::move(request));
      status_.publishRunStatus(toStatus(waiting_.front()));
      enterPassiveState(*readiness);
      return;
    }
  }

  TrkLoadResult loaded = loader_.load(active_->path);
  if (!loaded.ok()) {
    active_->state = MotionState::Failed;
    active_->frame = 0;
    active_->err = toCoreErrorCode(loaded.code);
    active_->stop_reason = StopReason::None;
    active_->ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(*active_));
    active_.reset();
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  const std::size_t loaded_frames = loaded.track->metadata.frames;
  const double loaded_duration_s = loaded.track->metadata.duration_s;
  active_->frames = loaded_frames;
  active_->fps = loaded.track->metadata.fps;
  active_->duration_s = loaded_duration_s;
  if (hasPolicyRuntime()) {
    try {
      policy_runner_.emplace(*deploy_config_,
                             std::move(*loaded.track),
                             *entry_low_state,
                             expected_mode_machine_);
    } catch (const std::exception&) {
      active_->state = MotionState::Failed;
      active_->frame = 0;
      active_->err = ErrorCode::ModelInferenceFailed;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      policy_runner_.reset();
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
  publishActive();
}

void RuntimeControlLoop::advanceActive() {
  if (hasPolicyRuntime()) {
    advanceActiveWithPolicy();
    return;
  }

  if (!active_) {
    enterGeneralTrackerIdleState();
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
      publishActive();
      return;
    }
    active_->frame = last_frame;
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  publishActive();
}

void RuntimeControlLoop::advanceActiveWithPolicy() {
  if (!active_) {
    policy_runner_.reset();
    enterGeneralTrackerIdleState();
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

  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
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

  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    const std::size_t last_frame = active_->frames == 0 ? 0 : active_->frames - 1;
    active_->frame = last_frame;
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  publishActive();
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
  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
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
  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
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

  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
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
  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
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

  active_->state = MotionState::Stopping;
  active_->stop_reason = reason;
  active_->err = ErrorCode::Ok;
  status_.publishRunStatus(toStatus(*active_));
  policy_runner_.reset();
}

void RuntimeControlLoop::completeStoppingActive(MotionState state, ErrorCode error) {
  if (!active_ || active_->state != MotionState::Stopping) {
    return;
  }

  active_->state = state;
  active_->err = error;
  active_->ended_at = std::chrono::steady_clock::now();
  status_.publishRunStatus(toStatus(*active_));
  active_.reset();
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
  status_.publishRunStatus(toStatus(completed));
  active_.reset();
  policy_runner_.reset();
  return completed;
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
  if (active_) {
    status_.publishRunStatus(toStatus(*active_));
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
  snapshot.queue.limit = config_.queue_limit;
  snapshot.queue.ids = waitingIds();
  snapshot.queue.n = snapshot.queue.ids.size();
  if (hasPolicyRuntime()) {
    snapshot.low_ms = runtime_state_.low_ms;
    snapshot.block = runtime_state_.block;
    snapshot.err = runtime_state_.err;
  } else {
    snapshot.err = ErrorCode::Ok;
  }
  if (active_) {
    snapshot.exec = toStatus(*active_);
  }
  const HealthSnapshot health = healthFromSnapshot(snapshot);
  status_.publishSnapshot(std::move(snapshot));
  status_.publishHealthSnapshot(health);
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

  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
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
