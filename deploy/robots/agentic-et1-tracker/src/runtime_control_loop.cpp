#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"

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
                                       std::uint8_t expected_mode_machine,
                                       RuntimeMode mode)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      robot_io_(&robot_io),
      policy_(&policy),
      deploy_config_(std::move(deploy_config)),
      expected_mode_machine_(expected_mode_machine),
      mode_(mode) {
  runtime_state_.ready = false;
  runtime_state_.robot = RobotState::Disconnected;
  runtime_state_.err = ErrorCode::ServiceNotReady;
  runtime_state_.block = "runtime_not_started";
  publishSnapshot();
}

void RuntimeControlLoop::tick() {
  if (ctrl_ == ControllerState::Stopping) {
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
      ctrl_ = ControllerState::Idle;
      stop_reason_ = StopReason::None;
    }
    publishSnapshot();
    return;
  }

  if (consumePendingCommands()) {
    publishSnapshot();
    return;
  }

  if (ctrl_ == ControllerState::Running) {
    advanceActive();
    publishSnapshot();
    return;
  }

  if (ctrl_ == ControllerState::Preparing) {
    completePreparing();
    publishSnapshot();
    return;
  }

  if (ctrl_ == ControllerState::Idle && !waiting_.empty()) {
    startNext();
    publishSnapshot();
    return;
  }

  if (ctrl_ == ControllerState::Idle && waiting_.empty()) {
    refreshReadinessForPolicyRuntime();
    publishIdleHoldIfReady();
  }
  publishSnapshot();
}

bool RuntimeControlLoop::consumePendingCommands() {
  bool consumed_motion = false;
  while (auto command = bridge_.consumeNextCommand()) {
    switch (command->kind) {
      case CommandKind::Stop:
        handleStop(command->sequence);
        return true;
      case CommandKind::Interrupt:
        handleInterrupt(std::move(command->request));
        return ctrl_ == ControllerState::Stopping;
      case CommandKind::Queue:
        waiting_.push_back(std::move(command->request));
        consumed_motion = true;
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
  if (ctrl_ == ControllerState::Fault) {
    return;
  }
  if (active_) {
    markActiveStopping(StopReason::Stop);
  }
  enterStopping(StopReason::Stop);
}

void RuntimeControlLoop::handleInterrupt(MotionRequest request) {
  cancelWaiting(StopReason::Interrupt);
  waiting_.push_back(std::move(request));
  if (active_) {
    markActiveStopping(StopReason::Interrupt);
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
  ctrl_ = ControllerState::Preparing;
  publishActive();
}

void RuntimeControlLoop::completePreparing() {
  if (!active_) {
    ctrl_ = ControllerState::Idle;
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
      ctrl_ = ControllerState::Idle;
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
    ctrl_ = ControllerState::Idle;
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
  active_first_advance_ = true;
  ctrl_ = ControllerState::Running;
  publishActive();
}

void RuntimeControlLoop::advanceActive() {
  if (hasPolicyRuntime()) {
    advanceActiveWithPolicy();
    return;
  }

  if (!active_) {
    ctrl_ = ControllerState::Idle;
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::size_t frame = active_->frame;
  if (active_first_advance_) {
    active_->started_at = now;
    active_first_advance_ = false;
    frame = 0;
  } else {
    const auto elapsed = std::chrono::duration<double>(now - active_->started_at).count();
    frame = referenceFrameIndex(elapsed, active_->fps, active_->frames);
  }

  active_->frame = frame;
  if (active_->frames == 0 || frame + 1 >= active_->frames) {
    const std::size_t last_frame = active_->frames == 0 ? 0 : active_->frames - 1;
    if (active_->frame < last_frame) {
      active_->frame = last_frame;
      publishActive();
      return;
    }
    active_->frame = last_frame;
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    ctrl_ = ControllerState::Idle;
    return;
  }

  publishActive();
}

void RuntimeControlLoop::advanceActiveWithPolicy() {
  if (!active_) {
    policy_runner_.reset();
    ctrl_ = ControllerState::Idle;
    return;
  }

  if (!policy_runner_) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed");
    return;
  }

  const std::optional<LowStateSample> low_state = robot_io_->readLowState();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    failActiveWithFault(readiness.err,
                        readiness.robot,
                        readiness.block,
                        readiness.low_ms);
    return;
  }
  applyReadiness(readiness);

  const auto now = std::chrono::steady_clock::now();
  if (active_first_advance_) {
    active_->started_at = now;
    active_->frame = 0;
    active_first_advance_ = false;
  } else {
    const auto elapsed = std::chrono::duration<double>(now - active_->started_at).count();
    active_->frame = referenceFrameIndex(elapsed, active_->fps, active_->frames);
  }
  publishActive();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_);
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed",
                        readiness.low_ms);
    return;
  }

  try {
    robot_io_->writeLowCmd(step.low_cmd);
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
    ctrl_ = ControllerState::Idle;
    return;
  }

  publishActive();
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
      applyReadiness(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  try {
    robot_io_->writeLowCmd(
        makeHoldLowCmdFrame(*deploy_config_, *low_state, expected_mode_machine_));
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
    enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    return false;
  }
  applyReadiness(readiness);

  try {
    robot_io_->writeLowCmd(
        makeHoldLowCmdFrame(*deploy_config_, *low_state, expected_mode_machine_));
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
  if (ctrl_ == ControllerState::Stopping) {
    return;
  }

  ctrl_ = ControllerState::Stopping;
  stop_reason_ = reason;
  stop_to_idle_pending_ = true;
  stopping_hold_ticks_remaining_ = hasPolicyRuntime() ? stopHoldTicks() : 0;
}

std::size_t RuntimeControlLoop::stopHoldTicks() const {
  if (!std::isfinite(config_.stop_hold_s) || !std::isfinite(config_.hz) ||
      config_.stop_hold_s <= 0.0 || config_.hz <= 0.0) {
    return 0;
  }

  const double max_ticks =
      static_cast<double>(std::numeric_limits<std::size_t>::max());
  if (config_.stop_hold_s >= max_ticks / config_.hz) {
    return std::numeric_limits<std::size_t>::max();
  }

  const double ticks = std::ceil(config_.stop_hold_s * config_.hz);
  if (!std::isfinite(ticks) || ticks <= 0.0) {
    return 0;
  }
  if (ticks >= max_ticks) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(ticks);
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
  return readiness.err == ErrorCode::RobotBadOrientation ||
         readiness.block == "lowcmd_occupied";
}

void RuntimeControlLoop::enterFault(ErrorCode error,
                                    RobotState robot,
                                    std::string block,
                                    std::optional<std::size_t> low_ms) {
  ctrl_ = ControllerState::Fault;
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
