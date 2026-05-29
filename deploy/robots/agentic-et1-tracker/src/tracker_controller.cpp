#include "agentic_et1_tracker/core/tracker_controller.hpp"

#include <chrono>
#include <utility>

namespace agentic_et1_tracker {
namespace {

MotionRequest makeMotionRequest(const ExecuteRequest& request) {
  MotionRequest motion;
  motion.id = request.id;
  motion.path = request.path;
  motion.frames = request.track.frames;
  motion.fps = request.track.fps;
  motion.duration_s = request.track.duration_s;
  motion.state = MotionState::Queued;
  return motion;
}

}  // namespace

bool Readiness::ready() const {
  return service_initialized && robot_connected && lowstate_fresh && mode_machine_ok &&
         policy_ready && !fault;
}

ErrorCode Readiness::acceptError() const {
  if (!service_initialized) {
    return ErrorCode::ServiceNotReady;
  }
  if (!robot_connected) {
    return ErrorCode::RobotDisconnected;
  }
  if (!lowstate_fresh || !mode_machine_ok) {
    return ErrorCode::RobotNotReady;
  }
  if (!policy_ready) {
    return ErrorCode::ModelNotReady;
  }
  if (fault) {
    return fault_code == ErrorCode::Ok ? ErrorCode::SafetyLimitTriggered : fault_code;
  }
  return ErrorCode::Ok;
}

TrackerController::TrackerController(TrackerControllerConfig config)
    : config_(config), queue_(config.queue_limit, config.recent_limit) {}

void TrackerController::setReadiness(Readiness readiness) {
  readiness_ = std::move(readiness);
  syncControllerWithReadiness();
}

const Readiness& TrackerController::readiness() const {
  return readiness_;
}

ExecuteResult TrackerController::execute(const ExecuteRequest& request) {
  const ErrorCode error = acceptError(request);
  if (error != ErrorCode::Ok) {
    return {error, request.id, MotionState::Queued, queue_.size()};
  }

  MotionRequest motion = makeMotionRequest(request);
  if (request.mode == MotionMode::Interrupt) {
    mailbox_.submitInterrupt(motion);
    auto command = mailbox_.consumeNext();
    return acceptInterrupt(command ? std::move(command->request) : std::move(motion));
  }

  return acceptQueue(std::move(motion));
}

StopResult TrackerController::stop() {
  mailbox_.submitStop();
  (void)mailbox_.consumeNext();
  return processStop();
}

void TrackerController::tick() {
  syncControllerWithReadiness();
  if (ctrl_ == ControllerState::Fault) {
    return;
  }

  if (ctrl_ == ControllerState::Stopping) {
    if (stop_to_idle_pending_) {
      stop_to_idle_pending_ = false;
      ctrl_ = readiness_.service_initialized ? ControllerState::Idle : ControllerState::Starting;
      stop_reason_ = StopReason::None;
    }
    return;
  }

  if (ctrl_ == ControllerState::Running) {
    advanceActive();
    return;
  }

  if (startGateOpen() && !queue_.empty()) {
    startNext();
  }
}

StatusSnapshot TrackerController::status() const {
  StatusSnapshot snapshot;
  snapshot.ready = readiness_.ready();
  snapshot.robot = robotState();
  snapshot.ctrl = ctrl_;
  snapshot.stop_reason = ctrl_ == ControllerState::Stopping ? stop_reason_ : StopReason::None;
  snapshot.hz = config_.hz;
  snapshot.queue = {queue_.size(), queue_.limit(), queue_.queuedIds()};
  snapshot.block = block();
  snapshot.err = statusError();
  if (active_) {
    snapshot.exec = toStatus(*active_);
  }
  return snapshot;
}

RunLookupResult TrackerController::findRun(const std::string& id) const {
  if (active_ && active_->id == id) {
    return {ErrorCode::Ok, toStatus(*active_)};
  }
  if (auto queued = queue_.findQueued(id)) {
    return {ErrorCode::Ok, toStatus(*queued)};
  }
  if (auto recent = queue_.findRecent(id)) {
    return {ErrorCode::Ok, toStatus(*recent)};
  }
  return {ErrorCode::RunNotFound, std::nullopt};
}

ErrorCode TrackerController::acceptError(const ExecuteRequest& request) const {
  const ErrorCode readiness_error = readiness_.acceptError();
  if (readiness_error != ErrorCode::Ok) {
    return readiness_error;
  }
  if (request.id.empty() || request.path.empty()) {
    return ErrorCode::RequestInvalid;
  }
  if (request.validation_error != ErrorCode::Ok) {
    return request.validation_error;
  }
  if (request.track.frames == 0) {
    return ErrorCode::TrkValidationFailed;
  }
  if (request.mode == MotionMode::Queue && queue_.full()) {
    return ErrorCode::QueueFull;
  }
  return ErrorCode::Ok;
}

ExecuteResult TrackerController::acceptQueue(MotionRequest request) {
  const std::string id = request.id;
  const QueueInsertResult inserted = queue_.enqueue(std::move(request));
  return {inserted.code, id, MotionState::Queued, inserted.queue_size};
}

ExecuteResult TrackerController::acceptInterrupt(MotionRequest request) {
  const std::string id = request.id;
  if (active_) {
    finishActive(MotionState::Stopped, StopReason::Interrupt, ErrorCode::Ok);
    enterStopping(StopReason::Interrupt);
  }

  const InterruptQueueResult result = queue_.interruptWith(std::move(request));
  return {result.code, id, MotionState::Queued, result.queue_size};
}

StopResult TrackerController::processStop() {
  const QueueCancelResult canceled = queue_.stopQueued();
  if (active_) {
    finishActive(MotionState::Stopped, StopReason::Stop, ErrorCode::Ok);
  }

  if (ctrl_ != ControllerState::Fault) {
    enterStopping(StopReason::Stop);
  }

  return {ErrorCode::Ok, ctrl_,
          ctrl_ == ControllerState::Stopping ? stop_reason_ : StopReason::None,
          canceled.canceled};
}

bool TrackerController::startGateOpen() const {
  return ctrl_ == ControllerState::Idle && readiness_.ready() && !active_;
}

void TrackerController::startNext() {
  auto next = queue_.popNext();
  if (!next) {
    return;
  }

  ctrl_ = ControllerState::Preparing;
  next->state = MotionState::Running;
  next->frame = 0;
  next->started_at = std::chrono::steady_clock::now();
  active_ = std::move(next);
  ctrl_ = ControllerState::Running;
}

void TrackerController::advanceActive() {
  if (!active_) {
    ctrl_ = ControllerState::Idle;
    return;
  }

  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    ctrl_ = readiness_.service_initialized ? ControllerState::Idle : ControllerState::Starting;
    return;
  }

  ++active_->frame;
}

void TrackerController::finishActive(MotionState state, StopReason reason, ErrorCode error) {
  if (!active_) {
    return;
  }

  active_->state = state;
  active_->stop_reason = reason;
  active_->err = error;
  active_->ended_at = std::chrono::steady_clock::now();
  queue_.addRecent(std::move(*active_));
  active_.reset();
}

void TrackerController::enterStopping(StopReason reason) {
  ctrl_ = ControllerState::Stopping;
  stop_reason_ = reason;
  stop_to_idle_pending_ = true;
}

void TrackerController::syncControllerWithReadiness() {
  if (readiness_.fault) {
    if (active_) {
      const ErrorCode error =
          readiness_.fault_code == ErrorCode::Ok ? ErrorCode::SafetyLimitTriggered
                                                 : readiness_.fault_code;
      finishActive(MotionState::Failed, StopReason::None, error);
    }
    ctrl_ = ControllerState::Fault;
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    return;
  }

  if (!readiness_.service_initialized) {
    if (!active_ && ctrl_ != ControllerState::Stopping) {
      ctrl_ = ControllerState::Starting;
    }
    return;
  }

  if (ctrl_ == ControllerState::Starting || ctrl_ == ControllerState::Fault) {
    ctrl_ = active_ ? ControllerState::Running : ControllerState::Idle;
  }
}

MotionStatus TrackerController::toStatus(const MotionRequest& request) const {
  return makeMotionStatus(request);
}

RobotState TrackerController::robotState() const {
  if (readiness_.fault || ctrl_ == ControllerState::Fault) {
    return RobotState::Fault;
  }
  if (!readiness_.robot_connected) {
    return RobotState::Disconnected;
  }
  if (!readiness_.lowstate_fresh || !readiness_.mode_machine_ok) {
    return RobotState::NotReady;
  }
  if (ctrl_ == ControllerState::Running || ctrl_ == ControllerState::Preparing) {
    return RobotState::Running;
  }
  if (ctrl_ == ControllerState::Stopping) {
    return RobotState::Holding;
  }
  return RobotState::Idle;
}

std::string TrackerController::block() const {
  if (!readiness_.block.empty()) {
    return readiness_.block;
  }
  if (!readiness_.service_initialized) {
    return "service_initializing";
  }
  if (!readiness_.robot_connected) {
    return "robot_disconnected";
  }
  if (!readiness_.lowstate_fresh) {
    return "lowstate_timeout";
  }
  if (!readiness_.mode_machine_ok) {
    return "mode_machine_mismatch";
  }
  if (!readiness_.policy_ready) {
    return "policy_not_loaded";
  }
  if (readiness_.fault) {
    return "fault";
  }
  return {};
}

ErrorCode TrackerController::statusError() const {
  if (readiness_.fault) {
    return readiness_.fault_code == ErrorCode::Ok ? ErrorCode::SafetyLimitTriggered
                                                  : readiness_.fault_code;
  }
  return readiness_.acceptError();
}

}  // namespace agentic_et1_tracker
