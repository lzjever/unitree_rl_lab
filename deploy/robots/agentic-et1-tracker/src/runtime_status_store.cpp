#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace agentic_et1_tracker {
namespace {

LocoRunStatus queuedLocoStatus(const LocoRunOptions& options) {
  LocoRunStatus status;
  status.max_radius_m = options.max_radius_m;
  status.distance_m = 0.0;
  status.radius_source.clear();
  status.phase = LocoPhase::Queued;
  status.radius_clamped = options.radius_clamped;
  status.envelope_clamped = options.envelope_clamped;
  status.upper_clamped = options.upper_clamped;
  status.upper_rate_limited = options.upper_rate_limited;
  return status;
}

MotionStatus queuedStatus(const ExecuteCommand& command, std::uint64_t sequence) {
  MotionStatus status;
  status.sequence = sequence;
  status.id = command.id;
  status.path = command.path;
  status.executor = command.executor;
  status.state = MotionState::Queued;
  status.frame = 0;
  status.frames = command.track.frames;
  status.time_s = 0.0;
  status.duration_s = command.track.duration_s;
  status.progress = computeProgress(0, command.track.frames, MotionState::Queued);
  status.hold = command.hold;
  if (status.executor == MotionExecutor::LocoUpper) {
    status.loco = queuedLocoStatus(command.loco_options);
  }
  status.stop_reason = StopReason::None;
  status.err = ErrorCode::Ok;
  return status;
}

void eraseById(std::deque<MotionStatus>& items, const std::string& id) {
  const auto it = std::find_if(items.begin(), items.end(), [&](const MotionStatus& item) {
    return item.id == id;
  });
  if (it != items.end()) {
    items.erase(it);
  }
}

auto findById(std::deque<MotionStatus>& items, const std::string& id) {
  return std::find_if(items.begin(), items.end(), [&](const MotionStatus& item) {
    return item.id == id;
  });
}

auto findById(const std::deque<MotionStatus>& items, const std::string& id) {
  return std::find_if(items.begin(), items.end(), [&](const MotionStatus& item) {
    return item.id == id;
  });
}

bool containsId(const std::vector<std::string>& ids, const std::string& id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool terminal(MotionState state) {
  return state == MotionState::Done || state == MotionState::Stopped ||
         state == MotionState::Canceled || state == MotionState::Failed;
}

bool activeLike(MotionState state) {
  return state == MotionState::Running || state == MotionState::Holding ||
         state == MotionState::Stopping;
}

void pushRecent(std::deque<MotionStatus>& recent,
                MotionStatus status,
                std::size_t limit) {
  if (limit == 0) {
    return;
  }

  eraseById(recent, status.id);
  recent.push_back(std::move(status));
  while (recent.size() > limit) {
    recent.pop_front();
  }
}

void upsertAccepted(std::deque<MotionStatus>& accepted, MotionStatus status) {
  eraseById(accepted, status.id);
  accepted.push_back(std::move(status));
}

bool recentHas(const std::deque<MotionStatus>& recent, const std::string& id) {
  return findById(recent, id) != recent.end();
}

bool acceptedNonQueued(const std::deque<MotionStatus>& accepted,
                       const std::string& id) {
  const auto it = findById(accepted, id);
  return it != accepted.end() && it->state != MotionState::Queued;
}

bool isCurrentExec(const StatusSnapshot& snapshot, const std::string& id) {
  return snapshot.exec && snapshot.exec->id == id;
}

bool isTransitionTarget(const StatusSnapshot& snapshot, const std::string& id) {
  return snapshot.transition.active && snapshot.transition.target == "user" &&
         !snapshot.transition.target_id.empty() &&
         snapshot.transition.target_id == id;
}

MotionStatus canceledStatus(const std::deque<MotionStatus>& accepted,
                            const std::string& id,
                            StopReason reason) {
  MotionStatus status;
  const auto existing = findById(accepted, id);
  if (existing != accepted.end()) {
    status = *existing;
  } else {
    status.id = id;
  }
  status.state = MotionState::Canceled;
  status.stop_reason = reason;
  status.progress = 0.0;
  if (status.executor == MotionExecutor::LocoUpper) {
    status.loco.phase = LocoPhase::Canceled;
  }
  status.err = ErrorCode::Ok;
  return status;
}

std::vector<std::string> acceptedQueuedIds(const std::deque<MotionStatus>& accepted) {
  std::vector<std::string> ids;
  ids.reserve(accepted.size());
  for (const auto& status : accepted) {
    if (status.state == MotionState::Queued) {
      ids.push_back(status.id);
    }
  }
  return ids;
}

bool hasActiveOrStopping(const StatusSnapshot& snapshot) {
  return snapshot.exec.has_value() || snapshot.ctrl == ControllerState::Preparing ||
         snapshot.ctrl == ControllerState::Running ||
         snapshot.ctrl == ControllerState::Stopping;
}

bool hasActivePublishedRun(const std::deque<MotionStatus>& statuses) {
  return std::any_of(statuses.begin(), statuses.end(), [](const MotionStatus& status) {
    return activeLike(status.state);
  });
}

IdleStatus idleStatusForConfig(const std::vector<IdleMotion>& motions) {
  IdleStatus status;
  status.enabled = !motions.empty();
  status.n = motions.size();
  status.active = false;
  return status;
}

bool sameIdleConfig(const IdleStatus& lhs, const IdleStatus& rhs) {
  return lhs.enabled == rhs.enabled && lhs.n == rhs.n;
}

void normalizeActive(StatusSnapshot& snapshot) {
  if (snapshot.active.kind == ActiveKind::Idle && !snapshot.idle.active) {
    snapshot.active = {ActiveKind::None, ""};
  }
  if (snapshot.active.kind == ActiveKind::None && snapshot.exec) {
    snapshot.active = {ActiveKind::User, snapshot.exec->id};
  }
}

}  // namespace

RuntimeStatusStore::RuntimeStatusStore(RuntimeConfig config)
    : config_(config) {
  snapshot_.hz = config_.hz;
  snapshot_.queue.limit = config_.queue_limit;
}

void RuntimeStatusStore::publishSnapshot(StatusSnapshot snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  const IdleStatus published_idle = snapshot.idle;
  snapshot_ = std::move(snapshot);
  if (sameIdleConfig(published_idle, idle_status_)) {
    idle_status_ = published_idle;
  }
  snapshot_.idle = idle_status_;
  normalizeActive(snapshot_);
  if (snapshot_.hz == 0.0) {
    snapshot_.hz = config_.hz;
  }
}

void RuntimeStatusStore::publishHealthSnapshot(HealthSnapshot health) {
  std::lock_guard<std::mutex> lock(mutex_);
  health_ = std::move(health);
}

void RuntimeStatusStore::publishRunStatus(MotionStatus status) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (terminal(status.state)) {
    if (snapshot_.exec && snapshot_.exec->id == status.id) {
      snapshot_.exec = status;
    }
    eraseById(accepted_, status.id);
    pushRecent(recent_, std::move(status), config_.recent_limit);
    return;
  }

  if (snapshot_.exec && snapshot_.exec->id == status.id) {
    snapshot_.exec = status;
  }
  upsertAccepted(accepted_, std::move(status));
}

StatusSnapshot RuntimeStatusStore::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  StatusSnapshot snapshot = snapshot_;
  snapshot.queue.ids = queuedIdsLocked();
  snapshot.queue.n = snapshot.queue.ids.size();
  snapshot.queue.limit = config_.queue_limit;
  if (snapshot.hz == 0.0) {
    snapshot.hz = config_.hz;
  }
  snapshot.idle = idle_status_;
  normalizeActive(snapshot);
  return snapshot;
}

RunLookupResult RuntimeStatusStore::findRun(const std::string& id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.exec && snapshot_.exec->id == id) {
    return {ErrorCode::Ok, snapshot_.exec};
  }

  const auto accepted = findById(accepted_, id);
  if (accepted != accepted_.end()) {
    return {ErrorCode::Ok, *accepted};
  }

  const auto recent = findById(recent_, id);
  if (recent != recent_.end()) {
    return {ErrorCode::Ok, *recent};
  }

  return {ErrorCode::RunNotFound, std::nullopt};
}

HealthSnapshot RuntimeStatusStore::health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return health_;
}

ExecuteResult RuntimeStatusStore::acceptQueued(const ExecuteCommand& command,
                                               std::uint64_t sequence) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t current_queue_size = queuedIdsLocked().size();
  if (current_queue_size >= config_.queue_limit) {
    return {ErrorCode::QueueFull, command.id, MotionState::Queued, current_queue_size};
  }

  upsertAccepted(accepted_, queuedStatus(command, sequence));
  return {ErrorCode::Ok, command.id, MotionState::Queued, queuedIdsLocked().size()};
}

ExecuteResult RuntimeStatusStore::acceptInterrupt(const ExecuteCommand& command,
                                                  std::uint64_t sequence) {
  std::lock_guard<std::mutex> lock(mutex_);
  cancelQueuedLocked(StopReason::Interrupt);
  upsertAccepted(accepted_, queuedStatus(command, sequence));
  return {ErrorCode::Ok, command.id, MotionState::Queued, queuedIdsLocked().size()};
}

ExecuteResult RuntimeStatusStore::acceptInterruptAfterStop(const ExecuteCommand& command,
                                                           std::uint64_t sequence,
                                                           std::uint64_t stop_sequence) {
  std::lock_guard<std::mutex> lock(mutex_);
  cancelQueuedAfterLocked(StopReason::Interrupt, stop_sequence);
  upsertAccepted(accepted_, queuedStatus(command, sequence));
  return {ErrorCode::Ok, command.id, MotionState::Queued, queuedIdsLocked().size()};
}

StopResult RuntimeStatusStore::acceptStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t waiting = queuedIdsLocked().size();

  if (waiting == 0 && !hasActiveOrStopping(snapshot_) &&
      !hasActivePublishedRun(accepted_) && !hasActivePublishedRun(recent_)) {
    return {ErrorCode::Ok, snapshot_.ctrl, StopReason::None, 0};
  }

  return {ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 0};
}

ControlResult RuntimeStatusStore::acceptControl(ControlMode mode, bool preserve_queued) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode == ControlMode::StandbyVelocity &&
      (snapshot_.ctrl == ControllerState::Passive ||
       snapshot_.ctrl == ControllerState::Fault)) {
    return {ErrorCode::ControlStateConflict};
  }
  if (mode == ControlMode::Passive || mode == ControlMode::StandbyVelocity) {
    cancelQueuedLocked(StopReason::Stop);
    if (mode == ControlMode::Passive) {
      return {ErrorCode::Ok};
    }
  }
  if (mode == ControlMode::FixStand && !preserve_queued) {
    cancelQueuedLocked(StopReason::Stop);
  }
  return {ErrorCode::Ok};
}

IdleResult RuntimeStatusStore::acceptIdleConfig(std::vector<IdleMotion> motions) {
  std::lock_guard<std::mutex> lock(mutex_);
  idle_config_ = std::move(motions);
  idle_status_ = idleStatusForConfig(idle_config_);
  snapshot_.idle = idle_status_;
  if (snapshot_.active.kind == ActiveKind::Idle) {
    snapshot_.active = {ActiveKind::None, ""};
  }
  IdleResult result;
  result.idle = idle_status_;
  return result;
}

bool RuntimeStatusStore::clearIdleConfig() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool had_idle = !idle_config_.empty() || idle_status_.enabled ||
                        snapshot_.idle.enabled ||
                        snapshot_.active.kind == ActiveKind::Idle;
  idle_config_.clear();
  idle_status_ = IdleStatus{};
  snapshot_.idle = idle_status_;
  if (snapshot_.active.kind == ActiveKind::Idle) {
    snapshot_.active = {ActiveKind::None, ""};
  }
  return had_idle;
}

std::size_t RuntimeStatusStore::cancelQueuedForStop(std::uint64_t sequence) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t canceled = 0;
  for (auto it = accepted_.begin(); it != accepted_.end();) {
    if (it->state != MotionState::Queued || it->sequence > sequence ||
        isCurrentExec(snapshot_, it->id) || recentHas(recent_, it->id)) {
      ++it;
      continue;
    }

    pushRecent(recent_, canceledStatus(accepted_, it->id, StopReason::Stop),
               config_.recent_limit);
    it = accepted_.erase(it);
    ++canceled;
  }
  return canceled;
}

std::vector<std::string> RuntimeStatusStore::queuedIdsLocked() const {
  std::vector<std::string> ids;
  ids.reserve(snapshot_.queue.ids.size() + accepted_.size());

  for (const auto& id : snapshot_.queue.ids) {
    if (!containsId(ids, id) && !isCurrentExec(snapshot_, id) &&
        !isTransitionTarget(snapshot_, id) &&
        !recentHas(recent_, id) && !acceptedNonQueued(accepted_, id)) {
      ids.push_back(id);
    }
  }

  for (const auto& id : acceptedQueuedIds(accepted_)) {
    if (!containsId(ids, id) && !isCurrentExec(snapshot_, id) &&
        !isTransitionTarget(snapshot_, id) &&
        !recentHas(recent_, id)) {
      ids.push_back(id);
    }
  }

  return ids;
}

std::size_t RuntimeStatusStore::cancelQueuedLocked(StopReason reason) {
  const std::vector<std::string> ids = queuedIdsLocked();
  for (const auto& id : ids) {
    pushRecent(recent_, canceledStatus(accepted_, id, reason), config_.recent_limit);
    eraseById(accepted_, id);
  }
  return ids.size();
}

std::size_t RuntimeStatusStore::cancelQueuedAfterLocked(StopReason reason,
                                                        std::uint64_t sequence) {
  std::size_t canceled = 0;
  for (auto it = accepted_.begin(); it != accepted_.end();) {
    if (it->state != MotionState::Queued || it->sequence <= sequence ||
        isCurrentExec(snapshot_, it->id) || recentHas(recent_, it->id)) {
      ++it;
      continue;
    }

    pushRecent(recent_, canceledStatus(accepted_, it->id, reason),
               config_.recent_limit);
    it = accepted_.erase(it);
    ++canceled;
  }
  return canceled;
}

}  // namespace agentic_et1_tracker
