#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"

#include <algorithm>
#include <utility>

namespace agentic_et1_tracker {
namespace {

MotionRequest motionRequest(const ExecuteCommand& command, std::uint64_t sequence) {
  MotionRequest request;
  request.sequence = sequence;
  request.id = command.id;
  request.path = command.path;
  request.state = MotionState::Queued;
  request.frame = 0;
  request.frames = command.track.frames;
  request.fps = command.track.fps;
  request.duration_s = command.track.duration_s;
  request.err = ErrorCode::Ok;
  request.stop_reason = StopReason::None;
  return request;
}

}  // namespace

RuntimeBridge::RuntimeBridge(RuntimeConfig, RuntimeStatusStore& status) : status_(status) {}

ExecuteResult RuntimeBridge::submitQueue(const ExecuteCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::uint64_t sequence = next_sequence_++;
  const ExecuteResult result = status_.acceptQueued(command, sequence);
  if (!result.ok()) {
    return result;
  }

  push(CommandKind::Queue, motionRequest(command, sequence), sequence);
  return result;
}

ExecuteResult RuntimeBridge::submitInterrupt(const ExecuteCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::uint64_t sequence = next_sequence_++;
  const std::optional<std::uint64_t> stop_sequence = latestPendingStopSequence();
  const ExecuteResult result =
      stop_sequence ? status_.acceptInterruptAfterStop(command, sequence, *stop_sequence)
                    : status_.acceptInterrupt(command, sequence);
  if (!result.ok()) {
    return result;
  }

  if (stop_sequence) {
    clearPendingMotionsAfter(*stop_sequence);
  } else {
    clearPendingMotions();
  }
  push(CommandKind::Interrupt, motionRequest(command, sequence), sequence);
  return result;
}

StopResult RuntimeBridge::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::uint64_t sequence = next_sequence_++;
  const StopResult result = status_.acceptStop();
  if (result.state == ControllerState::Stopping) {
    MotionRequest request;
    request.sequence = sequence;
    push(CommandKind::Stop, request, sequence);
  }
  return result;
}

std::optional<Command> RuntimeBridge::consumeNextCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_.empty()) {
    return std::nullopt;
  }

  auto best = pending_.begin();
  for (auto it = pending_.begin(); it != pending_.end(); ++it) {
    const int it_priority = priority(it->kind);
    const int best_priority = priority(best->kind);
    if (it_priority > best_priority ||
        (it_priority == best_priority && it->sequence < best->sequence)) {
      best = it;
    }
  }

  Command command = *best;
  pending_.erase(best);
  if (command.kind == CommandKind::Stop) {
    status_.cancelQueuedForStop(command.sequence);
    clearPendingMotionsThrough(command.sequence);
  }
  return command;
}

int RuntimeBridge::priority(CommandKind kind) {
  switch (kind) {
    case CommandKind::Stop:
      return 3;
    case CommandKind::Interrupt:
      return 2;
    case CommandKind::Queue:
      return 1;
  }
  return 0;
}

void RuntimeBridge::push(CommandKind kind, MotionRequest request, std::uint64_t sequence) {
  pending_.push_back({kind, std::move(request), sequence});
}

std::optional<std::uint64_t> RuntimeBridge::latestPendingStopSequence() const {
  std::optional<std::uint64_t> latest;
  for (const auto& command : pending_) {
    if (command.kind == CommandKind::Stop &&
        (!latest || command.sequence > *latest)) {
      latest = command.sequence;
    }
  }
  return latest;
}

void RuntimeBridge::clearPendingMotions() {
  pending_.erase(std::remove_if(pending_.begin(),
                                pending_.end(),
                                [](const Command& command) {
                                  return command.kind == CommandKind::Queue ||
                                         command.kind == CommandKind::Interrupt;
                                }),
                 pending_.end());
}

void RuntimeBridge::clearPendingMotionsAfter(std::uint64_t sequence) {
  pending_.erase(std::remove_if(pending_.begin(),
                                pending_.end(),
                                [sequence](const Command& command) {
                                  return (command.kind == CommandKind::Queue ||
                                          command.kind == CommandKind::Interrupt) &&
                                         command.sequence > sequence;
                                }),
                 pending_.end());
}

void RuntimeBridge::clearPendingMotionsThrough(std::uint64_t sequence) {
  pending_.erase(std::remove_if(pending_.begin(),
                                pending_.end(),
                                [sequence](const Command& command) {
                                  return (command.kind == CommandKind::Queue ||
                                          command.kind == CommandKind::Interrupt) &&
                                         command.sequence <= sequence;
                                }),
                 pending_.end());
}

}  // namespace agentic_et1_tracker
