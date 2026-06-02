#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"

namespace agentic_et1_tracker {

class RuntimeStatusStore final : public StatusReader {
 public:
  explicit RuntimeStatusStore(RuntimeConfig config = {});

  void publishSnapshot(StatusSnapshot snapshot);
  void publishHealthSnapshot(HealthSnapshot health);
  void publishRunStatus(MotionStatus status);

  StatusSnapshot snapshot() const override;
  RunLookupResult findRun(const std::string& id) const override;
  HealthSnapshot health() const override;

 private:
  friend class RuntimeBridge;

  ExecuteResult acceptQueued(const ExecuteCommand& command, std::uint64_t sequence);
  ExecuteResult acceptInterrupt(const ExecuteCommand& command, std::uint64_t sequence);
  ExecuteResult acceptInterruptAfterStop(const ExecuteCommand& command,
                                         std::uint64_t sequence,
                                         std::uint64_t stop_sequence);
  StopResult acceptStop();
  ControlResult acceptControl(ControlMode mode, bool preserve_queued = false);
  IdleResult acceptIdleConfig(std::vector<IdleMotion> motions);
  bool clearIdleConfig();
  std::size_t cancelQueuedForStop(std::uint64_t sequence);
  std::vector<std::string> queuedIdsLocked() const;
  std::size_t cancelQueuedLocked(StopReason reason);
  std::size_t cancelQueuedAfterLocked(StopReason reason, std::uint64_t sequence);

  RuntimeConfig config_;
  mutable std::mutex mutex_;
  StatusSnapshot snapshot_;
  HealthSnapshot health_;
  std::deque<MotionStatus> accepted_;
  std::deque<MotionStatus> recent_;
  std::vector<IdleMotion> idle_config_;
  IdleStatus idle_status_;
};

}  // namespace agentic_et1_tracker
