#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/core/command_mailbox.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"

namespace agentic_et1_tracker {

class RuntimeBridge final : public ExecutionCommandSink {
 public:
  RuntimeBridge(RuntimeConfig config, RuntimeStatusStore& status);

  ExecuteResult submitQueue(const ExecuteCommand& command) override;
  ExecuteResult submitInterrupt(const ExecuteCommand& command) override;
  StopResult stop() override;
  ControlResult fixStand() override;
  ControlResult standbyVelocity() override;
  IdleResult configureIdle(std::vector<IdleMotion> motions) override;

  std::optional<Command> consumeNextCommand();

 private:
  static int priority(CommandKind kind);

  void push(CommandKind kind,
            MotionRequest request,
            std::uint64_t sequence,
            bool stop_requires_stopping = false);
  std::optional<std::uint64_t> latestPendingStopSequence() const;
  void clearPendingMotions();
  void clearPendingCommands();
  void clearPendingIdleConfigs();
  void clearPendingControlsAfter(std::uint64_t sequence);
  void clearPendingMotionsAfter(std::uint64_t sequence);
  void clearPendingMotionsThrough(std::uint64_t sequence);

  RuntimeStatusStore& status_;
  std::mutex mutex_;
  std::uint64_t next_sequence_{0};
  std::deque<Command> pending_;
};

}  // namespace agentic_et1_tracker
