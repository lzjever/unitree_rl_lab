#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

enum class CommandKind {
  Queue,
  Interrupt,
  Stop,
  FixStand,
  StandbyVelocity,
};

struct Command {
  CommandKind kind{CommandKind::Queue};
  MotionRequest request;
  std::uint64_t sequence{0};
  ControlMode control{ControlMode::StandbyVelocity};
};

class CommandMailbox {
 public:
  void submitQueue(MotionRequest request);
  void submitInterrupt(MotionRequest request);
  void submitStop();

  std::optional<Command> consumeNext();
  bool empty() const;
  std::size_t size() const;
  void clear();

 private:
  static int priority(CommandKind kind);

  std::uint64_t next_sequence_{0};
  std::deque<Command> pending_;
};

}  // namespace agentic_et1_tracker
