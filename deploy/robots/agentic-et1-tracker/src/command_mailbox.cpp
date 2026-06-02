#include "agentic_et1_tracker/core/command_mailbox.hpp"

#include <algorithm>

namespace agentic_et1_tracker {

void CommandMailbox::submitQueue(MotionRequest request) {
  pending_.push_back({CommandKind::Queue, std::move(request), next_sequence_++});
}

void CommandMailbox::submitInterrupt(MotionRequest request) {
  pending_.push_back({CommandKind::Interrupt, std::move(request), next_sequence_++});
}

void CommandMailbox::submitStop() {
  Command command;
  command.kind = CommandKind::Stop;
  command.sequence = next_sequence_++;
  command.stop_requires_stopping = true;
  pending_.push_back(std::move(command));
}

std::optional<Command> CommandMailbox::consumeNext() {
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
  return command;
}

bool CommandMailbox::empty() const {
  return pending_.empty();
}

std::size_t CommandMailbox::size() const {
  return pending_.size();
}

void CommandMailbox::clear() {
  pending_.clear();
}

int CommandMailbox::priority(CommandKind kind) {
  switch (kind) {
    case CommandKind::Stop:
    case CommandKind::FixStand:
    case CommandKind::StandbyVelocity:
      return 3;
    case CommandKind::Interrupt:
      return 2;
    case CommandKind::Queue:
    case CommandKind::IdleConfig:
      return 1;
  }
  return 0;
}

}  // namespace agentic_et1_tracker
