#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/recent_ring.hpp"
#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

struct QueueInsertResult {
  ErrorCode code{ErrorCode::Ok};
  std::size_t queue_size{0};
};

struct QueueCancelResult {
  std::size_t canceled{0};
  std::vector<std::string> ids;
};

struct InterruptQueueResult {
  ErrorCode code{ErrorCode::Ok};
  std::size_t canceled{0};
  std::size_t queue_size{0};
};

class MotionQueue {
 public:
  explicit MotionQueue(std::size_t limit = 8, std::size_t recent_limit = 32);

  std::size_t limit() const;
  std::size_t size() const;
  bool empty() const;
  bool full() const;

  QueueInsertResult enqueue(MotionRequest request);
  QueueInsertResult enqueueFront(MotionRequest request);
  std::optional<MotionRequest> popNext();

  QueueCancelResult cancelQueued(StopReason reason);
  QueueCancelResult stopQueued();
  InterruptQueueResult interruptWith(MotionRequest next);

  void addRecent(MotionRequest request);
  std::optional<MotionRequest> findQueued(const std::string& id) const;
  std::optional<MotionRequest> findRecent(const std::string& id) const;
  std::optional<MotionRequest> find(const std::string& id) const;
  std::vector<std::string> queuedIds() const;

 private:
  QueueInsertResult insert(MotionRequest request, bool front);

  std::size_t limit_;
  std::deque<MotionRequest> queue_;
  RecentRing recent_;
};

}  // namespace agentic_et1_tracker
