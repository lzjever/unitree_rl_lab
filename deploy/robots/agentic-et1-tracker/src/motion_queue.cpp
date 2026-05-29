#include "agentic_et1_tracker/core/motion_queue.hpp"

#include <algorithm>
#include <chrono>

namespace agentic_et1_tracker {

MotionQueue::MotionQueue(std::size_t limit, std::size_t recent_limit)
    : limit_(limit), recent_(recent_limit) {}

std::size_t MotionQueue::limit() const {
  return limit_;
}

std::size_t MotionQueue::size() const {
  return queue_.size();
}

bool MotionQueue::empty() const {
  return queue_.empty();
}

bool MotionQueue::full() const {
  return queue_.size() >= limit_;
}

QueueInsertResult MotionQueue::enqueue(MotionRequest request) {
  return insert(std::move(request), false);
}

QueueInsertResult MotionQueue::enqueueFront(MotionRequest request) {
  return insert(std::move(request), true);
}

std::optional<MotionRequest> MotionQueue::popNext() {
  if (queue_.empty()) {
    return std::nullopt;
  }
  MotionRequest request = queue_.front();
  queue_.pop_front();
  return request;
}

QueueCancelResult MotionQueue::cancelQueued(StopReason reason) {
  QueueCancelResult result;
  const auto now = std::chrono::steady_clock::now();
  while (!queue_.empty()) {
    MotionRequest request = queue_.front();
    queue_.pop_front();
    request.state = MotionState::Canceled;
    request.stop_reason = reason;
    request.ended_at = now;
    result.ids.push_back(request.id);
    ++result.canceled;
    recent_.push(std::move(request));
  }
  return result;
}

QueueCancelResult MotionQueue::stopQueued() {
  return cancelQueued(StopReason::Stop);
}

InterruptQueueResult MotionQueue::interruptWith(MotionRequest next) {
  const QueueCancelResult canceled = cancelQueued(StopReason::Interrupt);
  const QueueInsertResult inserted = enqueueFront(std::move(next));
  return {inserted.code, canceled.canceled, inserted.queue_size};
}

void MotionQueue::addRecent(MotionRequest request) {
  recent_.push(std::move(request));
}

std::optional<MotionRequest> MotionQueue::findQueued(const std::string& id) const {
  const auto it = std::find_if(queue_.begin(), queue_.end(), [&](const MotionRequest& request) {
    return request.id == id;
  });
  if (it == queue_.end()) {
    return std::nullopt;
  }
  return *it;
}

std::optional<MotionRequest> MotionQueue::findRecent(const std::string& id) const {
  return recent_.find(id);
}

std::optional<MotionRequest> MotionQueue::find(const std::string& id) const {
  if (auto queued = findQueued(id)) {
    return queued;
  }
  return findRecent(id);
}

std::vector<std::string> MotionQueue::queuedIds() const {
  std::vector<std::string> ids;
  ids.reserve(queue_.size());
  for (const auto& request : queue_) {
    ids.push_back(request.id);
  }
  return ids;
}

QueueInsertResult MotionQueue::insert(MotionRequest request, bool front) {
  if (full()) {
    return {ErrorCode::QueueFull, queue_.size()};
  }
  request.state = MotionState::Queued;
  request.enqueued_at = std::chrono::steady_clock::now();
  if (front) {
    queue_.push_front(std::move(request));
  } else {
    queue_.push_back(std::move(request));
  }
  return {ErrorCode::Ok, queue_.size()};
}

}  // namespace agentic_et1_tracker
