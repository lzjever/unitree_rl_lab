#include "agentic_et1_tracker/core/recent_ring.hpp"

#include <algorithm>

namespace agentic_et1_tracker {

RecentRing::RecentRing(std::size_t limit) : limit_(limit) {}

std::size_t RecentRing::limit() const {
  return limit_;
}

std::size_t RecentRing::size() const {
  return items_.size();
}

bool RecentRing::empty() const {
  return items_.empty();
}

void RecentRing::push(MotionRequest request) {
  if (limit_ == 0) {
    return;
  }

  const auto existing = std::find_if(items_.begin(), items_.end(), [&](const MotionRequest& item) {
    return item.id == request.id;
  });
  if (existing != items_.end()) {
    items_.erase(existing);
  }

  items_.push_back(std::move(request));
  while (items_.size() > limit_) {
    items_.pop_front();
  }
}

std::optional<MotionRequest> RecentRing::find(const std::string& id) const {
  const auto it = std::find_if(items_.begin(), items_.end(), [&](const MotionRequest& item) {
    return item.id == id;
  });
  if (it == items_.end()) {
    return std::nullopt;
  }
  return *it;
}

std::vector<std::string> RecentRing::ids() const {
  std::vector<std::string> out;
  out.reserve(items_.size());
  for (const auto& item : items_) {
    out.push_back(item.id);
  }
  return out;
}

void RecentRing::clear() {
  items_.clear();
}

}  // namespace agentic_et1_tracker
