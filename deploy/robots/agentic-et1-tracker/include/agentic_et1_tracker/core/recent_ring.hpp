#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

class RecentRing {
 public:
  explicit RecentRing(std::size_t limit = 32);

  std::size_t limit() const;
  std::size_t size() const;
  bool empty() const;

  void push(MotionRequest request);
  std::optional<MotionRequest> find(const std::string& id) const;
  std::vector<std::string> ids() const;
  void clear();

 private:
  std::size_t limit_;
  std::deque<MotionRequest> items_;
};

}  // namespace agentic_et1_tracker
