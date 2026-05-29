#pragma once

#include <cstddef>

namespace agentic_et1_tracker {

struct RuntimeConfig {
  std::size_t queue_limit{8};
  std::size_t recent_limit{32};
  double hz{50.0};
  double stop_hold_s{0.5};
};

}  // namespace agentic_et1_tracker
