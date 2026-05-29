#include "agentic_et1_tracker/core/status.hpp"

#include <algorithm>
#include <limits>

namespace agentic_et1_tracker {

double computeProgress(std::size_t frame, std::size_t frames, MotionState state) {
  if (state == MotionState::Queued || state == MotionState::Canceled ||
      state == MotionState::Failed) {
    return 0.0;
  }
  if (state == MotionState::Done) {
    return 1.0;
  }
  if (frames == 0) {
    return 0.0;
  }
  if (frame + 1 >= frames || frame == std::numeric_limits<std::size_t>::max()) {
    return 1.0;
  }
  return std::min(1.0,
                  static_cast<double>(frame + 1) / static_cast<double>(frames));
}

Progress makeProgress(std::size_t frame, std::size_t frames, MotionState state) {
  return {frame, frames, computeProgress(frame, frames, state)};
}

MotionStatus makeMotionStatus(const MotionRequest& request) {
  MotionStatus status;
  status.sequence = request.sequence;
  status.id = request.id;
  status.path = request.path;
  status.state = request.state;
  status.frame = request.frame;
  status.frames = request.frames;
  status.time_s = request.fps > 0.0 ? static_cast<double>(request.frame) / request.fps
                                    : 0.0;
  status.duration_s = request.duration_s;
  status.progress = computeProgress(request.frame, request.frames, request.state);
  status.stop_reason = request.stop_reason;
  status.err = request.err;
  return status;
}

}  // namespace agentic_et1_tracker
