#include "agentic_et1_tracker/core/status.hpp"

#include <algorithm>
#include <limits>

namespace agentic_et1_tracker {
namespace {

LocoPhase locoPhaseForState(MotionState state) {
  switch (state) {
    case MotionState::Queued:
      return LocoPhase::Queued;
    case MotionState::Running:
      return LocoPhase::Motion;
    case MotionState::Holding:
      return LocoPhase::Holding;
    case MotionState::Stopping:
      return LocoPhase::Stopping;
    case MotionState::Done:
      return LocoPhase::Done;
    case MotionState::Stopped:
      return LocoPhase::Stopped;
    case MotionState::Canceled:
      return LocoPhase::Canceled;
    case MotionState::Failed:
      return LocoPhase::Failed;
  }
  return LocoPhase::Failed;
}

}  // namespace

double computeProgress(std::size_t frame, std::size_t frames, MotionState state) {
  if (state == MotionState::Queued || state == MotionState::Canceled ||
      state == MotionState::Failed) {
    return 0.0;
  }
  if (state == MotionState::Done || state == MotionState::Holding) {
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
  status.executor = request.executor;
  status.state = request.state;
  status.frame = request.frame;
  status.frames = request.frames;
  status.time_s = request.fps > 0.0 ? static_cast<double>(request.frame) / request.fps
                                    : 0.0;
  status.duration_s = request.duration_s;
  status.progress = computeProgress(request.frame, request.frames, request.state);
  status.hold = request.hold;
  status.loco = request.loco;
  if (status.executor == MotionExecutor::LocoUpper) {
    if (status.loco.max_radius_m <= 0.0) {
      status.loco.max_radius_m = request.loco_options.max_radius_m;
    }
    if (request.state == MotionState::Queued ||
        status.loco.phase == LocoPhase::Queued) {
      status.loco.phase = locoPhaseForState(request.state);
    }
  }
  status.stop_reason = request.stop_reason;
  status.err = request.err;
  return status;
}

bool idleConfigBlockedByController(ControllerState ctrl) {
  return ctrl == ControllerState::Starting || ctrl == ControllerState::Idle ||
         ctrl == ControllerState::Passive || ctrl == ControllerState::FixStand ||
         ctrl == ControllerState::Stopping ||
         ctrl == ControllerState::UrgentStopping ||
         ctrl == ControllerState::Fault;
}

bool controlHandoffBlocksUserWork(const StatusSnapshot& snapshot) {
  return snapshot.pending_control.has_value() ||
         (snapshot.active.kind == ActiveKind::Transition &&
          snapshot.transition.target == "standby");
}

}  // namespace agentic_et1_tracker
