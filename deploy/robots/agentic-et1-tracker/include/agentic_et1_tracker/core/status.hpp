#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

struct Progress {
  std::size_t frame{0};
  std::size_t frames{0};
  double progress{0.0};
};

struct QueueStatus {
  std::size_t n{0};
  std::size_t limit{0};
  std::vector<std::string> ids;
};

struct MotionStatus {
  std::uint64_t sequence{0};
  std::string id;
  std::string path;
  MotionState state{MotionState::Queued};
  std::size_t frame{0};
  std::size_t frames{0};
  double time_s{0.0};
  double duration_s{0.0};
  double progress{0.0};
  StopReason stop_reason{StopReason::None};
  ErrorCode err{ErrorCode::Ok};
};

struct PoseSnapshot {
  std::optional<std::array<float, 4>> q_wxyz;
  std::optional<std::array<float, 3>> gyro_xyz;
  std::optional<std::array<float, 3>> position_xyz;
  std::optional<std::array<float, 3>> velocity_xyz;
};

struct StatusSnapshot {
  bool ready{false};
  RuntimeMode mode{RuntimeMode::Unknown};
  RobotState robot{RobotState::Disconnected};
  ControllerState ctrl{ControllerState::Starting};
  StopReason stop_reason{StopReason::None};
  double hz{0.0};
  std::optional<MotionStatus> exec;
  QueueStatus queue;
  std::size_t low_ms{0};
  std::string block;
  ErrorCode err{ErrorCode::Ok};
  PoseSnapshot pose;
};

double computeProgress(std::size_t frame, std::size_t frames, MotionState state);
Progress makeProgress(std::size_t frame, std::size_t frames, MotionState state);
MotionStatus makeMotionStatus(const MotionRequest& request);

}  // namespace agentic_et1_tracker
