#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace agentic_et1_tracker {

enum class ErrorCode {
  Ok,
  RequestInvalid,
  ServiceNotReady,
  RobotDisconnected,
  RobotNotReady,
  RobotBadOrientation,
  ModelNotReady,
  ModelInferenceFailed,
  TrkPathNotAllowed,
  TrkFileNotFound,
  TrkParseFailed,
  TrkValidationFailed,
  QueueFull,
  RunNotFound,
  RunStateConflict,
  SafetyLimitTriggered,
  InternalError,
};

enum class NextAction {
  Status,
  Retry,
  WaitRobot,
  Fix,
  Stop,
  Manual,
};

struct ErrorInfo {
  ErrorCode code{ErrorCode::Ok};
  std::string message;
  bool retryable{false};
  NextAction next{NextAction::Status};
};

enum class MotionState {
  Queued,
  Running,
  Stopping,
  Done,
  Stopped,
  Canceled,
  Failed,
};

enum class MotionMode {
  Queue,
  Interrupt,
};

enum class RuntimeMode {
  Unknown,
  Sim,
  Real,
};

enum class ControllerState {
  Starting,
  Idle,
  FixStand,
  StandbyVelocity,
  Preparing,
  Running,
  Stopping,
  Fault,
};

enum class ControlMode {
  FixStand,
  StandbyVelocity,
};

enum class RobotState {
  Disconnected,
  NotReady,
  Idle,
  Running,
  Holding,
  Fault,
};

enum class StopReason {
  None,
  Stop,
  Interrupt,
};

struct MotionRequest {
  std::uint64_t sequence{0};
  std::string id;
  std::string path;
  MotionState state{MotionState::Queued};
  std::size_t frame{0};
  std::size_t frames{0};
  double fps{50.0};
  double duration_s{0.0};
  ErrorCode err{ErrorCode::Ok};
  StopReason stop_reason{StopReason::None};
  std::chrono::steady_clock::time_point enqueued_at{};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point ended_at{};
};

std::string toString(ErrorCode code);
std::string toString(NextAction next);
std::string toString(MotionState state);
std::string toString(MotionMode mode);
std::string toString(RuntimeMode mode);
std::string toString(ControllerState state);
std::string toString(ControlMode mode);
std::string toString(RobotState state);
std::string toString(StopReason reason);

ErrorInfo errorInfo(ErrorCode code);

}  // namespace agentic_et1_tracker
