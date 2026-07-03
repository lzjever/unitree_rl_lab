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
  ControlStateConflict,
  ControlRouteRenamed,
  SafetyLimitTriggered,
  InternalError,
};

enum class NextAction {
  Status,
  Retry,
  WaitRobot,
  Fix,
  FixStand,
  Standby,
  StandbyVelocity,
  UrgentStop,
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
  Holding,
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

enum class MotionExecutor {
  GeneralTracker,
  LocoUpper,
};

enum class LocoPhase {
  Queued,
  Entry,
  Motion,
  Holding,
  Exit,
  Stopping,
  Done,
  Stopped,
  Failed,
  Canceled,
};

enum class LocoReason {
  None,
  RootInvalid,
  UpperLimit,
  UpperDynamic,
  RadiusLimit,
  PoseMissing,
  PoseJump,
  PolicyNan,
  PolicyInfer,
  LowerLimit,
  MappingInvalid,
  HoldTimeout,
  PathError,
  DeadlineMiss,
};

enum class ActiveKind {
  None,
  User,
  Idle,
  Transition,
};

enum class RuntimeMode {
  Unknown,
  Sim,
  Real,
};

enum class ControllerState {
  Starting,
  Idle,
  Passive,
  FixStand,
  StandbyVelocity,
  Preparing,
  Running,
  Stopping,
  UrgentStopping,
  Fault,
};

enum class ControlMode {
  Passive,
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
  UrgentStop,
  Interrupt,
};

struct LocoRunOptions {
  double max_radius_m{0.0};
  bool hold{false};
  bool radius_clamped{false};
  bool envelope_clamped{false};
  bool upper_clamped{false};
  bool upper_rate_limited{false};
};

struct LocoRunStatus {
  double max_radius_m{0.0};
  double distance_m{0.0};
  std::string radius_source;
  LocoPhase phase{LocoPhase::Queued};
  bool radius_clamped{false};
  bool radius_limit_reached{false};
  bool envelope_clamped{false};
  bool upper_clamped{false};
  bool upper_rate_limited{false};
  bool raw_action_clamped{false};
  bool lower_q_limited{false};
  bool lower_action_clamped{false};
  LocoReason reason{LocoReason::None};
};

struct LocoUpperCapability {
  bool enabled{false};
  bool ready{false};
  double default_radius_m{0.0};
  double max_radius_m{0.0};
  bool strict_pose{false};
};

struct MotionRequest {
  std::uint64_t sequence{0};
  std::string id;
  std::string path;
  MotionExecutor executor{MotionExecutor::GeneralTracker};
  MotionState state{MotionState::Queued};
  std::size_t frame{0};
  std::size_t frames{0};
  double fps{50.0};
  double duration_s{0.0};
  bool hold{false};
  LocoRunOptions loco_options;
  LocoRunStatus loco;
  ErrorCode err{ErrorCode::Ok};
  StopReason stop_reason{StopReason::None};
  std::chrono::steady_clock::time_point enqueued_at{};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point ended_at{};
};

struct TrackMetadata {
  std::size_t frames{0};
  double duration_s{0.0};
  double fps{50.0};
  std::string canonical_path;
};

struct IdleMotion {
  std::string path;
  TrackMetadata track;
};

std::string toString(ErrorCode code);
std::string toString(NextAction next);
std::string toString(MotionState state);
std::string toString(MotionMode mode);
std::string toString(MotionExecutor executor);
std::string toString(LocoPhase phase);
std::string toString(LocoReason reason);
std::string toString(ActiveKind kind);
std::string toString(RuntimeMode mode);
std::string toString(ControllerState state);
std::string toString(ControlMode mode);
std::string toString(RobotState state);
std::string toString(StopReason reason);

ErrorInfo errorInfo(ErrorCode code);

}  // namespace agentic_et1_tracker
