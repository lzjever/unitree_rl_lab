#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

std::string toString(ErrorCode code) {
  switch (code) {
    case ErrorCode::Ok:
      return "OK";
    case ErrorCode::RequestInvalid:
      return "REQUEST_INVALID";
    case ErrorCode::ServiceNotReady:
      return "SERVICE_NOT_READY";
    case ErrorCode::RobotDisconnected:
      return "ROBOT_DISCONNECTED";
    case ErrorCode::RobotNotReady:
      return "ROBOT_NOT_READY";
    case ErrorCode::RobotBadOrientation:
      return "ROBOT_BAD_ORIENTATION";
    case ErrorCode::ModelNotReady:
      return "MODEL_NOT_READY";
    case ErrorCode::ModelInferenceFailed:
      return "MODEL_INFERENCE_FAILED";
    case ErrorCode::TrkPathNotAllowed:
      return "TRK_PATH_NOT_ALLOWED";
    case ErrorCode::TrkFileNotFound:
      return "TRK_FILE_NOT_FOUND";
    case ErrorCode::TrkParseFailed:
      return "TRK_PARSE_FAILED";
    case ErrorCode::TrkValidationFailed:
      return "TRK_VALIDATION_FAILED";
    case ErrorCode::QueueFull:
      return "QUEUE_FULL";
    case ErrorCode::RunNotFound:
      return "RUN_NOT_FOUND";
    case ErrorCode::RunStateConflict:
      return "RUN_STATE_CONFLICT";
    case ErrorCode::ControlStateConflict:
      return "CONTROL_STATE_CONFLICT";
    case ErrorCode::SafetyLimitTriggered:
      return "SAFETY_LIMIT_TRIGGERED";
    case ErrorCode::InternalError:
      return "INTERNAL_ERROR";
  }
  return "INTERNAL_ERROR";
}

std::string toString(NextAction next) {
  switch (next) {
    case NextAction::Status:
      return "status";
    case NextAction::Retry:
      return "retry";
    case NextAction::WaitRobot:
      return "wait_robot";
    case NextAction::Fix:
      return "fix";
    case NextAction::FixStand:
      return "fixstand";
    case NextAction::StandbyVelocity:
      return "standby_velocity";
    case NextAction::Stop:
      return "stop";
    case NextAction::Manual:
      return "manual";
  }
  return "manual";
}

std::string toString(MotionState state) {
  switch (state) {
    case MotionState::Queued:
      return "queued";
    case MotionState::Running:
      return "running";
    case MotionState::Stopping:
      return "stopping";
    case MotionState::Done:
      return "done";
    case MotionState::Stopped:
      return "stopped";
    case MotionState::Canceled:
      return "canceled";
    case MotionState::Failed:
      return "failed";
  }
  return "failed";
}

std::string toString(MotionMode mode) {
  switch (mode) {
    case MotionMode::Queue:
      return "queue";
    case MotionMode::Interrupt:
      return "interrupt";
  }
  return "queue";
}

std::string toString(RuntimeMode mode) {
  switch (mode) {
    case RuntimeMode::Unknown:
      return "unknown";
    case RuntimeMode::Sim:
      return "sim";
    case RuntimeMode::Real:
      return "real";
  }
  return "unknown";
}

std::string toString(ControllerState state) {
  switch (state) {
    case ControllerState::Starting:
      return "starting";
    case ControllerState::Idle:
      return "idle";
    case ControllerState::Passive:
      return "passive";
    case ControllerState::FixStand:
      return "fixstand";
    case ControllerState::StandbyVelocity:
      return "standby_velocity";
    case ControllerState::Preparing:
      return "preparing";
    case ControllerState::Running:
      return "running";
    case ControllerState::Stopping:
      return "stopping";
    case ControllerState::Fault:
      return "fault";
  }
  return "fault";
}

std::string toString(ControlMode mode) {
  switch (mode) {
    case ControlMode::FixStand:
      return "fixstand";
    case ControlMode::StandbyVelocity:
      return "standby_velocity";
  }
  return "standby_velocity";
}

std::string toString(RobotState state) {
  switch (state) {
    case RobotState::Disconnected:
      return "disconnected";
    case RobotState::NotReady:
      return "not_ready";
    case RobotState::Idle:
      return "idle";
    case RobotState::Running:
      return "running";
    case RobotState::Holding:
      return "holding";
    case RobotState::Fault:
      return "fault";
  }
  return "fault";
}

std::string toString(StopReason reason) {
  switch (reason) {
    case StopReason::None:
      return "null";
    case StopReason::Stop:
      return "stop";
    case StopReason::Interrupt:
      return "interrupt";
  }
  return "null";
}

ErrorInfo errorInfo(ErrorCode code) {
  switch (code) {
    case ErrorCode::Ok:
      return {code, "", false, NextAction::Status};
    case ErrorCode::RequestInvalid:
      return {code, "request is invalid", false, NextAction::Fix};
    case ErrorCode::ServiceNotReady:
      return {code, "service is not ready", true, NextAction::Status};
    case ErrorCode::RobotDisconnected:
      return {code, "robot lowstate is disconnected", true, NextAction::WaitRobot};
    case ErrorCode::RobotNotReady:
      return {code, "robot is not ready", true, NextAction::WaitRobot};
    case ErrorCode::RobotBadOrientation:
      return {code, "robot orientation is outside safe limits", false, NextAction::Manual};
    case ErrorCode::ModelNotReady:
      return {code, "policy model is not ready", true, NextAction::Status};
    case ErrorCode::ModelInferenceFailed:
      return {code, "policy inference failed", false, NextAction::Manual};
    case ErrorCode::TrkPathNotAllowed:
      return {code, "track path is not allowed", false, NextAction::Fix};
    case ErrorCode::TrkFileNotFound:
      return {code, "track file was not found", false, NextAction::Fix};
    case ErrorCode::TrkParseFailed:
      return {code, "track file could not be parsed", false, NextAction::Fix};
    case ErrorCode::TrkValidationFailed:
      return {code, "track file failed validation", false, NextAction::Fix};
    case ErrorCode::QueueFull:
      return {code, "motion queue is full", true, NextAction::Status};
    case ErrorCode::RunNotFound:
      return {code, "run id was not found", false, NextAction::Status};
    case ErrorCode::RunStateConflict:
      return {code, "run state conflicts with the requested action", true, NextAction::Stop};
    case ErrorCode::ControlStateConflict:
      return {code, "wrong ctrl; check /status", false, NextAction::Status};
    case ErrorCode::SafetyLimitTriggered:
      return {code, "safety limit triggered", false, NextAction::Manual};
    case ErrorCode::InternalError:
      return {code, "internal error", false, NextAction::Manual};
  }
  return {ErrorCode::InternalError, "internal error", false, NextAction::Manual};
}

}  // namespace agentic_et1_tracker
