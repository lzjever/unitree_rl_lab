#include "agentic_et1_tracker/api/json_codec.hpp"

namespace agentic_et1_tracker {
namespace {

nlohmann::json nullableString(const std::string& value) {
  return value.empty() ? nlohmann::json(nullptr) : nlohmann::json(value);
}

ErrorCode publicErrorCode(ErrorCode code) {
  if (code == ErrorCode::RunStateConflict) {
    return ErrorCode::InternalError;
  }
  return code;
}

}  // namespace

const char* toString(ServiceHealth health) {
  switch (health) {
    case ServiceHealth::Starting:
      return "starting";
    case ServiceHealth::Ready:
      return "ready";
    case ServiceHealth::Error:
      return "error";
  }
  return "error";
}

nlohmann::json errorJson(ErrorCode code) {
  const auto info = apiErrorInfo(code);
  return {
      {"code", toString(info.code)},
      {"message", info.message},
      {"retryable", info.retryable},
  };
}

nlohmann::json nullableErrorJson(ErrorCode code) {
  return code == ErrorCode::Ok ? nlohmann::json(nullptr) : errorJson(code);
}

ErrorInfo apiErrorInfo(ErrorCode code) {
  return errorInfo(publicErrorCode(code));
}

nlohmann::json stopReasonJson(StopReason reason) {
  if (reason == StopReason::None) {
    return nullptr;
  }
  return toString(reason);
}

nlohmann::json motionStatusJson(const MotionStatus& status, bool include_path) {
  auto out = nlohmann::json{
      {"id", status.id},
      {"state", toString(status.state)},
      {"frame", status.frame},
      {"frames", status.frames},
      {"time_s", status.time_s},
      {"duration_s", status.duration_s},
      {"progress", status.progress},
      {"stop_reason", stopReasonJson(status.stop_reason)},
      {"err", nullableErrorJson(status.err)},
  };
  if (include_path) {
    out["path"] = status.path;
  }
  return out;
}

nlohmann::json queueStatusJson(const QueueStatus& status) {
  return {
      {"n", status.n},
      {"limit", status.limit},
      {"ids", status.ids},
  };
}

nlohmann::json statusSnapshotJson(const StatusSnapshot& snapshot) {
  return {
      {"ok", true},
      {"ready", snapshot.ready},
      {"mode", toString(snapshot.mode)},
      {"robot", toString(snapshot.robot)},
      {"ctrl", toString(snapshot.ctrl)},
      {"stop_reason", stopReasonJson(snapshot.stop_reason)},
      {"hz", snapshot.hz},
      {"exec", snapshot.exec ? motionStatusJson(*snapshot.exec, false)
                             : nlohmann::json(nullptr)},
      {"queue", queueStatusJson(snapshot.queue)},
      {"low_ms", snapshot.low_ms},
      {"block", nullableString(snapshot.block)},
      {"err", nullableErrorJson(snapshot.err)},
  };
}

}  // namespace agentic_et1_tracker
