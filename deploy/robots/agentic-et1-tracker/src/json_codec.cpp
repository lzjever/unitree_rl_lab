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

template <std::size_t N>
nlohmann::json optionalArrayJson(const std::optional<std::array<float, N>>& value) {
  if (!value) {
    return nullptr;
  }
  nlohmann::json out = nlohmann::json::array();
  for (const float item : *value) {
    out.push_back(item);
  }
  return out;
}

nlohmann::json poseSnapshotJson(const PoseSnapshot& pose) {
  return {
      {"q", optionalArrayJson(pose.q_wxyz)},
      {"g", optionalArrayJson(pose.gyro_xyz)},
      {"p", optionalArrayJson(pose.position_xyz)},
      {"v", optionalArrayJson(pose.velocity_xyz)},
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
      {"pose", poseSnapshotJson(snapshot.pose)},
  };
}

}  // namespace agentic_et1_tracker
