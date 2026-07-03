#include "agentic_et1_tracker/api/json_codec.hpp"

namespace agentic_et1_tracker {
namespace {

nlohmann::json nullableString(const std::string& value) {
  return value.empty() ? nlohmann::json(nullptr) : nlohmann::json(value);
}

nlohmann::json nullableLocoReason(LocoReason reason) {
  return reason == LocoReason::None ? nlohmann::json(nullptr)
                                    : nlohmann::json(toString(reason));
}

ErrorCode publicErrorCode(ErrorCode code) {
  if (code == ErrorCode::RunStateConflict) {
    return ErrorCode::InternalError;
  }
  return code;
}

nlohmann::json locoRunStatusJson(const LocoRunStatus& status) {
  return {
      {"max_radius_m", status.max_radius_m},
      {"distance_m", status.distance_m},
      {"radius_source", nullableString(status.radius_source)},
      {"phase", toString(status.phase)},
      {"radius_clamped", status.radius_clamped},
      {"radius_limit_reached", status.radius_limit_reached},
      {"envelope_clamped", status.envelope_clamped},
      {"upper_clamped", status.upper_clamped},
      {"upper_rate_limited", status.upper_rate_limited},
      {"raw_action_clamped", status.raw_action_clamped},
      {"lower_q_limited", status.lower_q_limited},
      {"lower_action_clamped", status.lower_action_clamped},
      {"reason", nullableLocoReason(status.reason)},
  };
}

nlohmann::json locoUpperCapabilityJson(LocoUpperCapability capability) {
  if (!capability.enabled) {
    capability.ready = false;
  }
  return {
      {"enabled", capability.enabled},
      {"ready", capability.ready},
      {"default_radius_m", capability.default_radius_m},
      {"max_radius_m", capability.max_radius_m},
      {"strict_pose", capability.strict_pose},
  };
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
      {"hold", status.hold},
      {"stop_reason", stopReasonJson(status.stop_reason)},
      {"err", nullableErrorJson(status.err)},
  };
  if (include_path) {
    out["path"] = status.path;
  }
  if (status.executor == MotionExecutor::LocoUpper) {
    out["executor"] = toString(status.executor);
    out["loco"] = locoRunStatusJson(status.loco);
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

nlohmann::json activeStatusJson(const ActiveStatus& status) {
  return {
      {"kind", toString(status.kind)},
      {"id", status.kind == ActiveKind::User ? nlohmann::json(status.id)
                                             : nlohmann::json(nullptr)},
  };
}

nlohmann::json idleStatusJson(const IdleStatus& status) {
  return {
      {"enabled", status.enabled},
      {"n", status.n},
      {"active", status.active},
      {"current", status.current ? nlohmann::json(*status.current)
                                 : nlohmann::json(nullptr)},
      {"frame", status.frame},
      {"frames", status.frames},
      {"time_s", status.time_s},
      {"duration_s", status.duration_s},
      {"progress", status.progress},
  };
}

nlohmann::json transitionStatusJson(const TransitionStatus& status) {
  return {
      {"active", status.active},
      {"target", nullableString(status.target)},
      {"target_id", nullableString(status.target_id)},
      {"target_state",
       status.target_state ? nlohmann::json(toString(*status.target_state))
                           : nlohmann::json(nullptr)},
      {"frame", status.frame},
      {"frames", status.frames},
      {"progress", status.progress},
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

nlohmann::json passiveReasonJson(const std::optional<PassiveReason>& reason) {
  if (!reason) {
    return nullptr;
  }
  return {
      {"code", toString(reason->code)},
      {"block", nullableString(reason->block)},
  };
}

ActiveStatus effectiveActiveStatus(const StatusSnapshot& snapshot) {
  if (snapshot.active.kind == ActiveKind::None && snapshot.exec) {
    return {ActiveKind::User, snapshot.exec->id};
  }
  return snapshot.active;
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
      {"active", activeStatusJson(effectiveActiveStatus(snapshot))},
      {"exec", snapshot.exec ? motionStatusJson(*snapshot.exec, false)
                             : nlohmann::json(nullptr)},
      {"queue", queueStatusJson(snapshot.queue)},
      {"idle", idleStatusJson(snapshot.idle)},
      {"transition", transitionStatusJson(snapshot.transition)},
      {"low_ms", snapshot.low_ms},
      {"block", nullableString(snapshot.block)},
      {"err", nullableErrorJson(snapshot.err)},
      {"passive_reason", passiveReasonJson(snapshot.passive_reason)},
      {"pose", poseSnapshotJson(snapshot.pose)},
      {"cap", {{"loco_upper", locoUpperCapabilityJson(snapshot.loco_upper)}}},
  };
}

}  // namespace agentic_et1_tracker
