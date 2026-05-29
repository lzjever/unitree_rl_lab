#pragma once

#include <nlohmann/json.hpp>

#include "agentic_et1_tracker/core/status.hpp"
#include "agentic_et1_tracker/core/tracker_controller.hpp"
#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

enum class ServiceHealth {
  Starting,
  Ready,
  Error,
};

nlohmann::json errorJson(ErrorCode code);
nlohmann::json nullableErrorJson(ErrorCode code);
ErrorInfo apiErrorInfo(ErrorCode code);
nlohmann::json stopReasonJson(StopReason reason);
nlohmann::json motionStatusJson(const MotionStatus& status, bool include_path = true);
nlohmann::json queueStatusJson(const QueueStatus& status);
nlohmann::json statusSnapshotJson(const StatusSnapshot& snapshot);
const char* toString(ServiceHealth health);

}  // namespace agentic_et1_tracker
