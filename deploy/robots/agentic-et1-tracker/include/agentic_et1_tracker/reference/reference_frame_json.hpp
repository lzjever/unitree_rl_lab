#pragma once

#include <chrono>

#include <nlohmann/json.hpp>

#include "agentic_et1_tracker/reference/reference_frame.hpp"

namespace agentic_et1_tracker {

nlohmann::json referenceFrameSnapshotJson(
    const ReferenceFrameSnapshot& snapshot,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

}  // namespace agentic_et1_tracker
