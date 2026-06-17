#pragma once

#include <algorithm>
#include <cmath>

#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"
#include "agentic_et1_tracker/loco_upper/planner.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"

namespace agentic_et1_tracker {

inline double maxAbsRangeValue(const LocoLowerRange& range) {
  return std::max(std::abs(range.min), std::abs(range.max));
}

inline LocoUpperCommandLimits locoUpperCommandLimitsFromConfig(
    const RuntimeConfig& runtime_config,
    const LocoLowerDeployConfig& lower_config) {
  LocoUpperCommandLimits limits;
  const double max_vx = maxAbsRangeValue(lower_config.command_ranges.lin_vel_x);
  const double max_vy = maxAbsRangeValue(lower_config.command_ranges.lin_vel_y);
  limits.max_linear_velocity = std::hypot(max_vx, max_vy);
  limits.max_yaw_rate = maxAbsRangeValue(lower_config.command_ranges.ang_vel_z);
  limits.max_linear_acceleration = runtime_config.loco_upper_max_lin_accel_mps2;
  limits.max_yaw_acceleration = runtime_config.loco_upper_max_yaw_accel_radps2;
  limits.smoothing_window_frames = runtime_config.loco_upper_smoothing_window_frames;
  return limits;
}

}  // namespace agentic_et1_tracker
