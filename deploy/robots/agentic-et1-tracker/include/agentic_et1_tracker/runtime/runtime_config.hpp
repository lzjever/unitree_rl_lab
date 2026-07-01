#pragma once

#include <cstddef>

namespace agentic_et1_tracker {

struct RuntimeConfig {
  std::size_t queue_limit{8};
  std::size_t recent_limit{32};
  double hz{1000.0};
  double stop_hold_s{0.0};
  double transition_duration_s{0.30};
  std::size_t transition_min_frames{2};
  double transition_duration_dt_tolerance_s{1.0e-9};
  double user_bridge_reduced_startup_hold_s{0.10};
  double transition_root_yaw_residual_limit_rad{0.05};
  double transition_max_velocity{250.0};
  double transition_max_acceleration{10000.0};
  double transition_max_jerk{1000000.0};
  double loco_upper_max_hold_s{10.0};
  double radius_tolerance_m{0.05};
  bool loco_upper_strict_pose{false};
  std::size_t loco_upper_pose_fresh_timeout_ms{100};
  double loco_upper_pose_jump_reject_m{0.25};
  double loco_upper_max_lin_accel_mps2{0.4};
  double loco_upper_max_yaw_accel_radps2{0.5};
  std::size_t loco_upper_smoothing_window_frames{5};
};

}  // namespace agentic_et1_tracker
