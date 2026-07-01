#pragma once

#include <cstddef>
#include <optional>

#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

struct SyntheticTransitionLimits {
  double max_velocity;
  double max_acceleration;
  double max_jerk;
};

[[nodiscard]] constexpr SyntheticTransitionLimits defaultSyntheticTransitionLimits() noexcept {
  return {250.0, 10000.0, 1000000.0};
}

enum class SyntheticTransitionRootYawMode {
  kDisabled,
};

struct SyntheticTransitionOptions {
  double max_duration_s = 0.0;
  std::size_t min_frames = 2;
  double duration_dt_tolerance_s = 1.0e-9;
  SyntheticTransitionLimits limits = defaultSyntheticTransitionLimits();
  // Root yaw is intentionally not controlled in this slice. Keep the option explicit so a
  // future runtime/config change has one place to opt in without overloading body channels.
  SyntheticTransitionRootYawMode root_yaw_mode = SyntheticTransitionRootYawMode::kDisabled;
};

[[nodiscard]] std::optional<TrkTrack> makeSyntheticTransitionTrk(
    const TrkFrameView& source,
    const TrkFrameView& target,
    double target_fps,
    SyntheticTransitionOptions options);

[[nodiscard]] std::optional<TrkTrack> makeSyntheticTransitionTrk(
    const TrkFrameView& source,
    const TrkFrameView& target,
    double target_fps,
    double duration_s);

}  // namespace agentic_et1_tracker
