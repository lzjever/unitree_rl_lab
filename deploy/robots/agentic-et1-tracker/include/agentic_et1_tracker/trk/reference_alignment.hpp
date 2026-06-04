#pragma once

#include <optional>

#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

// Align target frame-0 root XY and yaw to the source root. Root Z and
// non-world/local arrays keep the target track semantics.
[[nodiscard]] std::optional<TrkTrack> alignTrackRootPlanarPose(
    const TrkTrack& target,
    const TrkFrameView& source);

}  // namespace agentic_et1_tracker
