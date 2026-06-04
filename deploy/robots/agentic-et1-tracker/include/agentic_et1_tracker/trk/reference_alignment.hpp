#pragma once

#include <optional>

#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

// Align only the world-root XY origin. Z and all non-position arrays retain the
// target track semantics.
[[nodiscard]] std::optional<TrkTrack> alignTrackRootTranslation(
    const TrkTrack& target,
    const TrkFrameView& source);

}  // namespace agentic_et1_tracker
