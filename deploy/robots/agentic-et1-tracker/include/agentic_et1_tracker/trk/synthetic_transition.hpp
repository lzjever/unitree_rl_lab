#pragma once

#include <optional>

#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

[[nodiscard]] std::optional<TrkTrack> makeSyntheticTransitionTrk(
    const TrkFrameView& source,
    const TrkFrameView& target,
    double target_fps,
    double duration_s);

}  // namespace agentic_et1_tracker
