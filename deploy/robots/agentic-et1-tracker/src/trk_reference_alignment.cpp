#include "agentic_et1_tracker/trk/reference_alignment.hpp"

#include <array>
#include <cmath>
#include <cstddef>

#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kRootBodyIndex = 0;
constexpr std::size_t kBodyPositionDimensions = 3;
constexpr std::size_t kPlanarPositionDimensions = 2;
constexpr std::size_t kBodyPosFrameSize =
    TrkSchema::kBodyCount * kBodyPositionDimensions;

bool validBodyPositions(const TrkArrayView<float>& view) {
  if (view.ptr == nullptr || view.size < kBodyPosFrameSize) {
    return false;
  }
  for (std::size_t i = 0; i < kPlanarPositionDimensions; ++i) {
    if (!std::isfinite(view.ptr[kRootBodyIndex * kBodyPositionDimensions + i])) {
      return false;
    }
  }
  return true;
}

bool validTargetBodyPositions(const TrkTrack& target) {
  return target.metadata.frames > 0 &&
         target.body_pos_w.frame_size == kBodyPosFrameSize &&
         target.body_pos_w.values.size() ==
             target.metadata.frames * target.body_pos_w.frame_size;
}

}  // namespace

std::optional<TrkTrack> alignTrackRootTranslation(const TrkTrack& target,
                                                  const TrkFrameView& source) {
  const std::optional<TrkFrameView> target_first_frame = target.frame(0);
  if (!target_first_frame || !validBodyPositions(source.body_pos_w) ||
      !validBodyPositions(target_first_frame->body_pos_w) ||
      !validTargetBodyPositions(target)) {
    return std::nullopt;
  }

  std::array<float, kPlanarPositionDimensions> delta{};
  for (std::size_t i = 0; i < delta.size(); ++i) {
    delta[i] = source.body_pos_w.ptr[kRootBodyIndex * kBodyPositionDimensions + i] -
               target_first_frame->body_pos_w.ptr[kRootBodyIndex * kBodyPositionDimensions + i];
    if (!std::isfinite(delta[i])) {
      return std::nullopt;
    }
  }

  TrkTrack aligned = target;
  for (std::size_t frame = 0; frame < aligned.metadata.frames; ++frame) {
    const std::size_t frame_offset = frame * aligned.body_pos_w.frame_size;
    for (std::size_t body = 0; body < TrkSchema::kBodyCount; ++body) {
      const std::size_t body_offset = frame_offset + body * kBodyPositionDimensions;
      for (std::size_t i = 0; i < delta.size(); ++i) {
        aligned.body_pos_w.values.at(body_offset + i) += delta[i];
      }
    }
  }
  return aligned;
}

}  // namespace agentic_et1_tracker
