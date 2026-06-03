#include "agentic_et1_tracker/trk/synthetic_transition.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointDim = TrkSchema::kJointDim;
constexpr std::size_t kBodyCount = TrkSchema::kBodyCount;
constexpr std::size_t kBodyPosFrameSize = kBodyCount * 3;
constexpr std::size_t kBodyQuatFrameSize = kBodyCount * 4;
constexpr std::size_t kFloatElementsPerFrame =
    kJointDim + kJointDim + kBodyPosFrameSize + kBodyQuatFrameSize + kBodyPosFrameSize +
    kBodyPosFrameSize + 3 + 3;
constexpr std::size_t kContactElementsPerFrame = 2;

bool finite(double value) {
  return std::isfinite(value);
}

bool finite(float value) {
  return std::isfinite(value);
}

bool validFloatView(TrkArrayView<float> view, std::size_t expected_size) {
  if (view.ptr == nullptr || view.size != expected_size) {
    return false;
  }
  for (std::size_t i = 0; i < view.size; ++i) {
    if (!finite(view.ptr[i])) {
      return false;
    }
  }
  return true;
}

bool validContactValue(std::int64_t value) {
  return value == 0 || value == 1 || value == 2;
}

bool validContactView(TrkArrayView<std::int64_t> view) {
  return view.ptr != nullptr && view.size == 1 && validContactValue(view.ptr[0]);
}

float quatNorm(const float* q) {
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    norm_sq += static_cast<double>(q[i]) * static_cast<double>(q[i]);
  }
  return static_cast<float>(std::sqrt(norm_sq));
}

bool validQuaternions(TrkArrayView<float> view) {
  if (!validFloatView(view, kBodyQuatFrameSize)) {
    return false;
  }
  for (std::size_t body = 0; body < kBodyCount; ++body) {
    if (quatNorm(view.ptr + body * 4) <= std::numeric_limits<float>::epsilon()) {
      return false;
    }
  }
  return true;
}

bool validFrame(const TrkFrameView& frame) {
  return validFloatView(frame.joint_pos, kJointDim) &&
         validFloatView(frame.joint_vel, kJointDim) &&
         validFloatView(frame.body_pos_w, kBodyPosFrameSize) &&
         validQuaternions(frame.body_quat_w) &&
         validFloatView(frame.body_lin_vel_w, kBodyPosFrameSize) &&
         validFloatView(frame.body_ang_vel_w, kBodyPosFrameSize) &&
         validContactView(frame.left_foot_contact_state) &&
         validContactView(frame.right_foot_contact_state) &&
         validFloatView(frame.ref_com_rel_navi, 3) &&
         validFloatView(frame.ref_com_vel_navi, 3);
}

std::optional<std::size_t> frameCount(double target_fps, double duration_s) {
  if (!finite(target_fps) || !finite(duration_s) || target_fps <= 0.0 || duration_s <= 0.0) {
    return std::nullopt;
  }
  const double intervals_d = duration_s * target_fps;
  constexpr std::uint64_t bytes_per_frame =
      static_cast<std::uint64_t>(kFloatElementsPerFrame) * sizeof(float) +
      static_cast<std::uint64_t>(kContactElementsPerFrame) * sizeof(std::int64_t);
  constexpr std::uint64_t max_frames =
      TrkSchema::kDefaultLimits.max_total_payload_bytes / bytes_per_frame;
  if (!finite(intervals_d) || intervals_d > static_cast<double>(max_frames - 1)) {
    return std::nullopt;
  }
  const std::size_t intervals =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(intervals_d - 1.0e-12)));
  const std::size_t frames = intervals + 1;
  if (static_cast<std::uint64_t>(frames) > max_frames) {
    return std::nullopt;
  }
  return frames;
}

void resizeFloatArray(TrkFloatArray& array,
                      std::size_t frames,
                      std::vector<std::uint64_t> shape,
                      std::size_t frame_size) {
  array.shape = std::move(shape);
  array.frame_size = frame_size;
  array.values.assign(frames * frame_size, 0.0F);
}

void resizeContactArray(TrkContactArray& array,
                        std::size_t frames,
                        std::vector<std::uint64_t> shape,
                        std::size_t frame_size) {
  array.shape = std::move(shape);
  array.frame_size = frame_size;
  array.values.assign(frames * frame_size, 0);
}

TrkTrack makeEmptyTrack(std::size_t frames, double target_fps) {
  TrkTrack track;
  track.metadata.frames = frames;
  track.metadata.duration_s = static_cast<double>(frames - 1) / target_fps;
  track.metadata.fps = target_fps;
  track.metadata.version = TrkSchema::kVersion;
  track.metadata.array_count = TrkSchema::kRequiredArrays.size();

  resizeFloatArray(track.joint_pos, frames, {frames, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.joint_vel, frames, {frames, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.body_pos_w, frames, {frames, TrkSchema::kBodyCount, 3},
                   kBodyPosFrameSize);
  resizeFloatArray(track.body_quat_w, frames, {frames, TrkSchema::kBodyCount, 4},
                   kBodyQuatFrameSize);
  resizeFloatArray(track.body_lin_vel_w, frames, {frames, TrkSchema::kBodyCount, 3},
                   kBodyPosFrameSize);
  resizeFloatArray(track.body_ang_vel_w, frames, {frames, TrkSchema::kBodyCount, 3},
                   kBodyPosFrameSize);
  resizeContactArray(track.left_foot_contact_state, frames, {frames}, 1);
  resizeContactArray(track.right_foot_contact_state, frames, {frames}, 1);
  resizeFloatArray(track.ref_com_rel_navi, frames, {frames, 3}, 3);
  resizeFloatArray(track.ref_com_vel_navi, frames, {frames, 3}, 3);
  return track;
}

float lerp(float source, float target, double t) {
  return static_cast<float>(static_cast<double>(source) +
                            (static_cast<double>(target) - static_cast<double>(source)) * t);
}

void interpolateLinear(const TrkArrayView<float>& source,
                       const TrkArrayView<float>& target,
                       std::size_t frame,
                       double t,
                       TrkFloatArray& out) {
  const std::size_t offset = frame * out.frame_size;
  for (std::size_t i = 0; i < out.frame_size; ++i) {
    out.values[offset + i] = lerp(source.ptr[i], target.ptr[i], t);
  }
}

std::array<float, 4> normalizedQuat(const float* q) {
  const float norm = quatNorm(q);
  return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

std::array<float, 4> nlerpQuat(const float* source, const float* target, double t) {
  const std::array<float, 4> source_q = normalizedQuat(source);
  std::array<float, 4> target_q = normalizedQuat(target);
  double dot = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    dot += static_cast<double>(source_q[i]) * static_cast<double>(target_q[i]);
  }
  if (dot < 0.0) {
    for (float& value : target_q) {
      value = -value;
    }
  }

  std::array<float, 4> out{};
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double value = static_cast<double>(source_q[i]) * (1.0 - t) +
                         static_cast<double>(target_q[i]) * t;
    out[i] = static_cast<float>(value);
    norm_sq += value * value;
  }
  const double norm = std::sqrt(norm_sq);
  if (norm <= static_cast<double>(std::numeric_limits<float>::epsilon())) {
    return {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F};
  }
  for (float& value : out) {
    value = static_cast<float>(static_cast<double>(value) / norm);
  }
  return out;
}

void interpolateQuats(const TrkArrayView<float>& source,
                      const TrkArrayView<float>& target,
                      std::size_t frame,
                      double t,
                      TrkFloatArray& out) {
  const std::size_t offset = frame * out.frame_size;
  for (std::size_t body = 0; body < kBodyCount; ++body) {
    const std::array<float, 4> quat =
        nlerpQuat(source.ptr + body * 4, target.ptr + body * 4, t);
    for (std::size_t component = 0; component < quat.size(); ++component) {
      out.values[offset + body * 4 + component] = quat[component];
    }
  }
}

void setContact(const TrkArrayView<std::int64_t>& source,
                const TrkArrayView<std::int64_t>& target,
                std::size_t frame,
                double t,
                TrkContactArray& out) {
  out.values[frame] = t < 0.5 ? source.ptr[0] : target.ptr[0];
}

void finiteDifference(const TrkFloatArray& positions, double fps, TrkFloatArray& velocities) {
  const std::size_t frames = static_cast<std::size_t>(positions.shape.front());
  for (std::size_t frame = 0; frame < frames; ++frame) {
    const std::size_t prev = frame == 0 ? frame : frame - 1;
    const std::size_t next = frame + 1 < frames ? frame + 1 : frame;
    const double scale = frame > 0 && frame + 1 < frames ? 0.5 * fps : fps;
    for (std::size_t i = 0; i < positions.frame_size; ++i) {
      const float before = positions.values[prev * positions.frame_size + i];
      const float after = positions.values[next * positions.frame_size + i];
      velocities.values[frame * velocities.frame_size + i] =
          static_cast<float>((static_cast<double>(after) - static_cast<double>(before)) * scale);
    }
  }
}

bool allFinite(const TrkFloatArray& array) {
  for (const float value : array.values) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

bool allFinite(const TrkTrack& track) {
  return allFinite(track.joint_pos) && allFinite(track.joint_vel) &&
         allFinite(track.body_pos_w) && allFinite(track.body_quat_w) &&
         allFinite(track.body_lin_vel_w) && allFinite(track.body_ang_vel_w) &&
         allFinite(track.ref_com_rel_navi) && allFinite(track.ref_com_vel_navi);
}

}  // namespace

std::optional<TrkTrack> makeSyntheticTransitionTrk(const TrkFrameView& source,
                                                   const TrkFrameView& target,
                                                   double target_fps,
                                                   double duration_s) {
  if (!validFrame(source) || !validFrame(target)) {
    return std::nullopt;
  }

  const std::optional<std::size_t> maybe_frames = frameCount(target_fps, duration_s);
  if (!maybe_frames.has_value()) {
    return std::nullopt;
  }

  TrkTrack track = makeEmptyTrack(*maybe_frames, target_fps);
  const double intervals = static_cast<double>(track.metadata.frames - 1);
  for (std::size_t frame = 0; frame < track.metadata.frames; ++frame) {
    const double t = static_cast<double>(frame) / intervals;
    interpolateLinear(source.joint_pos, target.joint_pos, frame, t, track.joint_pos);
    interpolateLinear(source.body_pos_w, target.body_pos_w, frame, t, track.body_pos_w);
    interpolateLinear(source.ref_com_rel_navi, target.ref_com_rel_navi, frame, t,
                      track.ref_com_rel_navi);
    interpolateQuats(source.body_quat_w, target.body_quat_w, frame, t, track.body_quat_w);
    setContact(source.left_foot_contact_state, target.left_foot_contact_state, frame, t,
               track.left_foot_contact_state);
    setContact(source.right_foot_contact_state, target.right_foot_contact_state, frame, t,
               track.right_foot_contact_state);
  }

  finiteDifference(track.joint_pos, target_fps, track.joint_vel);
  finiteDifference(track.body_pos_w, target_fps, track.body_lin_vel_w);
  finiteDifference(track.ref_com_rel_navi, target_fps, track.ref_com_vel_navi);
  std::fill(track.body_ang_vel_w.values.begin(), track.body_ang_vel_w.values.end(), 0.0F);

  if (!allFinite(track)) {
    return std::nullopt;
  }
  return track;
}

}  // namespace agentic_et1_tracker
