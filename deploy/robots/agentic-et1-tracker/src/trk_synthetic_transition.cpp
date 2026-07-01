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

#include <ruckig/ruckig.hpp>

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
constexpr double kRuckigDurationEpsilon = 1.0e-9;

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

constexpr std::uint64_t bytesPerFrame() {
  return static_cast<std::uint64_t>(kFloatElementsPerFrame) * sizeof(float) +
         static_cast<std::uint64_t>(kContactElementsPerFrame) * sizeof(std::int64_t);
}

constexpr std::uint64_t maxPayloadFrames() {
  return TrkSchema::kDefaultLimits.max_total_payload_bytes / bytesPerFrame();
}

bool validTimingCap(double target_fps, double duration_s) {
  if (!finite(target_fps) || !finite(duration_s) || target_fps <= 0.0 || duration_s <= 0.0) {
    return false;
  }
  const double intervals_d = duration_s * target_fps;
  constexpr std::uint64_t max_frames = maxPayloadFrames();
  if (!finite(intervals_d) || intervals_d > static_cast<double>(max_frames - 1)) {
    return false;
  }
  return true;
}

std::optional<std::size_t> quantizedFrameCount(double target_fps,
                                               double duration_s,
                                               std::size_t min_frames) {
  const double intervals_d = duration_s * target_fps;
  constexpr std::uint64_t max_frames = maxPayloadFrames();
  const std::size_t min_intervals =
      min_frames > 0 ? std::max<std::size_t>(1, min_frames - 1) : 1;
  const std::size_t intervals =
      std::max<std::size_t>(min_intervals,
                            static_cast<std::size_t>(std::ceil(intervals_d - 1.0e-12)));
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

bool sameContacts(const TrkFrameView& source, const TrkFrameView& target) {
  const std::int64_t source_left = source.left_foot_contact_state.ptr[0];
  const std::int64_t source_right = source.right_foot_contact_state.ptr[0];
  return source_left != 0 && source_right != 0 &&
         source_left == target.left_foot_contact_state.ptr[0] &&
         source_right == target.right_foot_contact_state.ptr[0];
}

void setContact(const TrkArrayView<std::int64_t>& source,
                std::size_t frame,
                TrkContactArray& out) {
  out.values[frame] = source.ptr[0];
}

void interpolateFloatViewEndpointPreserving(const TrkArrayView<float>& source,
                                            const TrkArrayView<float>& target,
                                            std::size_t frame,
                                            double t,
                                            TrkFloatArray& out) {
  const std::size_t offset = frame * out.frame_size;
  for (std::size_t i = 0; i < out.frame_size; ++i) {
    const double value = static_cast<double>(source.ptr[i]) * (1.0 - t) +
                         static_cast<double>(target.ptr[i]) * t;
    out.values[offset + i] = static_cast<float>(value);
  }
}

std::optional<ruckig::Trajectory<ruckig::DynamicDOFs>> makeJointTrajectory(
    const TrkArrayView<float>& source_position,
    const TrkArrayView<float>& source_velocity,
    const TrkArrayView<float>& target_position,
    const TrkArrayView<float>& target_velocity,
    double dt) {
  const std::size_t dofs = source_position.size;
  if (dofs != kJointDim || source_velocity.size != dofs || target_position.size != dofs ||
      target_velocity.size != dofs) {
    return std::nullopt;
  }

  ruckig::Ruckig<ruckig::DynamicDOFs> ruckig(dofs, dt);
  ruckig::InputParameter<ruckig::DynamicDOFs> input(dofs);
  ruckig::Trajectory<ruckig::DynamicDOFs> trajectory(dofs);
  const SyntheticTransitionLimits limits = defaultSyntheticTransitionLimits();

  for (std::size_t i = 0; i < dofs; ++i) {
    input.current_position[i] = static_cast<double>(source_position.ptr[i]);
    input.current_velocity[i] = static_cast<double>(source_velocity.ptr[i]);
    input.current_acceleration[i] = 0.0;
    input.target_position[i] = static_cast<double>(target_position.ptr[i]);
    input.target_velocity[i] = static_cast<double>(target_velocity.ptr[i]);
    input.target_acceleration[i] = 0.0;
    input.max_velocity[i] = limits.max_velocity;
    input.max_acceleration[i] = limits.max_acceleration;
    input.max_jerk[i] = limits.max_jerk;
  }

  if (!ruckig.validate_input<false>(input, true, true)) {
    return std::nullopt;
  }

  const ruckig::Result result = ruckig.calculate(input, trajectory);
  if (result != ruckig::Result::Working) {
    return std::nullopt;
  }

  const double trajectory_duration = trajectory.get_duration();
  if (!finite(trajectory_duration) || trajectory_duration <= 0.0) {
    return std::nullopt;
  }
  return trajectory;
}

bool sampleRuckigTrajectory(const ruckig::Trajectory<ruckig::DynamicDOFs>& trajectory,
                            std::size_t frames,
                            double dt,
                            TrkFloatArray& out_position,
                            TrkFloatArray& out_velocity) {
  const std::size_t dofs = out_position.frame_size;
  if (dofs == 0 || out_velocity.frame_size != dofs ||
      out_position.values.size() != frames * dofs || out_velocity.values.size() != frames * dofs) {
    return false;
  }

  const double trajectory_duration = trajectory.get_duration();
  std::vector<double> position(dofs, 0.0);
  std::vector<double> velocity(dofs, 0.0);
  std::vector<double> acceleration(dofs, 0.0);
  for (std::size_t frame = 0; frame < frames; ++frame) {
    const double sample_time = std::min(static_cast<double>(frame) * dt, trajectory_duration);
    trajectory.at_time(sample_time, position, velocity, acceleration);
    for (std::size_t i = 0; i < dofs; ++i) {
      if (!finite(position[i]) || !finite(velocity[i])) {
        return false;
      }
      out_position.values[frame * dofs + i] = static_cast<float>(position[i]);
      out_velocity.values[frame * dofs + i] = static_cast<float>(velocity[i]);
    }
  }

  return true;
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
                                                   SyntheticTransitionOptions options) {
  if (!validFrame(source) || !validFrame(target)) {
    return std::nullopt;
  }

  if (options.root_yaw_mode != SyntheticTransitionRootYawMode::kDisabled ||
      options.min_frames == 0 ||
      !finite(options.duration_dt_tolerance_s) ||
      options.duration_dt_tolerance_s < 0.0 ||
      !validTimingCap(target_fps, options.max_duration_s) || !sameContacts(source, target)) {
    return std::nullopt;
  }

  const double dt = 1.0 / target_fps;
  const std::optional<ruckig::Trajectory<ruckig::DynamicDOFs>> joint_trajectory =
      makeJointTrajectory(source.joint_pos, source.joint_vel, target.joint_pos, target.joint_vel,
                          dt);
  if (!joint_trajectory.has_value()) {
    return std::nullopt;
  }

  const double feasible_duration = joint_trajectory->get_duration();
  const std::optional<std::size_t> maybe_frames =
      quantizedFrameCount(target_fps, feasible_duration, options.min_frames);
  if (!maybe_frames.has_value()) {
    return std::nullopt;
  }
  const double actual_duration = static_cast<double>(*maybe_frames - 1) * dt;
  if (!finite(actual_duration) ||
      actual_duration > options.max_duration_s + options.duration_dt_tolerance_s ||
      actual_duration + kRuckigDurationEpsilon < feasible_duration) {
    return std::nullopt;
  }

  TrkTrack track = makeEmptyTrack(*maybe_frames, target_fps);
  if (!sampleRuckigTrajectory(*joint_trajectory, track.metadata.frames, dt, track.joint_pos,
                              track.joint_vel)) {
    return std::nullopt;
  }

  const double intervals = static_cast<double>(track.metadata.frames - 1);
  for (std::size_t frame = 0; frame < track.metadata.frames; ++frame) {
    const double t = static_cast<double>(frame) / intervals;
    interpolateFloatViewEndpointPreserving(source.body_pos_w,
                                           target.body_pos_w,
                                           frame,
                                           t,
                                           track.body_pos_w);
    interpolateFloatViewEndpointPreserving(source.body_lin_vel_w,
                                           target.body_lin_vel_w,
                                           frame,
                                           t,
                                           track.body_lin_vel_w);
    interpolateFloatViewEndpointPreserving(source.ref_com_rel_navi,
                                           target.ref_com_rel_navi,
                                           frame,
                                           t,
                                           track.ref_com_rel_navi);
    interpolateFloatViewEndpointPreserving(source.ref_com_vel_navi,
                                           target.ref_com_vel_navi,
                                           frame,
                                           t,
                                           track.ref_com_vel_navi);
    interpolateQuats(source.body_quat_w, target.body_quat_w, frame, t, track.body_quat_w);
    // This preserves angular velocity endpoints only; it does not enforce consistency
    // with the interpolated quaternion derivative.
    interpolateFloatViewEndpointPreserving(source.body_ang_vel_w,
                                           target.body_ang_vel_w,
                                           frame,
                                           t,
                                           track.body_ang_vel_w);
    setContact(source.left_foot_contact_state, frame, track.left_foot_contact_state);
    setContact(source.right_foot_contact_state, frame, track.right_foot_contact_state);
  }

  if (!allFinite(track)) {
    return std::nullopt;
  }
  return track;
}

std::optional<TrkTrack> makeSyntheticTransitionTrk(const TrkFrameView& source,
                                                   const TrkFrameView& target,
                                                   double target_fps,
                                                   double duration_s) {
  return makeSyntheticTransitionTrk(source,
                                    target,
                                    target_fps,
                                    SyntheticTransitionOptions{duration_s});
}

}  // namespace agentic_et1_tracker
