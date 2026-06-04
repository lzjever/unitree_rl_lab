#include "agentic_et1_tracker/trk/reference_alignment.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kRootBodyIndex = 0;
constexpr std::size_t kBodyPositionDimensions = 3;
constexpr std::size_t kBodyQuatDimensions = 4;
constexpr std::size_t kPlanarPositionDimensions = 2;
constexpr std::size_t kBodyPosFrameSize =
    TrkSchema::kBodyCount * kBodyPositionDimensions;
constexpr std::size_t kBodyQuatFrameSize =
    TrkSchema::kBodyCount * kBodyQuatDimensions;

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

bool validRootQuaternion(const TrkArrayView<float>& view) {
  if (view.ptr == nullptr || view.size < kBodyQuatFrameSize) {
    return false;
  }
  const float* q = view.ptr + kRootBodyIndex * kBodyQuatDimensions;
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < kBodyQuatDimensions; ++i) {
    if (!std::isfinite(q[i])) {
      return false;
    }
    norm_sq += static_cast<double>(q[i]) * static_cast<double>(q[i]);
  }
  return norm_sq > static_cast<double>(std::numeric_limits<float>::epsilon()) *
                       static_cast<double>(std::numeric_limits<float>::epsilon());
}

bool validTargetFloatArray(const TrkFloatArray& array,
                           std::size_t frames,
                           std::size_t frame_size) {
  return frames > 0 && array.frame_size == frame_size &&
         array.values.size() == frames * frame_size;
}

bool validTargetBodyQuaternions(const TrkTrack& target) {
  if (!validTargetFloatArray(target.body_quat_w,
                             target.metadata.frames,
                             kBodyQuatFrameSize)) {
    return false;
  }
  for (std::size_t offset = 0; offset < target.body_quat_w.values.size();
       offset += kBodyQuatDimensions) {
    double norm_sq = 0.0;
    for (std::size_t i = 0; i < kBodyQuatDimensions; ++i) {
      const float value = target.body_quat_w.values.at(offset + i);
      if (!std::isfinite(value)) {
        return false;
      }
      norm_sq += static_cast<double>(value) * static_cast<double>(value);
    }
    if (norm_sq <= static_cast<double>(std::numeric_limits<float>::epsilon()) *
                       static_cast<double>(std::numeric_limits<float>::epsilon())) {
      return false;
    }
  }
  return true;
}

std::array<double, kBodyQuatDimensions> normalizedQuat(const float* q) {
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < kBodyQuatDimensions; ++i) {
    norm_sq += static_cast<double>(q[i]) * static_cast<double>(q[i]);
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  return {static_cast<double>(q[0]) * inv_norm,
          static_cast<double>(q[1]) * inv_norm,
          static_cast<double>(q[2]) * inv_norm,
          static_cast<double>(q[3]) * inv_norm};
}

double yawFromQuat(const float* q) {
  const std::array<double, kBodyQuatDimensions> nq = normalizedQuat(q);
  const double w = nq[0];
  const double x = nq[1];
  const double y = nq[2];
  const double z = nq[3];
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

std::array<double, kBodyQuatDimensions> multiplyQuat(
    const std::array<double, kBodyQuatDimensions>& lhs,
    const float* rhs) {
  const std::array<double, kBodyQuatDimensions> r = normalizedQuat(rhs);
  return {lhs[0] * r[0] - lhs[1] * r[1] - lhs[2] * r[2] - lhs[3] * r[3],
          lhs[0] * r[1] + lhs[1] * r[0] + lhs[2] * r[3] - lhs[3] * r[2],
          lhs[0] * r[2] - lhs[1] * r[3] + lhs[2] * r[0] + lhs[3] * r[1],
          lhs[0] * r[3] + lhs[1] * r[2] - lhs[2] * r[1] + lhs[3] * r[0]};
}

std::array<float, kBodyQuatDimensions> normalizeQuat(
    const std::array<double, kBodyQuatDimensions>& q) {
  double norm_sq = 0.0;
  for (const double value : q) {
    norm_sq += value * value;
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  return {static_cast<float>(q[0] * inv_norm),
          static_cast<float>(q[1] * inv_norm),
          static_cast<float>(q[2] * inv_norm),
          static_cast<float>(q[3] * inv_norm)};
}

void rotatePlanar(float x,
                  float y,
                  double cos_yaw,
                  double sin_yaw,
                  float& out_x,
                  float& out_y) {
  out_x = static_cast<float>(cos_yaw * static_cast<double>(x) -
                             sin_yaw * static_cast<double>(y));
  out_y = static_cast<float>(sin_yaw * static_cast<double>(x) +
                             cos_yaw * static_cast<double>(y));
}

}  // namespace

std::optional<TrkTrack> alignTrackRootPlanarPose(const TrkTrack& target,
                                                 const TrkFrameView& source) {
  const std::optional<TrkFrameView> target_first_frame = target.frame(0);
  if (!target_first_frame || !validBodyPositions(source.body_pos_w) ||
      !validRootQuaternion(source.body_quat_w) ||
      !validBodyPositions(target_first_frame->body_pos_w) ||
      !validRootQuaternion(target_first_frame->body_quat_w) ||
      !validTargetFloatArray(target.body_pos_w,
                             target.metadata.frames,
                             kBodyPosFrameSize) ||
      !validTargetBodyQuaternions(target) ||
      !validTargetFloatArray(target.body_lin_vel_w,
                             target.metadata.frames,
                             kBodyPosFrameSize) ||
      !validTargetFloatArray(target.body_ang_vel_w,
                             target.metadata.frames,
                             kBodyPosFrameSize)) {
    return std::nullopt;
  }

  const float source_root_x =
      source.body_pos_w.ptr[kRootBodyIndex * kBodyPositionDimensions + 0];
  const float source_root_y =
      source.body_pos_w.ptr[kRootBodyIndex * kBodyPositionDimensions + 1];
  const float target_root_x =
      target_first_frame->body_pos_w.ptr[kRootBodyIndex * kBodyPositionDimensions + 0];
  const float target_root_y =
      target_first_frame->body_pos_w.ptr[kRootBodyIndex * kBodyPositionDimensions + 1];
  const double source_yaw =
      yawFromQuat(source.body_quat_w.ptr + kRootBodyIndex * kBodyQuatDimensions);
  const double target_yaw =
      yawFromQuat(target_first_frame->body_quat_w.ptr +
                  kRootBodyIndex * kBodyQuatDimensions);
  if (!std::isfinite(source_yaw) || !std::isfinite(target_yaw)) {
    return std::nullopt;
  }

  const double yaw_delta = source_yaw - target_yaw;
  const double cos_yaw = std::cos(yaw_delta);
  const double sin_yaw = std::sin(yaw_delta);
  const std::array<double, kBodyQuatDimensions> yaw_delta_quat = {
      std::cos(0.5 * yaw_delta), 0.0, 0.0, std::sin(0.5 * yaw_delta)};

  TrkTrack aligned = target;
  for (std::size_t frame = 0; frame < aligned.metadata.frames; ++frame) {
    const std::size_t pos_frame_offset = frame * aligned.body_pos_w.frame_size;
    const std::size_t quat_frame_offset = frame * aligned.body_quat_w.frame_size;
    for (std::size_t body = 0; body < TrkSchema::kBodyCount; ++body) {
      const std::size_t body_pos_offset =
          pos_frame_offset + body * kBodyPositionDimensions;
      float rotated_x = 0.0F;
      float rotated_y = 0.0F;
      rotatePlanar(target.body_pos_w.values.at(body_pos_offset + 0) - target_root_x,
                   target.body_pos_w.values.at(body_pos_offset + 1) - target_root_y,
                   cos_yaw,
                   sin_yaw,
                   rotated_x,
                   rotated_y);
      aligned.body_pos_w.values.at(body_pos_offset + 0) = source_root_x + rotated_x;
      aligned.body_pos_w.values.at(body_pos_offset + 1) = source_root_y + rotated_y;

      const std::size_t body_quat_offset =
          quat_frame_offset + body * kBodyQuatDimensions;
      const std::array<float, kBodyQuatDimensions> rotated_quat =
          normalizeQuat(multiplyQuat(yaw_delta_quat,
                                     target.body_quat_w.values.data() +
                                         body_quat_offset));
      for (std::size_t i = 0; i < rotated_quat.size(); ++i) {
        aligned.body_quat_w.values.at(body_quat_offset + i) = rotated_quat[i];
      }
    }
  }

  for (TrkFloatArray* velocities :
       {&aligned.body_lin_vel_w, &aligned.body_ang_vel_w}) {
    for (std::size_t frame = 0; frame < aligned.metadata.frames; ++frame) {
      const std::size_t frame_offset = frame * velocities->frame_size;
      for (std::size_t body = 0; body < TrkSchema::kBodyCount; ++body) {
        const std::size_t body_offset =
            frame_offset + body * kBodyPositionDimensions;
        rotatePlanar(velocities->values.at(body_offset + 0),
                     velocities->values.at(body_offset + 1),
                     cos_yaw,
                     sin_yaw,
                     velocities->values.at(body_offset + 0),
                     velocities->values.at(body_offset + 1));
      }
    }
  }
  return aligned;
}

}  // namespace agentic_et1_tracker
