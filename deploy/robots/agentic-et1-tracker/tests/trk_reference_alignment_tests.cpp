#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/trk/reference_alignment.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointDim = TrkSchema::kJointDim;
constexpr std::size_t kBodyCount = TrkSchema::kBodyCount;

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

TrkTrack makeTrack(std::size_t frames) {
  TrkTrack track;
  track.metadata.frames = frames;
  track.metadata.fps = TrkSchema::kDefaultFps;
  track.metadata.duration_s =
      frames > 0 ? static_cast<double>(frames - 1) / track.metadata.fps : 0.0;

  resizeFloatArray(track.joint_pos, frames, {frames, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.joint_vel, frames, {frames, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.body_pos_w, frames, {frames, TrkSchema::kBodyCount, 3},
                   kBodyCount * 3);
  resizeFloatArray(track.body_quat_w, frames, {frames, TrkSchema::kBodyCount, 4},
                   kBodyCount * 4);
  resizeFloatArray(track.body_lin_vel_w, frames, {frames, TrkSchema::kBodyCount, 3},
                   kBodyCount * 3);
  resizeFloatArray(track.body_ang_vel_w, frames, {frames, TrkSchema::kBodyCount, 3},
                   kBodyCount * 3);
  resizeContactArray(track.left_foot_contact_state, frames, {frames}, 1);
  resizeContactArray(track.right_foot_contact_state, frames, {frames}, 1);
  resizeFloatArray(track.ref_com_rel_navi, frames, {frames, 3}, 3);
  resizeFloatArray(track.ref_com_vel_navi, frames, {frames, 3}, 3);

  for (std::size_t frame = 0; frame < frames; ++frame) {
    for (std::size_t joint = 0; joint < kJointDim; ++joint) {
      track.joint_pos.values.at(frame * track.joint_pos.frame_size + joint) =
          static_cast<float>(frame * 100 + joint);
      track.joint_vel.values.at(frame * track.joint_vel.frame_size + joint) =
          static_cast<float>(frame * 200 + joint);
    }
    for (std::size_t body = 0; body < kBodyCount; ++body) {
      const std::size_t pos = frame * track.body_pos_w.frame_size + body * 3;
      track.body_pos_w.values.at(pos + 0) =
          static_cast<float>(frame * 100 + body * 10 + 1);
      track.body_pos_w.values.at(pos + 1) =
          static_cast<float>(frame * 100 + body * 10 + 2);
      track.body_pos_w.values.at(pos + 2) =
          static_cast<float>(frame * 100 + body * 10 + 3);
      track.body_lin_vel_w.values.at(pos + 0) =
          static_cast<float>(frame * 1000 + body * 10 + 4);
      track.body_lin_vel_w.values.at(pos + 1) =
          static_cast<float>(frame * 1000 + body * 10 + 5);
      track.body_lin_vel_w.values.at(pos + 2) =
          static_cast<float>(frame * 1000 + body * 10 + 6);
      track.body_ang_vel_w.values.at(pos + 0) =
          static_cast<float>(frame * 2000 + body * 10 + 7);
      track.body_ang_vel_w.values.at(pos + 1) =
          static_cast<float>(frame * 2000 + body * 10 + 8);
      track.body_ang_vel_w.values.at(pos + 2) =
          static_cast<float>(frame * 2000 + body * 10 + 9);
    }
    for (std::size_t body = 0; body < kBodyCount; ++body) {
      track.body_quat_w.values.at(frame * track.body_quat_w.frame_size + body * 4) =
          1.0F;
    }
    track.left_foot_contact_state.values.at(frame) = static_cast<std::int64_t>(frame % 3);
    track.right_foot_contact_state.values.at(frame) =
        static_cast<std::int64_t>((frame + 1) % 3);
    for (std::size_t i = 0; i < 3; ++i) {
      track.ref_com_rel_navi.values.at(frame * 3 + i) =
          static_cast<float>(frame * 10 + i);
      track.ref_com_vel_navi.values.at(frame * 3 + i) =
          static_cast<float>(frame * 20 + i);
    }
  }
  return track;
}

float atBodyPos(const TrkTrack& track,
                std::size_t frame,
                std::size_t body,
                std::size_t component) {
  return track.body_pos_w.values.at(frame * track.body_pos_w.frame_size +
                                    body * 3 + component);
}

float atBodyVector(const TrkFloatArray& array,
                   std::size_t frame,
                   std::size_t body,
                   std::size_t component) {
  return array.values.at(frame * array.frame_size + body * 3 + component);
}

std::array<float, 4> yawQuat(float yaw_rad) {
  return {std::cos(0.5F * yaw_rad), 0.0F, 0.0F, std::sin(0.5F * yaw_rad)};
}

void setBodyQuat(TrkTrack& track,
                 std::size_t frame,
                 std::size_t body,
                 std::array<float, 4> quat) {
  const std::size_t offset = frame * track.body_quat_w.frame_size + body * 4;
  for (std::size_t i = 0; i < quat.size(); ++i) {
    track.body_quat_w.values.at(offset + i) = quat.at(i);
  }
}

float yawFromQuat(const TrkTrack& track, std::size_t frame, std::size_t body) {
  const std::size_t offset = frame * track.body_quat_w.frame_size + body * 4;
  const float w = track.body_quat_w.values.at(offset + 0);
  const float x = track.body_quat_w.values.at(offset + 1);
  const float y = track.body_quat_w.values.at(offset + 2);
  const float z = track.body_quat_w.values.at(offset + 3);
  return std::atan2(2.0F * (w * z + x * y),
                    1.0F - 2.0F * (y * y + z * z));
}

void requireAllZUnchanged(const TrkTrack& aligned, const TrkTrack& target) {
  for (std::size_t frame = 0; frame < target.metadata.frames; ++frame) {
    for (std::size_t body = 0; body < kBodyCount; ++body) {
      REQUIRE(atBodyPos(aligned, frame, body, 2) ==
              Catch::Approx(atBodyPos(target, frame, body, 2)));
    }
  }
}

void requireNonPositionArraysUnchanged(const TrkTrack& aligned,
                                       const TrkTrack& target) {
  REQUIRE(aligned.joint_pos.values == target.joint_pos.values);
  REQUIRE(aligned.joint_vel.values == target.joint_vel.values);
  REQUIRE(aligned.left_foot_contact_state.values ==
          target.left_foot_contact_state.values);
  REQUIRE(aligned.right_foot_contact_state.values ==
          target.right_foot_contact_state.values);
  REQUIRE(aligned.ref_com_rel_navi.values == target.ref_com_rel_navi.values);
  REQUIRE(aligned.ref_com_vel_navi.values == target.ref_com_vel_navi.values);
}

}  // namespace

TEST_CASE("TRK reference alignment maps target frame0 root XY yaw to source") {
  constexpr float kHalfPi = 1.57079632679F;
  TrkTrack source = makeTrack(1);
  source.body_pos_w.values.at(0) = 5.0F;
  source.body_pos_w.values.at(1) = -2.0F;
  source.body_pos_w.values.at(2) = 0.75F;
  setBodyQuat(source, 0, 0, yawQuat(kHalfPi));

  TrkTrack target = makeTrack(2);
  target.body_pos_w.values.at(0) = 100.0F;
  target.body_pos_w.values.at(1) = -20.0F;
  target.body_pos_w.values.at(2) = 1.25F;
  setBodyQuat(target, 0, 0, yawQuat(0.0F));
  const std::optional<TrkTrack> aligned =
      alignTrackRootPlanarPose(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  REQUIRE(atBodyPos(*aligned, 0, 0, 0) == Catch::Approx(5.0F));
  REQUIRE(atBodyPos(*aligned, 0, 0, 1) == Catch::Approx(-2.0F));
  REQUIRE(atBodyPos(*aligned, 0, 0, 2) == Catch::Approx(atBodyPos(target, 0, 0, 2)));
  REQUIRE(yawFromQuat(*aligned, 0, 0) == Catch::Approx(kHalfPi).margin(1.0e-5F));
  requireAllZUnchanged(*aligned, target);
}

TEST_CASE("TRK reference alignment rotates body positions around target root") {
  constexpr float kHalfPi = 1.57079632679F;
  TrkTrack source = makeTrack(1);
  source.body_pos_w.values.at(0) = 5.0F;
  source.body_pos_w.values.at(1) = -2.0F;
  setBodyQuat(source, 0, 0, yawQuat(kHalfPi));

  TrkTrack target = makeTrack(2);
  target.body_pos_w.values.at(0) = 100.0F;
  target.body_pos_w.values.at(1) = -20.0F;
  target.body_pos_w.values.at(3 * 3 + 0) = 102.0F;
  target.body_pos_w.values.at(3 * 3 + 1) = -19.0F;
  target.body_pos_w.values.at(3 * 3 + 2) = 4.5F;
  setBodyQuat(target, 0, 0, yawQuat(0.0F));

  const std::optional<TrkTrack> aligned =
      alignTrackRootPlanarPose(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  REQUIRE(atBodyPos(*aligned, 0, 3, 0) == Catch::Approx(4.0F).margin(1.0e-5F));
  REQUIRE(atBodyPos(*aligned, 0, 3, 1) == Catch::Approx(0.0F).margin(1.0e-5F));
  REQUIRE(atBodyPos(*aligned, 0, 3, 2) == Catch::Approx(4.5F));
}

TEST_CASE("TRK reference alignment uses yaw delta when both roots have yaw") {
  constexpr float kSourceYaw = 1.04719755120F;
  constexpr float kTargetYaw = 0.52359877559F;
  constexpr float kYawDelta = kSourceYaw - kTargetYaw;
  const float cos_delta = std::cos(kYawDelta);
  const float sin_delta = std::sin(kYawDelta);

  TrkTrack source = makeTrack(1);
  source.body_pos_w.values.at(0) = 10.0F;
  source.body_pos_w.values.at(1) = -3.0F;
  setBodyQuat(source, 0, 0, yawQuat(kSourceYaw));

  TrkTrack target = makeTrack(2);
  target.body_pos_w.values.at(0) = 2.0F;
  target.body_pos_w.values.at(1) = 5.0F;
  target.body_pos_w.values.at(4 * 3 + 0) = 4.0F;
  target.body_pos_w.values.at(4 * 3 + 1) = 5.0F;
  target.body_pos_w.values.at(4 * 3 + 2) = 8.0F;
  target.body_lin_vel_w.values.at(4 * 3 + 0) = 0.0F;
  target.body_lin_vel_w.values.at(4 * 3 + 1) = 2.0F;
  target.body_lin_vel_w.values.at(4 * 3 + 2) = 9.0F;
  setBodyQuat(target, 0, 0, yawQuat(kTargetYaw));

  const std::optional<TrkTrack> aligned =
      alignTrackRootPlanarPose(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  REQUIRE(yawFromQuat(*aligned, 0, 0) ==
          Catch::Approx(kSourceYaw).margin(1.0e-5F));
  REQUIRE(atBodyPos(*aligned, 0, 4, 0) ==
          Catch::Approx(10.0F + 2.0F * cos_delta).margin(1.0e-5F));
  REQUIRE(atBodyPos(*aligned, 0, 4, 1) ==
          Catch::Approx(-3.0F + 2.0F * sin_delta).margin(1.0e-5F));
  REQUIRE(atBodyPos(*aligned, 0, 4, 2) == Catch::Approx(8.0F));
  REQUIRE(atBodyVector(aligned->body_lin_vel_w, 0, 4, 0) ==
          Catch::Approx(-2.0F * sin_delta).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_lin_vel_w, 0, 4, 1) ==
          Catch::Approx(2.0F * cos_delta).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_lin_vel_w, 0, 4, 2) == Catch::Approx(9.0F));
}

TEST_CASE("TRK reference alignment rotates world quats and velocities") {
  constexpr float kHalfPi = 1.57079632679F;
  constexpr float kQuarterPi = 0.78539816339F;
  TrkTrack source = makeTrack(1);
  setBodyQuat(source, 0, 0, yawQuat(kHalfPi));

  TrkTrack target = makeTrack(2);
  setBodyQuat(target, 0, 0, yawQuat(0.0F));
  setBodyQuat(target, 0, 3, yawQuat(kQuarterPi));
  target.body_lin_vel_w.values.at(3 * 3 + 0) = 3.0F;
  target.body_lin_vel_w.values.at(3 * 3 + 1) = 4.0F;
  target.body_lin_vel_w.values.at(3 * 3 + 2) = 5.0F;
  target.body_ang_vel_w.values.at(3 * 3 + 0) = 6.0F;
  target.body_ang_vel_w.values.at(3 * 3 + 1) = 7.0F;
  target.body_ang_vel_w.values.at(3 * 3 + 2) = 8.0F;

  const std::optional<TrkTrack> aligned =
      alignTrackRootPlanarPose(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  REQUIRE(yawFromQuat(*aligned, 0, 0) == Catch::Approx(kHalfPi).margin(1.0e-5F));
  REQUIRE(yawFromQuat(*aligned, 0, 3) ==
          Catch::Approx(kHalfPi + kQuarterPi).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_lin_vel_w, 0, 3, 0) ==
          Catch::Approx(-4.0F).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_lin_vel_w, 0, 3, 1) ==
          Catch::Approx(3.0F).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_lin_vel_w, 0, 3, 2) == Catch::Approx(5.0F));
  REQUIRE(atBodyVector(aligned->body_ang_vel_w, 0, 3, 0) ==
          Catch::Approx(-7.0F).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_ang_vel_w, 0, 3, 1) ==
          Catch::Approx(6.0F).margin(1.0e-5F));
  REQUIRE(atBodyVector(aligned->body_ang_vel_w, 0, 3, 2) == Catch::Approx(8.0F));
}

TEST_CASE("TRK reference alignment keeps local and contact arrays unchanged") {
  TrkTrack source = makeTrack(1);
  source.body_pos_w.values.at(0) = 5.0F;
  source.body_pos_w.values.at(1) = -2.0F;
  setBodyQuat(source, 0, 0, yawQuat(1.57079632679F));

  TrkTrack target = makeTrack(2);
  setBodyQuat(target, 0, 0, yawQuat(0.0F));
  const std::optional<TrkTrack> aligned =
      alignTrackRootPlanarPose(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  requireNonPositionArraysUnchanged(*aligned, target);
}

TEST_CASE("TRK reference alignment rejects missing root pose inputs but preserves Z") {
  TrkTrack source = makeTrack(1);
  setBodyQuat(source, 0, 0, yawQuat(1.57079632679F));
  TrkTrack target = makeTrack(2);
  target.body_pos_w.values.at(2) = 123.0F;
  setBodyQuat(target, 0, 0, yawQuat(0.0F));

  std::optional<TrkTrack> aligned =
      alignTrackRootPlanarPose(target, *source.frame(0));
  REQUIRE(aligned.has_value());
  REQUIRE(atBodyPos(*aligned, 0, 0, 2) == Catch::Approx(123.0F));

  target.body_quat_w.values.clear();
  REQUIRE_FALSE(alignTrackRootPlanarPose(target, *source.frame(0)).has_value());

  target = makeTrack(2);
  source.body_pos_w.values.at(0) = std::numeric_limits<float>::quiet_NaN();
  REQUIRE_FALSE(alignTrackRootPlanarPose(target, *source.frame(0)).has_value());
}

}  // namespace agentic_et1_tracker
