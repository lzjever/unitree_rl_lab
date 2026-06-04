#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
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
  REQUIRE(aligned.body_quat_w.values == target.body_quat_w.values);
  REQUIRE(aligned.body_lin_vel_w.values == target.body_lin_vel_w.values);
  REQUIRE(aligned.body_ang_vel_w.values == target.body_ang_vel_w.values);
  REQUIRE(aligned.left_foot_contact_state.values ==
          target.left_foot_contact_state.values);
  REQUIRE(aligned.right_foot_contact_state.values ==
          target.right_foot_contact_state.values);
  REQUIRE(aligned.ref_com_rel_navi.values == target.ref_com_rel_navi.values);
  REQUIRE(aligned.ref_com_vel_navi.values == target.ref_com_vel_navi.values);
}

}  // namespace

TEST_CASE("TRK reference alignment translates target root XY to source root XY") {
  TrkTrack source = makeTrack(1);
  source.body_pos_w.values.at(0) = 5.0F;
  source.body_pos_w.values.at(1) = -2.0F;
  source.body_pos_w.values.at(2) = 0.75F;

  const TrkTrack target = makeTrack(2);
  const std::optional<TrkTrack> aligned =
      alignTrackRootTranslation(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  REQUIRE(atBodyPos(*aligned, 0, 0, 0) == Catch::Approx(5.0F));
  REQUIRE(atBodyPos(*aligned, 0, 0, 1) == Catch::Approx(-2.0F));
  REQUIRE(atBodyPos(*aligned, 0, 0, 2) == Catch::Approx(atBodyPos(target, 0, 0, 2)));

  REQUIRE(atBodyPos(*aligned, 0, 3, 0) ==
          Catch::Approx(atBodyPos(target, 0, 3, 0) + 4.0F));
  REQUIRE(atBodyPos(*aligned, 1, 4, 1) ==
          Catch::Approx(atBodyPos(target, 1, 4, 1) - 4.0F));
  REQUIRE(atBodyPos(*aligned, 1, 5, 2) ==
          Catch::Approx(atBodyPos(target, 1, 5, 2)));
  requireAllZUnchanged(*aligned, target);
}

TEST_CASE("TRK reference alignment keeps non-position arrays unchanged") {
  TrkTrack source = makeTrack(1);
  source.body_pos_w.values.at(0) = 5.0F;
  source.body_pos_w.values.at(1) = -2.0F;
  source.body_pos_w.values.at(2) = 0.75F;

  const TrkTrack target = makeTrack(2);
  const std::optional<TrkTrack> aligned =
      alignTrackRootTranslation(target, *source.frame(0));

  REQUIRE(aligned.has_value());
  requireNonPositionArraysUnchanged(*aligned, target);
}

}  // namespace agentic_et1_tracker
