#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/synthetic_transition.hpp"

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

TrkTrack makeSingleFrameTrack() {
  TrkTrack track;
  track.metadata.frames = 1;
  track.metadata.fps = TrkSchema::kDefaultFps;
  track.metadata.duration_s = 0.0;

  resizeFloatArray(track.joint_pos, 1, {1, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.joint_vel, 1, {1, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.body_pos_w, 1, {1, TrkSchema::kBodyCount, 3}, kBodyCount * 3);
  resizeFloatArray(track.body_quat_w, 1, {1, TrkSchema::kBodyCount, 4}, kBodyCount * 4);
  resizeFloatArray(track.body_lin_vel_w, 1, {1, TrkSchema::kBodyCount, 3}, kBodyCount * 3);
  resizeFloatArray(track.body_ang_vel_w, 1, {1, TrkSchema::kBodyCount, 3}, kBodyCount * 3);
  resizeContactArray(track.left_foot_contact_state, 1, {1}, 1);
  resizeContactArray(track.right_foot_contact_state, 1, {1}, 1);
  resizeFloatArray(track.ref_com_rel_navi, 1, {1, 3}, 3);
  resizeFloatArray(track.ref_com_vel_navi, 1, {1, 3}, 3);

  for (std::size_t i = 0; i < kJointDim; ++i) {
    track.joint_pos.values.at(i) = 10.0F + static_cast<float>(i);
  }
  for (std::size_t i = 0; i < kBodyCount * 3; ++i) {
    track.body_pos_w.values.at(i) = 100.0F + static_cast<float>(i);
  }
  for (std::size_t body = 0; body < kBodyCount; ++body) {
    track.body_quat_w.values.at(body * 4) = 1.0F;
  }
  track.left_foot_contact_state.values.at(0) = 1;
  track.right_foot_contact_state.values.at(0) = 2;
  track.ref_com_rel_navi.values = {1.0F, 2.0F, 3.0F};
  return track;
}

TrkTrack makeTargetTrack() {
  TrkTrack track = makeSingleFrameTrack();
  for (std::size_t i = 0; i < kJointDim; ++i) {
    track.joint_pos.values.at(i) = 16.0F + static_cast<float>(i);
  }
  for (std::size_t i = 0; i < kBodyCount * 3; ++i) {
    track.body_pos_w.values.at(i) = 112.0F + static_cast<float>(i);
  }
  for (std::size_t body = 0; body < kBodyCount; ++body) {
    track.body_quat_w.values.at(body * 4) = 0.0F;
    track.body_quat_w.values.at(body * 4 + 1) = 1.0F;
    track.body_quat_w.values.at(body * 4 + 2) = 0.0F;
    track.body_quat_w.values.at(body * 4 + 3) = 0.0F;
  }
  track.ref_com_rel_navi.values = {4.0F, 8.0F, 12.0F};
  return track;
}

void requireFloatArrayShape(const TrkFloatArray& array,
                            const std::vector<std::uint64_t>& shape,
                            std::size_t frame_size) {
  REQUIRE(array.shape == shape);
  REQUIRE(array.frame_size == frame_size);
  REQUIRE(array.values.size() == static_cast<std::size_t>(shape.front()) * frame_size);
}

void requireContactArrayShape(const TrkContactArray& array,
                              const std::vector<std::uint64_t>& shape,
                              std::size_t frame_size) {
  REQUIRE(array.shape == shape);
  REQUIRE(array.frame_size == frame_size);
  REQUIRE(array.values.size() == static_cast<std::size_t>(shape.front()) * frame_size);
}

void requireAllFinite(const TrkFloatArray& array) {
  for (const float value : array.values) {
    REQUIRE(std::isfinite(value));
  }
}

void requireAllFloatsFinite(const TrkTrack& track) {
  requireAllFinite(track.joint_pos);
  requireAllFinite(track.joint_vel);
  requireAllFinite(track.body_pos_w);
  requireAllFinite(track.body_quat_w);
  requireAllFinite(track.body_lin_vel_w);
  requireAllFinite(track.body_ang_vel_w);
  requireAllFinite(track.ref_com_rel_navi);
  requireAllFinite(track.ref_com_vel_navi);
}

float atFrame(const TrkFloatArray& array, std::size_t frame, std::size_t element) {
  return array.values.at(frame * array.frame_size + element);
}

std::int64_t contactAt(const TrkContactArray& array, std::size_t frame) {
  return array.values.at(frame * array.frame_size);
}

double normalizedTime(const TrkTrack& track, std::size_t frame) {
  return static_cast<double>(frame) / static_cast<double>(track.metadata.frames - 1);
}

float lerp(float source, float target, double t) {
  return static_cast<float>(static_cast<double>(source) * (1.0 - t) +
                            static_cast<double>(target) * t);
}

}  // namespace

TEST_CASE("Trk synthetic transition builds required arrays and metadata in memory") {
  const TrkTrack source_track = makeSingleFrameTrack();
  const TrkTrack target_track = makeTargetTrack();

  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20);

  REQUIRE(transition.has_value());
  const TrkTrack& track = *transition;
  REQUIRE(track.metadata.frames >= 2);
  REQUIRE(track.metadata.fps == 25.0);
  REQUIRE(track.metadata.duration_s ==
          Catch::Approx(static_cast<double>(track.metadata.frames - 1) / 25.0));
  REQUIRE(track.metadata.duration_s <= 0.20);
  REQUIRE(track.metadata.version == TrkSchema::kVersion);
  REQUIRE(track.metadata.array_count == TrkSchema::kRequiredArrays.size());
  REQUIRE(track.metadata.file_size == 0);
  REQUIRE(track.metadata.canonical_path.empty());

  const std::uint64_t frames = track.metadata.frames;
  requireFloatArrayShape(track.joint_pos, {frames, TrkSchema::kJointDim}, kJointDim);
  requireFloatArrayShape(track.joint_vel, {frames, TrkSchema::kJointDim}, kJointDim);
  requireFloatArrayShape(track.body_pos_w, {frames, TrkSchema::kBodyCount, 3}, kBodyCount * 3);
  requireFloatArrayShape(track.body_quat_w, {frames, TrkSchema::kBodyCount, 4}, kBodyCount * 4);
  requireFloatArrayShape(track.body_lin_vel_w, {frames, TrkSchema::kBodyCount, 3},
                         kBodyCount * 3);
  requireFloatArrayShape(track.body_ang_vel_w, {frames, TrkSchema::kBodyCount, 3},
                         kBodyCount * 3);
  requireContactArrayShape(track.left_foot_contact_state, {frames}, 1);
  requireContactArrayShape(track.right_foot_contact_state, {frames}, 1);
  requireFloatArrayShape(track.ref_com_rel_navi, {frames, 3}, 3);
  requireFloatArrayShape(track.ref_com_vel_navi, {frames, 3}, 3);
  requireAllFloatsFinite(track);
}

TEST_CASE("Trk synthetic transition samples smooth positions and velocity boundaries") {
  const TrkTrack source_track = makeSingleFrameTrack();
  const TrkTrack target_track = makeTargetTrack();

  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20);

  REQUIRE(transition.has_value());
  const TrkTrack& track = *transition;
  const std::size_t last = track.metadata.frames - 1;
  REQUIRE(last >= 2);

  REQUIRE(atFrame(track.joint_pos, 0, 0) == Catch::Approx(10.0F));
  REQUIRE(atFrame(track.joint_pos, last, 0) == Catch::Approx(16.0F));
  REQUIRE(atFrame(track.joint_pos, 1, 0) > atFrame(track.joint_pos, 0, 0));
  REQUIRE(atFrame(track.joint_pos, last - 1, 0) < atFrame(track.joint_pos, last, 0));
  REQUIRE(atFrame(track.joint_vel, 0, 0) == Catch::Approx(0.0F).margin(1.0e-4F));
  REQUIRE(atFrame(track.joint_vel, 1, 0) > 0.0F);
  REQUIRE(std::isfinite(atFrame(track.joint_vel, 1, 0)));
  REQUIRE(atFrame(track.joint_vel, last, 0) == Catch::Approx(0.0F).margin(1.0e-4F));

  REQUIRE(atFrame(track.body_pos_w, 0, 0) == Catch::Approx(100.0F));
  REQUIRE(atFrame(track.body_pos_w, last, 0) == Catch::Approx(112.0F));
  REQUIRE(atFrame(track.body_pos_w, 1, 0) ==
          Catch::Approx(lerp(100.0F, 112.0F, normalizedTime(track, 1))));
  REQUIRE(atFrame(track.body_pos_w, last - 1, 0) ==
          Catch::Approx(lerp(100.0F, 112.0F, normalizedTime(track, last - 1))));
  REQUIRE(atFrame(track.body_lin_vel_w, 0, 0) == Catch::Approx(0.0F).margin(1.0e-4F));
  REQUIRE(atFrame(track.body_lin_vel_w, 1, 0) == Catch::Approx(0.0F).margin(1.0e-4F));
  REQUIRE(atFrame(track.body_lin_vel_w, last, 0) == Catch::Approx(0.0F).margin(1.0e-4F));
  for (std::size_t frame = 0; frame < track.metadata.frames; ++frame) {
    REQUIRE(atFrame(track.body_ang_vel_w, frame, 0) == Catch::Approx(0.0F));
  }

  REQUIRE(atFrame(track.ref_com_rel_navi, 0, 0) == Catch::Approx(1.0F));
  REQUIRE(atFrame(track.ref_com_rel_navi, last, 0) == Catch::Approx(4.0F));
  REQUIRE(atFrame(track.ref_com_rel_navi, 1, 0) ==
          Catch::Approx(lerp(1.0F, 4.0F, normalizedTime(track, 1))));
  REQUIRE(atFrame(track.ref_com_rel_navi, last - 1, 0) ==
          Catch::Approx(lerp(1.0F, 4.0F, normalizedTime(track, last - 1))));
  REQUIRE(atFrame(track.ref_com_vel_navi, 0, 0) == Catch::Approx(0.0F).margin(1.0e-4F));
  REQUIRE(atFrame(track.ref_com_vel_navi, 1, 0) == Catch::Approx(0.0F).margin(1.0e-4F));
  REQUIRE(atFrame(track.ref_com_vel_navi, last, 0) == Catch::Approx(0.0F).margin(1.0e-4F));

  for (std::size_t frame = 0; frame < track.metadata.frames; ++frame) {
    REQUIRE(contactAt(track.left_foot_contact_state, frame) == 1);
    REQUIRE(contactAt(track.right_foot_contact_state, frame) == 2);
  }

  REQUIRE(atFrame(track.body_quat_w, 0, 0) == Catch::Approx(1.0F));
  const double q_t = normalizedTime(track, 1);
  const double q_norm = std::sqrt((1.0 - q_t) * (1.0 - q_t) + q_t * q_t);
  REQUIRE(atFrame(track.body_quat_w, 1, 0) ==
          Catch::Approx(static_cast<float>((1.0 - q_t) / q_norm)).margin(1.0e-5F));
  REQUIRE(atFrame(track.body_quat_w, 1, 1) ==
          Catch::Approx(static_cast<float>(q_t / q_norm)).margin(1.0e-5F));
  REQUIRE(atFrame(track.body_quat_w, last, 1) == Catch::Approx(1.0F));
}

TEST_CASE("Trk synthetic transition preserves source and target velocity boundaries") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();

  source_track.joint_vel.values.at(0) = 0.75F;
  target_track.joint_vel.values.at(0) = -0.25F;
  source_track.body_lin_vel_w.values.at(0) = 1.25F;
  target_track.body_lin_vel_w.values.at(0) = -0.5F;
  source_track.body_ang_vel_w.values.at(0) = -0.375F;
  target_track.body_ang_vel_w.values.at(0) = 0.625F;
  source_track.ref_com_vel_navi.values.at(0) = 0.5F;
  target_track.ref_com_vel_navi.values.at(0) = -0.125F;

  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20);

  REQUIRE(transition.has_value());
  const TrkTrack& track = *transition;
  const std::size_t last = track.metadata.frames - 1;

  REQUIRE(atFrame(track.joint_vel, 0, 0) == Catch::Approx(0.75F).margin(1.0e-4F));
  REQUIRE(atFrame(track.joint_vel, last, 0) == Catch::Approx(-0.25F).margin(1.0e-4F));
  REQUIRE(atFrame(track.body_lin_vel_w, 0, 0) == Catch::Approx(1.25F).margin(1.0e-4F));
  REQUIRE(atFrame(track.body_lin_vel_w, last, 0) == Catch::Approx(-0.5F).margin(1.0e-4F));
  REQUIRE(atFrame(track.body_ang_vel_w, 0, 0) == Catch::Approx(-0.375F).margin(1.0e-4F));
  REQUIRE(atFrame(track.body_ang_vel_w, last, 0) == Catch::Approx(0.625F).margin(1.0e-4F));
  REQUIRE(atFrame(track.ref_com_vel_navi, 0, 0) == Catch::Approx(0.5F).margin(1.0e-4F));
  REQUIRE(atFrame(track.ref_com_vel_navi, last, 0) == Catch::Approx(-0.125F).margin(1.0e-4F));
}

TEST_CASE("Trk synthetic transition grows to quantized Ruckig duration within cap") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();
  target_track.joint_pos.values.at(0) = source_track.joint_pos.values.at(0) + 100.0F;

  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 1.0);

  REQUIRE(transition.has_value());
  REQUIRE(transition->metadata.duration_s > 0.12);
  REQUIRE(transition->metadata.duration_s <= 1.0);
  REQUIRE(transition->metadata.duration_s == Catch::Approx(0.44));
  REQUIRE(transition->metadata.frames == 12);
  REQUIRE(atFrame(transition->joint_pos, 0, 0) == Catch::Approx(source_track.joint_pos.values.at(0)));
  REQUIRE(atFrame(transition->joint_pos, transition->metadata.frames - 1, 0) ==
          Catch::Approx(target_track.joint_pos.values.at(0)));
}

TEST_CASE("Trk synthetic transition honors configured minimum frame count") {
  const TrkTrack source_track = makeSingleFrameTrack();
  const TrkTrack target_track = makeTargetTrack();
  SyntheticTransitionOptions options;
  options.max_duration_s = 1.0;
  options.min_frames = 10;

  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, options);

  REQUIRE(transition.has_value());
  REQUIRE(transition->metadata.frames == 10);
  REQUIRE(transition->metadata.duration_s == Catch::Approx(0.36));
}

TEST_CASE("Trk synthetic transition rejects quantized Ruckig duration beyond cap") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();
  target_track.joint_pos.values.at(0) = source_track.joint_pos.values.at(0) + 100.0F;

  REQUIRE_FALSE(
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.30)
          .has_value());
}

TEST_CASE("Trk synthetic transition applies configured cap tolerance") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();
  target_track.joint_pos.values.at(0) = source_track.joint_pos.values.at(0) + 100.0F;

  SyntheticTransitionOptions options;
  options.max_duration_s = 0.4399;
  options.duration_dt_tolerance_s = 0.0002;
  REQUIRE(makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, options)
              .has_value());

  options.duration_dt_tolerance_s = 0.00001;
  REQUIRE_FALSE(
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, options)
          .has_value());
}

TEST_CASE("Trk synthetic transition only applies velocity limits to controlled joints") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();
  const float over_limit =
      static_cast<float>(defaultSyntheticTransitionLimits().max_velocity + 1.0);

  source_track.body_lin_vel_w.values.at(0) = over_limit;
  target_track.body_lin_vel_w.values.at(0) = -over_limit;
  source_track.ref_com_vel_navi.values.at(0) = over_limit;
  target_track.ref_com_vel_navi.values.at(0) = -over_limit;
  REQUIRE(makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20)
              .has_value());

  source_track.joint_vel.values.at(0) = over_limit;
  REQUIRE_FALSE(
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20)
          .has_value());
}

TEST_CASE("Trk synthetic transition rejects contact mismatch and preserves same contact") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();

  target_track.left_foot_contact_state.values.at(0) = 2;
  REQUIRE_FALSE(
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20)
          .has_value());

  target_track = makeTargetTrack();
  target_track.right_foot_contact_state.values.at(0) = 0;
  REQUIRE_FALSE(
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20)
          .has_value());

  target_track = makeTargetTrack();
  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.20);
  REQUIRE(transition.has_value());
  for (std::size_t frame = 0; frame < transition->metadata.frames; ++frame) {
    REQUIRE(contactAt(transition->left_foot_contact_state, frame) == 1);
    REQUIRE(contactAt(transition->right_foot_contact_state, frame) == 2);
  }
}

TEST_CASE("Trk synthetic transition uses shortest-path quaternion nlerp") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();
  for (std::size_t body = 0; body < kBodyCount; ++body) {
    target_track.body_quat_w.values.at(body * 4) = -0.6F;
    target_track.body_quat_w.values.at(body * 4 + 1) = -0.8F;
    target_track.body_quat_w.values.at(body * 4 + 2) = 0.0F;
    target_track.body_quat_w.values.at(body * 4 + 3) = 0.0F;
  }

  const std::optional<TrkTrack> transition =
      makeSyntheticTransitionTrk(*source_track.frame(0), *target_track.frame(0), 25.0, 0.12);

  REQUIRE(transition.has_value());
  const std::size_t frame = 1;
  const double t = normalizedTime(*transition, frame);
  const double q0 = 1.0 * (1.0 - t) + 0.6 * t;
  const double q1 = 0.8 * t;
  const double norm = std::sqrt(q0 * q0 + q1 * q1);
  REQUIRE(atFrame(transition->body_quat_w, 1, 0) ==
          Catch::Approx(static_cast<float>(q0 / norm)).margin(1.0e-5F));
  REQUIRE(atFrame(transition->body_quat_w, 1, 1) ==
          Catch::Approx(static_cast<float>(q1 / norm)).margin(1.0e-5F));
  const std::size_t last = transition->metadata.frames - 1;
  REQUIRE(atFrame(transition->body_quat_w, last, 0) == Catch::Approx(0.6F).margin(1.0e-5F));
  REQUIRE(atFrame(transition->body_quat_w, last, 1) == Catch::Approx(0.8F).margin(1.0e-5F));
}

TEST_CASE("Trk synthetic transition rejects invalid numeric inputs") {
  TrkTrack source_track = makeSingleFrameTrack();
  TrkTrack target_track = makeTargetTrack();
  const TrkFrameView source = *source_track.frame(0);
  const TrkFrameView target = *target_track.frame(0);

  REQUIRE_FALSE(makeSyntheticTransitionTrk(source, target, 0.0, 0.12).has_value());
  REQUIRE_FALSE(makeSyntheticTransitionTrk(source, target, -25.0, 0.12).has_value());
  REQUIRE_FALSE(makeSyntheticTransitionTrk(source, target,
                                           std::numeric_limits<double>::quiet_NaN(), 0.12)
                    .has_value());
  REQUIRE_FALSE(makeSyntheticTransitionTrk(source, target, 25.0,
                                           std::numeric_limits<double>::quiet_NaN())
                    .has_value());

  source_track.joint_pos.values.at(0) = std::numeric_limits<float>::quiet_NaN();
  REQUIRE_FALSE(makeSyntheticTransitionTrk(*source_track.frame(0), target, 25.0, 0.12)
                    .has_value());

  source_track = makeSingleFrameTrack();
  for (std::size_t i = 0; i < 4; ++i) {
    source_track.body_quat_w.values.at(i) = 0.0F;
  }
  REQUIRE_FALSE(makeSyntheticTransitionTrk(*source_track.frame(0), target, 25.0, 0.12)
                    .has_value());

  source_track = makeSingleFrameTrack();
  source_track.left_foot_contact_state.values.at(0) = 3;
  REQUIRE_FALSE(makeSyntheticTransitionTrk(*source_track.frame(0), target, 25.0, 0.12)
                    .has_value());
}

}  // namespace agentic_et1_tracker
