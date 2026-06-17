#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/loco_upper/compiler.hpp"
#include "agentic_et1_tracker/loco_upper/precheck.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

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

std::array<float, 4> yawQuat(double yaw_rad) {
  return {static_cast<float>(std::cos(0.5 * yaw_rad)),
          0.0F,
          0.0F,
          static_cast<float>(std::sin(0.5 * yaw_rad))};
}

TrkTrack makeTrack(std::size_t frames, double fps = 10.0) {
  TrkTrack track;
  track.metadata.frames = frames;
  track.metadata.fps = fps;
  track.metadata.duration_s = frames > 0 ? static_cast<double>(frames - 1) / fps : 0.0;

  resizeFloatArray(track.joint_pos, frames, {frames, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.joint_vel, frames, {frames, TrkSchema::kJointDim}, kJointDim);
  resizeFloatArray(track.body_pos_w,
                   frames,
                   {frames, TrkSchema::kBodyCount, 3},
                   kBodyCount * 3);
  resizeFloatArray(track.body_quat_w,
                   frames,
                   {frames, TrkSchema::kBodyCount, 4},
                   kBodyCount * 4);
  resizeFloatArray(track.body_lin_vel_w,
                   frames,
                   {frames, TrkSchema::kBodyCount, 3},
                   kBodyCount * 3);
  resizeFloatArray(track.body_ang_vel_w,
                   frames,
                   {frames, TrkSchema::kBodyCount, 3},
                   kBodyCount * 3);
  resizeContactArray(track.left_foot_contact_state, frames, {frames}, 1);
  resizeContactArray(track.right_foot_contact_state, frames, {frames}, 1);
  resizeFloatArray(track.ref_com_rel_navi, frames, {frames, 3}, 3);
  resizeFloatArray(track.ref_com_vel_navi, frames, {frames, 3}, 3);

  for (std::size_t frame = 0; frame < frames; ++frame) {
    for (std::size_t body = 0; body < kBodyCount; ++body) {
      const std::array<float, 4> quat = yawQuat(0.0);
      const std::size_t quat_offset = frame * track.body_quat_w.frame_size + body * 4;
      for (std::size_t i = 0; i < quat.size(); ++i) {
        track.body_quat_w.values.at(quat_offset + i) = quat.at(i);
      }
    }
  }
  return track;
}

void setRootPose(TrkTrack& track,
                 std::size_t frame,
                 double x,
                 double y,
                 double yaw_rad = 0.0) {
  const std::size_t pos_offset = frame * track.body_pos_w.frame_size;
  track.body_pos_w.values.at(pos_offset + 0) = static_cast<float>(x);
  track.body_pos_w.values.at(pos_offset + 1) = static_cast<float>(y);
  const std::array<float, 4> quat = yawQuat(yaw_rad);
  const std::size_t quat_offset = frame * track.body_quat_w.frame_size;
  for (std::size_t i = 0; i < quat.size(); ++i) {
    track.body_quat_w.values.at(quat_offset + i) = quat.at(i);
  }
}

void setJoint(TrkTrack& track, std::size_t frame, std::size_t joint, float value) {
  track.joint_pos.values.at(frame * track.joint_pos.frame_size + joint) = value;
}

LocoUpperCompileOptions compileOptions() {
  LocoUpperCompileOptions options;
  options.max_radius_m = 10.0;
  options.command_limits.max_linear_velocity = 10.0;
  options.command_limits.max_yaw_rate = 10.0;
  options.command_limits.max_linear_acceleration = 100.0;
  options.command_limits.max_yaw_acceleration = 100.0;
  options.upper_joint_limits.min_positions.assign(kLocoUpperJointCount, -1.0);
  options.upper_joint_limits.max_positions.assign(kLocoUpperJointCount, 1.0);
  options.upper_joint_limits.max_velocities.assign(kLocoUpperJointCount, 2.0);
  options.upper_joint_limits.max_accelerations.assign(kLocoUpperJointCount, 5.0);
  return options;
}

LocoUpperCompileResult compileOk(const TrkTrack& track,
                                 const LocoUpperCompileOptions& options) {
  const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);
  REQUIRE(result.ok());
  return result;
}

void requireUpperJointsWithinBounds(const CompiledLocoUpperPlan& plan,
                                    double min_position,
                                    double max_position) {
  for (const LocoUpperLogicalJointFrame& frame : plan.joint_pos_frames) {
    for (std::size_t joint = kLocoUpperJointFirst;
         joint < kLocoUpperJointLastExclusive;
         ++joint) {
      REQUIRE(frame.at(joint) >= min_position);
      REQUIRE(frame.at(joint) <= max_position);
    }
  }
}

}  // namespace

TEST_CASE("loco upper compiler clamps upper position and emits full logical frames") {
  TrkTrack track = makeTrack(2);
  LocoUpperCompileOptions options = compileOptions();
  options.upper_joint_limits.max_velocities.clear();
  options.upper_joint_limits.max_accelerations.clear();
  setJoint(track, 0, 0, 0.25F);
  setJoint(track, 0, kLocoUpperJointFirst, 2.5F);
  setJoint(track, 1, kLocoUpperJointFirst, -2.5F);

  const LocoUpperCompileResult result = compileOk(track, options);

  REQUIRE(result.flags.upper_clamped);
  REQUIRE(result.plan.joint_pos_frames.size() == 2);
  REQUIRE(result.plan.joint_pos_frames.front().size() == kJointDim);
  REQUIRE(result.plan.joint_pos_frames.at(0).at(0) == Catch::Approx(0.25));
  REQUIRE(result.plan.joint_pos_frames.at(0).at(kLocoUpperJointFirst) ==
          Catch::Approx(1.0));
  REQUIRE(result.plan.joint_pos_frames.at(1).at(kLocoUpperJointFirst) ==
          Catch::Approx(-1.0));
}

TEST_CASE("loco upper compiler rate limits upper velocity without rejecting") {
  TrkTrack track = makeTrack(2, 10.0);
  setJoint(track, 0, kLocoUpperJointFirst, 0.0F);
  setJoint(track, 1, kLocoUpperJointFirst, 1.0F);

  const LocoUpperCompileResult result = compileOk(track, compileOptions());

  REQUIRE(result.flags.upper_velocity_limited);
  REQUIRE(result.flags.upper_rate_limited);
  REQUIRE(result.plan.joint_pos_frames.at(1).at(kLocoUpperJointFirst) ==
          Catch::Approx(0.2));
}

TEST_CASE("loco upper compiler acceleration limiting maps to public rate flag") {
  TrkTrack track = makeTrack(3, 10.0);
  LocoUpperCompileOptions options = compileOptions();
  options.upper_joint_limits.max_velocities.at(0) = 100.0;
  options.upper_joint_limits.max_accelerations.at(0) = 5.0;
  setJoint(track, 0, kLocoUpperJointFirst, 0.0F);
  setJoint(track, 1, kLocoUpperJointFirst, 0.0F);
  setJoint(track, 2, kLocoUpperJointFirst, 1.0F);

  const LocoUpperCompileResult result = compileOk(track, options);

  REQUIRE(result.flags.upper_accel_limited);
  REQUIRE(result.flags.upper_rate_limited);
  REQUIRE(result.plan.joint_pos_frames.at(2).at(kLocoUpperJointFirst) ==
          Catch::Approx(0.05));
}

TEST_CASE("loco upper compiler keeps acceleration-limited upper joints in position bounds") {
  LocoUpperCompileOptions options = compileOptions();
  options.upper_joint_limits.max_velocities.at(0) = 100.0;
  options.upper_joint_limits.max_accelerations.at(0) = 1.0;

  SECTION("max bound") {
    TrkTrack track = makeTrack(3, 10.0);
    setJoint(track, 0, kLocoUpperJointFirst, 0.95F);
    setJoint(track, 1, kLocoUpperJointFirst, 1.0F);
    setJoint(track, 2, kLocoUpperJointFirst, 1.0F);

    const LocoUpperCompileResult result = compileOk(track, options);

    REQUIRE(result.flags.upper_rate_limited);
    REQUIRE(result.flags.upper_accel_limited);
    requireUpperJointsWithinBounds(result.plan, -1.0, 1.0);
  }

  SECTION("min bound") {
    TrkTrack track = makeTrack(3, 10.0);
    setJoint(track, 0, kLocoUpperJointFirst, -0.95F);
    setJoint(track, 1, kLocoUpperJointFirst, -1.0F);
    setJoint(track, 2, kLocoUpperJointFirst, -1.0F);

    const LocoUpperCompileResult result = compileOk(track, options);

    REQUIRE(result.flags.upper_rate_limited);
    REQUIRE(result.flags.upper_accel_limited);
    requireUpperJointsWithinBounds(result.plan, -1.0, 1.0);
  }
}

TEST_CASE("loco upper compiler projects radius and strict precheck accepts it") {
  TrkTrack track = makeTrack(2);
  setRootPose(track, 0, 0.0, 0.0);
  setRootPose(track, 1, 2.0, 0.0);
  LocoUpperCompileOptions options = compileOptions();
  options.max_radius_m = 1.0;

  const LocoUpperCompileResult result = compileOk(track, options);

  REQUIRE(result.flags.radius_clamped);
  REQUIRE(result.plan.root_plan.samples.at(1).x == Catch::Approx(1.0));

  LocoUpperPrecheckOptions precheck_options;
  precheck_options.max_radius_m = 1.0;
  precheck_options.strict_pose = true;
  precheck_options.upper_joint_limits = options.upper_joint_limits;
  const LocoUpperPrecheckResult precheck =
      precheckLocoUpperTrack(track, precheck_options);

  REQUIRE(precheck.ok());
  REQUIRE(precheck.flags.radius_clamped);
}

TEST_CASE("loco upper compiler reports root command envelope clamping") {
  TrkTrack track = makeTrack(3, 10.0);
  setRootPose(track, 0, 0.0, 0.0);
  setRootPose(track, 1, 0.5, 0.0);
  setRootPose(track, 2, 1.0, 0.0);
  LocoUpperCompileOptions options = compileOptions();
  options.command_limits.max_linear_velocity = 1.0;

  const LocoUpperCompileResult result = compileOk(track, options);

  REQUIRE_FALSE(result.flags.radius_clamped);
  REQUIRE(result.flags.envelope_clamped);
  REQUIRE(result.plan.root_velocity_commands.size() == result.plan.frame_count);
  REQUIRE(result.plan.root_velocity_commands.size() == 3);
  REQUIRE(result.plan.root_velocity_commands.at(0).vx == Catch::Approx(1.0));
  REQUIRE(result.plan.root_velocity_commands.at(1).vx == Catch::Approx(1.0));
  REQUIRE(result.plan.root_velocity_commands.at(2).vx == Catch::Approx(0.0));
  REQUIRE(result.plan.root_velocity_commands.at(2).vy == Catch::Approx(0.0));
  REQUIRE(result.plan.root_velocity_commands.at(2).yaw_rate == Catch::Approx(0.0));
}

TEST_CASE("loco upper compiler preserves frame timing and indexing") {
  TrkTrack track = makeTrack(4, 20.0);
  track.metadata.duration_s = 999.0;
  for (std::size_t frame = 0; frame < track.metadata.frames; ++frame) {
    setRootPose(track, frame, static_cast<double>(frame) * 0.1, 0.0);
    setJoint(track,
             frame,
             kLocoUpperJointFirst,
             static_cast<float>(0.01 * static_cast<double>(frame)));
  }

  const LocoUpperCompileResult result = compileOk(track, compileOptions());

  REQUIRE(result.plan.fps == Catch::Approx(20.0));
  REQUIRE(result.plan.frame_count == 4);
  REQUIRE(result.plan.duration_s == Catch::Approx(0.15));
  REQUIRE(result.plan.root_plan.samples.size() == 4);
  REQUIRE(result.plan.root_velocity_commands.size() == result.plan.frame_count);
  REQUIRE(result.plan.joint_pos_frames.size() == 4);
  for (std::size_t frame = 0; frame < result.plan.frame_count; ++frame) {
    REQUIRE(result.plan.root_plan.samples.at(frame).x ==
            Catch::Approx(static_cast<double>(frame) * 0.1));
    REQUIRE(result.plan.joint_pos_frames.at(frame).at(kLocoUpperJointFirst) ==
            Catch::Approx(0.01 * static_cast<double>(frame)));
  }
}

TEST_CASE("loco upper compiler still rejects illegal track and config inputs") {
  LocoUpperCompileOptions options = compileOptions();

  SECTION("non-finite joint") {
    TrkTrack track = makeTrack(2);
    setJoint(track, 0, kLocoUpperJointFirst, std::numeric_limits<float>::infinity());

    const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.failure_kind == LocoUpperCompileFailureKind::InvalidTrack);
    REQUIRE_THAT(result.message, ContainsSubstring("finite"));
  }

  SECTION("invalid joint shape") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.frame_size = kLocoUpperJointLastExclusive - 1;
    track.joint_pos.shape = {2, kLocoUpperJointLastExclusive - 1};
    track.joint_pos.values.resize(track.metadata.frames * track.joint_pos.frame_size);

    const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.failure_kind == LocoUpperCompileFailureKind::InvalidTrack);
    REQUIRE_THAT(result.message, ContainsSubstring("shape"));
  }

  SECTION("invalid fps") {
    TrkTrack track = makeTrack(2);
    track.metadata.fps = 0.0;

    const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.failure_kind == LocoUpperCompileFailureKind::InvalidTrack);
    REQUIRE_THAT(result.message, ContainsSubstring("fps"));
  }

  SECTION("invalid frame count") {
    TrkTrack track = makeTrack(1);

    const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.failure_kind == LocoUpperCompileFailureKind::InvalidTrack);
    REQUIRE_THAT(result.message, ContainsSubstring("frames"));
  }

  SECTION("invalid radius config") {
    TrkTrack track = makeTrack(2);
    options.max_radius_m = 0.0;

    const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.failure_kind == LocoUpperCompileFailureKind::InvalidOptions);
    REQUIRE_THAT(result.message, ContainsSubstring("radius"));
  }

  SECTION("invalid upper limit mapping") {
    TrkTrack track = makeTrack(2);
    options.upper_joint_limits.min_positions.assign(kLocoUpperJointCount - 1, -1.0);

    const LocoUpperCompileResult result = compileLocoUpperPlan(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.failure_kind == LocoUpperCompileFailureKind::InvalidConfig);
    REQUIRE_THAT(result.message, ContainsSubstring("14 entries"));
  }
}

}  // namespace agentic_et1_tracker
