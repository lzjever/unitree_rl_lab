#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/loco_upper/planner.hpp"
#include "agentic_et1_tracker/loco_upper/precheck.hpp"
#include "agentic_et1_tracker/loco_upper/validator.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr std::size_t kJointDim = TrkSchema::kJointDim;
constexpr std::size_t kBodyCount = TrkSchema::kBodyCount;
constexpr double kPi = 3.14159265358979323846;

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

TrkTrack makeTrack(std::size_t frames, double fps = 1.0) {
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
                 double yaw_rad) {
  const std::size_t pos_offset = frame * track.body_pos_w.frame_size;
  track.body_pos_w.values.at(pos_offset + 0) = static_cast<float>(x);
  track.body_pos_w.values.at(pos_offset + 1) = static_cast<float>(y);
  const std::array<float, 4> quat = yawQuat(yaw_rad);
  const std::size_t quat_offset = frame * track.body_quat_w.frame_size;
  for (std::size_t i = 0; i < quat.size(); ++i) {
    track.body_quat_w.values.at(quat_offset + i) = quat.at(i);
  }
}

LocoUpperRootPlan extractPlan(const TrkTrack& track) {
  const LocoUpperRootPlanResult result = extractRootPlanarPath(track);
  REQUIRE(result.ok());
  return result.plan;
}

LocoUpperJointValidationOptions testUpperLimits() {
  LocoUpperJointValidationOptions options;
  options.min_positions.assign(kLocoUpperJointCount, -10.0);
  options.max_positions.assign(kLocoUpperJointCount, 10.0);
  options.max_velocities.assign(kLocoUpperJointCount, 2.0);
  options.max_accelerations.assign(kLocoUpperJointCount, 5.0);
  return options;
}

struct TempDir {
  TempDir() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_loco_upper_precheck_" + std::to_string(stamp));
    std::filesystem::create_directories(root);
  }

  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path root;
};

}  // namespace

TEST_CASE("loco upper planner emits static zero velocity commands") {
  TrkTrack track = makeTrack(3);
  setRootPose(track, 0, 1.0, 2.0, 0.25);
  setRootPose(track, 1, 1.0, 2.0, 0.25);
  setRootPose(track, 2, 1.0, 2.0, 0.25);

  const LocoUpperRootPlan aligned = alignRootPlanToStart(extractPlan(track));
  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(aligned);

  REQUIRE(commands.size() == 2);
  for (const LocoUpperVelocityCommand& command : commands) {
    REQUIRE(command.vx == Catch::Approx(0.0));
    REQUIRE(command.vy == Catch::Approx(0.0));
    REQUIRE(command.yaw_rate == Catch::Approx(0.0));
  }
}

TEST_CASE("loco upper planner preserves forward velocity sign") {
  TrkTrack track = makeTrack(3);
  setRootPose(track, 0, 0.0, 0.0, 0.0);
  setRootPose(track, 1, 0.5, 0.0, 0.0);
  setRootPose(track, 2, 1.0, 0.0, 0.0);

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(alignRootPlanToStart(extractPlan(track)));

  REQUIRE(commands.size() == 2);
  REQUIRE(commands.at(0).vx > 0.0);
  REQUIRE(commands.at(0).vy == Catch::Approx(0.0));
}

TEST_CASE("loco upper planner preserves lateral velocity sign") {
  TrkTrack track = makeTrack(3);
  setRootPose(track, 0, 0.0, 0.0, 0.0);
  setRootPose(track, 1, 0.0, -0.5, 0.0);
  setRootPose(track, 2, 0.0, -1.0, 0.0);

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(alignRootPlanToStart(extractPlan(track)));

  REQUIRE(commands.size() == 2);
  REQUIRE(commands.at(0).vx == Catch::Approx(0.0));
  REQUIRE(commands.at(0).vy < 0.0);
}

TEST_CASE("loco upper planner preserves yaw-only command sign") {
  TrkTrack track = makeTrack(2);
  setRootPose(track, 0, 0.0, 0.0, 0.0);
  setRootPose(track, 1, 0.0, 0.0, 0.5);

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(alignRootPlanToStart(extractPlan(track)));

  REQUIRE(commands.size() == 1);
  REQUIRE(commands.at(0).vx == Catch::Approx(0.0));
  REQUIRE(commands.at(0).vy == Catch::Approx(0.0));
  REQUIRE(commands.at(0).yaw_rate > 0.0);
}

TEST_CASE("loco upper planner unwraps root yaw before differencing") {
  TrkTrack track = makeTrack(2);
  setRootPose(track, 0, 0.0, 0.0, 179.0 * kPi / 180.0);
  setRootPose(track, 1, 0.0, 0.0, -179.0 * kPi / 180.0);

  const LocoUpperRootPlan plan = extractPlan(track);
  REQUIRE(plan.samples.at(1).yaw > plan.samples.at(0).yaw);
  REQUIRE(plan.samples.at(1).yaw - plan.samples.at(0).yaw ==
          Catch::Approx(2.0 * kPi / 180.0).margin(1.0e-5));
}

TEST_CASE("loco upper planner is invariant to global XY offsets after start alignment") {
  TrkTrack near_origin = makeTrack(3);
  setRootPose(near_origin, 0, 1.0, -2.0, 0.0);
  setRootPose(near_origin, 1, 2.0, -2.0, 0.0);
  setRootPose(near_origin, 2, 3.0, -2.0, 0.0);

  TrkTrack offset = makeTrack(3);
  setRootPose(offset, 0, 101.0, 98.0, 0.0);
  setRootPose(offset, 1, 102.0, 98.0, 0.0);
  setRootPose(offset, 2, 103.0, 98.0, 0.0);

  const LocoUpperRootPlan aligned_near = alignRootPlanToStart(extractPlan(near_origin));
  const LocoUpperRootPlan aligned_offset = alignRootPlanToStart(extractPlan(offset));

  REQUIRE(aligned_near.samples.size() == aligned_offset.samples.size());
  for (std::size_t i = 0; i < aligned_near.samples.size(); ++i) {
    REQUIRE(aligned_near.samples.at(i).x ==
            Catch::Approx(aligned_offset.samples.at(i).x));
    REQUIRE(aligned_near.samples.at(i).y ==
            Catch::Approx(aligned_offset.samples.at(i).y));
    REQUIRE(aligned_near.samples.at(i).yaw ==
            Catch::Approx(aligned_offset.samples.at(i).yaw));
  }
}

TEST_CASE("loco upper planner projects root plan to radius") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{0.0, 0.0, 0.0}, {3.0, 4.0, 0.0}, {6.0, 8.0, 0.0}};

  const LocoUpperProjectionResult projected = projectRootPlanToRadius(plan, 5.0);

  REQUIRE(projected.radius_clamped);
  REQUIRE(projected.plan.samples.at(1).x == Catch::Approx(3.0));
  REQUIRE(projected.plan.samples.at(1).y == Catch::Approx(4.0));
  REQUIRE(projected.plan.samples.at(2).x == Catch::Approx(3.0));
  REQUIRE(projected.plan.samples.at(2).y == Catch::Approx(4.0));
}

TEST_CASE("loco upper planner projects radius around action start") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{10.0, 0.0, 0.0}, {10.5, 0.0, 0.0}, {12.0, 0.0, 0.0}};

  const LocoUpperProjectionResult projected =
      projectRootPlanToRadius(plan, 1.0, {10.0, 0.0, 0.0});

  REQUIRE(projected.radius_clamped);
  REQUIRE(projected.plan.samples.at(0).x == Catch::Approx(10.0));
  REQUIRE(projected.plan.samples.at(1).x == Catch::Approx(10.5));
  REQUIRE(projected.plan.samples.at(2).x == Catch::Approx(11.0));
  REQUIRE(projected.plan.samples.at(0).y == Catch::Approx(0.0));
  REQUIRE(projected.plan.samples.at(1).y == Catch::Approx(0.0));
  REQUIRE(projected.plan.samples.at(2).y == Catch::Approx(0.0));
}

TEST_CASE("loco upper planner clamps max linear velocity") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{0.0, 0.0, 0.0}, {3.0, 4.0, 0.0}};
  LocoUpperCommandLimits limits;
  limits.max_linear_velocity = 2.0;

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(plan, limits);

  REQUIRE(commands.size() == 1);
  REQUIRE(commands.at(0).vx == Catch::Approx(1.2));
  REQUIRE(commands.at(0).vy == Catch::Approx(1.6));
}

TEST_CASE("loco upper planner clamps max yaw rate") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{0.0, 0.0, 0.0}, {0.0, 0.0, 4.0}};
  LocoUpperCommandLimits limits;
  limits.max_yaw_rate = 1.5;

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(plan, limits);

  REQUIRE(commands.size() == 1);
  REQUIRE(commands.at(0).yaw_rate == Catch::Approx(1.5));
}

TEST_CASE("loco upper planner clamps linear acceleration") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  LocoUpperCommandLimits limits;
  limits.max_linear_acceleration = 2.0;

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(plan, limits);

  REQUIRE(commands.size() == 2);
  REQUIRE(commands.at(0).vx == Catch::Approx(0.0));
  REQUIRE(commands.at(1).vx == Catch::Approx(2.0));
  REQUIRE(commands.at(1).vy == Catch::Approx(0.0));
}

TEST_CASE("loco upper planner clamps yaw acceleration") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 10.0}};
  LocoUpperCommandLimits limits;
  limits.max_yaw_acceleration = 2.0;

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(plan, limits);

  REQUIRE(commands.size() == 2);
  REQUIRE(commands.at(0).yaw_rate == Catch::Approx(0.0));
  REQUIRE(commands.at(1).yaw_rate == Catch::Approx(2.0));
}

TEST_CASE("loco upper planner smooths velocity commands with trailing window") {
  LocoUpperRootPlan plan;
  plan.dt_s = 1.0;
  plan.samples = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  LocoUpperCommandLimits limits;
  limits.smoothing_window_frames = 2;

  const std::vector<LocoUpperVelocityCommand> commands =
      rootPlanToVelocityCommands(plan, limits);

  REQUIRE(commands.size() == 2);
  REQUIRE(commands.at(0).vx == Catch::Approx(0.0));
  REQUIRE(commands.at(1).vx == Catch::Approx(5.0));
  REQUIRE(commands.at(1).vy == Catch::Approx(0.0));
}

TEST_CASE("loco upper planner suppresses outward radial velocity near radius") {
  LocoUpperVelocityCommand command;
  command.vx = 1.0;
  command.vy = 2.0;

  const LocoUpperVelocityCommand limited =
      suppressOutwardRadialVelocityNearRadius({4.9, 0.0, 0.0}, command, 5.0, 0.2);

  REQUIRE(limited.radius_limit_reached);
  REQUIRE(limited.vx == Catch::Approx(0.0));
  REQUIRE(limited.vy == Catch::Approx(2.0));
}

TEST_CASE("loco upper planner suppresses radial velocity around action start") {
  LocoUpperVelocityCommand command;
  command.vx = 1.0;

  const LocoUpperVelocityCommand limited =
      suppressOutwardRadialVelocityNearRadius(
          {10.1, 0.0, 0.0}, command, 1.0, 0.2, {10.0, 0.0, 0.0});

  REQUIRE_FALSE(limited.radius_limit_reached);
  REQUIRE(limited.vx == Catch::Approx(1.0));
  REQUIRE(limited.vy == Catch::Approx(0.0));
}

TEST_CASE("loco upper planner marks radius reached only when velocity is suppressed") {
  SECTION("inward velocity") {
    LocoUpperVelocityCommand command;
    command.vx = -1.0;
    command.radius_limit_reached = true;

    const LocoUpperVelocityCommand limited =
        suppressOutwardRadialVelocityNearRadius(
            {4.9, 0.0, 0.0}, command, 5.0, 0.2);

    REQUIRE_FALSE(limited.radius_limit_reached);
    REQUIRE(limited.vx == Catch::Approx(-1.0));
    REQUIRE(limited.vy == Catch::Approx(0.0));
  }

  SECTION("tangent velocity") {
    LocoUpperVelocityCommand command;
    command.vy = 1.0;

    const LocoUpperVelocityCommand limited =
        suppressOutwardRadialVelocityNearRadius(
            {4.9, 0.0, 0.0}, command, 5.0, 0.2);

    REQUIRE_FALSE(limited.radius_limit_reached);
    REQUIRE(limited.vx == Catch::Approx(0.0));
    REQUIRE(limited.vy == Catch::Approx(1.0));
  }
}

TEST_CASE("loco upper planner converts world velocity command to body frame") {
  LocoUpperVelocityCommand command_world;
  command_world.vy = 2.0;
  command_world.yaw_rate = -0.5;
  command_world.radius_limit_reached = true;

  const LocoUpperVelocityCommand command_body =
      worldVelocityToBodyCommand(command_world, kPi / 2.0);

  REQUIRE(command_body.vx == Catch::Approx(2.0));
  REQUIRE(command_body.vy == Catch::Approx(0.0).margin(1.0e-12));
  REQUIRE(command_body.yaw_rate == Catch::Approx(-0.5));
  REQUIRE(command_body.radius_limit_reached);
}

TEST_CASE("loco upper planner rejects invalid root samples") {
  SECTION("NaN root position") {
    TrkTrack track = makeTrack(2);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 1.0, 0.0, 0.0);
    track.body_pos_w.values.at(track.body_pos_w.frame_size) =
        std::numeric_limits<float>::quiet_NaN();

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("finite"));
  }

  SECTION("zero root quaternion") {
    TrkTrack track = makeTrack(2);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 1.0, 0.0, 0.0);
    for (std::size_t i = 0; i < 4; ++i) {
      track.body_quat_w.values.at(i) = 0.0F;
    }

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("quaternion"));
  }
}

TEST_CASE("loco upper planner rejects invalid fps root shape and root index") {
  SECTION("invalid fps") {
    TrkTrack track = makeTrack(2);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 1.0, 0.0, 0.0);
    track.metadata.fps = 0.0;

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("fps"));
  }

  SECTION("root shape") {
    TrkTrack track = makeTrack(2);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 1.0, 0.0, 0.0);
    track.body_pos_w.frame_size = 2;
    track.body_pos_w.shape = {2, 2};
    track.body_pos_w.values.resize(track.metadata.frames * track.body_pos_w.frame_size);

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("root arrays"));
  }

  SECTION("root index") {
    TrkTrack track = makeTrack(2);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 1.0, 0.0, 0.0);
    LocoUpperPlannerOptions options;
    options.root_body_index = std::numeric_limits<std::size_t>::max();

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("root body index"));
  }
}

TEST_CASE("loco upper planner rejects empty one-frame and too-short tracks") {
  SECTION("empty") {
    const TrkTrack track = makeTrack(0);

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("at least two frames"));
  }

  SECTION("one frame") {
    TrkTrack track = makeTrack(1);
    setRootPose(track, 0, 0.0, 0.0, 0.0);

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("at least two frames"));
  }

  SECTION("too short") {
    TrkTrack track = makeTrack(2, 1000.0);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 0.01, 0.0, 0.0);

    const LocoUpperRootPlanResult result = extractRootPlanarPath(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("too short"));
  }
}

TEST_CASE("loco upper validator extracts finite upper joint targets") {
  TrkTrack track = makeTrack(3);
  for (std::size_t frame = 0; frame < track.metadata.frames; ++frame) {
    for (std::size_t joint = kLocoUpperJointFirst;
         joint < kLocoUpperJointLastExclusive;
         ++joint) {
      track.joint_pos.values.at(frame * track.joint_pos.frame_size + joint) =
          static_cast<float>(0.1 * static_cast<double>(joint - kLocoUpperJointFirst));
    }
  }

  const LocoUpperJointValidationResult result =
      extractAndValidateUpperJointTargets(track);

  REQUIRE(result.ok());
  REQUIRE(result.plan.frames.size() == 3);
  REQUIRE(result.plan.frames.at(0).at(0) == Catch::Approx(0.0));
}

TEST_CASE("loco upper validator rejects invalid upper joint targets") {
  SECTION("non-finite joint") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.values.at(kLocoUpperJointFirst) =
        std::numeric_limits<float>::quiet_NaN();

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("finite"));
    REQUIRE(result.joint_index == kLocoUpperJointFirst);
  }

  SECTION("position limit") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.values.at(kLocoUpperJointFirst) = 2.0F;
    LocoUpperJointValidationOptions options;
    options.max_position = 1.0;

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("above limit"));
  }

  SECTION("velocity limit") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.values.at(kLocoUpperJointFirst) = 0.0F;
    track.joint_pos.values.at(track.joint_pos.frame_size + kLocoUpperJointFirst) =
        3.0F;
    LocoUpperJointValidationOptions options;
    options.max_velocity = 2.0;

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("velocity"));
  }

  SECTION("acceleration limit") {
    TrkTrack track = makeTrack(3);
    track.joint_pos.values.at(kLocoUpperJointFirst) = 0.0F;
    track.joint_pos.values.at(track.joint_pos.frame_size + kLocoUpperJointFirst) =
        0.0F;
    track.joint_pos.values.at(2 * track.joint_pos.frame_size +
                              kLocoUpperJointFirst) = 3.0F;
    LocoUpperJointValidationOptions options;
    options.max_acceleration = 2.0;

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("acceleration"));
  }
}

TEST_CASE("loco upper validator covers upper joint limit boundaries") {
  SECTION("below lower position limit at last upper joint") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.values.at(kLocoUpperJointLastExclusive - 1) = -2.0F;
    LocoUpperJointValidationOptions options;
    options.min_position = -1.0;

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("below limit"));
    REQUIRE(result.joint_index == kLocoUpperJointLastExclusive - 1);
  }

  SECTION("invalid position limit range") {
    TrkTrack track = makeTrack(2);
    LocoUpperJointValidationOptions options;
    options.min_position = 1.0;
    options.max_position = -1.0;

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("position limits"));
  }

  SECTION("invalid joint_pos shape") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.frame_size = kLocoUpperJointLastExclusive - 1;
    track.joint_pos.shape = {2, kLocoUpperJointLastExclusive - 1};
    track.joint_pos.values.resize(track.metadata.frames * track.joint_pos.frame_size);

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track);

    REQUIRE_FALSE(result.ok());
    REQUIRE_THAT(result.message, ContainsSubstring("invalid shape"));
  }

  SECTION("last upper joint is included") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.values.at(kLocoUpperJointLastExclusive - 1) = 1.25F;

    const LocoUpperJointValidationResult result =
        extractAndValidateUpperJointTargets(track);

    REQUIRE(result.ok());
    REQUIRE(result.plan.frames.at(0).at(kLocoUpperJointCount - 1) ==
            Catch::Approx(1.25));
  }
}

TEST_CASE("loco upper precheck combines root and upper joint validation") {
  LocoUpperPrecheckOptions options;
  options.max_radius_m = 1.0;
  options.upper_joint_limits = testUpperLimits();

  SECTION("valid track") {
    TrkTrack track = makeTrack(2);

    const LocoUpperPrecheckResult result = precheckLocoUpperTrack(track, options);

    REQUIRE(result.ok());
  }

  SECTION("invalid root") {
    TrkTrack track = makeTrack(2);
    track.body_pos_w.values.at(track.body_pos_w.frame_size) =
        std::numeric_limits<float>::quiet_NaN();

    const LocoUpperPrecheckResult result = precheckLocoUpperTrack(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.code == ErrorCode::TrkValidationFailed);
    REQUIRE_THAT(result.message, ContainsSubstring("root"));
  }

  SECTION("invalid upper joints") {
    TrkTrack track = makeTrack(2);
    options.upper_joint_limits = testUpperLimits();
    options.upper_joint_limits->max_positions.at(0) = 1.0;
    track.joint_pos.values.at(kLocoUpperJointFirst) = 1.5F;

    const LocoUpperPrecheckResult result = precheckLocoUpperTrack(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.code == ErrorCode::TrkValidationFailed);
    REQUIRE_THAT(result.message, ContainsSubstring("above limit"));
  }

  SECTION("velocity jump uses app-owned upper dynamic limits") {
    TrkTrack track = makeTrack(2);
    track.joint_pos.values.at(kLocoUpperJointFirst) = 0.0F;
    track.joint_pos.values.at(track.joint_pos.frame_size + kLocoUpperJointFirst) = 3.0F;

    const LocoUpperPrecheckResult result = precheckLocoUpperTrack(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.code == ErrorCode::TrkValidationFailed);
    REQUIRE_THAT(result.message, ContainsSubstring("velocity"));
  }

  SECTION("acceleration jump uses app-owned upper dynamic limits") {
    TrkTrack track = makeTrack(3);
    options.upper_joint_limits = testUpperLimits();
    options.upper_joint_limits->max_velocities.at(0) = 10.0;
    options.upper_joint_limits->max_accelerations.at(0) = 2.0;
    track.joint_pos.values.at(kLocoUpperJointFirst) = 0.0F;
    track.joint_pos.values.at(track.joint_pos.frame_size + kLocoUpperJointFirst) = 0.0F;
    track.joint_pos.values.at(2 * track.joint_pos.frame_size + kLocoUpperJointFirst) =
        3.0F;

    const LocoUpperPrecheckResult result = precheckLocoUpperTrack(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.code == ErrorCode::TrkValidationFailed);
    REQUIRE_THAT(result.message, ContainsSubstring("acceleration"));
  }

  SECTION("strict pose rejects radius clamping") {
    TrkTrack track = makeTrack(2);
    setRootPose(track, 0, 0.0, 0.0, 0.0);
    setRootPose(track, 1, 2.0, 0.0, 0.0);
    options.strict_pose = true;

    const LocoUpperPrecheckResult result = precheckLocoUpperTrack(track, options);

    REQUIRE_FALSE(result.ok());
    REQUIRE(result.code == ErrorCode::TrkValidationFailed);
    REQUIRE_THAT(result.message, ContainsSubstring("radius"));
  }
}

TEST_CASE("loco upper file precheck maps loader errors for app adapter") {
  TempDir tmp;
  TrkValidationConfig config;
  config.allowlist_dirs = {tmp.root};
  TrkLoader loader(config);
  LocoUpperPrecheckOptions options;
  options.max_radius_m = 1.0;

  const LocoUpperPrecheckResult missing =
      precheckLocoUpperTrackFile(loader, tmp.root / "missing.trk", options);

  REQUIRE_FALSE(missing.ok());
  REQUIRE(missing.code == ErrorCode::TrkFileNotFound);

  const LocoUpperPrecheckResult bad_options =
      precheckLocoUpperTrackFile(loader, tmp.root / "missing.trk", {});

  REQUIRE_FALSE(bad_options.ok());
  REQUIRE(bad_options.code == ErrorCode::RequestInvalid);
}

}  // namespace agentic_et1_tracker
