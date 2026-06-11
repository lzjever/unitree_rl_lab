#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "agentic_et1_tracker/policy/observation_builder.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

using Vec = std::vector<float>;

constexpr std::size_t kJointDim = TrkSchema::kJointDim;
constexpr std::size_t kBodyCount = TrkSchema::kBodyCount;
constexpr std::size_t kAnchorBodyIndex = 14;
constexpr std::size_t kClnFootstateHistoryLength = 5;
constexpr std::size_t kClnFootstateHistoryWidth = 41;
constexpr std::size_t kClnFootstateFutureFootOffset = 35;
constexpr float kPi = 3.14159265358979323846F;

std::array<float, 4> yawQuat(float radians) {
  return {std::cos(radians * 0.5F), 0.0F, 0.0F, std::sin(radians * 0.5F)};
}

struct TestQuat {
  float w{1.0F};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

TestQuat quatFromArray(const std::array<float, 4>& values) {
  return {values[0], values[1], values[2], values[3]};
}

TestQuat multiply(TestQuat a, TestQuat b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

TestQuat conjugate(TestQuat q) {
  return {q.w, -q.x, -q.y, -q.z};
}

TestQuat angleAxisX(float radians) {
  return {std::cos(radians * 0.5F), std::sin(radians * 0.5F), 0.0F, 0.0F};
}

float yawFromQuat(TestQuat q) {
  return std::atan2(2.0F * (q.w * q.z + q.x * q.y),
                    1.0F - 2.0F * (q.y * q.y + q.z * q.z));
}

float et1AnchorOffsetYaw(const std::array<float, 4>& robot_root,
                         float joint_12,
                         float joint_13,
                         const std::array<float, 4>& ref_anchor) {
  const TestQuat robot_anchor =
      multiply(multiply(quatFromArray(robot_root), angleAxisX(joint_12)),
               quatFromArray(yawQuat(joint_13)));
  return yawFromQuat(multiply(robot_anchor, conjugate(quatFromArray(ref_anchor))));
}

Vec rootOriFromYaw(float yaw) {
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  return {c, -s, s, c, 0.0F, 0.0F};
}

Vec rotateByYaw(float yaw, float x, float y, float z) {
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  return {c * x - s * y, s * x + c * y, z};
}

Vec seq(float start, std::size_t count) {
  Vec values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(start + static_cast<float>(i));
  }
  return values;
}

std::vector<double> doubleSeq(double start, double step, std::size_t count) {
  std::vector<double> values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(start + step * static_cast<double>(i));
  }
  return values;
}

std::vector<int> frozenSdkMap() {
  return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
          13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30};
}

DeployConfig validConfig() {
  DeployConfig config;
  config.joint_dim = kJointDim;
  config.sdk_joint_ids_map = frozenSdkMap();
  config.default_joint_pos = doubleSeq(0.25, 0.5, kJointDim);
  return config;
}

DeployConfig validClnFootstateConfig() {
  DeployConfig config = validConfig();
  config.observation_contract = ObservationContract::GeneralTrackerCLNFootstate;
  config.obs_history_width = kClnFootstateHistoryWidth;
  config.obs_history_length = kClnFootstateHistoryLength;
  return config;
}

ObservationBuilderConfig motionCommandConfig() {
  ObservationBuilderConfig config;
  config.use_motion_root_command = true;
  config.use_motion_velocity_command = true;
  return config;
}

struct FrameStorage {
  std::array<float, kJointDim> joint_pos{};
  std::array<float, kJointDim> joint_vel{};
  std::array<float, kBodyCount * 3> body_pos_w{};
  std::array<float, kBodyCount * 4> body_quat_w{};
  std::array<float, kBodyCount * 3> body_lin_vel_w{};
  std::array<float, kBodyCount * 3> body_ang_vel_w{};
  std::array<std::int64_t, 1> left_contact{{0}};
  std::array<std::int64_t, 1> right_contact{{0}};
  std::array<float, 3> ref_com_rel_navi{};
  std::array<float, 3> ref_com_vel_navi{};

  FrameStorage() {
    const auto identity = yawQuat(0.0F);
    for (std::size_t body = 0; body < kBodyCount; ++body) {
      for (std::size_t i = 0; i < 4; ++i) {
        body_quat_w[body * 4 + i] = identity[i];
      }
    }
  }

  void setRootQuat(const std::array<float, 4>& quat_wxyz) {
    for (std::size_t i = 0; i < quat_wxyz.size(); ++i) {
      body_quat_w[i] = quat_wxyz[i];
    }
  }

  void setBodyQuat(std::size_t body, const std::array<float, 4>& quat_wxyz) {
    const std::size_t offset = body * 4;
    for (std::size_t i = 0; i < quat_wxyz.size(); ++i) {
      body_quat_w[offset + i] = quat_wxyz[i];
    }
  }

  void setRootLinVel(float x, float y, float z) {
    body_lin_vel_w[0] = x;
    body_lin_vel_w[1] = y;
    body_lin_vel_w[2] = z;
  }

  void setRootAngVel(float x, float y, float z) {
    body_ang_vel_w[0] = x;
    body_ang_vel_w[1] = y;
    body_ang_vel_w[2] = z;
  }

  TrkFrameView view() const {
    TrkFrameView out;
    out.joint_pos = {joint_pos.data(), joint_pos.size()};
    out.joint_vel = {joint_vel.data(), joint_vel.size()};
    out.body_pos_w = {body_pos_w.data(), body_pos_w.size()};
    out.body_quat_w = {body_quat_w.data(), body_quat_w.size()};
    out.body_lin_vel_w = {body_lin_vel_w.data(), body_lin_vel_w.size()};
    out.body_ang_vel_w = {body_ang_vel_w.data(), body_ang_vel_w.size()};
    out.left_foot_contact_state = {left_contact.data(), left_contact.size()};
    out.right_foot_contact_state = {right_contact.data(), right_contact.size()};
    out.ref_com_rel_navi = {ref_com_rel_navi.data(), ref_com_rel_navi.size()};
    out.ref_com_vel_navi = {ref_com_vel_navi.data(), ref_com_vel_navi.size()};
    return out;
  }
};

LowStateSample liveState(const std::array<float, 4>& quat_wxyz) {
  LowStateSample low;
  low.quat_wxyz = quat_wxyz;
  return low;
}

void fillPolicyMotors(LowStateSample& low, const std::vector<int>& sdk_map) {
  for (std::size_t policy_index = 0; policy_index < sdk_map.size(); ++policy_index) {
    const int sdk_index = sdk_map[policy_index];
    low.motors.at(static_cast<std::size_t>(sdk_index)).q =
        10.0F + static_cast<float>(policy_index);
    low.motors.at(static_cast<std::size_t>(sdk_index)).dq =
        20.0F + static_cast<float>(policy_index);
  }
}

void requireVecApprox(const Vec& actual, const Vec& expected, float margin = 1.0e-5F) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    REQUIRE(actual[i] == Catch::Approx(expected[i]).margin(margin));
  }
}

void requireSeqApprox(const Vec& actual, float start, std::size_t count) {
  requireVecApprox(actual, seq(start, count));
}

void resizeFloatArray(TrkFloatArray& array,
                      std::size_t frames,
                      std::size_t frame_size) {
  array.frame_size = frame_size;
  array.values.assign(frames * frame_size, 0.0F);
}

void resizeContactArray(TrkContactArray& array,
                        std::size_t frames,
                        std::size_t frame_size) {
  array.frame_size = frame_size;
  array.values.assign(frames * frame_size, 0);
}

float frameJointBase(std::size_t frame_index) {
  return 100.0F + 100.0F * static_cast<float>(frame_index);
}

void fillTrackFrame(TrkTrack& track,
                    std::size_t frame_index,
                    std::int64_t left_contact,
                    std::int64_t right_contact) {
  const std::size_t joint_offset = frame_index * track.joint_pos.frame_size;
  const std::size_t body_quat_offset = frame_index * track.body_quat_w.frame_size;
  const std::size_t body_lin_offset = frame_index * track.body_lin_vel_w.frame_size;
  const std::size_t body_ang_offset = frame_index * track.body_ang_vel_w.frame_size;
  const float base = frameJointBase(frame_index);

  for (std::size_t i = 0; i < kJointDim; ++i) {
    track.joint_pos.values.at(joint_offset + i) = base + static_cast<float>(i);
  }
  for (std::size_t body = 0; body < kBodyCount; ++body) {
    track.body_quat_w.values.at(body_quat_offset + body * 4) = 1.0F;
  }
  track.body_lin_vel_w.values.at(body_lin_offset) = 1.0F + static_cast<float>(frame_index);
  track.body_lin_vel_w.values.at(body_lin_offset + 1) =
      -2.0F - static_cast<float>(frame_index);
  track.body_ang_vel_w.values.at(body_ang_offset + 2) =
      0.25F + static_cast<float>(frame_index);
  track.left_foot_contact_state.values.at(frame_index) = left_contact;
  track.right_foot_contact_state.values.at(frame_index) = right_contact;
}

TrkTrack makeTrack(std::size_t frames) {
  TrkTrack track;
  track.metadata.frames = frames;
  track.metadata.fps = TrkSchema::kDefaultFps;
  resizeFloatArray(track.joint_pos, frames, kJointDim);
  resizeFloatArray(track.joint_vel, frames, kJointDim);
  resizeFloatArray(track.body_pos_w, frames, kBodyCount * 3);
  resizeFloatArray(track.body_quat_w, frames, kBodyCount * 4);
  resizeFloatArray(track.body_lin_vel_w, frames, kBodyCount * 3);
  resizeFloatArray(track.body_ang_vel_w, frames, kBodyCount * 3);
  resizeContactArray(track.left_foot_contact_state, frames, 1);
  resizeContactArray(track.right_foot_contact_state, frames, 1);
  resizeFloatArray(track.ref_com_rel_navi, frames, 3);
  resizeFloatArray(track.ref_com_vel_navi, frames, 3);

  for (std::size_t frame = 0; frame < frames; ++frame) {
    fillTrackFrame(track,
                   frame,
                   static_cast<std::int64_t>(frame % 3),
                   static_cast<std::int64_t>((frame + 1) % 3));
  }
  return track;
}

void setTrackBodyQuat(TrkTrack& track,
                      std::size_t frame_index,
                      std::size_t body_index,
                      const std::array<float, 4>& quat_wxyz) {
  const std::size_t offset =
      frame_index * track.body_quat_w.frame_size + body_index * 4;
  for (std::size_t i = 0; i < quat_wxyz.size(); ++i) {
    track.body_quat_w.values.at(offset + i) = quat_wxyz[i];
  }
}

void requireSliceApprox(const Vec& actual,
                        std::size_t offset,
                        const Vec& expected,
                        float margin = 1.0e-5F) {
  REQUIRE(offset + expected.size() <= actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    REQUIRE(actual[offset + i] == Catch::Approx(expected[i]).margin(margin));
  }
}

}  // namespace

TEST_CASE("referenceFrameIndex rounds elapsed frames and clamps to valid track bounds") {
  REQUIRE(referenceFrameIndex(0.0, 50.0, 7) == 0);
  REQUIRE(referenceFrameIndex(0.009, 50.0, 7) == 0);
  REQUIRE(referenceFrameIndex(0.011, 50.0, 7) == 1);
  REQUIRE(referenceFrameIndex(-0.25, 50.0, 7) == 0);
  REQUIRE(referenceFrameIndex(99.0, 50.0, 7) == 6);
}

TEST_CASE("ObservationBuilder emits identity reference and live observation parts") {
  DeployConfig config = validConfig();
  FrameStorage first;
  FrameStorage frame;
  LowStateSample low = liveState(yawQuat(0.0F));
  low.gyro = {0.3F, -0.4F, 0.5F};
  fillPolicyMotors(low, config.sdk_joint_ids_map);

  for (std::size_t i = 0; i < kJointDim; ++i) {
    frame.joint_pos[i] = 100.0F + static_cast<float>(i);
  }
  frame.setRootLinVel(1.5F, -2.0F, 9.0F);
  frame.setRootAngVel(7.0F, 8.0F, -0.75F);
  frame.left_contact[0] = 1;
  frame.right_contact[0] = 2;
  frame.ref_com_rel_navi = {0.1F, 0.2F, 0.3F};
  frame.ref_com_vel_navi = {-0.1F, -0.2F, -0.3F};
  const Vec last_action = seq(-1.0F, kJointDim);

  const ObservationBuilderState state =
      makeObservationBuilderState(first.view(), low, motionCommandConfig());
  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, last_action, state, motionCommandConfig());

  requireVecApprox(parts.command_yaw, {1.0F, 0.0F});
  requireVecApprox(parts.command_root_ori_b, {1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F});
  requireVecApprox(parts.command_xy_yaw_vel, {1.5F, -2.0F, -0.75F});
  requireSeqApprox(parts.command_jnt_pos, 100.0F, kJointDim);
  requireVecApprox(parts.command_foot_support_state,
                   {0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F});
  requireVecApprox(parts.ref_com_rel_navi, {0.1F, 0.2F, 0.3F});
  requireVecApprox(parts.ref_com_vel_navi, {-0.1F, -0.2F, -0.3F});
  requireVecApprox(parts.projected_gravity, {0.0F, 0.0F, -1.0F});
  requireVecApprox(parts.base_ang_vel, {0.3F, -0.4F, 0.5F});
  requireVecApprox(parts.last_action, last_action);

  Vec expected_q_rel;
  Vec expected_dq;
  expected_q_rel.reserve(kJointDim);
  expected_dq.reserve(kJointDim);
  for (std::size_t i = 0; i < kJointDim; ++i) {
    expected_q_rel.push_back(10.0F + static_cast<float>(i) -
                             static_cast<float>(config.default_joint_pos[i]));
    expected_dq.push_back(20.0F + static_cast<float>(i));
  }
  requireVecApprox(parts.joint_pos_rel, expected_q_rel);
  requireVecApprox(parts.joint_vel_rel, expected_dq);
}

TEST_CASE("ObservationBuilder emits CLNFootstate current foot support one-hot") {
  DeployConfig config = validClnFootstateConfig();
  FrameStorage first;
  FrameStorage frame;
  LowStateSample low = liveState(yawQuat(0.0F));
  fillPolicyMotors(low, config.sdk_joint_ids_map);
  frame.setRootQuat(yawQuat(kPi * 0.5F));
  frame.setRootLinVel(1.25F, -2.5F, 0.0F);
  frame.setRootAngVel(0.0F, 0.0F, 3.75F);
  frame.left_contact[0] = 2;
  frame.right_contact[0] = 1;

  const ObservationBuilderState state =
      makeObservationBuilderState(first.view(), low, ObservationBuilderConfig{});
  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim), state);

  requireVecApprox(parts.command_yaw, {0.0F, 1.0F});
  requireVecApprox(parts.command_root_ori_b, {1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F});
  requireVecApprox(parts.command_xy_yaw_vel, {0.0F, 0.0F, 0.0F});
  requireVecApprox(parts.command_foot_support_state,
                   {0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F});
}

TEST_CASE("ObservationBuilder keeps CLNFootstate future motion root velocity foot support and clamps to final frame") {
  const DeployConfig config = validClnFootstateConfig();
  TrkTrack track = makeTrack(3);
  track.left_foot_contact_state.values.at(2) = 2;
  track.right_foot_contact_state.values.at(2) = 0;
  const std::size_t frame_2_quat_offset = 2 * track.body_quat_w.frame_size;
  const auto frame_2_quat = yawQuat(kPi * 0.5F);
  for (std::size_t i = 0; i < frame_2_quat.size(); ++i) {
    track.body_quat_w.values.at(frame_2_quat_offset + i) = frame_2_quat[i];
  }
  const std::size_t frame_2_lin_offset = 2 * track.body_lin_vel_w.frame_size;
  const std::size_t frame_2_ang_offset = 2 * track.body_ang_vel_w.frame_size;
  track.body_lin_vel_w.values.at(frame_2_lin_offset) = 4.0F;
  track.body_lin_vel_w.values.at(frame_2_lin_offset + 1) = -5.0F;
  track.body_ang_vel_w.values.at(frame_2_ang_offset + 2) = 6.0F;
  const LowStateSample low = liveState(yawQuat(0.0F));
  const ObservationBuilderState state =
      makeObservationBuilderState(*track.frame(0), low, ObservationBuilderConfig{});

  const PolicyObservationParts parts =
      buildObservationParts(config, track, 1, low, seq(0.0F, kJointDim), state);

  REQUIRE(parts.future_commands.size() ==
          kClnFootstateHistoryLength * kClnFootstateHistoryWidth);
  requireSliceApprox(parts.future_commands, 0, {0.0F, -1.0F, 1.0F, 0.0F, 0.0F, 0.0F});
  requireSliceApprox(parts.future_commands, 6, {-5.0F, -4.0F, 6.0F});
  requireSliceApprox(parts.future_commands, 9, seq(frameJointBase(2), kJointDim));
  requireSliceApprox(parts.future_commands,
                     kClnFootstateFutureFootOffset,
                     {0.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F});
  requireSliceApprox(parts.future_commands,
                     (kClnFootstateHistoryLength - 1) * kClnFootstateHistoryWidth,
                     {0.0F, -1.0F, 1.0F, 0.0F, 0.0F, 0.0F});
  requireSliceApprox(parts.future_commands,
                     (kClnFootstateHistoryLength - 1) * kClnFootstateHistoryWidth + 6,
                     {-5.0F, -4.0F, 6.0F});
  requireSliceApprox(parts.future_commands,
                     (kClnFootstateHistoryLength - 1) * kClnFootstateHistoryWidth + 9,
                     seq(frameJointBase(2), kJointDim));
  requireSliceApprox(parts.future_commands,
                     (kClnFootstateHistoryLength - 1) * kClnFootstateHistoryWidth +
                         kClnFootstateFutureFootOffset,
                         {0.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F});
}

TEST_CASE("ObservationBuilder rejects invalid CLNFootstate future contact state") {
  const DeployConfig config = validClnFootstateConfig();
  TrkTrack track = makeTrack(3);
  track.left_foot_contact_state.values.at(2) = 3;
  const LowStateSample low = liveState(yawQuat(0.0F));
  const ObservationBuilderState state =
      makeObservationBuilderState(*track.frame(0), low, ObservationBuilderConfig{});

  REQUIRE_THROWS_AS(buildObservationParts(config, track, 1, low, seq(0.0F, kJointDim), state),
                    ObservationBuilderError);
}

TEST_CASE("ObservationBuilder applies ET1 anchor yaw alignment to root orientation and reference velocity") {
  DeployConfig config = validConfig();
  FrameStorage first;
  first.setRootQuat(yawQuat(kPi * 0.5F));

  LowStateSample entry_low = liveState(yawQuat(kPi / 6.0F));
  const ObservationBuilderState state =
      makeObservationBuilderState(first.view(), entry_low, motionCommandConfig());

  FrameStorage frame;
  frame.setRootQuat(yawQuat(5.0F * kPi / 6.0F));
  frame.setRootLinVel(2.0F, 3.0F, 4.0F);
  frame.setRootAngVel(0.0F, 0.0F, 1.25F);
  LowStateSample low = liveState(yawQuat(5.0F * kPi / 18.0F));
  fillPolicyMotors(low, config.sdk_joint_ids_map);

  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim), state,
                            motionCommandConfig());

  const float relative_yaw = 13.0F * kPi / 18.0F;
  const float c = std::cos(relative_yaw);
  const float s = std::sin(relative_yaw);
  requireVecApprox(parts.command_yaw, {c, s});
  requireVecApprox(parts.command_root_ori_b, {c, -s, s, c, 0.0F, 0.0F});

  const Vec expected_xy = rotateByYaw(-5.0F * kPi / 6.0F, 2.0F, 3.0F, 1.25F);
  requireVecApprox(parts.command_xy_yaw_vel, expected_xy);
}

TEST_CASE("ObservationBuilder no_global_mode calibrates from body14 anchor yaw and mapped robot joints") {
  DeployConfig config = validConfig();
  config.sdk_joint_ids_map = {30, 29, 26, 25, 24, 23, 22, 19, 18, 17, 16, 15, 13,
                              12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,  0};

  FrameStorage first;
  first.setRootQuat(yawQuat(1.20F));
  const auto ref_anchor = yawQuat(-0.40F);
  first.setBodyQuat(kAnchorBodyIndex, ref_anchor);

  LowStateSample entry_low = liveState(yawQuat(0.20F));
  const float robot_joint_12 = 0.35F;
  const float robot_joint_13 = 0.55F;
  entry_low.motors.at(static_cast<std::size_t>(config.sdk_joint_ids_map.at(12))).q =
      robot_joint_12;
  entry_low.motors.at(static_cast<std::size_t>(config.sdk_joint_ids_map.at(13))).q =
      robot_joint_13;

  const ObservationBuilderState state =
      makeObservationBuilderState(config, first.view(), entry_low, motionCommandConfig());
  const float expected_offset =
      et1AnchorOffsetYaw(entry_low.quat_wxyz, robot_joint_12, robot_joint_13, ref_anchor);
  REQUIRE(state.ref_world_align_yaw == Catch::Approx(expected_offset).margin(1.0e-5F));

  FrameStorage frame;
  frame.setRootQuat(yawQuat(0.75F));
  LowStateSample current_low = liveState(yawQuat(-0.10F));
  fillPolicyMotors(current_low, config.sdk_joint_ids_map);

  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), current_low, seq(0.0F, kJointDim), state,
                            motionCommandConfig());

  const float relative_yaw = expected_offset + 0.75F - -0.10F;
  requireVecApprox(parts.command_yaw,
                   {std::cos(relative_yaw), std::sin(relative_yaw)});
  requireVecApprox(parts.command_root_ori_b, rootOriFromYaw(relative_yaw));
}

TEST_CASE("ObservationBuilder no_global_mode uses ET1 anchor alignment for forced Footstate future motion") {
  const DeployConfig config = validClnFootstateConfig();
  TrkTrack track = makeTrack(2);
  setTrackBodyQuat(track, 0, 0, yawQuat(1.10F));
  const auto ref_anchor = yawQuat(-0.30F);
  setTrackBodyQuat(track, 0, kAnchorBodyIndex, ref_anchor);
  setTrackBodyQuat(track, 1, 0, yawQuat(0.60F));
  const std::size_t frame_1_lin_offset = track.body_lin_vel_w.frame_size;
  const std::size_t frame_1_ang_offset = track.body_ang_vel_w.frame_size;
  track.body_lin_vel_w.values.at(frame_1_lin_offset + 0) = 3.0F;
  track.body_lin_vel_w.values.at(frame_1_lin_offset + 1) = -4.0F;
  track.body_ang_vel_w.values.at(frame_1_ang_offset + 2) = 1.50F;

  LowStateSample entry_low = liveState(yawQuat(0.25F));
  const float robot_joint_12 = -0.20F;
  const float robot_joint_13 = 0.45F;
  entry_low.motors.at(static_cast<std::size_t>(config.sdk_joint_ids_map.at(12))).q =
      robot_joint_12;
  entry_low.motors.at(static_cast<std::size_t>(config.sdk_joint_ids_map.at(13))).q =
      robot_joint_13;

  const ObservationBuilderState state =
      makeObservationBuilderState(config, *track.frame(0), entry_low, ObservationBuilderConfig{});
  const float expected_offset =
      et1AnchorOffsetYaw(entry_low.quat_wxyz, robot_joint_12, robot_joint_13, ref_anchor);

  const PolicyObservationParts parts =
      buildObservationParts(config, track, 0, entry_low, seq(0.0F, kJointDim), state);

  requireVecApprox(parts.command_root_ori_b,
                   {1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F});
  requireVecApprox(parts.command_xy_yaw_vel, {0.0F, 0.0F, 0.0F});

  const float future_relative_yaw = expected_offset + 0.60F - 0.25F;
  requireSliceApprox(parts.future_commands, 0, rootOriFromYaw(future_relative_yaw));
  const Vec aligned_velocity =
      rotateByYaw(expected_offset, 3.0F, -4.0F, 1.50F);
  const Vec expected_future_velocity =
      rotateByYaw(-(expected_offset + 0.60F),
                  aligned_velocity[0],
                  aligned_velocity[1],
                  aligned_velocity[2]);
  requireSliceApprox(parts.future_commands, 6, expected_future_velocity);
}

TEST_CASE("ObservationBuilder maps SDK motor slots into policy order before default subtraction") {
  DeployConfig config = validConfig();
  config.sdk_joint_ids_map = {30, 29, 26, 25, 24, 23, 22, 19, 18, 17, 16, 15, 13,
                              12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,  0};
  config.default_joint_pos = doubleSeq(-2.0, 0.25, kJointDim);

  FrameStorage frame;
  LowStateSample low = liveState(yawQuat(0.0F));
  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const int sdk_index = config.sdk_joint_ids_map[policy_index];
    low.motors.at(static_cast<std::size_t>(sdk_index)).q =
        1000.0F + static_cast<float>(sdk_index);
    low.motors.at(static_cast<std::size_t>(sdk_index)).dq =
        -1000.0F - static_cast<float>(sdk_index);
  }

  const ObservationBuilderState state =
      makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim), state);

  Vec expected_q_rel;
  Vec expected_dq;
  expected_q_rel.reserve(kJointDim);
  expected_dq.reserve(kJointDim);
  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const int sdk_index = config.sdk_joint_ids_map[policy_index];
    expected_q_rel.push_back(1000.0F + static_cast<float>(sdk_index) -
                             static_cast<float>(config.default_joint_pos[policy_index]));
    expected_dq.push_back(-1000.0F - static_cast<float>(sdk_index));
  }
  requireVecApprox(parts.joint_pos_rel, expected_q_rel);
  requireVecApprox(parts.joint_vel_rel, expected_dq);
}

TEST_CASE("ObservationBuilder rejects invalid input contracts") {
  DeployConfig config = validConfig();
  FrameStorage frame;
  LowStateSample low = liveState(yawQuat(0.0F));
  fillPolicyMotors(low, config.sdk_joint_ids_map);

  SECTION("zero reference root quaternion") {
    frame.setRootQuat({0.0F, 0.0F, 0.0F, 0.0F});
    REQUIRE_THROWS_AS(
        makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{}),
        ObservationBuilderError);
  }

  SECTION("zero live root quaternion") {
    LowStateSample bad_low = low;
    bad_low.quat_wxyz = {0.0F, 0.0F, 0.0F, 0.0F};
    const ObservationBuilderState state =
        makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
    REQUIRE_THROWS_AS(buildObservationParts(config, frame.view(), bad_low, seq(0.0F, kJointDim),
                                            state),
                      ObservationBuilderError);
  }

  SECTION("invalid contact values") {
    frame.left_contact[0] = 3;
    const ObservationBuilderState state =
        makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
    REQUIRE_THROWS_AS(buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim),
                                            state),
                      ObservationBuilderError);
  }

  SECTION("last action size") {
    const ObservationBuilderState state =
        makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
    REQUIRE_THROWS_AS(buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim - 1),
                                            state),
                      ObservationBuilderError);
  }

  SECTION("default joint position size") {
    config.default_joint_pos.pop_back();
    const ObservationBuilderState state =
        makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
    REQUIRE_THROWS_AS(buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim),
                                            state),
                      ObservationBuilderError);
  }

}

}  // namespace agentic_et1_tracker
