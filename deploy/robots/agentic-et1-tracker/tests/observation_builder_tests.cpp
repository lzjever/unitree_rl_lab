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
constexpr float kPi = 3.14159265358979323846F;

std::array<float, 4> yawQuat(float radians) {
  return {std::cos(radians * 0.5F), 0.0F, 0.0F, std::sin(radians * 0.5F)};
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
      makeObservationBuilderState(first.view(), low, ObservationBuilderConfig{});
  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, last_action, state);

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

TEST_CASE("ObservationBuilder applies yaw bias to root orientation and reference velocity") {
  DeployConfig config = validConfig();
  FrameStorage first;
  first.setRootQuat(yawQuat(kPi * 0.5F));

  LowStateSample entry_low = liveState(yawQuat(kPi / 6.0F));
  const ObservationBuilderState state =
      makeObservationBuilderState(first.view(), entry_low, ObservationBuilderConfig{});

  FrameStorage frame;
  frame.setRootQuat(yawQuat(5.0F * kPi / 6.0F));
  frame.setRootLinVel(2.0F, 3.0F, 4.0F);
  frame.setRootAngVel(0.0F, 0.0F, 1.25F);
  LowStateSample low = liveState(yawQuat(5.0F * kPi / 18.0F));
  fillPolicyMotors(low, config.sdk_joint_ids_map);

  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim), state);

  const float relative_yaw = 2.0F * kPi / 9.0F;
  const float c = std::cos(relative_yaw);
  const float s = std::sin(relative_yaw);
  requireVecApprox(parts.command_root_ori_b, {c, -s, s, c, 0.0F, 0.0F});

  const float nav_yaw = kPi / 3.0F;
  const float first_ref_yaw = kPi * 0.5F;
  const float aligned_x = std::cos(-first_ref_yaw) * 2.0F -
                          std::sin(-first_ref_yaw) * 3.0F;
  const float aligned_y = std::sin(-first_ref_yaw) * 2.0F +
                          std::cos(-first_ref_yaw) * 3.0F;
  const float expected_x =
      std::cos(-nav_yaw) * aligned_x - std::sin(-nav_yaw) * aligned_y;
  const float expected_y =
      std::sin(-nav_yaw) * aligned_x + std::cos(-nav_yaw) * aligned_y;
  requireVecApprox(parts.command_xy_yaw_vel, {expected_x, expected_y, 1.25F});
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

TEST_CASE("ObservationBuilder overrides configured joint_pos_rel from reference joint_pos") {
  DeployConfig config = validConfig();
  config.override_joint_ids = {24, 25};

  FrameStorage frame;
  LowStateSample low = liveState(yawQuat(0.0F));
  fillPolicyMotors(low, config.sdk_joint_ids_map);
  for (std::size_t i = 0; i < kJointDim; ++i) {
    frame.joint_pos[i] = 1000.0F + static_cast<float>(i);
  }

  const std::size_t head_yaw_sdk =
      static_cast<std::size_t>(config.sdk_joint_ids_map.at(24));
  const std::size_t head_pitch_sdk =
      static_cast<std::size_t>(config.sdk_joint_ids_map.at(25));
  low.motors.at(head_yaw_sdk).q = -300.0F;
  low.motors.at(head_pitch_sdk).q = -400.0F;

  const ObservationBuilderState state =
      makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
  const PolicyObservationParts parts =
      buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim), state);

  Vec expected_q_rel;
  expected_q_rel.reserve(kJointDim);
  for (std::size_t i = 0; i < kJointDim; ++i) {
    const float source_q =
        i == 24 || i == 25 ? frame.joint_pos[i] : 10.0F + static_cast<float>(i);
    expected_q_rel.push_back(source_q -
                             static_cast<float>(config.default_joint_pos.at(i)));
  }
  requireVecApprox(parts.joint_pos_rel, expected_q_rel);
  REQUIRE(parts.joint_vel_rel.at(24) == 20.0F + 24.0F);
  REQUIRE(parts.joint_vel_rel.at(25) == 20.0F + 25.0F);
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

  SECTION("override joint id outside policy order") {
    config.override_joint_ids = {26};
    const ObservationBuilderState state =
        makeObservationBuilderState(frame.view(), low, ObservationBuilderConfig{});
    REQUIRE_THROWS_AS(buildObservationParts(config, frame.view(), low, seq(0.0F, kJointDim),
                                            state),
                      ObservationBuilderError);
  }
}

}  // namespace agentic_et1_tracker
