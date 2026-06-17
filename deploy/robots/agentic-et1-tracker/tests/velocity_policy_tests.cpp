#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/policy_io_contract.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr std::uint8_t kExpectedModeMachine = 7;

std::filesystem::path repoFile(const std::string& relative) {
  return (std::filesystem::path(__FILE__).parent_path().parent_path() / relative)
      .lexically_normal();
}

VelocityDeployConfig minimalVelocityConfig() {
  VelocityDeployConfig config;
  config.joint_dim = kVelocityPolicyJointDim;
  config.joint_ids_map = {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11};
  config.stiffness = std::vector<double>(kVelocityPolicyJointDim, 20.0);
  config.damping = std::vector<double>(kVelocityPolicyJointDim, 0.5);
  config.default_joint_pos = std::vector<double>(kVelocityPolicyJointDim, 0.0);
  config.action_scale = std::vector<double>(kVelocityPolicyJointDim, 0.25);
  config.action_offset = std::vector<double>(kVelocityPolicyJointDim, -0.1);
  config.observation_terms = {
      {"base_ang_vel", 3, 0, {0.2, 0.2, 0.2}},
      {"projected_gravity", 3, 3, {1.0, 1.0, 1.0}},
      {"keyboard_velocity_commands", 3, 6, {1.0, 1.0, 1.0}},
      {"joint_pos_rel", kVelocityPolicyJointDim, 9,
       std::vector<double>(kVelocityPolicyJointDim, 1.0)},
      {"joint_vel_rel", kVelocityPolicyJointDim, 21,
       std::vector<double>(kVelocityPolicyJointDim, 0.05)},
      {"last_action", kVelocityPolicyJointDim, 33,
       std::vector<double>(kVelocityPolicyJointDim, 1.0)},
  };
  config.obs_row_width = kVelocityPolicyObsRowWidth;
  config.obs_history_length = kVelocityPolicyHistoryLength;
  config.obs_dim = kVelocityPolicyObsDim;
  config.step_dt = 0.02;
  return config;
}

PolicyModelMetadata validVelocityMetadata() {
  return {
      {{"obs", PolicyTensorElementType::Float32,
        {1, static_cast<std::int64_t>(kVelocityPolicyObsDim)}}},
      {{"actions", PolicyTensorElementType::Float32,
        {1, static_cast<std::int64_t>(kVelocityPolicyJointDim)}}},
  };
}

class CaptureVelocityPolicy final : public VelocityPolicyInference {
 public:
  Vec infer(const VelocityPolicyInputs& inputs) override {
    ++calls;
    inputs_seen.push_back(inputs);
    return raw_actions;
  }

  Vec raw_actions{0.0F, 0.1F, 0.2F, 0.3F, 0.4F, 0.5F,
                  0.6F, 0.7F, 0.8F, 0.9F, 1.0F, 1.1F};
  int calls{0};
  std::vector<VelocityPolicyInputs> inputs_seen;
};

LowStateSample readyLowState(const VelocityDeployConfig& config) {
  LowStateSample low;
  low.fresh = true;
  low.age_ms = 4;
  low.mode_machine = kExpectedModeMachine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low.gyro = {1.0F, -2.0F, 3.0F};
  for (std::size_t i = 0; i < config.joint_ids_map.size(); ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.joint_ids_map.at(i));
    low.motors.at(sdk_slot).q = 0.5F + static_cast<float>(i);
    low.motors.at(sdk_slot).dq = 10.0F + static_cast<float>(i);
  }
  return low;
}

void requireTermHistory(const Vec& obs, std::size_t offset, const Vec& values) {
  for (std::size_t h = 0; h < kVelocityPolicyHistoryLength; ++h) {
    for (std::size_t i = 0; i < values.size(); ++i) {
      REQUIRE(obs.at(offset + h * values.size() + i) == values.at(i));
    }
  }
}

Vec velocityCommandValues(const VelocityCommand& command) {
  return {command.vx, command.vy, command.yaw_rate};
}

std::size_t keyboardCommandObsOffset() {
  return kVelocityPolicyHistoryLength * (3 + 3);
}

std::size_t lastActionObsOffset() {
  return kVelocityPolicyHistoryLength *
         (3 + 3 + 3 + kVelocityPolicyJointDim + kVelocityPolicyJointDim);
}

}  // namespace

TEST_CASE("VelocityDeployConfig accepts app-owned ET1 velocity deploy") {
  const VelocityDeployConfig config = loadVelocityDeployConfig(
      repoFile("config/policy/velocity/v0/params/deploy.yaml"));

  REQUIRE(config.joint_dim == kVelocityPolicyJointDim);
  REQUIRE(config.joint_ids_map == std::vector<int>{0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11});
  REQUIRE(config.sdk_joint_ids_map.empty());
  REQUIRE(config.stiffness.size() == kVelocityPolicyJointDim);
  REQUIRE(config.damping.size() == kVelocityPolicyJointDim);
  REQUIRE(config.default_joint_pos.size() == kVelocityPolicyJointDim);
  REQUIRE(config.action_scale.size() == kVelocityPolicyJointDim);
  REQUIRE(config.action_offset.size() == kVelocityPolicyJointDim);
  REQUIRE(config.step_dt == 0.02);
  REQUIRE(config.obs_history_length == 5);
  REQUIRE(config.obs_row_width == 45);
  REQUIRE(config.obs_dim == 225);
  REQUIRE(config.observation_terms.size() == 6);
  REQUIRE(config.observation_terms.at(0).name == "base_ang_vel");
  REQUIRE(config.observation_terms.at(0).width == 3);
  REQUIRE(config.observation_terms.at(0).offset == 0);
  REQUIRE(config.observation_terms.at(5).name == "last_action");
  REQUIRE(config.observation_terms.at(5).width == 12);
  REQUIRE(config.observation_terms.at(5).offset == 33);
}

TEST_CASE("VelocityDeployConfig rejects GeneralTracker deploy schema") {
  REQUIRE_THROWS_WITH(
      loadVelocityDeployConfig(
          repoFile("config/policy/general_tracker/params/deploy.yaml")),
      ContainsSubstring("GeneralTracker"));
}

TEST_CASE("Velocity policy IO contract accepts single obs input and 12 actions output") {
  const VelocityDeployConfig config = minimalVelocityConfig();

  REQUIRE_NOTHROW(validateVelocityDeployConfig(config));
  REQUIRE_NOTHROW(validateVelocityPolicyIoContract(config, validVelocityMetadata()));

  auto ga_metadata = validVelocityMetadata();
  ga_metadata.inputs = {
      {"obs_current", PolicyTensorElementType::Float32, {1, 131}},
      {"obs_history", PolicyTensorElementType::Float32, {1, 25, 105}},
  };
  ga_metadata.outputs = {
      {"actions", PolicyTensorElementType::Float32, {1, 26}},
  };
  REQUIRE_THROWS_WITH(validateVelocityPolicyIoContract(config, ga_metadata),
                      ContainsSubstring("input count"));
}

TEST_CASE("Velocity policy inputs accept explicit velocity command and preserve zero default") {
  const VelocityDeployConfig config = minimalVelocityConfig();
  const LowStateSample low_state = readyLowState(config);
  const Vec last_action(kVelocityPolicyJointDim, 0.25F);

  const VelocityPolicyInputs default_inputs =
      makeVelocityPolicyInputs(config, low_state, last_action);
  const VelocityPolicyInputs explicit_zero_inputs =
      makeVelocityPolicyInputs(config, low_state, last_action, VelocityCommand{});
  REQUIRE(explicit_zero_inputs.obs == default_inputs.obs);

  const VelocityCommand command{1.25F, -0.5F, 0.75F};
  const VelocityPolicyInputs commanded_inputs =
      makeVelocityPolicyInputs(config, low_state, last_action, command);

  requireTermHistory(commanded_inputs.obs, keyboardCommandObsOffset(),
                     velocityCommandValues(command));
}

TEST_CASE("StandbyVelocity runner builds zero-command obs and maps 12 actions to SDK slots") {
  VelocityDeployConfig config = minimalVelocityConfig();
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    config.stiffness.at(i) = 30.0 + static_cast<double>(i);
    config.damping.at(i) = 1.0 + static_cast<double>(i) * 0.1;
    config.default_joint_pos.at(i) = 0.25 * static_cast<double>(i);
    config.action_offset.at(i) = -0.5 + 0.1 * static_cast<double>(i);
  }
  const LowStateSample low_state = readyLowState(config);
  CaptureVelocityPolicy policy;
  VelocityStepRunner runner(config, kExpectedModeMachine);

  const VelocityStepResult step = runner.step(low_state, policy);

  REQUIRE(policy.calls == 1);
  REQUIRE(policy.inputs_seen.size() == 1);
  const Vec& obs = policy.inputs_seen.back().obs;
  REQUIRE(obs.size() == kVelocityPolicyObsDim);
  std::size_t offset = 0;
  requireTermHistory(obs, offset, Vec{0.2F, -0.4F, 0.6F});
  offset += kVelocityPolicyHistoryLength * 3;
  requireTermHistory(obs, offset, Vec{0.0F, 0.0F, -1.0F});
  offset += kVelocityPolicyHistoryLength * 3;
  requireTermHistory(obs, offset, Vec{0.0F, 0.0F, 0.0F});
  offset += kVelocityPolicyHistoryLength * 3;

  Vec expected_joint_pos;
  Vec expected_joint_vel;
  expected_joint_pos.reserve(kVelocityPolicyJointDim);
  expected_joint_vel.reserve(kVelocityPolicyJointDim);
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.joint_ids_map.at(i));
    expected_joint_pos.push_back(low_state.motors.at(sdk_slot).q -
                                 static_cast<float>(config.default_joint_pos.at(i)));
    expected_joint_vel.push_back(low_state.motors.at(sdk_slot).dq * 0.05F);
  }
  requireTermHistory(obs, offset, expected_joint_pos);
  offset += kVelocityPolicyHistoryLength * kVelocityPolicyJointDim;
  requireTermHistory(obs, offset, expected_joint_vel);
  offset += kVelocityPolicyHistoryLength * kVelocityPolicyJointDim;
  requireTermHistory(obs, offset, Vec(kVelocityPolicyJointDim, 0.0F));
  REQUIRE(offset + kVelocityPolicyHistoryLength * kVelocityPolicyJointDim ==
          kVelocityPolicyObsDim);

  REQUIRE(step.raw_action == policy.raw_actions);
  REQUIRE(step.processed_action.size() == kVelocityPolicyJointDim);
  REQUIRE(step.low_cmd.mode_machine == kExpectedModeMachine);
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.joint_ids_map.at(i));
    REQUIRE(step.processed_action.at(i) ==
            policy.raw_actions.at(i) * 0.25F +
                static_cast<float>(config.action_offset.at(i)));
    const MotorCommand& motor = step.low_cmd.motors.at(sdk_slot);
    REQUIRE(motor.mode == 1);
    REQUIRE(motor.q == step.processed_action.at(i));
    REQUIRE(motor.dq == 0.0F);
    REQUIRE(motor.kp == static_cast<float>(config.stiffness.at(i)));
    REQUIRE(motor.kd == static_cast<float>(config.damping.at(i)));
    REQUIRE(motor.tau == 0.0F);
  }
  REQUIRE(step.low_cmd.motors.at(12).kp == 0.0F);
  REQUIRE(step.low_cmd.motors.at(12).kd == 0.0F);
}

TEST_CASE("StandbyVelocity runner injects explicit velocity command into obs history") {
  const VelocityDeployConfig config = minimalVelocityConfig();
  const LowStateSample low_state = readyLowState(config);
  CaptureVelocityPolicy policy;
  VelocityStepRunner runner(config, kExpectedModeMachine);
  const VelocityCommand first_command{0.8F, -0.3F, 1.1F};
  const VelocityCommand second_command{-0.2F, 0.4F, -0.6F};

  runner.step(low_state, policy, first_command);

  REQUIRE(policy.inputs_seen.size() == 1);
  requireTermHistory(policy.inputs_seen.back().obs, keyboardCommandObsOffset(),
                     velocityCommandValues(first_command));

  const Vec first_raw = policy.raw_actions;
  policy.raw_actions = Vec(kVelocityPolicyJointDim, -0.5F);
  runner.step(low_state, policy, second_command);

  REQUIRE(policy.inputs_seen.size() == 2);
  const Vec& second_obs = policy.inputs_seen.back().obs;
  for (std::size_t h = 0; h < kVelocityPolicyHistoryLength - 1; ++h) {
    for (std::size_t i = 0; i < 3; ++i) {
      REQUIRE(second_obs.at(keyboardCommandObsOffset() + h * 3 + i) ==
              velocityCommandValues(first_command).at(i));
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    REQUIRE(second_obs.at(keyboardCommandObsOffset() +
                          (kVelocityPolicyHistoryLength - 1) * 3 + i) ==
            velocityCommandValues(second_command).at(i));
  }
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    REQUIRE(second_obs.at(lastActionObsOffset() +
                          (kVelocityPolicyHistoryLength - 1) *
                              kVelocityPolicyJointDim +
                          i) == first_raw.at(i));
  }

  runner.reset();
  policy.inputs_seen.clear();
  runner.step(low_state, policy, second_command);

  REQUIRE(policy.inputs_seen.size() == 1);
  requireTermHistory(policy.inputs_seen.back().obs, keyboardCommandObsOffset(),
                     velocityCommandValues(second_command));
}

TEST_CASE("StandbyVelocity projected gravity matches ET1 inverse quaternion rotation") {
  const VelocityDeployConfig config = minimalVelocityConfig();
  LowStateSample low_state = readyLowState(config);
  low_state.quat_wxyz = {0.70710677F, 0.70710677F, 0.0F, 0.0F};
  CaptureVelocityPolicy policy;
  VelocityStepRunner runner(config, kExpectedModeMachine);

  runner.step(low_state, policy);

  const Vec& obs = policy.inputs_seen.back().obs;
  const std::size_t offset = kVelocityPolicyHistoryLength * 3;
  for (std::size_t h = 0; h < kVelocityPolicyHistoryLength; ++h) {
    REQUIRE(obs.at(offset + h * 3 + 0) == Catch::Approx(0.0F).margin(1.0e-5F));
    REQUIRE(obs.at(offset + h * 3 + 1) == Catch::Approx(-1.0F).margin(1.0e-5F));
    REQUIRE(obs.at(offset + h * 3 + 2) == Catch::Approx(0.0F).margin(1.0e-5F));
  }
}

TEST_CASE("StandbyVelocity runner overlays policy joints on a base LowCmd frame") {
  VelocityDeployConfig config = minimalVelocityConfig();
  const LowStateSample low_state = readyLowState(config);
  CaptureVelocityPolicy policy;
  VelocityStepRunner runner(config, kExpectedModeMachine);
  LowCmdFrame base;
  base.mode_machine = 3;
  base.mode_pr = 9;
  for (std::size_t i = 0; i < base.motors.size(); ++i) {
    MotorCommand& motor = base.motors.at(i);
    motor.mode = static_cast<std::uint8_t>(10 + i);
    motor.q = 100.0F + static_cast<float>(i);
    motor.dq = 200.0F + static_cast<float>(i);
    motor.kp = 300.0F + static_cast<float>(i);
    motor.kd = 400.0F + static_cast<float>(i);
    motor.tau = 500.0F + static_cast<float>(i);
  }

  const VelocityStepResult step = runner.step(low_state, policy, &base);

  REQUIRE(step.low_cmd.mode_machine == kExpectedModeMachine);
  REQUIRE(step.low_cmd.mode_pr == 0);
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.joint_ids_map.at(i));
    const MotorCommand& motor = step.low_cmd.motors.at(sdk_slot);
    REQUIRE(motor.mode == 1);
    REQUIRE(motor.q == policy.raw_actions.at(i) * 0.25F +
                           static_cast<float>(config.action_offset.at(i)));
    REQUIRE(motor.kp == static_cast<float>(config.stiffness.at(i)));
    REQUIRE(motor.kd == static_cast<float>(config.damping.at(i)));
  }

  const std::size_t unmapped_slot = 12;
  REQUIRE(step.low_cmd.motors.at(unmapped_slot).mode ==
          base.motors.at(unmapped_slot).mode);
  REQUIRE(step.low_cmd.motors.at(unmapped_slot).q ==
          base.motors.at(unmapped_slot).q);
  REQUIRE(step.low_cmd.motors.at(unmapped_slot).dq ==
          base.motors.at(unmapped_slot).dq);
  REQUIRE(step.low_cmd.motors.at(unmapped_slot).kp ==
          base.motors.at(unmapped_slot).kp);
  REQUIRE(step.low_cmd.motors.at(unmapped_slot).kd ==
          base.motors.at(unmapped_slot).kd);
  REQUIRE(step.low_cmd.motors.at(unmapped_slot).tau ==
          base.motors.at(unmapped_slot).tau);
}

TEST_CASE("StandbyVelocity last_action history keeps previous raw action newest") {
  VelocityDeployConfig config = minimalVelocityConfig();
  const LowStateSample low_state = readyLowState(config);
  CaptureVelocityPolicy policy;
  const Vec first_raw = policy.raw_actions;
  VelocityStepRunner runner(config, kExpectedModeMachine);

  runner.step(low_state, policy);
  policy.raw_actions = Vec(kVelocityPolicyJointDim, -0.5F);
  const VelocityStepResult second = runner.step(low_state, policy);

  const Vec& obs = policy.inputs_seen.back().obs;
  const std::size_t last_action_offset =
      kVelocityPolicyHistoryLength * (3 + 3 + 3 + kVelocityPolicyJointDim +
                                      kVelocityPolicyJointDim);
  for (std::size_t h = 0; h < kVelocityPolicyHistoryLength - 1; ++h) {
    for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
      REQUIRE(obs.at(last_action_offset + h * kVelocityPolicyJointDim + i) == 0.0F);
    }
  }
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    REQUIRE(obs.at(last_action_offset +
                   (kVelocityPolicyHistoryLength - 1) * kVelocityPolicyJointDim + i) ==
            first_raw.at(i));
  }
  REQUIRE(second.raw_action == Vec(kVelocityPolicyJointDim, -0.5F));
}

TEST_CASE("StandbyVelocity maps policy joints through logical sdk slots") {
  VelocityDeployConfig config = minimalVelocityConfig();
  config.joint_ids_map = {2, 4, 1, 3, 0, 5, 11, 10, 9, 8, 7, 6};
  config.sdk_joint_ids_map = {19, 14, 22, 13, 21, 12, 20, 15, 23, 16, 18, 17};
  config.default_joint_pos = std::vector<double>(kVelocityPolicyJointDim, 1.0);

  LowStateSample low;
  low.fresh = true;
  low.age_ms = 4;
  low.mode_machine = kExpectedModeMachine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low.gyro = {0.0F, 0.0F, 0.0F};
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const auto logical = static_cast<std::size_t>(config.joint_ids_map.at(i));
    low.motors.at(logical).q = -100.0F;
    low.motors.at(logical).dq = -200.0F;
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(logical));
    low.motors.at(sdk_slot).q = 10.0F + static_cast<float>(i);
    low.motors.at(sdk_slot).dq = 20.0F + static_cast<float>(i);
  }

  CaptureVelocityPolicy policy;
  VelocityStepRunner runner(config, kExpectedModeMachine);
  const VelocityStepResult step = runner.step(low, policy);

  const Vec& obs = policy.inputs_seen.back().obs;
  const std::size_t joint_pos_offset = kVelocityPolicyHistoryLength * (3 + 3 + 3);
  const std::size_t joint_vel_offset =
      joint_pos_offset + kVelocityPolicyHistoryLength * kVelocityPolicyJointDim;
  for (std::size_t h = 0; h < kVelocityPolicyHistoryLength; ++h) {
    for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
      REQUIRE(obs.at(joint_pos_offset + h * kVelocityPolicyJointDim + i) ==
              9.0F + static_cast<float>(i));
      REQUIRE(obs.at(joint_vel_offset + h * kVelocityPolicyJointDim + i) ==
              (20.0F + static_cast<float>(i)) * 0.05F);
    }
  }

  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const auto logical = static_cast<std::size_t>(config.joint_ids_map.at(i));
    REQUIRE(step.low_cmd.motors.at(logical).kp == 0.0F);

    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(logical));
    const MotorCommand& motor = step.low_cmd.motors.at(sdk_slot);
    REQUIRE(motor.mode == 1);
    REQUIRE(motor.q == policy.raw_actions.at(i) * 0.25F +
                           static_cast<float>(config.action_offset.at(i)));
    REQUIRE(motor.kp == static_cast<float>(config.stiffness.at(i)));
    REQUIRE(motor.kd == static_cast<float>(config.damping.at(i)));
  }
}

}  // namespace agentic_et1_tracker
