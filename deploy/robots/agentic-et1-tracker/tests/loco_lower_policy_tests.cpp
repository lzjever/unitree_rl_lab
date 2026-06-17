#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr std::uint8_t kExpectedModeMachine = 7;

std::filesystem::path repoFile(const std::string& relative) {
  return (std::filesystem::path(__FILE__).parent_path().parent_path() / relative)
      .lexically_normal();
}

class CaptureLowerPolicy final : public VelocityPolicyInference {
 public:
  Vec infer(const VelocityPolicyInputs& inputs) override {
    ++calls;
    inputs_seen.push_back(inputs);
    return raw_actions;
  }

  Vec raw_actions{0.0F, 0.05F, 0.10F, 0.15F, 0.20F, 0.25F,
                  0.30F, 0.35F, 0.40F, 0.45F, 0.50F, 0.55F};
  int calls{0};
  std::vector<VelocityPolicyInputs> inputs_seen;
};

LowStateSample readyLowState(const LocoLowerDeployConfig& config) {
  LowStateSample low;
  low.fresh = true;
  low.age_ms = 4;
  low.mode_machine = kExpectedModeMachine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low.gyro = {1.0F, -2.0F, 3.0F};
  for (std::size_t i = 0; i < config.joint_ids_map.size(); ++i) {
    const auto slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
    low.motors.at(slot).q =
        0.5F + static_cast<float>(i) + static_cast<float>(config.default_joint_pos.at(i));
    low.motors.at(slot).dq = 10.0F + static_cast<float>(i);
  }
  return low;
}

std::size_t keyboardCommandObsOffset() {
  return 3 + 3;
}

std::size_t lastActionObsOffset() {
  return 3 + 3 + 3 + kLocoLowerPolicyJointDim + kLocoLowerPolicyJointDim;
}

void requireLocoLowerError(const std::filesystem::path& path,
                           const std::string& message) {
  try {
    (void)loadLocoLowerDeployConfig(path);
    FAIL("expected LocoLowerDeployConfigError");
  } catch (const LocoLowerDeployConfigError& err) {
    REQUIRE_THAT(std::string(err.what()), ContainsSubstring(message));
  }
}

std::string readTextFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to read " + path.string());
  }
  return std::string(std::istreambuf_iterator<char>(in),
                     std::istreambuf_iterator<char>());
}

std::string yamlList(const std::vector<int>& values, const std::string& indent) {
  std::ostringstream out;
  for (const int value : values) {
    out << indent << "- " << value << '\n';
  }
  return out.str();
}

std::string listIndent(const std::string& text, std::size_t list_begin) {
  const std::size_t line_begin = text.rfind('\n', list_begin);
  const std::size_t indent_begin =
      line_begin == std::string::npos ? 0 : line_begin + 1;
  return text.substr(indent_begin, list_begin - indent_begin);
}

std::size_t listLineBegin(const std::string& text, std::size_t list_begin) {
  const std::size_t line_begin = text.rfind('\n', list_begin);
  return line_begin == std::string::npos ? 0 : line_begin + 1;
}

void replaceListAfterLabel(std::string& text,
                           const std::string& label,
                           const std::vector<int>& values) {
  const std::size_t label_pos = text.find(label);
  if (label_pos == std::string::npos) {
    throw std::runtime_error("missing yaml label " + label);
  }
  std::size_t list_begin = text.find("- ", label_pos);
  if (list_begin == std::string::npos) {
    throw std::runtime_error("missing yaml list for " + label);
  }
  std::size_t list_end = list_begin;
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    list_end = text.find('\n', list_end);
    if (list_end == std::string::npos) {
      throw std::runtime_error("short yaml list for " + label);
    }
    ++list_end;
  }
  const std::size_t replace_begin = listLineBegin(text, list_begin);
  text.replace(replace_begin,
               list_end - replace_begin,
               yamlList(values, listIndent(text, list_begin)));
}

void replaceListAfterNestedLabel(std::string& text,
                                 const std::string& parent_label,
                                 const std::string& list_label,
                                 const std::vector<int>& values) {
  const std::size_t parent_pos = text.find(parent_label);
  if (parent_pos == std::string::npos) {
    throw std::runtime_error("missing yaml parent " + parent_label);
  }
  const std::size_t label_pos = text.find(list_label, parent_pos);
  if (label_pos == std::string::npos) {
    throw std::runtime_error("missing yaml label " + list_label);
  }
  std::size_t list_begin = text.find("- ", label_pos);
  if (list_begin == std::string::npos) {
    throw std::runtime_error("missing yaml list for " + list_label);
  }
  std::size_t list_end = list_begin;
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    list_end = text.find('\n', list_end);
    if (list_end == std::string::npos) {
      throw std::runtime_error("short yaml list for " + list_label);
    }
    ++list_end;
  }
  const std::size_t replace_begin = listLineBegin(text, list_begin);
  text.replace(replace_begin,
               list_end - replace_begin,
               yamlList(values, listIndent(text, list_begin)));
}

void eraseListAfterLabel(std::string& text, const std::string& label) {
  const std::size_t label_pos = text.find(label);
  if (label_pos == std::string::npos) {
    throw std::runtime_error("missing yaml label " + label);
  }
  std::size_t list_begin = text.find("- ", label_pos);
  if (list_begin == std::string::npos) {
    throw std::runtime_error("missing yaml list for " + label);
  }
  std::size_t erase_end = list_begin;
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    erase_end = text.find('\n', erase_end);
    if (erase_end == std::string::npos) {
      throw std::runtime_error("short yaml list for " + label);
    }
    ++erase_end;
  }
  text.erase(label_pos, erase_end - label_pos);
}

std::string actionClipRows(double lo, double hi) {
  std::ostringstream out;
  out << '\n';
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    out << "    - [" << lo << ", " << hi << "]\n";
  }
  return out.str();
}

void replaceActionClip(std::string& text, const std::optional<std::string>& replacement) {
  const std::string needle = "    clip: null\n";
  const std::size_t pos = text.find(needle);
  if (pos == std::string::npos) {
    throw std::runtime_error("missing JointPositionAction clip");
  }
  text.replace(pos,
               needle.size(),
               replacement ? "    clip: " + *replacement : needle);
}

std::filesystem::path writeTempDeployConfig(const std::string& name,
                                            const std::string& text) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / ("agentic_et1_" + name + ".yaml");
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("failed to write " + path.string());
  }
  out << text;
  return path;
}

std::filesystem::path writeDeployConfigWithMaps(const std::string& name,
                                                const std::vector<int>& joint_ids_map,
                                                const std::vector<int>& sdk_joint_ids_map) {
  std::string text = readTextFile(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  replaceListAfterLabel(text, "joint_ids_map:", joint_ids_map);
  replaceListAfterLabel(text, "sdk_joint_ids_map:", sdk_joint_ids_map);
  replaceListAfterNestedLabel(text,
                              "actions:\n  JointPositionAction:",
                              "joint_ids:",
                              joint_ids_map);
  replaceListAfterLabel(text, "joint_pos_rel:", joint_ids_map);
  replaceListAfterLabel(text, "joint_vel_rel:", joint_ids_map);
  return writeTempDeployConfig(name, text);
}

}  // namespace

TEST_CASE("LocoLowerDeployConfig accepts app-owned ET1 lowobs10k deploy") {
  const LocoLowerDeployConfig config = loadLocoLowerDeployConfig(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));

  REQUIRE(config.joint_dim == kLocoLowerPolicyJointDim);
  REQUIRE(config.joint_ids_map == std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  REQUIRE(config.sdk_joint_ids_map == config.joint_ids_map);
  REQUIRE(config.policy_decimation == 10);
  REQUIRE(config.obs_history_length == 1);
  REQUIRE(config.obs_row_width == 45);
  REQUIRE(config.obs_dim == 45);
  REQUIRE(config.command_ranges.lin_vel_x.min == Catch::Approx(-0.5));
  REQUIRE(config.command_ranges.lin_vel_x.max == Catch::Approx(1.0));
  REQUIRE(config.command_ranges.lin_vel_y.min == Catch::Approx(-0.3));
  REQUIRE(config.command_ranges.lin_vel_y.max == Catch::Approx(0.3));
  REQUIRE(config.command_ranges.ang_vel_z.min == Catch::Approx(-0.2));
  REQUIRE(config.command_ranges.ang_vel_z.max == Catch::Approx(0.2));
  REQUIRE(config.stiffness.at(0) == Catch::Approx(80.0));
  REQUIRE(config.damping.at(4) == Catch::Approx(3.0));
  REQUIRE(config.default_joint_pos.at(3) == Catch::Approx(0.75));
  REQUIRE(config.action_scale.at(4) == Catch::Approx(0.3125));
  REQUIRE(config.action_offset == config.default_joint_pos);
  REQUIRE_FALSE(config.action_clip.has_value());
  REQUIRE(config.observation_terms.size() == 6);
  REQUIRE(config.observation_terms.at(0).name == "base_ang_vel");
  REQUIRE(config.observation_terms.at(0).offset == 0);
  REQUIRE(config.observation_terms.at(5).name == "last_action");
  REQUIRE(config.observation_terms.at(5).offset == 33);
}

TEST_CASE("LocoLowerDeployConfig defaults missing SDK joint map to identity") {
  std::string text = readTextFile(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  eraseListAfterLabel(text, "sdk_joint_ids_map:");

  const LocoLowerDeployConfig config =
      loadLocoLowerDeployConfig(writeTempDeployConfig("loco_lower_missing_sdk_map", text));

  REQUIRE(config.sdk_joint_ids_map ==
          std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
}

TEST_CASE("LocoLowerDeployConfig rejects missing or wrong nested obs schema") {
  requireLocoLowerError(repoFile("config/policy/velocity/v0/params/deploy.yaml"),
                        "observations.obs");
  requireLocoLowerError(repoFile("config/policy/general_tracker/params/deploy.yaml"),
                        "observations.obs");
}

TEST_CASE("LocoLowerDeployConfig rejects non-identity lower joint maps") {
  const std::vector<int> identity{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  SECTION("duplicate joint_ids_map entry") {
    requireLocoLowerError(
        writeDeployConfigWithMaps("loco_lower_duplicate_joint_map",
                                  {0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11},
                                  identity),
        "joint_ids_map");
  }

  SECTION("out-of-range joint_ids_map entry") {
    requireLocoLowerError(
        writeDeployConfigWithMaps("loco_lower_out_of_range_joint_map",
                                  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12},
                                  identity),
        "joint_ids_map");
  }

  SECTION("wrong-order joint_ids_map entry") {
    requireLocoLowerError(
        writeDeployConfigWithMaps("loco_lower_wrong_order_joint_map",
                                  {0, 1, 2, 3, 4, 5, 7, 6, 8, 9, 10, 11},
                                  identity),
        "joint_ids_map");
  }

  SECTION("duplicate sdk_joint_ids_map entry") {
    requireLocoLowerError(
        writeDeployConfigWithMaps("loco_lower_duplicate_sdk_map",
                                  identity,
                                  {0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11}),
        "sdk_joint_ids_map");
  }

  SECTION("out-of-range sdk_joint_ids_map entry") {
    requireLocoLowerError(
        writeDeployConfigWithMaps("loco_lower_out_of_range_sdk_map",
                                  identity,
                                  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12}),
        "sdk_joint_ids_map");
  }

  SECTION("wrong-order sdk_joint_ids_map entry") {
    requireLocoLowerError(
        writeDeployConfigWithMaps("loco_lower_wrong_order_sdk_map",
                                  identity,
                                  {0, 1, 2, 3, 4, 5, 7, 6, 8, 9, 10, 11}),
        "sdk_joint_ids_map");
  }
}

TEST_CASE("LocoLowerStepRunner injects command and writes lower lowcmd slots") {
  LocoLowerDeployConfig config = loadLocoLowerDeployConfig(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  config.action_scale = std::vector<double>(kLocoLowerPolicyJointDim, 0.5);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    config.action_offset.at(i) = -0.25 + 0.1 * static_cast<double>(i);
    config.stiffness.at(i) = 30.0 + static_cast<double>(i);
    config.damping.at(i) = 1.0 + 0.1 * static_cast<double>(i);
  }
  const LowStateSample low_state = readyLowState(config);
  CaptureLowerPolicy policy;
  LocoLowerStepRunner runner(config, kExpectedModeMachine);

  const VelocityCommand command{0.4F, -0.2F, 0.1F};
  for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
    const LocoLowerStepResult held = runner.step(low_state, command, policy);
    REQUIRE_FALSE(held.policy_evaluated);
  }
  const LocoLowerStepResult step = runner.step(low_state, command, policy);

  REQUIRE(policy.calls == 1);
  REQUIRE(step.policy_evaluated == true);
  REQUIRE(step.command_clamped == false);
  REQUIRE(step.action_clamped == false);
  REQUIRE(policy.inputs_seen.back().obs.size() == kLocoLowerPolicyObsDim);
  REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 0) == command.vx);
  REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 1) == command.vy);
  REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 2) ==
          command.yaw_rate);
  REQUIRE(step.raw_action == policy.raw_actions);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const auto slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
    const float expected_q =
        policy.raw_actions.at(i) * 0.5F + static_cast<float>(config.action_offset.at(i));
    REQUIRE(step.processed_action.at(i) == expected_q);
    REQUIRE(step.low_cmd.motors.at(slot).mode == 1);
    REQUIRE(step.low_cmd.motors.at(slot).q == expected_q);
    REQUIRE(step.low_cmd.motors.at(slot).dq == 0.0F);
    REQUIRE(step.low_cmd.motors.at(slot).kp == static_cast<float>(config.stiffness.at(i)));
    REQUIRE(step.low_cmd.motors.at(slot).kd == static_cast<float>(config.damping.at(i)));
    REQUIRE(step.low_cmd.motors.at(slot).tau == 0.0F);
  }
  REQUIRE(step.low_cmd.motors.at(12).kp == 0.0F);
}

TEST_CASE("LocoLowerStepRunner honors null action clip contract") {
  LocoLowerDeployConfig config = loadLocoLowerDeployConfig(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  config.action_scale = std::vector<double>(kLocoLowerPolicyJointDim, 0.25);
  config.action_offset = std::vector<double>(kLocoLowerPolicyJointDim, 0.0);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    config.joint_min_q.at(i) = -2.0;
    config.joint_max_q.at(i) = 2.0;
  }
  const LowStateSample low_state = readyLowState(config);
  CaptureLowerPolicy policy;
  policy.raw_actions = Vec(kLocoLowerPolicyJointDim, 2.0F);
  LocoLowerStepRunner runner(config, kExpectedModeMachine);

  for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
    const LocoLowerStepResult held = runner.step(low_state, VelocityCommand{}, policy);
    REQUIRE_FALSE(held.policy_evaluated);
    REQUIRE(held.raw_action == Vec(kLocoLowerPolicyJointDim, 0.0F));
    REQUIRE_FALSE(held.raw_action_clamped);
    REQUIRE_FALSE(held.action_clamped);
  }
  const LocoLowerStepResult step = runner.step(low_state, VelocityCommand{}, policy);

  REQUIRE(policy.calls == 1);
  REQUIRE(step.policy_evaluated == true);
  REQUIRE_FALSE(step.raw_action_clamped);
  REQUIRE_FALSE(step.action_clamped);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const auto slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
    const float expected_q = 2.0F * static_cast<float>(config.action_scale.at(i));
    REQUIRE(step.raw_action.at(i) == 2.0F);
    REQUIRE(step.processed_action.at(i) == expected_q);
    REQUIRE(step.low_cmd.motors.at(slot).q == expected_q);
  }
}

TEST_CASE("LocoLowerStepRunner applies configured action clip when present") {
  std::string text = readTextFile(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  replaceActionClip(text, actionClipRows(-0.5, 0.75));
  LocoLowerDeployConfig config =
      loadLocoLowerDeployConfig(writeTempDeployConfig("loco_lower_action_clip", text));
  config.action_scale = std::vector<double>(kLocoLowerPolicyJointDim, 0.25);
  config.action_offset = std::vector<double>(kLocoLowerPolicyJointDim, 0.0);

  const LowStateSample low_state = readyLowState(config);
  CaptureLowerPolicy policy;
  policy.raw_actions = Vec(kLocoLowerPolicyJointDim, 2.0F);
  LocoLowerStepRunner runner(config, kExpectedModeMachine);
  for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
    (void)runner.step(low_state, VelocityCommand{}, policy);
  }
  const LocoLowerStepResult step = runner.step(low_state, VelocityCommand{}, policy);

  REQUIRE(config.action_clip.has_value());
  REQUIRE(policy.calls == 1);
  REQUIRE(step.policy_evaluated);
  REQUIRE(step.raw_action_clamped);
  REQUIRE(step.action_clamped);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    REQUIRE(step.raw_action.at(i) == Catch::Approx(0.75F));
    REQUIRE(step.processed_action.at(i) == Catch::Approx(0.1875F));
  }
}

TEST_CASE("LocoLowerStepRunner clamps command ranges before policy inputs") {
  LocoLowerDeployConfig config = loadLocoLowerDeployConfig(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  config.command_ranges.lin_vel_x = {-0.1, 0.5};
  config.command_ranges.lin_vel_y = {-0.2, 0.2};
  config.command_ranges.ang_vel_z = {-0.3, 0.3};
  const LowStateSample low_state = readyLowState(config);

  SECTION("in range stays unclamped") {
    CaptureLowerPolicy policy;
    LocoLowerStepRunner runner(config, kExpectedModeMachine);
    const VelocityCommand command{0.4F, -0.2F, 0.1F};
    for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
      const LocoLowerStepResult held = runner.step(low_state, command, policy);
      REQUIRE_FALSE(held.policy_evaluated);
      REQUIRE_FALSE(held.command_clamped);
    }

    const LocoLowerStepResult step = runner.step(low_state, command, policy);

    REQUIRE(step.command_clamped == false);
    REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 0) ==
            command.vx);
    REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 1) ==
            command.vy);
    REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 2) ==
            command.yaw_rate);
  }

  SECTION("out of range is clamped") {
    CaptureLowerPolicy policy;
    LocoLowerStepRunner runner(config, kExpectedModeMachine);
    const VelocityCommand command{0.75F, -0.25F, 0.6F};
    for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
      const LocoLowerStepResult held = runner.step(low_state, command, policy);
      REQUIRE_FALSE(held.policy_evaluated);
      REQUIRE(held.command_clamped);
    }

    const LocoLowerStepResult step = runner.step(low_state, command, policy);

    REQUIRE(step.command_clamped == true);
    REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 0) ==
            Catch::Approx(0.5F));
    REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 1) ==
            Catch::Approx(-0.2F));
    REQUIRE(policy.inputs_seen.back().obs.at(keyboardCommandObsOffset() + 2) ==
            Catch::Approx(0.3F));
  }
}

TEST_CASE("LocoLowerLowCmdFrame rejects duplicate SDK slots before writing") {
  LocoLowerDeployConfig config = loadLocoLowerDeployConfig(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  config.sdk_joint_ids_map.at(1) = 0;

  LowCmdFrame base;
  base.motors.at(0).q = 123.0F;
  base.motors.at(0).kp = 456.0F;
  REQUIRE_THROWS_AS(makeLocoLowerLowCmdFrame(config,
                                             Vec(kLocoLowerPolicyJointDim, 0.0F),
                                             kExpectedModeMachine,
                                             &base),
                    LocoLowerPolicyError);
  REQUIRE(base.motors.at(0).q == 123.0F);
  REQUIRE(base.motors.at(0).kp == 456.0F);
}

TEST_CASE("LocoLowerStepRunner reset aligns decimation phase with held zeros") {
  const LocoLowerDeployConfig config = loadLocoLowerDeployConfig(repoFile(
      "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  const LowStateSample low_state = readyLowState(config);
  CaptureLowerPolicy policy;
  LocoLowerStepRunner runner(config, kExpectedModeMachine);

  for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
    const LocoLowerStepResult held = runner.step(low_state, VelocityCommand{}, policy);
    CAPTURE(step_index);
    REQUIRE(policy.calls == 0);
    REQUIRE(held.raw_action == Vec(kLocoLowerPolicyJointDim, 0.0F));
    REQUIRE_FALSE(held.policy_evaluated);
    REQUIRE(policy.inputs_seen.empty());
  }
  const LocoLowerStepResult first = runner.step(low_state, VelocityCommand{}, policy);
  REQUIRE(first.policy_evaluated == true);
  REQUIRE(policy.calls == 1);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    REQUIRE(policy.inputs_seen.back().obs.at(lastActionObsOffset() + i) == 0.0F);
  }
  policy.raw_actions = Vec(kLocoLowerPolicyJointDim, -0.5F);
  for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
    const LocoLowerStepResult held = runner.step(low_state, VelocityCommand{}, policy);
    REQUIRE(policy.calls == 1);
    REQUIRE(held.raw_action == first.raw_action);
    REQUIRE(held.policy_evaluated == false);
  }
  const LocoLowerStepResult decimated = runner.step(low_state, VelocityCommand{}, policy);
  REQUIRE(policy.calls == 2);
  REQUIRE(decimated.policy_evaluated == true);
  REQUIRE(decimated.raw_action == Vec(kLocoLowerPolicyJointDim, -0.5F));

  runner.reset();
  for (std::size_t step_index = 1; step_index < config.policy_decimation; ++step_index) {
    const LocoLowerStepResult held = runner.step(low_state, VelocityCommand{}, policy);
    CAPTURE(step_index);
    REQUIRE(policy.calls == 2);
    REQUIRE(held.raw_action == Vec(kLocoLowerPolicyJointDim, 0.0F));
    REQUIRE_FALSE(held.policy_evaluated);
  }
  const LocoLowerStepResult after_reset =
      runner.step(low_state, VelocityCommand{}, policy);

  REQUIRE(policy.calls == 3);
  REQUIRE(after_reset.policy_evaluated == true);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    REQUIRE(policy.inputs_seen.back().obs.at(lastActionObsOffset() + i) == 0.0F);
  }
}

TEST_CASE("LocoLower app-owned copied assets match expected sha256") {
  const auto policy_path =
      repoFile("config/policy/loco_lower/et1_low/exported/policy.onnx");
  const auto deploy_path =
      repoFile("config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml");

  REQUIRE(std::filesystem::exists(policy_path));
  REQUIRE(std::filesystem::exists(deploy_path));
  REQUIRE(sha256File(policy_path) ==
          "c76686a5b952a10eded30b87673cf098d23d469f596ad6289bbc05b81bdb5203");
  REQUIRE(sha256File(deploy_path) ==
          "b88727d5a2c7c87b2ee8f053c1a5ecf367bfaa2f1b0a54bb8362f6a0107aa48a");
}

}  // namespace agentic_et1_tracker
