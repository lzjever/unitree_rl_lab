#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/onnx_policy_runtime.hpp"

namespace agentic_et1_tracker {
namespace {

struct TempDir {
  TempDir() {
    const auto suffix =
        std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_onnx_policy_runtime_tests_" + std::to_string(suffix));
    std::filesystem::create_directories(root);
  }

  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path root;
};

std::vector<ObservationTerm> terms(
    std::initializer_list<std::pair<const char*, std::size_t>> specs) {
  std::vector<ObservationTerm> out;
  std::size_t offset = 0;
  for (const auto& spec : specs) {
    out.push_back({spec.first, spec.second, offset});
    offset += spec.second;
  }
  return out;
}

std::vector<ObservationTerm> currentTerms() {
  return terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kGaPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kGaPolicyJointDim},
      {"joint_vel_rel", kGaPolicyJointDim},
      {"last_action", kGaPolicyJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
}

std::vector<ObservationTerm> historyTerms() {
  return terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kGaPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kGaPolicyJointDim},
      {"joint_vel_rel", kGaPolicyJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
}

std::filesystem::path repoFile(const std::string& relative) {
  return (std::filesystem::path(__FILE__).parent_path().parent_path() / relative)
      .lexically_normal();
}

DeployConfig validConfig() {
  DeployConfig config;
  config.joint_dim = kGaPolicyJointDim;
  config.joint_ids_map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                          10, 11, 12, 13, 14, 15, 16, 17,
                          18, 19, 20, 21, 22, 23, 24, 25};
  config.sdk_joint_ids_map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                              10, 11, 12, 13, 15, 16, 17, 18,
                              19, 22, 23, 24, 25, 26, 29, 30};
  config.action_scale = std::vector<double>(kGaPolicyJointDim, 0.1);
  config.action_offset = std::vector<double>(kGaPolicyJointDim, 0.0);
  config.policy_kp = std::vector<double>(kGaPolicyJointDim, 1.0);
  config.policy_kd = std::vector<double>(kGaPolicyJointDim, 0.1);
  config.default_joint_pos = std::vector<double>(kGaPolicyJointDim, 0.0);
  config.obs_current_dim = kGaPolicyObsCurrentDim;
  config.obs_history_width = kGaPolicyObsHistoryWidth;
  config.obs_history_length = kGaPolicyObsHistoryLength;
  config.obs_current_terms = currentTerms();
  config.obs_history_terms = historyTerms();
  return config;
}

VelocityDeployConfig validVelocityConfig() {
  VelocityDeployConfig config;
  config.joint_dim = kVelocityPolicyJointDim;
  config.joint_ids_map = {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11};
  config.stiffness = std::vector<double>(kVelocityPolicyJointDim, 20.0);
  config.damping = std::vector<double>(kVelocityPolicyJointDim, 0.5);
  config.default_joint_pos = std::vector<double>(kVelocityPolicyJointDim, 0.0);
  config.action_scale = std::vector<double>(kVelocityPolicyJointDim, 0.25);
  config.action_offset = std::vector<double>(kVelocityPolicyJointDim, 0.0);
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

void writeText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
}

void writeMinimalGaOnnx(const std::filesystem::path& path) {
  // Constant-zero GA policy model with obs_current/obs_history inputs and actions output.
  static constexpr unsigned char kModel[] = {
      0x08, 0x07, 0x12, 0x19, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f,
      0x65, 0x74, 0x31, 0x5f, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x5f,
      0x74, 0x65, 0x73, 0x74, 0x73, 0x3a, 0x93, 0x02, 0x0a, 0x99, 0x01, 0x12,
      0x07, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x22, 0x08, 0x43, 0x6f,
      0x6e, 0x73, 0x74, 0x61, 0x6e, 0x74, 0x2a, 0x83, 0x01, 0x0a, 0x05, 0x76,
      0x61, 0x6c, 0x75, 0x65, 0x2a, 0x77, 0x08, 0x01, 0x08, 0x1a, 0x10, 0x01,
      0x22, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x05,
      0x76, 0x61, 0x6c, 0x75, 0x65, 0xa0, 0x01, 0x04, 0x12, 0x17, 0x61, 0x67,
      0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f, 0x65, 0x74, 0x31, 0x5f, 0x74, 0x65,
      0x73, 0x74, 0x5f, 0x70, 0x6f, 0x6c, 0x69, 0x63, 0x79, 0x5a, 0x1e, 0x0a,
      0x0b, 0x6f, 0x62, 0x73, 0x5f, 0x63, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74,
      0x12, 0x0f, 0x0a, 0x0d, 0x08, 0x01, 0x12, 0x09, 0x0a, 0x02, 0x08, 0x01,
      0x0a, 0x03, 0x08, 0x83, 0x01, 0x5a, 0x21, 0x0a, 0x0b, 0x6f, 0x62, 0x73,
      0x5f, 0x68, 0x69, 0x73, 0x74, 0x6f, 0x72, 0x79, 0x12, 0x12, 0x0a, 0x10,
      0x08, 0x01, 0x12, 0x0c, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x02, 0x08, 0x19,
      0x0a, 0x02, 0x08, 0x69, 0x62, 0x19, 0x0a, 0x07, 0x61, 0x63, 0x74, 0x69,
      0x6f, 0x6e, 0x73, 0x12, 0x0e, 0x0a, 0x0c, 0x08, 0x01, 0x12, 0x08, 0x0a,
      0x02, 0x08, 0x01, 0x0a, 0x02, 0x08, 0x1a, 0x42, 0x04, 0x0a, 0x00, 0x10,
      0x0d};
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out.write(reinterpret_cast<const char*>(kModel), sizeof(kModel));
}

PolicyInputs validInputs() {
  return {Vec(kGaPolicyObsCurrentDim, 0.0F),
          Vec(kGaPolicyObsHistoryLength * kGaPolicyObsHistoryWidth, 0.0F)};
}

PolicyInputs validClnInputs() {
  return {Vec(kClnPolicyObsCurrentDim, 0.0F),
          Vec(kClnPolicyObsHistoryLength * kClnPolicyObsHistoryWidth, 0.0F)};
}

PolicyInputs validFootstateInputs() {
  return {Vec(kClnFootstatePolicyObsCurrentDim, 0.0F),
          Vec(kClnFootstatePolicyObsHistoryLength * kClnFootstatePolicyObsHistoryWidth,
              0.0F)};
}

void requireRuntimeRejects(const OnnxPolicyRuntimeConfig& config,
                           const std::string& message) {
  try {
    OnnxPolicyRuntime runtime(config);
  } catch (const PolicyRuntimeError& err) {
    REQUIRE(std::string(err.what()).find(message) != std::string::npos);
    return;
  }
  FAIL("expected PolicyRuntimeError");
}

void requireVelocityRuntimeRejects(const OnnxVelocityPolicyRuntimeConfig& config,
                                   const std::string& message) {
  try {
    OnnxVelocityPolicyRuntime runtime(config);
  } catch (const PolicyRuntimeError& err) {
    REQUIRE(std::string(err.what()).find(message) != std::string::npos);
    return;
  }
  FAIL("expected PolicyRuntimeError");
}

}  // namespace

TEST_CASE("OnnxPolicyRuntime rejects a missing model file before ORT session load") {
  TempDir tmp;

  requireRuntimeRejects({tmp.root / "missing.onnx", validConfig()},
                        "model file missing");
}

TEST_CASE("OnnxPolicyRuntime rejects an empty model file before ORT session load") {
  TempDir tmp;
  const auto model = tmp.root / "empty.onnx";
  writeText(model, "");

  requireRuntimeRejects({model, validConfig()}, "model file empty");
}

TEST_CASE("OnnxPolicyRuntime rejects a Git LFS pointer before ORT session load") {
  TempDir tmp;
  const auto model = tmp.root / "pointer.onnx";
  writeText(model,
            "version https://git-lfs.github.com/spec/v1\n"
            "oid sha256:0000000000000000000000000000000000000000000000000000000000000000\n"
            "size 1234\n");

  requireRuntimeRejects({model, validConfig()}, "Git LFS pointer");
}

TEST_CASE("OnnxPolicyRuntime rejects deploy action dimension drift before ORT session load") {
  TempDir tmp;
  const auto model = tmp.root / "not-a-real-model.onnx";
  writeText(model, "not a Git LFS pointer, and not a valid ONNX model");

  auto config = validConfig();
  config.action_scale.pop_back();

  requireRuntimeRejects({model, config}, "action_scale");
}

TEST_CASE("OnnxPolicyRuntime constructs and runs a minimal GA ONNX policy") {
  TempDir tmp;
  const auto model = tmp.root / "minimal-ga.onnx";
  writeMinimalGaOnnx(model);

  OnnxPolicyRuntime runtime({model, validConfig()});

  const Vec actions = runtime.infer(validInputs());

  REQUIRE(actions.size() == kGaPolicyJointDim);
  REQUIRE(std::all_of(actions.begin(), actions.end(),
                      [](float value) { return std::isfinite(value); }));
  REQUIRE(std::all_of(actions.begin(), actions.end(),
                      [](float value) { return value == 0.0F; }));
}

TEST_CASE("OnnxPolicyRuntime constructs and runs the app-owned GeneralTrackerCLNFootstate policy") {
  const DeployConfig deploy_config = loadDeployConfig(
      repoFile("config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml"));
  REQUIRE(deploy_config.observation_contract == ObservationContract::GeneralTrackerCLNFootstate);

  OnnxPolicyRuntime runtime(
      {repoFile("config/policy/general_tracker_cln/exported/multi_policy_footstate3.onnx"),
       deploy_config});

  const Vec actions = runtime.infer(validFootstateInputs());

  REQUIRE(actions.size() == kGaPolicyJointDim);
  REQUIRE(std::all_of(actions.begin(), actions.end(),
                      [](float value) { return std::isfinite(value); }));
}

TEST_CASE("OnnxPolicyRuntime constructs and runs the app-owned GeneralTrackerCLN policy") {
  const DeployConfig deploy_config =
      loadDeployConfig(repoFile("config/policy/general_tracker_cln/params/deploy.yaml"));
  REQUIRE(deploy_config.observation_contract == ObservationContract::GeneralTrackerCLN);

  OnnxPolicyRuntime runtime(
      {repoFile("config/policy/general_tracker_cln/exported/multi_policy_v17c2_70k.onnx"),
       deploy_config});

  const Vec actions = runtime.infer(validClnInputs());

  REQUIRE(actions.size() == kGaPolicyJointDim);
  REQUIRE(std::all_of(actions.begin(), actions.end(),
                      [](float value) { return std::isfinite(value); }));
}

TEST_CASE("OnnxVelocityPolicyRuntime rejects bad inputs before ORT session load") {
  TempDir tmp;
  const auto model = tmp.root / "not-a-real-model.onnx";
  writeText(model, "not a Git LFS pointer, and not a valid ONNX model");

  auto config = validVelocityConfig();
  config.action_scale.pop_back();

  requireVelocityRuntimeRejects({model, config}, "action_scale");
}

TEST_CASE("OnnxVelocityPolicyRuntime rejects a GA model contract as fail-fast metadata drift") {
  TempDir tmp;
  const auto model = tmp.root / "minimal-ga.onnx";
  writeMinimalGaOnnx(model);

  requireVelocityRuntimeRejects({model, validVelocityConfig()}, "input count");
}

TEST_CASE("OnnxVelocityPolicyRuntime constructs and runs the app-owned velocity policy") {
  const VelocityDeployConfig deploy_config = loadVelocityDeployConfig(
      repoFile("config/policy/velocity/v0/params/deploy.yaml"));
  OnnxVelocityPolicyRuntime runtime(
      {repoFile("config/policy/velocity/v0/exported/policy.onnx"), deploy_config});

  const Vec actions = runtime.infer({Vec(kVelocityPolicyObsDim, 0.0F)});

  REQUIRE(actions.size() == kVelocityPolicyJointDim);
  REQUIRE(std::all_of(actions.begin(), actions.end(),
                      [](float value) { return std::isfinite(value); }));
}

}  // namespace agentic_et1_tracker
