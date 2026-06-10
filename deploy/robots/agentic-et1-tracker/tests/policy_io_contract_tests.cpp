#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/policy_io_contract.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointDim = 26;
constexpr std::size_t kObsCurrentDim = 131;
constexpr std::size_t kHistoryLength = 25;
constexpr std::size_t kHistoryWidth = 105;
constexpr std::size_t kClnObsCurrentDim = 121;
constexpr std::size_t kClnHistoryWidth = 35;
constexpr std::size_t kClnFootstateObsCurrentDim = 127;
constexpr std::size_t kClnFootstateHistoryLength = 5;
constexpr std::size_t kClnFootstateHistoryWidth = 41;

std::vector<int> intRange(std::size_t count) {
  std::vector<int> values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(static_cast<int>(i));
  }
  return values;
}

std::vector<double> doubles(std::size_t count, double value) {
  return std::vector<double>(count, value);
}

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
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
}

std::vector<ObservationTerm> historyTerms() {
  return terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
}

std::vector<ObservationTerm> clnCurrentTerms() {
  return terms({
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
  });
}

std::vector<ObservationTerm> clnHistoryTerms() {
  return terms({
      {"future_commands", kClnHistoryWidth},
  });
}

std::vector<ObservationTerm> clnFootstateCurrentTerms() {
  return terms({
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
      {"command_foot_support_state", 6},
  });
}

std::vector<ObservationTerm> clnFootstateHistoryTerms() {
  return terms({
      {"future_command_with_foot_support_state", kClnFootstateHistoryWidth},
  });
}

DeployConfig validConfig() {
  DeployConfig config;
  config.joint_dim = kJointDim;
  config.joint_ids_map = intRange(kJointDim);
  config.sdk_joint_ids_map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                              10, 11, 12, 13, 15, 16, 17, 18,
                              19, 22, 23, 24, 25, 26, 29, 30};
  config.action_scale = doubles(kJointDim, 0.1);
  config.action_offset = doubles(kJointDim, 0.0);
  config.policy_kp = doubles(kJointDim, 1.0);
  config.policy_kd = doubles(kJointDim, 0.1);
  config.default_joint_pos = doubles(kJointDim, 0.0);
  config.obs_current_dim = kObsCurrentDim;
  config.obs_history_width = kHistoryWidth;
  config.obs_history_length = kHistoryLength;
  config.obs_current_terms = currentTerms();
  config.obs_history_terms = historyTerms();
  return config;
}

DeployConfig validClnConfig() {
  DeployConfig config = validConfig();
  config.observation_contract = ObservationContract::GeneralTrackerCLN;
  config.obs_current_dim = kClnObsCurrentDim;
  config.obs_history_width = kClnHistoryWidth;
  config.obs_history_length = kHistoryLength;
  config.obs_current_terms = clnCurrentTerms();
  config.obs_history_terms = clnHistoryTerms();
  return config;
}

DeployConfig validClnFootstateConfig() {
  DeployConfig config = validConfig();
  config.observation_contract = ObservationContract::GeneralTrackerCLNFootstate;
  config.obs_current_dim = kClnFootstateObsCurrentDim;
  config.obs_history_width = kClnFootstateHistoryWidth;
  config.obs_history_length = kClnFootstateHistoryLength;
  config.obs_current_terms = clnFootstateCurrentTerms();
  config.obs_history_terms = clnFootstateHistoryTerms();
  return config;
}

PolicyModelMetadata validMetadata() {
  return {
      {
          {"obs_current", PolicyTensorElementType::Float32, {1, 131}},
          {"obs_history", PolicyTensorElementType::Float32, {1, 25, 105}},
      },
      {
          {"actions", PolicyTensorElementType::Float32, {1, 26}},
      },
  };
}

PolicyModelMetadata validClnMetadata() {
  return {
      {
          {"obs_current", PolicyTensorElementType::Float32, {1, 121}},
          {"obs_history", PolicyTensorElementType::Float32, {1, 25, 35}},
      },
      {
          {"actions", PolicyTensorElementType::Float32, {1, 26}},
      },
  };
}

PolicyModelMetadata validClnFootstateMetadata() {
  return {
      {
          {"obs_current", PolicyTensorElementType::Float32, {1, 127}},
          {"obs_history", PolicyTensorElementType::Float32, {1, 5, 41}},
      },
      {
          {"actions", PolicyTensorElementType::Float32, {1, 26}},
      },
  };
}

void requireContractRejects(const DeployConfig& config,
                            const PolicyModelMetadata& metadata,
                            const std::string& message) {
  try {
    validateGaPolicyIoContract(config, metadata);
  } catch (const PolicyIoContractError& err) {
    REQUIRE(std::string(err.what()).find(message) != std::string::npos);
    return;
  }
  FAIL("expected PolicyIoContractError");
}

void requireDeployRejects(const DeployConfig& config,
                          const std::string& message) {
  try {
    validateGaDeployConfig(config);
  } catch (const PolicyIoContractError& err) {
    REQUIRE(std::string(err.what()).find(message) != std::string::npos);
    return;
  }
  FAIL("expected PolicyIoContractError");
}

}  // namespace

TEST_CASE("GA policy IO contract accepts frozen deploy dims and ONNX metadata") {
  REQUIRE_NOTHROW(validateGaDeployConfig(validConfig()));
  REQUIRE_NOTHROW(validateGaPolicyIoContract(validConfig(), validMetadata()));
}

TEST_CASE("GA policy IO contract accepts GeneralTrackerCLN deploy dims and ONNX metadata") {
  REQUIRE_NOTHROW(validateGaDeployConfig(validClnConfig()));
  REQUIRE_NOTHROW(validateGaPolicyIoContract(validClnConfig(), validClnMetadata()));
}

TEST_CASE("GA policy IO contract accepts GeneralTrackerCLNFootstate deploy dims and ONNX metadata") {
  REQUIRE_NOTHROW(validateGaDeployConfig(validClnFootstateConfig()));
  REQUIRE_NOTHROW(
      validateGaPolicyIoContract(validClnFootstateConfig(), validClnFootstateMetadata()));
}

TEST_CASE("GA policy IO contract rejects old CLN ONNX shapes for Footstate profile") {
  requireContractRejects(validClnFootstateConfig(), validClnMetadata(), "obs_current shape");
}

TEST_CASE("GA policy IO contract rejects input name drift") {
  auto metadata = validMetadata();
  metadata.inputs[0].name = "observation";

  requireContractRejects(validConfig(), metadata, "input[0] name");
}

TEST_CASE("GA policy IO contract rejects extra inputs") {
  auto metadata = validMetadata();
  metadata.inputs.push_back(
      {"unused", PolicyTensorElementType::Float32, {1, 1}});

  requireContractRejects(validConfig(), metadata, "input count");
}

TEST_CASE("GA policy IO contract rejects input shape drift") {
  auto metadata = validMetadata();
  metadata.inputs[1].shape = {1, 24, 105};

  requireContractRejects(validConfig(), metadata, "obs_history shape");
}

TEST_CASE("GA policy IO contract rejects output size drift") {
  auto metadata = validMetadata();
  metadata.outputs[0].shape = {1, 25};

  requireContractRejects(validConfig(), metadata, "actions shape");
}

TEST_CASE("GA policy IO contract rejects non-float32 tensors") {
  auto metadata = validMetadata();
  metadata.outputs[0].element_type = PolicyTensorElementType::Float64;

  requireContractRejects(validConfig(), metadata, "actions dtype");
}

TEST_CASE("GA deploy config contract rejects scalar dimension drift") {
  SECTION("joint_dim") {
    auto config = validConfig();
    config.joint_dim = 25;

    requireDeployRejects(config, "joint_dim");
    requireContractRejects(config, validMetadata(), "joint_dim");
  }

  SECTION("obs_current_dim") {
    auto config = validConfig();
    config.obs_current_dim = 130;

    requireDeployRejects(config, "obs_current_dim");
    requireContractRejects(config, validMetadata(), "obs_current_dim");
  }

  SECTION("obs_history_width") {
    auto config = validConfig();
    config.obs_history_width = 104;

    requireDeployRejects(config, "obs_history_width");
    requireContractRejects(config, validMetadata(), "obs_history_width");
  }

  SECTION("obs_history_length") {
    auto config = validConfig();
    config.obs_history_length = 24;

    requireDeployRejects(config, "obs_history_length");
    requireContractRejects(config, validMetadata(), "obs_history_length");
  }
}

TEST_CASE("GA deploy config contract rejects action vector length drift") {
  SECTION("action_scale") {
    auto config = validConfig();
    config.action_scale.pop_back();

    requireDeployRejects(config, "action_scale");
    requireContractRejects(config, validMetadata(), "action_scale");
  }

  SECTION("action_offset") {
    auto config = validConfig();
    config.action_offset.pop_back();

    requireDeployRejects(config, "action_offset");
    requireContractRejects(config, validMetadata(), "action_offset");
  }

  SECTION("action_clip") {
    auto config = validConfig();
    config.action_clip.assign(kJointDim - 1, std::array<double, 2>{-1.0, 1.0});

    requireDeployRejects(config, "action_clip");
    requireContractRejects(config, validMetadata(), "action_clip");
  }
}

TEST_CASE("GA deploy config contract rejects policy gain vector length drift") {
  SECTION("policy_kp") {
    auto config = validConfig();
    config.policy_kp.pop_back();

    requireDeployRejects(config, "policy_kp");
    requireContractRejects(config, validMetadata(), "policy_kp");
  }

  SECTION("policy_kd") {
    auto config = validConfig();
    config.policy_kd.pop_back();

    requireDeployRejects(config, "policy_kd");
    requireContractRejects(config, validMetadata(), "policy_kd");
  }
}

TEST_CASE("GA deploy config contract rejects joint vector length drift") {
  SECTION("default_joint_pos") {
    auto config = validConfig();
    config.default_joint_pos.pop_back();

    requireDeployRejects(config, "default_joint_pos");
    requireContractRejects(config, validMetadata(), "default_joint_pos");
  }

  SECTION("sdk_joint_ids_map") {
    auto config = validConfig();
    config.sdk_joint_ids_map.pop_back();

    requireDeployRejects(config, "sdk_joint_ids_map");
    requireContractRejects(config, validMetadata(), "sdk_joint_ids_map");
  }

  SECTION("joint_ids_map") {
    auto config = validConfig();
    config.joint_ids_map.pop_back();

    requireDeployRejects(config, "joint_ids_map");
    requireContractRejects(config, validMetadata(), "joint_ids_map");
  }
}

TEST_CASE("GA deploy config contract rejects non-finite action and gain values") {
  SECTION("default_joint_pos NaN") {
    auto config = validConfig();
    config.default_joint_pos[0] = std::numeric_limits<double>::quiet_NaN();

    requireDeployRejects(config, "default_joint_pos");
    requireContractRejects(config, validMetadata(), "default_joint_pos");
  }

  SECTION("action_offset Inf") {
    auto config = validConfig();
    config.action_offset[0] = std::numeric_limits<double>::infinity();

    requireDeployRejects(config, "action_offset");
    requireContractRejects(config, validMetadata(), "action_offset");
  }

  SECTION("action_scale NaN") {
    auto config = validConfig();
    config.action_scale[0] = std::numeric_limits<double>::quiet_NaN();

    requireDeployRejects(config, "action_scale");
    requireContractRejects(config, validMetadata(), "action_scale");
  }

  SECTION("policy_kp Inf") {
    auto config = validConfig();
    config.policy_kp[0] = std::numeric_limits<double>::infinity();

    requireDeployRejects(config, "policy_kp");
    requireContractRejects(config, validMetadata(), "policy_kp");
  }

  SECTION("policy_kd NaN") {
    auto config = validConfig();
    config.policy_kd[0] = std::numeric_limits<double>::quiet_NaN();

    requireDeployRejects(config, "policy_kd");
    requireContractRejects(config, validMetadata(), "policy_kd");
  }

  SECTION("action_clip Inf") {
    auto config = validConfig();
    config.action_clip.assign(kJointDim, std::array<double, 2>{-1.0, 1.0});
    config.action_clip[0][1] = std::numeric_limits<double>::infinity();

    requireDeployRejects(config, "action_clip");
    requireContractRejects(config, validMetadata(), "action_clip");
  }
}

TEST_CASE("GA deploy config contract rejects non-positive policy scale and gains") {
  SECTION("zero action_scale") {
    auto config = validConfig();
    config.action_scale[0] = 0.0;

    requireDeployRejects(config, "action_scale");
    requireContractRejects(config, validMetadata(), "action_scale");
  }

  SECTION("negative action_scale") {
    auto config = validConfig();
    config.action_scale[0] = -0.1;

    requireDeployRejects(config, "action_scale");
    requireContractRejects(config, validMetadata(), "action_scale");
  }

  SECTION("zero policy_kp") {
    auto config = validConfig();
    config.policy_kp[0] = 0.0;

    requireDeployRejects(config, "policy_kp");
    requireContractRejects(config, validMetadata(), "policy_kp");
  }

  SECTION("negative policy_kp") {
    auto config = validConfig();
    config.policy_kp[0] = -1.0;

    requireDeployRejects(config, "policy_kp");
    requireContractRejects(config, validMetadata(), "policy_kp");
  }

  SECTION("zero policy_kd") {
    auto config = validConfig();
    config.policy_kd[0] = 0.0;

    requireDeployRejects(config, "policy_kd");
    requireContractRejects(config, validMetadata(), "policy_kd");
  }

  SECTION("negative policy_kd") {
    auto config = validConfig();
    config.policy_kd[0] = -0.1;

    requireDeployRejects(config, "policy_kd");
    requireContractRejects(config, validMetadata(), "policy_kd");
  }

  SECTION("inverted action_clip") {
    auto config = validConfig();
    config.action_clip.assign(kJointDim, std::array<double, 2>{-1.0, 1.0});
    config.action_clip[0] = {2.0, 1.0};

    requireDeployRejects(config, "action_clip");
    requireContractRejects(config, validMetadata(), "action_clip");
  }
}

TEST_CASE("GA deploy config contract rejects frozen observation term drift") {
  SECTION("current wrong term name") {
    auto config = validConfig();
    config.obs_current_terms[0].name = "wrong";

    requireDeployRejects(config, "obs_current_terms[0].name");
    requireContractRejects(config, validMetadata(), "obs_current_terms[0].name");
  }

  SECTION("current wrong order") {
    auto config = validConfig();
    std::swap(config.obs_current_terms[0], config.obs_current_terms[1]);

    requireDeployRejects(config, "obs_current_terms[0].name");
    requireContractRejects(config, validMetadata(), "obs_current_terms[0].name");
  }

  SECTION("current wrong width") {
    auto config = validConfig();
    config.obs_current_terms[2].width = 25;

    requireDeployRejects(config, "obs_current_terms[2].width");
    requireContractRejects(config, validMetadata(), "obs_current_terms[2].width");
  }

  SECTION("current wrong offset") {
    auto config = validConfig();
    config.obs_current_terms[3].offset = 999;

    requireDeployRejects(config, "obs_current_terms[3].offset");
    requireContractRejects(config, validMetadata(), "obs_current_terms[3].offset");
  }

  SECTION("history includes last_action") {
    auto config = validConfig();
    config.obs_history_terms[7].name = "last_action";
    config.obs_history_terms[7].width = kJointDim;

    requireDeployRejects(config, "obs_history_terms[7].name");
    requireContractRejects(config, validMetadata(), "obs_history_terms[7].name");
  }

  SECTION("current missing last_action") {
    auto config = validConfig();
    config.obs_current_terms.erase(config.obs_current_terms.begin() + 7);

    requireDeployRejects(config, "obs_current_terms");
    requireContractRejects(config, validMetadata(), "obs_current_terms");
  }
}

}  // namespace agentic_et1_tracker
