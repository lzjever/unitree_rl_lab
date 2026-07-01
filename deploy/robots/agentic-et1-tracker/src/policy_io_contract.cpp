#include "agentic_et1_tracker/policy/policy_io_contract.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <vector>

namespace agentic_et1_tracker {
namespace {

struct ExpectedObservationTerm {
  const char* name;
  std::size_t width;
};

const std::vector<int>& expectedJointIdsMap() {
  static const std::vector<int> values{0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                       10, 11, 12, 13, 14, 15, 16, 17,
                                       18, 19, 20, 21, 22, 23, 24, 25};
  return values;
}

const std::vector<int>& expectedSdkJointIdsMap() {
  static const std::vector<int> values{0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                       10, 11, 12, 13, 15, 16, 17, 18,
                                       19, 22, 23, 24, 25, 26, 29, 30};
  return values;
}

const std::array<ExpectedObservationTerm, 11>& expectedCurrentTerms() {
  static constexpr std::array<ExpectedObservationTerm, 11> values{{
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
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 10>& expectedHistoryTerms() {
  static constexpr std::array<ExpectedObservationTerm, 10> values{{
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
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 9>& expectedClnCurrentTerms() {
  static constexpr std::array<ExpectedObservationTerm, 9> values{{
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kGaPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kGaPolicyJointDim},
      {"joint_vel_rel", kGaPolicyJointDim},
      {"last_action", kGaPolicyJointDim},
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 1>& expectedClnHistoryTerms() {
  static constexpr std::array<ExpectedObservationTerm, 1> values{{
      {"future_commands", kClnPolicyObsHistoryWidth},
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 10>& expectedClnFootstateCurrentTerms() {
  static constexpr std::array<ExpectedObservationTerm, 10> values{{
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kGaPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kGaPolicyJointDim},
      {"joint_vel_rel", kGaPolicyJointDim},
      {"last_action", kGaPolicyJointDim},
      {"command_foot_support_state", 6},
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 1>& expectedClnFootstateHistoryTerms() {
  static constexpr std::array<ExpectedObservationTerm, 1> values{{
      {"future_command_with_foot_support_state", kClnFootstatePolicyObsHistoryWidth},
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 8>& expectedDr3CurrentTerms() {
  static constexpr std::array<ExpectedObservationTerm, 8> values{{
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_jnt_pos", kGaPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kGaPolicyJointDim},
      {"joint_vel_rel", kGaPolicyJointDim},
      {"last_action", kGaPolicyJointDim},
  }};
  return values;
}

const std::array<ExpectedObservationTerm, 1>& expectedDr3HistoryTerms() {
  static constexpr std::array<ExpectedObservationTerm, 1> values{{
      {"future_command", kDr3PolicyObsHistoryWidth},
  }};
  return values;
}

PolicyIoContractError error(const std::string& message) {
  return PolicyIoContractError("policy io contract error: " + message);
}

std::string shapeToString(const std::vector<std::int64_t>& shape) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < shape.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << shape[i];
  }
  out << "]";
  return out.str();
}

void requireSize(const std::string& field,
                 std::size_t actual,
                 std::size_t expected) {
  if (actual != expected) {
    std::ostringstream out;
    out << field << " must be " << expected << ", got " << actual;
    throw error(out.str());
  }
}

void requireShape(const std::string& field,
                  const std::vector<std::int64_t>& actual,
                  const std::vector<std::int64_t>& expected) {
  if (actual != expected) {
    throw error(field + " shape must be " + shapeToString(expected) + ", got " +
                shapeToString(actual));
  }
}

void requireDtype(const std::string& field, PolicyTensorElementType actual) {
  if (actual != PolicyTensorElementType::Float32) {
    throw error(field + " dtype must be float32");
  }
}

void requireName(const std::string& field,
                 const std::string& actual,
                 const std::string& expected) {
  if (actual != expected) {
    throw error(field + " name must be '" + expected + "', got '" + actual + "'");
  }
}

void requireFrozenMap(const std::string& field,
                      const std::vector<int>& actual,
                      const std::vector<int>& expected) {
  requireSize(field, actual.size(), expected.size());
  if (actual != expected) {
    throw error(field + " must match the frozen GA deploy map");
  }
}

void requireFiniteDoubles(const std::string& field,
                          const std::vector<double>& values,
                          bool require_positive) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    const std::string element = field + "[" + std::to_string(i) + "]";
    if (!std::isfinite(values[i])) {
      throw error(element + " must be finite");
    }
    if (require_positive && values[i] <= 0.0) {
      throw error(element + " must be positive");
    }
  }
}

void requireActionClip(const DeployConfig& config) {
  if (config.action_clip.empty()) {
    return;
  }
  requireSize("action_clip", config.action_clip.size(), kGaPolicyJointDim);
  for (std::size_t i = 0; i < config.action_clip.size(); ++i) {
    const std::string field = "action_clip[" + std::to_string(i) + "]";
    const auto& clip = config.action_clip[i];
    if (!std::isfinite(clip[0]) || !std::isfinite(clip[1])) {
      throw error(field + " must be finite");
    }
    if (clip[0] > clip[1]) {
      throw error(field + " min must be <= max");
    }
  }
}

template <std::size_t N>
void requireFrozenObservationTerms(
    const std::string& field,
    const std::vector<ObservationTerm>& actual,
    const std::array<ExpectedObservationTerm, N>& expected) {
  requireSize(field, actual.size(), expected.size());

  std::size_t expected_offset = 0;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    const std::string prefix = field + "[" + std::to_string(i) + "]";
    if (actual[i].name != expected[i].name) {
      throw error(prefix + ".name must be '" + expected[i].name + "', got '" +
                  actual[i].name + "'");
    }
    if (actual[i].width != expected[i].width) {
      std::ostringstream out;
      out << prefix << ".width must be " << expected[i].width << ", got "
          << actual[i].width;
      throw error(out.str());
    }
    if (actual[i].offset != expected_offset) {
      std::ostringstream out;
      out << prefix << ".offset must be " << expected_offset << ", got "
          << actual[i].offset;
      throw error(out.str());
    }
    expected_offset += expected[i].width;
  }
}

}  // namespace

void validateGaDeployConfig(const DeployConfig& config) {
  requireSize("joint_dim", config.joint_dim, kGaPolicyJointDim);
  requireSize("action_scale", config.action_scale.size(), kGaPolicyJointDim);
  requireSize("action_offset", config.action_offset.size(), kGaPolicyJointDim);
  requireSize("policy_kp", config.policy_kp.size(), kGaPolicyJointDim);
  requireSize("policy_kd", config.policy_kd.size(), kGaPolicyJointDim);
  requireSize("default_joint_pos", config.default_joint_pos.size(),
              kGaPolicyJointDim);
  requireFiniteDoubles("default_joint_pos", config.default_joint_pos, false);
  requireFiniteDoubles("action_offset", config.action_offset, false);
  requireFiniteDoubles("action_scale", config.action_scale, true);
  requireActionClip(config);
  requireFiniteDoubles("policy_kp", config.policy_kp, true);
  requireFiniteDoubles("policy_kd", config.policy_kd, true);
  requireFrozenMap("joint_ids_map", config.joint_ids_map, expectedJointIdsMap());
  requireFrozenMap("sdk_joint_ids_map", config.sdk_joint_ids_map,
                   expectedSdkJointIdsMap());
  switch (config.observation_contract) {
    case ObservationContract::GeneralTracker:
      requireSize("obs_current_dim", config.obs_current_dim, kGaPolicyObsCurrentDim);
      requireSize("obs_history_width", config.obs_history_width,
                  kGaPolicyObsHistoryWidth);
      requireSize("obs_history_length", config.obs_history_length,
                  kGaPolicyObsHistoryLength);
      requireFrozenObservationTerms("obs_current_terms", config.obs_current_terms,
                                    expectedCurrentTerms());
      requireFrozenObservationTerms("obs_history_terms", config.obs_history_terms,
                                    expectedHistoryTerms());
      break;
    case ObservationContract::GeneralTrackerCLN:
      requireSize("obs_current_dim", config.obs_current_dim, kClnPolicyObsCurrentDim);
      requireSize("obs_history_width", config.obs_history_width,
                  kClnPolicyObsHistoryWidth);
      requireSize("obs_history_length", config.obs_history_length,
                  kClnPolicyObsHistoryLength);
      requireFrozenObservationTerms("obs_current_terms", config.obs_current_terms,
                                    expectedClnCurrentTerms());
      requireFrozenObservationTerms("obs_history_terms", config.obs_history_terms,
                                    expectedClnHistoryTerms());
      break;
    case ObservationContract::GeneralTrackerCLNFootstate:
      requireSize("obs_current_dim", config.obs_current_dim,
                  kClnFootstatePolicyObsCurrentDim);
      requireSize("obs_history_width", config.obs_history_width,
                  kClnFootstatePolicyObsHistoryWidth);
      requireSize("obs_history_length", config.obs_history_length,
                  kClnFootstatePolicyObsHistoryLength);
      requireFrozenObservationTerms("obs_current_terms", config.obs_current_terms,
                                    expectedClnFootstateCurrentTerms());
      requireFrozenObservationTerms("obs_history_terms", config.obs_history_terms,
                                    expectedClnFootstateHistoryTerms());
      break;
    case ObservationContract::GeneralTrackerDR3:
      requireSize("obs_current_dim", config.obs_current_dim, kDr3PolicyObsCurrentDim);
      requireSize("obs_history_width", config.obs_history_width,
                  kDr3PolicyObsHistoryWidth);
      requireSize("obs_history_length", config.obs_history_length,
                  kDr3PolicyObsHistoryLength);
      requireFrozenObservationTerms("obs_current_terms", config.obs_current_terms,
                                    expectedDr3CurrentTerms());
      requireFrozenObservationTerms("obs_history_terms", config.obs_history_terms,
                                    expectedDr3HistoryTerms());
      break;
  }
}

void validateGaPolicyIoContract(const DeployConfig& config,
                                const PolicyModelMetadata& metadata) {
  validateGaDeployConfig(config);

  requireSize("input count", metadata.inputs.size(), 2);
  requireSize("output count", metadata.outputs.size(), 1);

  const PolicyTensorMetadata& obs_current = metadata.inputs[0];
  requireName("input[0]", obs_current.name, "obs_current");
  requireDtype("obs_current", obs_current.element_type);
  requireShape("obs_current", obs_current.shape,
               {1, static_cast<std::int64_t>(config.obs_current_dim)});

  const PolicyTensorMetadata& obs_history = metadata.inputs[1];
  requireName("input[1]", obs_history.name, "obs_history");
  requireDtype("obs_history", obs_history.element_type);
  requireShape("obs_history", obs_history.shape,
               {1, static_cast<std::int64_t>(config.obs_history_length),
                static_cast<std::int64_t>(config.obs_history_width)});

  const PolicyTensorMetadata& actions = metadata.outputs[0];
  requireName("output[0]", actions.name, "actions");
  requireDtype("actions", actions.element_type);
  requireShape("actions", actions.shape,
               {1, static_cast<std::int64_t>(kGaPolicyJointDim)});
}

void validateGaPolicyInputs(const DeployConfig& config, const PolicyInputs& inputs) {
  validateGaDeployConfig(config);
  requireSize("obs_current", inputs.obs_current.size(), config.obs_current_dim);
  requireSize("obs_history", inputs.obs_history.size(),
              config.obs_history_length * config.obs_history_width);
}

void validateVelocityDeployConfig(const VelocityDeployConfig& config) {
  requireSize("joint_dim", config.joint_dim, kVelocityPolicyJointDim);
  requireSize("joint_ids_map", config.joint_ids_map.size(), kVelocityPolicyJointDim);
  if (!config.sdk_joint_ids_map.empty()) {
    requireSize("sdk_joint_ids_map", config.sdk_joint_ids_map.size(),
                kVelocityPolicyJointDim);
  }
  requireSize("stiffness", config.stiffness.size(), kVelocityPolicyJointDim);
  requireSize("damping", config.damping.size(), kVelocityPolicyJointDim);
  requireSize("default_joint_pos", config.default_joint_pos.size(),
              kVelocityPolicyJointDim);
  requireSize("action_scale", config.action_scale.size(), kVelocityPolicyJointDim);
  requireSize("action_offset", config.action_offset.size(), kVelocityPolicyJointDim);
  requireSize("obs_row_width", config.obs_row_width, kVelocityPolicyObsRowWidth);
  requireSize("obs_history_length", config.obs_history_length,
              kVelocityPolicyHistoryLength);
  requireSize("obs_dim", config.obs_dim, kVelocityPolicyObsDim);
  requireFiniteDoubles("stiffness", config.stiffness, false);
  requireFiniteDoubles("damping", config.damping, false);
  requireFiniteDoubles("default_joint_pos", config.default_joint_pos, false);
  requireFiniteDoubles("action_scale", config.action_scale, true);
  requireFiniteDoubles("action_offset", config.action_offset, false);
  requireSize("observation_terms", config.observation_terms.size(), 6);

  const std::array<ExpectedObservationTerm, 6> expected{{
      {"base_ang_vel", 3},
      {"projected_gravity", 3},
      {"keyboard_velocity_commands", 3},
      {"joint_pos_rel", kVelocityPolicyJointDim},
      {"joint_vel_rel", kVelocityPolicyJointDim},
      {"last_action", kVelocityPolicyJointDim},
  }};
  std::size_t offset = 0;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    const auto& actual = config.observation_terms.at(i);
    const std::string prefix = "observation_terms[" + std::to_string(i) + "]";
    if (actual.name != expected.at(i).name) {
      throw error(prefix + ".name must be '" + expected.at(i).name + "'");
    }
    requireSize(prefix + ".width", actual.width, expected.at(i).width);
    requireSize(prefix + ".offset", actual.offset, offset);
    requireSize(prefix + ".scale", actual.scale.size(), expected.at(i).width);
    requireFiniteDoubles(prefix + ".scale", actual.scale, false);
    offset += actual.width;
  }
}

void validateVelocityPolicyIoContract(const VelocityDeployConfig& config,
                                      const PolicyModelMetadata& metadata) {
  validateVelocityDeployConfig(config);
  requireSize("input count", metadata.inputs.size(), 1);
  requireSize("output count", metadata.outputs.size(), 1);

  const PolicyTensorMetadata& obs = metadata.inputs[0];
  requireName("input[0]", obs.name, "obs");
  requireDtype("obs", obs.element_type);
  requireShape("obs", obs.shape,
               {1, static_cast<std::int64_t>(kVelocityPolicyObsDim)});

  const PolicyTensorMetadata& actions = metadata.outputs[0];
  requireName("output[0]", actions.name, "actions");
  requireDtype("actions", actions.element_type);
  requireShape("actions", actions.shape,
               {1, static_cast<std::int64_t>(kVelocityPolicyJointDim)});
}

void validateVelocityPolicyInputs(const Vec& obs) {
  requireSize("obs", obs.size(), kVelocityPolicyObsDim);
}

void validateLocoLowerDeployConfig(const LocoLowerDeployConfig& config) {
  requireSize("joint_dim", config.joint_dim, kLocoLowerPolicyJointDim);
  requireSize("joint_ids_map", config.joint_ids_map.size(), kLocoLowerPolicyJointDim);
  if (!config.sdk_joint_ids_map.empty()) {
    requireSize("sdk_joint_ids_map", config.sdk_joint_ids_map.size(),
                kLocoLowerPolicyJointDim);
  }
  requireSize("stiffness", config.stiffness.size(), kLocoLowerPolicyJointDim);
  requireSize("damping", config.damping.size(), kLocoLowerPolicyJointDim);
  requireSize("default_joint_pos", config.default_joint_pos.size(),
              kLocoLowerPolicyJointDim);
  requireSize("action_scale", config.action_scale.size(), kLocoLowerPolicyJointDim);
  requireSize("action_offset", config.action_offset.size(), kLocoLowerPolicyJointDim);
  requireSize("obs_row_width", config.obs_row_width, kLocoLowerPolicyObsRowWidth);
  requireSize("obs_history_length", config.obs_history_length,
              kLocoLowerPolicyHistoryLength);
  requireSize("obs_dim", config.obs_dim, kLocoLowerPolicyObsDim);
  requireFiniteDoubles("stiffness", config.stiffness, false);
  requireFiniteDoubles("damping", config.damping, false);
  requireFiniteDoubles("default_joint_pos", config.default_joint_pos, false);
  requireFiniteDoubles("action_scale", config.action_scale, true);
  requireFiniteDoubles("action_offset", config.action_offset, false);
  requireSize("observation_terms", config.observation_terms.size(), 6);

  const std::array<ExpectedObservationTerm, 6> expected{{
      {"base_ang_vel", 3},
      {"projected_gravity", 3},
      {"keyboard_velocity_commands", 3},
      {"joint_pos_rel", kLocoLowerPolicyJointDim},
      {"joint_vel_rel", kLocoLowerPolicyJointDim},
      {"last_action", kLocoLowerPolicyJointDim},
  }};
  std::size_t offset = 0;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    const auto& actual = config.observation_terms.at(i);
    const std::string prefix = "observation_terms[" + std::to_string(i) + "]";
    if (actual.name != expected.at(i).name) {
      throw error(prefix + ".name must be '" + expected.at(i).name + "'");
    }
    requireSize(prefix + ".width", actual.width, expected.at(i).width);
    requireSize(prefix + ".offset", actual.offset, offset);
    requireSize(prefix + ".scale", actual.scale.size(), expected.at(i).width);
    requireFiniteDoubles(prefix + ".scale", actual.scale, false);
    offset += actual.width;
  }
}

void validateLocoLowerPolicyIoContract(const LocoLowerDeployConfig& config,
                                       const PolicyModelMetadata& metadata) {
  validateLocoLowerDeployConfig(config);
  requireSize("input count", metadata.inputs.size(), 1);
  requireSize("output count", metadata.outputs.size(), 1);

  const PolicyTensorMetadata& obs = metadata.inputs[0];
  requireName("input[0]", obs.name, "obs");
  requireDtype("obs", obs.element_type);
  requireShape("obs", obs.shape, {1, static_cast<std::int64_t>(config.obs_dim)});

  const PolicyTensorMetadata& actions = metadata.outputs[0];
  requireName("output[0]", actions.name, "actions");
  requireDtype("actions", actions.element_type);
  requireShape("actions", actions.shape,
               {1, static_cast<std::int64_t>(config.joint_dim)});
}

void validateLocoLowerPolicyInputs(const LocoLowerDeployConfig& config,
                                   const Vec& obs) {
  validateLocoLowerDeployConfig(config);
  requireSize("obs", obs.size(), config.obs_dim);
}

}  // namespace agentic_et1_tracker
