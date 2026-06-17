#include "agentic_et1_tracker/loco_upper/lowcmd_composer.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kLowerEndJointExclusive = 12;

LocoUpperLowCmdComposerError error(const std::string& message) {
  return LocoUpperLowCmdComposerError("loco upper lowcmd composer error: " + message);
}

void requireSize(const std::string& field, std::size_t actual, std::size_t expected) {
  if (actual == expected) {
    return;
  }

  std::ostringstream out;
  out << field << " must contain " << expected << " entries, got " << actual;
  throw error(out.str());
}

void requireFinite(const std::string& field, float value) {
  if (!std::isfinite(value)) {
    throw error(field + " must be finite");
  }
}

void validateMotorCommand(const std::string& field, const MotorCommand& motor) {
  requireFinite(field + ".q", motor.q);
  requireFinite(field + ".dq", motor.dq);
  requireFinite(field + ".kp", motor.kp);
  requireFinite(field + ".kd", motor.kd);
  requireFinite(field + ".tau", motor.tau);
}

void validateConfig(const LocoUpperLowCmdComposerConfig& config) {
  if (config.upper_start_joint != kLowerEndJointExclusive ||
      config.upper_end_joint_exclusive != kPolicyJointCount ||
      config.upper_start_joint >= config.upper_end_joint_exclusive) {
    throw error("upper joint range must be [12, 26)");
  }

  requireSize("logical_to_sdk", config.logical_to_sdk.size(), kPolicyJointCount);
  requireSize("upper_kp", config.upper_kp.size(), kPolicyJointCount);
  requireSize("upper_kd", config.upper_kd.size(), kPolicyJointCount);
  requireSize("lower_min_q", config.lower_min_q.size(), kLowerEndJointExclusive);
  requireSize("lower_max_q", config.lower_max_q.size(), kLowerEndJointExclusive);
  requireSize("upper_min_q", config.upper_min_q.size(), kPolicyJointCount);
  requireSize("upper_max_q", config.upper_max_q.size(), kPolicyJointCount);
  requireSize("upper_max_vel_radps", config.upper_max_vel_radps.size(), kPolicyJointCount);
  requireSize("upper_max_accel_radps2",
              config.upper_max_accel_radps2.size(),
              kPolicyJointCount);

  std::array<bool, kSdkMotorCount> used{};
  for (std::size_t logical = 0; logical < config.logical_to_sdk.size(); ++logical) {
    const int slot = config.logical_to_sdk.at(logical);
    if (slot < 0 || slot >= static_cast<int>(kSdkMotorCount)) {
      std::ostringstream out;
      out << "logical_to_sdk[" << logical << "] out of range: " << slot;
      throw error(out.str());
    }
    const std::size_t sdk_slot = static_cast<std::size_t>(slot);
    if (used.at(sdk_slot)) {
      std::ostringstream out;
      out << "logical_to_sdk duplicate SDK slot: " << slot;
      throw error(out.str());
    }
    used.at(sdk_slot) = true;
  }

  for (std::size_t logical = 0; logical < config.upper_start_joint; ++logical) {
    requireFinite("lower_min_q", config.lower_min_q.at(logical));
    requireFinite("lower_max_q", config.lower_max_q.at(logical));
    if (config.lower_min_q.at(logical) > config.lower_max_q.at(logical)) {
      std::ostringstream out;
      out << "lower limits invalid at logical joint " << logical;
      throw error(out.str());
    }
  }

  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive;
       ++logical) {
    requireFinite("upper_kp", config.upper_kp.at(logical));
    requireFinite("upper_kd", config.upper_kd.at(logical));
    requireFinite("upper_min_q", config.upper_min_q.at(logical));
    requireFinite("upper_max_q", config.upper_max_q.at(logical));
    requireFinite("upper_max_vel_radps", config.upper_max_vel_radps.at(logical));
    requireFinite("upper_max_accel_radps2",
                  config.upper_max_accel_radps2.at(logical));
    if (config.upper_min_q.at(logical) > config.upper_max_q.at(logical)) {
      std::ostringstream out;
      out << "upper limits invalid at logical joint " << logical;
      throw error(out.str());
    }
    if (config.upper_max_vel_radps.at(logical) < 0.0F ||
        config.upper_max_accel_radps2.at(logical) < 0.0F) {
      std::ostringstream out;
      out << "upper dynamic limits invalid at logical joint " << logical;
      throw error(out.str());
    }
  }

  requireFinite("unowned_safe.q", config.unowned_safe.q);
  requireFinite("unowned_safe.kp", config.unowned_safe.kp);
  requireFinite("unowned_safe.kd", config.unowned_safe.kd);
}

MotorCommand safeMotorCommand(const LocoUpperSafeCommand& safe) {
  MotorCommand motor;
  motor.mode = safe.mode;
  motor.q = safe.q;
  motor.dq = 0.0F;
  motor.kp = safe.kp;
  motor.kd = safe.kd;
  motor.tau = 0.0F;
  return motor;
}

void validateUpperTargets(const LocoUpperLowCmdComposerConfig& config,
                          const std::vector<float>& targets) {
  requireSize("upper joint targets", targets.size(), kPolicyJointCount);
  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive;
       ++logical) {
    const float target = targets.at(logical);
    if (!std::isfinite(target)) {
      std::ostringstream out;
      out << "upper target[" << logical << "] must be finite";
      throw error(out.str());
    }
    if (target < config.upper_min_q.at(logical) ||
        target > config.upper_max_q.at(logical)) {
      std::ostringstream out;
      out << "upper target[" << logical << "] outside configured limits";
      throw error(out.str());
    }
  }
}

void applyLowerJoints(const LocoUpperLowCmdComposerConfig& config,
                      const LowCmdFrame& lower_frame,
                      LowCmdFrame& output,
                      bool& lower_q_limited) {
  for (std::size_t logical = 0; logical < config.upper_start_joint; ++logical) {
    const std::size_t sdk_slot =
        static_cast<std::size_t>(config.logical_to_sdk.at(logical));
    MotorCommand lower_motor = lower_frame.motors.at(sdk_slot);
    std::ostringstream field;
    field << "lower motor[" << sdk_slot << "]";
    validateMotorCommand(field.str(), lower_motor);
    if (!std::isfinite(lower_motor.q)) {
      throw error(field.str() + ".q must be finite");
    }
    const float clamped_q = std::clamp(lower_motor.q,
                                       config.lower_min_q.at(logical),
                                       config.lower_max_q.at(logical));
    if (clamped_q != lower_motor.q) {
      lower_motor.q = clamped_q;
      lower_q_limited = true;
    }
    output.motors.at(sdk_slot) = lower_motor;
  }
}

void applyUpperJoints(const LocoUpperLowCmdComposerConfig& config,
                      const std::vector<float>& targets,
                      LowCmdFrame& output) {
  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive;
       ++logical) {
    const std::size_t sdk_slot =
        static_cast<std::size_t>(config.logical_to_sdk.at(logical));
    MotorCommand& motor = output.motors.at(sdk_slot);
    motor.mode = 1;
    motor.q = targets.at(logical);
    motor.dq = 0.0F;
    motor.kp = config.upper_kp.at(logical);
    motor.kd = config.upper_kd.at(logical);
    motor.tau = 0.0F;
  }
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& err) {
    throw error(field + " has invalid value: " + err.what());
  }
}

YAML::Node loadYamlMap(const std::filesystem::path& path, const std::string& field) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception& err) {
    throw error(field + " failed to load: " + err.what());
  }
  if (!root || !root.IsMap()) {
    throw error(field + " must be a map");
  }
  return root;
}

YAML::Node requiredNode(const YAML::Node& root,
                        const char* key,
                        const std::string& field) {
  const YAML::Node node = root[key];
  if (!node) {
    throw error(field + " is required");
  }
  return node;
}

YAML::Node optionalAliasNode(const YAML::Node& root,
                             const char* key,
                             const char* alias) {
  const YAML::Node node = root[key];
  if (node) {
    return node;
  }
  if (alias == nullptr) {
    return {};
  }
  return root[alias];
}

std::vector<int> readIntVector(const YAML::Node& root,
                               const char* key,
                               std::size_t expected_size,
                               const std::string& field) {
  const YAML::Node node = requiredNode(root, key, field);
  if (!node.IsSequence() || node.size() != expected_size) {
    std::ostringstream out;
    out << field << " must contain " << expected_size << " entries";
    throw error(out.str());
  }
  std::vector<int> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < expected_size; ++i) {
    values.push_back(scalarAs<int>(node[i], field + "[" + std::to_string(i) + "]"));
  }
  return values;
}

std::vector<float> readUpperLimitVector(const YAML::Node& root,
                                        const char* key,
                                        const char* alias,
                                        const std::string& field) {
  const YAML::Node node = optionalAliasNode(root, key, alias);
  constexpr std::size_t kUpperJointCount = kPolicyJointCount - kLowerEndJointExclusive;
  if (!node || !node.IsSequence() || node.size() != kUpperJointCount) {
    std::ostringstream out;
    out << field << " must contain " << kUpperJointCount << " entries";
    throw error(out.str());
  }
  std::vector<float> values;
  values.reserve(kUpperJointCount);
  for (std::size_t i = 0; i < kUpperJointCount; ++i) {
    const float value = scalarAs<float>(node[i], field + "[" + std::to_string(i) + "]");
    if (!std::isfinite(value)) {
      throw error(field + "[" + std::to_string(i) + "] must be finite");
    }
    values.push_back(value);
  }
  return values;
}

std::vector<float> readGains(const std::vector<double>& values, const std::string& field) {
  requireSize(field, values.size(), kPolicyJointCount);
  std::vector<float> out;
  out.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    const float value = static_cast<float>(values.at(i));
    if (!std::isfinite(value)) {
      throw error(field + "[" + std::to_string(i) + "] must be finite");
    }
    out.push_back(value);
  }
  return out;
}

std::vector<float> readFiniteFloats(const std::vector<double>& values,
                                    std::size_t expected_size,
                                    const std::string& field) {
  requireSize(field, values.size(), expected_size);
  std::vector<float> out;
  out.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    const float value = static_cast<float>(values.at(i));
    if (!std::isfinite(value)) {
      throw error(field + "[" + std::to_string(i) + "] must be finite");
    }
    out.push_back(value);
  }
  return out;
}

}  // namespace

LocoUpperLowCmdComposeResult composeLocoUpperLowCmd(
    const LocoUpperLowCmdComposerConfig& config,
    const LowCmdFrame& lower_frame,
    const std::vector<float>& upper_joint_targets) {
  validateConfig(config);
  validateUpperTargets(config, upper_joint_targets);

  LocoUpperLowCmdComposeResult result;
  result.frame.mode_machine = config.expected_mode_machine;
  result.frame.mode_pr = 0;
  for (MotorCommand& motor : result.frame.motors) {
    motor = safeMotorCommand(config.unowned_safe);
  }

  applyLowerJoints(config, lower_frame, result.frame, result.lower_q_limited);
  result.lower_action_clamped = result.lower_q_limited;
  applyUpperJoints(config, upper_joint_targets, result.frame);
  return result;
}

LocoUpperLowCmdComposeResult composeLocoUpperLowCmd(
    const LocoUpperLowCmdComposerConfig& config,
    const LowCmdFrame& lower_frame,
    const TrkFrameView& upper_frame) {
  if (upper_frame.joint_pos.ptr == nullptr) {
    throw error("TRK frame joint_pos is missing");
  }
  requireSize("TRK frame joint_pos", upper_frame.joint_pos.size, kPolicyJointCount);
  const std::vector<float> targets(upper_frame.joint_pos.ptr,
                                   upper_frame.joint_pos.ptr +
                                       upper_frame.joint_pos.size);
  return composeLocoUpperLowCmd(config, lower_frame, targets);
}

LocoUpperLowCmdComposerConfig loadLocoUpperLowCmdComposerConfig(
    const std::filesystem::path& limits_path,
    const std::filesystem::path& joint_map_path,
    const DeployConfig& deploy_config,
    const LocoLowerDeployConfig& lower_deploy_config) {
  const YAML::Node limits = loadYamlMap(limits_path, "limits");
  const YAML::Node joint_map = loadYamlMap(joint_map_path, "joint_map");

  LocoUpperLowCmdComposerConfig config;
  config.logical_to_sdk =
      readIntVector(joint_map, "logical_to_sdk", kPolicyJointCount, "logical_to_sdk");
  if (!deploy_config.sdk_joint_ids_map.empty() &&
      config.logical_to_sdk != deploy_config.sdk_joint_ids_map) {
    throw error("logical_to_sdk must match DeployConfig.sdk_joint_ids_map");
  }

  config.upper_kp = readGains(deploy_config.policy_kp, "DeployConfig.policy_kp");
  config.upper_kd = readGains(deploy_config.policy_kd, "DeployConfig.policy_kd");
  requireSize("LocoLowerDeployConfig.joint_ids_map",
              lower_deploy_config.joint_ids_map.size(),
              kLowerEndJointExclusive);
  requireSize("LocoLowerDeployConfig.sdk_joint_ids_map",
              lower_deploy_config.sdk_joint_ids_map.size(),
              kLowerEndJointExclusive);
  for (std::size_t lower_joint = 0; lower_joint < kLowerEndJointExclusive; ++lower_joint) {
    const int logical = lower_deploy_config.joint_ids_map.at(lower_joint);
    if (logical < 0 || logical >= static_cast<int>(kLowerEndJointExclusive)) {
      std::ostringstream out;
      out << "LocoLowerDeployConfig.joint_ids_map[" << lower_joint
          << "] out of lower range: " << logical;
      throw error(out.str());
    }
    const int lower_sdk_slot =
        lower_deploy_config.sdk_joint_ids_map.at(static_cast<std::size_t>(logical));
    const int composer_sdk_slot = config.logical_to_sdk.at(lower_joint);
    if (lower_sdk_slot != composer_sdk_slot) {
      std::ostringstream out;
      out << "lower SDK slot map mismatch at logical joint " << lower_joint
          << ": composer=" << composer_sdk_slot
          << " lower_policy=" << lower_sdk_slot;
      throw error(out.str());
    }
  }
  config.lower_min_q = readFiniteFloats(lower_deploy_config.joint_min_q,
                                        kLowerEndJointExclusive,
                                        "LocoLowerDeployConfig.joint_min_q");
  config.lower_max_q = readFiniteFloats(lower_deploy_config.joint_max_q,
                                        kLowerEndJointExclusive,
                                        "LocoLowerDeployConfig.joint_max_q");
  config.upper_min_q.assign(kPolicyJointCount, 0.0F);
  config.upper_max_q.assign(kPolicyJointCount, 0.0F);
  config.upper_max_vel_radps.assign(kPolicyJointCount, 0.0F);
  config.upper_max_accel_radps2.assign(kPolicyJointCount, 0.0F);

  const std::vector<float> upper_min =
      readUpperLimitVector(limits, "upper_min_q", "joint_min", "upper_min_q");
  const std::vector<float> upper_max =
      readUpperLimitVector(limits, "upper_max_q", "joint_max", "upper_max_q");
  const std::vector<float> upper_max_vel =
      readUpperLimitVector(limits, "max_vel_radps", nullptr, "max_vel_radps");
  const std::vector<float> upper_max_accel = readUpperLimitVector(
      limits, "max_accel_radps2", nullptr, "max_accel_radps2");

  for (std::size_t i = 0; i < upper_min.size(); ++i) {
    const std::size_t logical = kLowerEndJointExclusive + i;
    config.upper_min_q.at(logical) = upper_min.at(i);
    config.upper_max_q.at(logical) = upper_max.at(i);
    config.upper_max_vel_radps.at(logical) = upper_max_vel.at(i);
    config.upper_max_accel_radps2.at(logical) = upper_max_accel.at(i);
  }
  validateConfig(config);
  return config;
}

}  // namespace agentic_et1_tracker
