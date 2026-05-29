#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace agentic_et1_tracker {
namespace {

constexpr double kStepDt = 0.02;
constexpr double kStepDtEpsilon = 1.0e-9;

struct ExpectedTerm {
  std::string_view name;
  std::size_t width;
};

constexpr std::array<ExpectedTerm, 6> kTerms{{
    {"base_ang_vel", 3},
    {"projected_gravity", 3},
    {"keyboard_velocity_commands", 3},
    {"joint_pos_rel", kVelocityPolicyJointDim},
    {"joint_vel_rel", kVelocityPolicyJointDim},
    {"last_action", kVelocityPolicyJointDim},
}};

VelocityDeployConfigError error(const std::string& message) {
  return VelocityDeployConfigError("velocity deploy config error: " + message);
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& err) {
    throw error(field + " has invalid value: " + err.what());
  }
}

YAML::Node requiredNode(const YAML::Node& map, const char* key, const std::string& field) {
  const YAML::Node node = map[key];
  if (!node) {
    throw error(field + " is required");
  }
  return node;
}

YAML::Node requiredMap(const YAML::Node& map, const char* key, const std::string& field) {
  const YAML::Node node = requiredNode(map, key, field);
  if (!node.IsMap()) {
    throw error(field + " must be a map");
  }
  return node;
}

std::vector<int> readIntVector(const YAML::Node& root,
                               const char* key,
                               std::size_t expected_size) {
  const YAML::Node node = requiredNode(root, key, key);
  if (!node.IsSequence() || node.size() != expected_size) {
    std::ostringstream out;
    out << key << " must contain " << expected_size << " entries";
    throw error(out.str());
  }

  std::vector<int> out;
  out.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    out.push_back(scalarAs<int>(node[i], std::string(key) + "[" + std::to_string(i) + "]"));
  }
  return out;
}

std::vector<double> readDoubleVector(const std::string& field,
                                     const YAML::Node& node,
                                     std::size_t expected_size,
                                     bool require_non_negative,
                                     bool require_positive) {
  if (!node || !node.IsSequence() || node.size() != expected_size) {
    std::ostringstream out;
    out << field << " must contain " << expected_size << " entries";
    throw error(out.str());
  }

  std::vector<double> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string element = field + "[" + std::to_string(i) + "]";
    const double value = scalarAs<double>(node[i], element);
    if (!std::isfinite(value)) {
      throw error(element + " must be finite");
    }
    if (require_positive && value <= 0.0) {
      throw error(element + " must be positive");
    }
    if (require_non_negative && value < 0.0) {
      throw error(element + " must be non-negative");
    }
    values.push_back(value);
  }
  return values;
}

std::vector<double> readRootDoubleVector(const YAML::Node& root,
                                         const char* key,
                                         bool require_non_negative,
                                         bool require_positive) {
  return readDoubleVector(key, requiredNode(root, key, key),
                          kVelocityPolicyJointDim,
                          require_non_negative,
                          require_positive);
}

double readStepDt(const YAML::Node& root) {
  const double value = scalarAs<double>(requiredNode(root, "step_dt", "step_dt"), "step_dt");
  if (!std::isfinite(value) || std::abs(value - kStepDt) > kStepDtEpsilon) {
    throw error("step_dt must equal 0.02");
  }
  return value;
}

std::size_t readHistoryLength(const YAML::Node& term, const std::string& field) {
  const long long value =
      scalarAs<long long>(requiredNode(term, "history_length", field + ".history_length"),
                          field + ".history_length");
  if (value != static_cast<long long>(kVelocityPolicyHistoryLength)) {
    throw error(field + ".history_length must be 5");
  }
  return static_cast<std::size_t>(value);
}

std::vector<double> readScale(const YAML::Node& term,
                              const std::string& field,
                              std::size_t width) {
  const YAML::Node scale = requiredNode(term, "scale", field + ".scale");
  return readDoubleVector(field + ".scale", scale, width, false, false);
}

void validateNoGeneralTrackerSchema(const YAML::Node& root) {
  const YAML::Node observations = requiredMap(root, "observations", "observations");
  if (observations["obs_current"] || observations["obs_history"]) {
    throw error("GeneralTracker observation groups are not allowed");
  }
}

void validateSdkMap(const VelocityDeployConfig& config) {
  const std::vector<int>& slots =
      config.sdk_joint_ids_map.empty() ? config.joint_ids_map : config.sdk_joint_ids_map;
  for (std::size_t i = 0; i < slots.size(); ++i) {
    if (slots[i] < 0 || slots[i] >= 35) {
      throw error("sdk joint slot out of range");
    }
  }
}

}  // namespace

VelocityDeployConfig loadVelocityDeployConfig(const std::filesystem::path& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root || !root.IsMap()) {
      throw error("root must be a map");
    }
    validateNoGeneralTrackerSchema(root);

    VelocityDeployConfig config;
    config.joint_ids_map =
        readIntVector(root, "joint_ids_map", kVelocityPolicyJointDim);
    const YAML::Node sdk_map = root["sdk_joint_ids_map"];
    if (sdk_map) {
      config.sdk_joint_ids_map =
          readIntVector(root, "sdk_joint_ids_map", kVelocityPolicyJointDim);
    }
    config.step_dt = readStepDt(root);
    config.stiffness = readRootDoubleVector(root, "stiffness", true, false);
    config.damping = readRootDoubleVector(root, "damping", true, false);
    config.default_joint_pos =
        readRootDoubleVector(root, "default_joint_pos", false, false);

    const YAML::Node actions = requiredMap(root, "actions", "actions");
    const YAML::Node action =
        requiredMap(actions, "JointPositionAction", "actions.JointPositionAction");
    config.action_scale =
        readDoubleVector("actions.JointPositionAction.scale",
                         requiredNode(action, "scale", "actions.JointPositionAction.scale"),
                         kVelocityPolicyJointDim,
                         false,
                         true);
    config.action_offset =
        readDoubleVector("actions.JointPositionAction.offset",
                         requiredNode(action, "offset", "actions.JointPositionAction.offset"),
                         kVelocityPolicyJointDim,
                         false,
                         false);

    const YAML::Node observations = requiredMap(root, "observations", "observations");
    std::size_t offset = 0;
    for (const ExpectedTerm& expected : kTerms) {
      const std::string name(expected.name);
      const YAML::Node term = requiredMap(observations, name.c_str(), "observations." + name);
      readHistoryLength(term, "observations." + name);
      VelocityObservationTerm out;
      out.name = name;
      out.width = expected.width;
      out.offset = offset;
      out.scale = readScale(term, "observations." + name, expected.width);
      config.observation_terms.push_back(std::move(out));
      offset += expected.width;
    }
    if (offset != kVelocityPolicyObsRowWidth) {
      throw error("observation row width mismatch");
    }
    config.obs_row_width = offset;
    config.obs_history_length = kVelocityPolicyHistoryLength;
    config.obs_dim = config.obs_row_width * config.obs_history_length;
    validateSdkMap(config);
    return config;
  } catch (const VelocityDeployConfigError&) {
    throw;
  } catch (const YAML::Exception& err) {
    throw error(err.what());
  }
}

}  // namespace agentic_et1_tracker
