#include "agentic_et1_tracker/policy/deploy_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointDim = 26;
constexpr double kStepDt = 0.02;
constexpr double kStepDtEpsilon = 1.0e-9;
constexpr std::size_t kClnFutureHorizon = 25;
constexpr std::size_t kClnFutureCommandWidth = 35;
constexpr std::size_t kClnFootstateFutureHorizon = 5;
constexpr std::size_t kClnFootstateFutureCommandWidth = 41;
constexpr std::size_t kDr3FutureHorizon = 5;
constexpr std::size_t kDr3FutureCommandWidth = 32;
constexpr std::array<int, kJointDim> kJointIdsMap{{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
}};
constexpr std::array<int, kJointDim> kSdkJointIdsMap{{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30,
}};

struct ObservationSpec {
  std::string_view name;
  std::size_t width;
};

constexpr std::array<ObservationSpec, 11> kCurrentObs{{
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
}};
constexpr std::array<ObservationSpec, 10> kHistoryObs{{
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
}};
constexpr std::array<ObservationSpec, 9> kClnCurrentObs{{
    {"command_yaw", 2},
    {"command_root_ori_b", 6},
    {"command_xy_yaw_vel", 3},
    {"command_jnt_pos", kJointDim},
    {"projected_gravity", 3},
    {"base_ang_vel", 3},
    {"joint_pos_rel", kJointDim},
    {"joint_vel_rel", kJointDim},
    {"last_action", kJointDim},
}};
constexpr std::array<ObservationSpec, 1> kClnHistoryObs{{
    {"future_commands", kClnFutureCommandWidth},
}};
constexpr std::array<ObservationSpec, 10> kClnFootstateCurrentObs{{
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
}};
constexpr std::array<ObservationSpec, 1> kClnFootstateHistoryObs{{
    {"future_command_with_foot_support_state", kClnFootstateFutureCommandWidth},
}};
constexpr std::array<ObservationSpec, 8> kDr3CurrentObs{{
    {"command_yaw", 2},
    {"command_root_ori_b", 6},
    {"command_jnt_pos", kJointDim},
    {"projected_gravity", 3},
    {"base_ang_vel", 3},
    {"joint_pos_rel", kJointDim},
    {"joint_vel_rel", kJointDim},
    {"last_action", kJointDim},
}};
constexpr std::array<ObservationSpec, 1> kDr3HistoryObs{{
    {"future_command", kDr3FutureCommandWidth},
}};

DeployConfigError error(const std::string& message) {
  return DeployConfigError("deploy config error: " + message);
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& e) {
    std::ostringstream out;
    out << field << " has invalid value: " << e.what();
    throw error(out.str());
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

template <std::size_t N>
bool contains(const std::array<ObservationSpec, N>& specs, std::string_view name) {
  return std::find_if(specs.begin(), specs.end(), [name](const ObservationSpec& spec) {
           return spec.name == name;
         }) != specs.end();
}

bool groupHasObservationKey(const YAML::Node& group, std::string_view name) {
  if (!group || !group.IsMap()) {
    return false;
  }
  for (const auto& entry : group) {
    const std::string key = scalarAs<std::string>(entry.first, "observation key");
    if (key == name) {
      return true;
    }
  }
  return false;
}

template <std::size_t N>
std::vector<std::string> toStrings(const std::array<ObservationSpec, N>& specs) {
  std::vector<std::string> out;
  out.reserve(specs.size());
  for (const ObservationSpec& spec : specs) {
    out.emplace_back(spec.name);
  }
  return out;
}

template <std::size_t N>
std::vector<ObservationTerm> toTerms(const std::array<ObservationSpec, N>& specs) {
  std::vector<ObservationTerm> out;
  out.reserve(specs.size());
  std::size_t offset = 0;
  for (const ObservationSpec& spec : specs) {
    out.push_back({std::string(spec.name), spec.width, offset});
    offset += spec.width;
  }
  return out;
}

template <std::size_t N>
std::size_t sumWidths(const std::array<ObservationSpec, N>& specs) {
  std::size_t total = 0;
  for (const ObservationSpec& spec : specs) {
    total += spec.width;
  }
  return total;
}

std::vector<int> readIntVector(const YAML::Node& root,
                               const char* key,
                               std::size_t expected_size) {
  const std::string field = key;
  const YAML::Node node = requiredNode(root, key, field);
  if (!node.IsSequence()) {
    throw error(field + " must be a sequence");
  }
  if (node.size() != expected_size) {
    std::ostringstream out;
    out << field << " must contain " << expected_size << " entries";
    throw error(out.str());
  }

  std::vector<int> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    values.push_back(scalarAs<int>(node[i], field + "[" + std::to_string(i) + "]"));
  }
  return values;
}

std::vector<double> readDoubleVector(const std::string& field,
                                     const YAML::Node& node,
                                     std::size_t expected_size,
                                     bool require_non_negative,
                                     bool require_positive) {
  if (!node || !node.IsSequence()) {
    throw error(field + " must be a sequence");
  }
  if (node.size() != expected_size) {
    std::ostringstream out;
    out << field << " must contain " << expected_size << " entries";
    throw error(out.str());
  }

  std::vector<double> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string element_field = field + "[" + std::to_string(i) + "]";
    const double value = scalarAs<double>(node[i], element_field);
    if (!std::isfinite(value)) {
      throw error(element_field + " must be finite");
    }
    if (require_positive && value <= 0.0) {
      throw error(element_field + " must be positive");
    }
    if (require_non_negative && value < 0.0) {
      throw error(element_field + " must be non-negative");
    }
    values.push_back(value);
  }
  return values;
}

std::vector<double> readRootDoubleVector(const YAML::Node& root,
                                         const char* key,
                                         bool require_non_negative,
                                         bool require_positive) {
  const std::string field = key;
  return readDoubleVector(field, requiredNode(root, key, field), kJointDim,
                          require_non_negative, require_positive);
}

std::vector<std::array<double, 2>> readOptionalActionClip(const YAML::Node& action) {
  const std::string field = "actions.JointPositionAction.clip";
  const YAML::Node node = action["clip"];
  if (!node || node.IsNull()) {
    return {};
  }
  if (!node.IsSequence()) {
    throw error(field + " must be null or a sequence");
  }
  if (node.size() != kJointDim) {
    std::ostringstream out;
    out << field << " must contain " << kJointDim << " entries";
    throw error(out.str());
  }

  std::vector<std::array<double, 2>> values;
  values.reserve(kJointDim);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string row_field = field + "[" + std::to_string(i) + "]";
    const YAML::Node row = node[i];
    if (!row.IsSequence()) {
      throw error(row_field + " must be a sequence");
    }
    if (row.size() != 2) {
      throw error(row_field + " must contain 2 entries");
    }
    std::array<double, 2> clip{};
    for (std::size_t j = 0; j < clip.size(); ++j) {
      const std::string element_field =
          row_field + "[" + std::to_string(j) + "]";
      const double value = scalarAs<double>(row[j], element_field);
      if (!std::isfinite(value)) {
        throw error(element_field + " must be finite");
      }
      clip[j] = value;
    }
    if (clip[0] > clip[1]) {
      throw error(row_field + " min must be <= max");
    }
    values.push_back(clip);
  }
  return values;
}

void validateJointIdsMap(const std::vector<int>& values) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] != kJointIdsMap[i]) {
      throw error("joint_ids_map must equal [0..25]");
    }
  }
}

void validateSdkJointIdsMap(const std::vector<int>& values) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] != kSdkJointIdsMap[i]) {
      throw error("sdk_joint_ids_map must equal the frozen GA general_tracker SDK map");
    }
  }
}

std::size_t historyLength(const YAML::Node& obs, const std::string& field) {
  const YAML::Node node = requiredNode(obs, "history_length", field + ".history_length");
  const long long value = scalarAs<long long>(node, field + ".history_length");
  if (value <= 0) {
    throw error(field + ".history_length must be positive");
  }
  return static_cast<std::size_t>(value);
}

std::size_t futureCommandsHorizon(const YAML::Node& obs_history) {
  const YAML::Node future_commands =
      requiredMap(obs_history, "future_commands", "observations.obs_history.future_commands");
  const YAML::Node params = future_commands["params"];
  if (params && !params.IsNull()) {
    if (!params.IsMap()) {
      throw error("observations.obs_history.future_commands.params must be a map");
    }
    const YAML::Node horizon = params["horizon"];
    if (horizon && !horizon.IsNull()) {
      const long long value =
          scalarAs<long long>(horizon, "observations.obs_history.future_commands.params.horizon");
      if (value != static_cast<long long>(kClnFutureHorizon)) {
        throw error("observations.obs_history.future_commands.params.horizon must be 25");
      }
    }
  }
  return kClnFutureHorizon;
}

std::size_t footstateFutureHorizon(const YAML::Node& obs_history) {
  const YAML::Node future_command =
      requiredMap(obs_history,
                  "future_command_with_foot_support_state",
                  "observations.obs_history.future_command_with_foot_support_state");
  const YAML::Node params =
      requiredMap(future_command,
                  "params",
                  "observations.obs_history.future_command_with_foot_support_state.params");
  const YAML::Node horizon =
      requiredNode(params,
                   "horizon",
                   "observations.obs_history.future_command_with_foot_support_state.params.horizon");
  const long long value = scalarAs<long long>(
      horizon,
      "observations.obs_history.future_command_with_foot_support_state.params.horizon");
  if (value != static_cast<long long>(kClnFootstateFutureHorizon)) {
    throw error(
        "observations.obs_history.future_command_with_foot_support_state.params.horizon must be 5");
  }
  return kClnFootstateFutureHorizon;
}

std::size_t dr3FutureHorizon(const YAML::Node& obs_history) {
  const YAML::Node future_command =
      requiredMap(obs_history, "future_command", "observations.obs_history.future_command");
  const YAML::Node params =
      requiredMap(future_command,
                  "params",
                  "observations.obs_history.future_command.params");
  const YAML::Node horizon =
      requiredNode(params,
                   "horizon",
                   "observations.obs_history.future_command.params.horizon");
  const long long value =
      scalarAs<long long>(horizon,
                          "observations.obs_history.future_command.params.horizon");
  if (value != static_cast<long long>(kDr3FutureHorizon)) {
    throw error("observations.obs_history.future_command.params.horizon must be 5");
  }
  return kDr3FutureHorizon;
}

template <std::size_t N>
void validateObsKeysInOrder(const YAML::Node& group,
                            const std::array<ObservationSpec, N>& allowed,
                            const std::string& field) {
  std::vector<std::string> actual;
  for (const auto& entry : group) {
    const std::string key = scalarAs<std::string>(entry.first, field + " key");
    if (key == "use_gym_history") {
      continue;
    }
    if (!contains(allowed, key)) {
      throw error(field + "." + key + " is not allowed for frozen GeneralTracker");
    }
    actual.push_back(key);
  }

  for (const ObservationSpec& spec : allowed) {
    const std::string expected(spec.name);
    if (std::find(actual.begin(), actual.end(), expected) == actual.end()) {
      throw error(field + "." + expected + " is required");
    }
  }

  if (actual.size() != allowed.size()) {
    std::ostringstream out;
    out << field << " must contain exactly " << allowed.size()
        << " frozen GA observations";
    throw error(out.str());
  }
  for (std::size_t i = 0; i < allowed.size(); ++i) {
    if (actual[i] != allowed[i].name) {
      std::ostringstream out;
      out << field << " must contain frozen GA observations in order; expected "
          << allowed[i].name << " at index " << i;
      throw error(out.str());
    }
  }
}

void validateUseGymHistory(const YAML::Node& group,
                           const std::string& field,
                           bool required_value) {
  const YAML::Node node =
      requiredNode(group, "use_gym_history", field + ".use_gym_history");
  const bool value = scalarAs<bool>(node, field + ".use_gym_history");
  if (value != required_value) {
    throw error(field + ".use_gym_history must be " +
                std::string(required_value ? "true" : "false"));
  }
}

void validateNullIfPresent(const YAML::Node& obs, const char* key, const std::string& field) {
  const YAML::Node node = obs[key];
  if (node && !node.IsNull()) {
    throw error(field + "." + key + " must be null or absent");
  }
}

template <std::size_t N>
std::size_t validateObservationGroup(const YAML::Node& group,
                                     const std::array<ObservationSpec, N>& allowed,
                                     const std::string& field,
                                     std::size_t required_history_length,
                                     bool required_use_gym_history = true) {
  validateUseGymHistory(group, field, required_use_gym_history);
  validateObsKeysInOrder(group, allowed, field);

  for (const ObservationSpec& spec : allowed) {
    const std::string name(spec.name);
    const std::string obs_field = field + "." + name;
    const YAML::Node obs = requiredMap(group, name.c_str(), obs_field);
    validateNullIfPresent(obs, "clip", obs_field);
    validateNullIfPresent(obs, "scale", obs_field);
    const std::size_t length = historyLength(obs, obs_field);
    if (length != required_history_length) {
      std::ostringstream out;
      out << obs_field << ".history_length must be " << required_history_length;
      throw error(out.str());
    }
  }

  return required_history_length;
}

void validateOptionalJointDim(const YAML::Node& root) {
  const YAML::Node node = root["joint_dim"];
  if (!node) {
    return;
  }
  const int value = scalarAs<int>(node, "joint_dim");
  if (value != static_cast<int>(kJointDim)) {
    throw error("joint_dim must be 26");
  }
}

double readStepDt(const YAML::Node& root) {
  const YAML::Node node = requiredNode(root, "step_dt", "step_dt");
  const double value = scalarAs<double>(node, "step_dt");
  if (!std::isfinite(value) || std::abs(value - kStepDt) > kStepDtEpsilon) {
    throw error("step_dt must equal 0.02 for the frozen 50Hz GeneralTracker profile");
  }
  return value;
}

void validateOnlyObservationGroups(const YAML::Node& observations) {
  for (const auto& entry : observations) {
    const std::string key = scalarAs<std::string>(entry.first, "observations key");
    if (key != "obs_current" && key != "obs_history") {
      throw error("observations." + key + " is not allowed");
    }
  }
}

void validateOnlyJointPositionAction(const YAML::Node& actions) {
  for (const auto& entry : actions) {
    const std::string key = scalarAs<std::string>(entry.first, "actions key");
    if (key != "JointPositionAction") {
      throw error("actions." + key + " is not allowed");
    }
  }
}

void validateActionJointIds(const YAML::Node& action) {
  const YAML::Node node = action["joint_ids"];
  if (node && !node.IsNull()) {
    throw error("actions.JointPositionAction.joint_ids must be null or absent");
  }
}

}  // namespace

DeployConfig loadDeployConfig(const std::filesystem::path& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root || !root.IsMap()) {
      throw error("root must be a map");
    }

    validateOptionalJointDim(root);

    DeployConfig config;
    config.joint_dim = kJointDim;
    config.joint_ids_map = readIntVector(root, "joint_ids_map", kJointDim);
    validateJointIdsMap(config.joint_ids_map);

    config.sdk_joint_ids_map = readIntVector(root, "sdk_joint_ids_map", kJointDim);
    validateSdkJointIdsMap(config.sdk_joint_ids_map);

    config.step_dt = readStepDt(root);
    config.policy_kp = readRootDoubleVector(root, "policy_kp", false, true);
    config.policy_kd = readRootDoubleVector(root, "policy_kd", false, true);
    config.default_joint_pos =
        readRootDoubleVector(root, "default_joint_pos", false, false);

    const YAML::Node actions = requiredMap(root, "actions", "actions");
    validateOnlyJointPositionAction(actions);
    const YAML::Node action =
        requiredMap(actions, "JointPositionAction", "actions.JointPositionAction");
    validateActionJointIds(action);
    config.action_scale =
        readDoubleVector("actions.JointPositionAction.scale",
                         requiredNode(action, "scale", "actions.JointPositionAction.scale"),
                         kJointDim, false, true);
    config.action_offset =
        readDoubleVector("actions.JointPositionAction.offset",
                         requiredNode(action, "offset", "actions.JointPositionAction.offset"),
                         kJointDim, false, false);
    config.action_clip = readOptionalActionClip(action);

    const YAML::Node observations = requiredMap(root, "observations", "observations");
    validateOnlyObservationGroups(observations);
    const YAML::Node obs_current =
        requiredMap(observations, "obs_current", "observations.obs_current");
    const YAML::Node obs_history =
        requiredMap(observations, "obs_history", "observations.obs_history");
    const bool is_cln_footstate =
        groupHasObservationKey(obs_history, "future_command_with_foot_support_state");
    const bool is_dr3 =
        groupHasObservationKey(obs_current, "command_yaw") &&
        groupHasObservationKey(obs_history, "future_command");
    const bool is_cln =
        groupHasObservationKey(obs_current, "command_yaw") &&
        groupHasObservationKey(obs_history, "future_commands");
    const bool looks_dr3_current =
        groupHasObservationKey(obs_current, "command_yaw") &&
        !groupHasObservationKey(obs_current, "command_xy_yaw_vel") &&
        !groupHasObservationKey(obs_current, "command_foot_support_state");
    if (looks_dr3_current && groupHasObservationKey(obs_history, "future_commands")) {
      throw error(
          "observations.obs_history.future_commands is not allowed for frozen GeneralTrackerDR3");
    }
    if (looks_dr3_current &&
        groupHasObservationKey(obs_history, "future_command_with_foot_support_state")) {
      throw error("observations.obs_history.future_command_with_foot_support_state is not "
                  "allowed for frozen GeneralTrackerDR3");
    }
    if (groupHasObservationKey(obs_history, "future_command") &&
        groupHasObservationKey(obs_current, "command_xy_yaw_vel")) {
      throw error(
          "observations.obs_current.command_xy_yaw_vel is not allowed for frozen GeneralTrackerDR3");
    }
    if (groupHasObservationKey(obs_history, "future_command") &&
        groupHasObservationKey(obs_current, "command_foot_support_state")) {
      throw error("observations.obs_current.command_foot_support_state is not allowed for "
                  "frozen GeneralTrackerDR3");
    }
    if (is_cln_footstate) {
      config.observation_contract = ObservationContract::GeneralTrackerCLNFootstate;
      validateObservationGroup(obs_current,
                               kClnFootstateCurrentObs,
                               "observations.obs_current",
                               1);
      validateObservationGroup(obs_history,
                               kClnFootstateHistoryObs,
                               "observations.obs_history",
                               1,
                               false);
      config.obs_history_length = footstateFutureHorizon(obs_history);
      config.obs_current = toStrings(kClnFootstateCurrentObs);
      config.obs_history = toStrings(kClnFootstateHistoryObs);
      config.obs_current_terms = toTerms(kClnFootstateCurrentObs);
      config.obs_history_terms = toTerms(kClnFootstateHistoryObs);
      config.obs_current_dim = sumWidths(kClnFootstateCurrentObs);
      config.obs_history_width = sumWidths(kClnFootstateHistoryObs);
    } else if (is_dr3) {
      config.observation_contract = ObservationContract::GeneralTrackerDR3;
      validateObservationGroup(obs_current,
                               kDr3CurrentObs,
                               "observations.obs_current",
                               1);
      validateObservationGroup(obs_history,
                               kDr3HistoryObs,
                               "observations.obs_history",
                               1,
                               false);
      config.obs_history_length = dr3FutureHorizon(obs_history);
      config.obs_current = toStrings(kDr3CurrentObs);
      config.obs_history = toStrings(kDr3HistoryObs);
      config.obs_current_terms = toTerms(kDr3CurrentObs);
      config.obs_history_terms = toTerms(kDr3HistoryObs);
      config.obs_current_dim = sumWidths(kDr3CurrentObs);
      config.obs_history_width = sumWidths(kDr3HistoryObs);
    } else if (is_cln) {
      config.observation_contract = ObservationContract::GeneralTrackerCLN;
      validateObservationGroup(obs_current, kClnCurrentObs, "observations.obs_current", 1);
      validateObservationGroup(obs_history, kClnHistoryObs, "observations.obs_history", 1);
      config.obs_history_length = futureCommandsHorizon(obs_history);
      config.obs_current = toStrings(kClnCurrentObs);
      config.obs_history = toStrings(kClnHistoryObs);
      config.obs_current_terms = toTerms(kClnCurrentObs);
      config.obs_history_terms = toTerms(kClnHistoryObs);
      config.obs_current_dim = sumWidths(kClnCurrentObs);
      config.obs_history_width = sumWidths(kClnHistoryObs);
    } else {
      config.observation_contract = ObservationContract::GeneralTracker;
      validateObservationGroup(obs_current, kCurrentObs, "observations.obs_current", 1);
      config.obs_history_length =
          validateObservationGroup(obs_history, kHistoryObs, "observations.obs_history", 25);
      config.obs_current = toStrings(kCurrentObs);
      config.obs_history = toStrings(kHistoryObs);
      config.obs_current_terms = toTerms(kCurrentObs);
      config.obs_history_terms = toTerms(kHistoryObs);
      config.obs_current_dim = sumWidths(kCurrentObs);
      config.obs_history_width = sumWidths(kHistoryObs);
    }

    return config;
  } catch (const DeployConfigError&) {
    throw;
  } catch (const YAML::Exception& e) {
    throw error(e.what());
  }
}

}  // namespace agentic_et1_tracker
