#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

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
    {"joint_pos_rel", kLocoLowerPolicyJointDim},
    {"joint_vel_rel", kLocoLowerPolicyJointDim},
    {"last_action", kLocoLowerPolicyJointDim},
}};

LocoLowerDeployConfigError configError(const std::string& message) {
  return LocoLowerDeployConfigError("loco lower deploy config error: " + message);
}

LocoLowerPolicyError policyError(const std::string& message) {
  return LocoLowerPolicyError("loco lower policy error: " + message);
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& err) {
    throw configError(field + " has invalid value: " + err.what());
  }
}

YAML::Node requiredNode(const YAML::Node& map,
                        const char* key,
                        const std::string& field) {
  const YAML::Node node = map[key];
  if (!node) {
    throw configError(field + " is required");
  }
  return node;
}

YAML::Node requiredMap(const YAML::Node& map,
                       const char* key,
                       const std::string& field) {
  const YAML::Node node = requiredNode(map, key, field);
  if (!node.IsMap()) {
    throw configError(field + " must be a map");
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
    throw configError(out.str());
  }

  std::vector<int> out;
  out.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    out.push_back(scalarAs<int>(node[i], std::string(key) + "[" + std::to_string(i) + "]"));
  }
  return out;
}

std::vector<int> lowerIdentityMap() {
  std::vector<int> out;
  out.reserve(kLocoLowerPolicyJointDim);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    out.push_back(static_cast<int>(i));
  }
  return out;
}

std::vector<int> readOptionalIntVector(const YAML::Node& root,
                                       const char* key,
                                       std::size_t expected_size,
                                       const std::vector<int>& fallback) {
  if (!root[key]) {
    return fallback;
  }
  return readIntVector(root, key, expected_size);
}

std::vector<double> readDoubleVector(const std::string& field,
                                     const YAML::Node& node,
                                     std::size_t expected_size,
                                     bool require_non_negative,
                                     bool require_positive) {
  if (!node || !node.IsSequence() || node.size() != expected_size) {
    std::ostringstream out;
    out << field << " must contain " << expected_size << " entries";
    throw configError(out.str());
  }

  std::vector<double> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string element = field + "[" + std::to_string(i) + "]";
    const double value = scalarAs<double>(node[i], element);
    if (!std::isfinite(value)) {
      throw configError(element + " must be finite");
    }
    if (require_positive && value <= 0.0) {
      throw configError(element + " must be positive");
    }
    if (require_non_negative && value < 0.0) {
      throw configError(element + " must be non-negative");
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
                          kLocoLowerPolicyJointDim,
                          require_non_negative,
                          require_positive);
}

void requireSize(const std::string& field,
                 std::size_t actual,
                 std::size_t expected) {
  if (actual != expected) {
    std::ostringstream out;
    out << field << " must contain " << expected << " entries, got " << actual;
    throw policyError(out.str());
  }
}

void requireConfigSize(const std::string& field,
                       std::size_t actual,
                       std::size_t expected) {
  if (actual != expected) {
    std::ostringstream out;
    out << field << " must contain " << expected << " entries, got " << actual;
    throw configError(out.str());
  }
}

void requireFinite(const std::string& field, const Vec& values) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      throw policyError(field + "[" + std::to_string(i) + "] must be finite");
    }
  }
}

double readStepDt(const YAML::Node& root) {
  const double value = scalarAs<double>(requiredNode(root, "step_dt", "step_dt"), "step_dt");
  if (!std::isfinite(value) || std::abs(value - kStepDt) > kStepDtEpsilon) {
    throw configError("step_dt must equal 0.02");
  }
  return value;
}

std::size_t readPolicyDecimation(const YAML::Node& root) {
  const long long value =
      scalarAs<long long>(requiredNode(root, "policy_decimation", "policy_decimation"),
                          "policy_decimation");
  if (value != 10) {
    throw configError("policy_decimation must equal 10");
  }
  return static_cast<std::size_t>(value);
}

std::size_t readHistoryLength(const YAML::Node& term, const std::string& field) {
  const long long value =
      scalarAs<long long>(requiredNode(term, "history_length", field + ".history_length"),
                          field + ".history_length");
  if (value != static_cast<long long>(kLocoLowerPolicyHistoryLength)) {
    throw configError(field + ".history_length must be 1");
  }
  return static_cast<std::size_t>(value);
}

std::vector<double> readScale(const YAML::Node& term,
                              const std::string& field,
                              std::size_t width) {
  return readDoubleVector(field + ".scale",
                          requiredNode(term, "scale", field + ".scale"),
                          width,
                          false,
                          false);
}

LocoLowerRange readRange(const YAML::Node& ranges, const char* key) {
  const YAML::Node node =
      requiredNode(ranges, key, std::string("commands.base_velocity.ranges.") + key);
  if (!node.IsSequence() || node.size() != 2) {
    throw configError(std::string("commands.base_velocity.ranges.") + key +
                      " must contain 2 entries");
  }
  LocoLowerRange range;
  range.min = scalarAs<double>(node[0], std::string(key) + "[0]");
  range.max = scalarAs<double>(node[1], std::string(key) + "[1]");
  if (!std::isfinite(range.min) || !std::isfinite(range.max) || range.min > range.max) {
    throw configError(std::string("commands.base_velocity.ranges.") + key +
                      " must be finite and ordered");
  }
  return range;
}

LocoLowerCommandRanges readCommandRanges(const YAML::Node& root) {
  const YAML::Node commands = requiredMap(root, "commands", "commands");
  const YAML::Node base_velocity =
      requiredMap(commands, "base_velocity", "commands.base_velocity");
  const YAML::Node ranges =
      requiredMap(base_velocity, "ranges", "commands.base_velocity.ranges");
  LocoLowerCommandRanges out;
  out.lin_vel_x = readRange(ranges, "lin_vel_x");
  out.lin_vel_y = readRange(ranges, "lin_vel_y");
  out.ang_vel_z = readRange(ranges, "ang_vel_z");
  return out;
}

std::optional<std::vector<LocoLowerRange>> readOptionalActionClip(
    const YAML::Node& action) {
  const YAML::Node clip = action["clip"];
  if (!clip || clip.IsNull()) {
    return std::nullopt;
  }
  if (!clip.IsSequence() || clip.size() != kLocoLowerPolicyJointDim) {
    std::ostringstream out;
    out << "actions.JointPositionAction.clip must contain "
        << kLocoLowerPolicyJointDim << " entries";
    throw configError(out.str());
  }
  std::vector<LocoLowerRange> ranges;
  ranges.reserve(kLocoLowerPolicyJointDim);
  for (std::size_t i = 0; i < clip.size(); ++i) {
    const YAML::Node row = clip[i];
    if (!row.IsSequence() || row.size() != 2) {
      throw configError("actions.JointPositionAction.clip[" + std::to_string(i) +
                        "] must contain 2 entries");
    }
    const double min =
        scalarAs<double>(row[0], "actions.JointPositionAction.clip[" +
                                    std::to_string(i) + "][0]");
    const double max =
        scalarAs<double>(row[1], "actions.JointPositionAction.clip[" +
                                    std::to_string(i) + "][1]");
    if (!std::isfinite(min) || !std::isfinite(max) || min > max) {
      throw configError("actions.JointPositionAction.clip[" + std::to_string(i) +
                        "] must be finite and ordered");
    }
    ranges.push_back({min, max});
  }
  return ranges;
}

void validateVectorEquals(const std::string& field,
                          const std::vector<int>& actual,
                          const std::vector<int>& expected) {
  if (actual != expected) {
    throw configError(field + " must match joint_ids_map");
  }
}

void validateIdentityMapConfig(const std::string& field,
                               const std::vector<int>& actual) {
  const std::vector<int> expected = lowerIdentityMap();
  if (actual != expected) {
    throw configError(field + " must be [0..11]");
  }
}

void validateIdentityMapPolicy(const std::string& field,
                               const std::vector<int>& actual) {
  const std::vector<int> expected = lowerIdentityMap();
  if (actual != expected) {
    throw policyError(field + " must be [0..11]");
  }
}

void validateJointIds(const YAML::Node& node,
                      const std::string& field,
                      const std::vector<int>& expected) {
  if (!node || !node.IsSequence() || node.size() != expected.size()) {
    throw configError(field + " must contain 12 entries");
  }
  std::vector<int> actual;
  actual.reserve(expected.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    actual.push_back(scalarAs<int>(node[i], field + "[" + std::to_string(i) + "]"));
  }
  validateVectorEquals(field, actual, expected);
}

void validateDeployConfig(const LocoLowerDeployConfig& config) {
  requireConfigSize("joint_dim", config.joint_dim, kLocoLowerPolicyJointDim);
  requireConfigSize("joint_ids_map", config.joint_ids_map.size(), kLocoLowerPolicyJointDim);
  requireConfigSize("sdk_joint_ids_map", config.sdk_joint_ids_map.size(),
                    kLocoLowerPolicyJointDim);
  requireConfigSize("stiffness", config.stiffness.size(), kLocoLowerPolicyJointDim);
  requireConfigSize("damping", config.damping.size(), kLocoLowerPolicyJointDim);
  requireConfigSize("default_joint_pos", config.default_joint_pos.size(),
                    kLocoLowerPolicyJointDim);
  requireConfigSize("joint_min_q", config.joint_min_q.size(), kLocoLowerPolicyJointDim);
  requireConfigSize("joint_max_q", config.joint_max_q.size(), kLocoLowerPolicyJointDim);
  if (config.action_clip.has_value()) {
    requireConfigSize("action_clip", config.action_clip->size(), kLocoLowerPolicyJointDim);
  }
  requireConfigSize("action_scale", config.action_scale.size(), kLocoLowerPolicyJointDim);
  requireConfigSize("action_offset", config.action_offset.size(), kLocoLowerPolicyJointDim);
  requireConfigSize("observation_terms", config.observation_terms.size(), kTerms.size());
  requireConfigSize("obs_row_width", config.obs_row_width, kLocoLowerPolicyObsRowWidth);
  requireConfigSize("obs_history_length", config.obs_history_length,
                    kLocoLowerPolicyHistoryLength);
  requireConfigSize("obs_dim", config.obs_dim, kLocoLowerPolicyObsDim);
  if (config.policy_decimation == 0) {
    throw configError("policy_decimation must be positive");
  }
  validateIdentityMapConfig("joint_ids_map", config.joint_ids_map);
  validateIdentityMapConfig("sdk_joint_ids_map", config.sdk_joint_ids_map);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const double min_q = config.joint_min_q.at(i);
    const double max_q = config.joint_max_q.at(i);
    if (!std::isfinite(min_q) || !std::isfinite(max_q) || min_q > max_q) {
      throw configError("joint q limits must be finite and ordered");
    }
  }
}

void validateConfig(const LocoLowerDeployConfig& config) {
  requireSize("joint_dim", config.joint_dim, kLocoLowerPolicyJointDim);
  requireSize("joint_ids_map", config.joint_ids_map.size(), kLocoLowerPolicyJointDim);
  requireSize("sdk_joint_ids_map", config.sdk_joint_ids_map.size(), kLocoLowerPolicyJointDim);
  requireSize("stiffness", config.stiffness.size(), kLocoLowerPolicyJointDim);
  requireSize("damping", config.damping.size(), kLocoLowerPolicyJointDim);
  requireSize("default_joint_pos", config.default_joint_pos.size(), kLocoLowerPolicyJointDim);
  requireSize("joint_min_q", config.joint_min_q.size(), kLocoLowerPolicyJointDim);
  requireSize("joint_max_q", config.joint_max_q.size(), kLocoLowerPolicyJointDim);
  if (config.action_clip.has_value()) {
    requireSize("action_clip", config.action_clip->size(), kLocoLowerPolicyJointDim);
  }
  requireSize("action_scale", config.action_scale.size(), kLocoLowerPolicyJointDim);
  requireSize("action_offset", config.action_offset.size(), kLocoLowerPolicyJointDim);
  requireSize("observation_terms", config.observation_terms.size(), kTerms.size());
  requireSize("obs_row_width", config.obs_row_width, kLocoLowerPolicyObsRowWidth);
  requireSize("obs_history_length", config.obs_history_length, kLocoLowerPolicyHistoryLength);
  requireSize("obs_dim", config.obs_dim, kLocoLowerPolicyObsDim);
  if (config.policy_decimation == 0) {
    throw policyError("policy_decimation must be positive");
  }
  validateIdentityMapPolicy("joint_ids_map", config.joint_ids_map);
  validateIdentityMapPolicy("sdk_joint_ids_map", config.sdk_joint_ids_map);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const double min_q = config.joint_min_q.at(i);
    const double max_q = config.joint_max_q.at(i);
    if (!std::isfinite(min_q) || !std::isfinite(max_q) || min_q > max_q) {
      throw policyError("joint q limits must be finite and ordered");
    }
  }
}

std::size_t sdkSlot(const LocoLowerDeployConfig& config, std::size_t policy_index) {
  const int logical = config.joint_ids_map.at(policy_index);
  return static_cast<std::size_t>(
      config.sdk_joint_ids_map.at(static_cast<std::size_t>(logical)));
}

Vec projectedGravity(const LowStateSample& low_state) {
  const float w = low_state.quat_wxyz[0];
  const float x = low_state.quat_wxyz[1];
  const float y = low_state.quat_wxyz[2];
  const float z = low_state.quat_wxyz[3];
  const float norm_sq = w * w + x * x + y * y + z * z;
  if (!std::isfinite(norm_sq) || norm_sq <= kMinSafeQuaternionNormSquared) {
    throw policyError("low_state.quat_wxyz must be a finite non-zero quaternion");
  }
  const float inv = 1.0F / norm_sq;
  return {
      2.0F * (w * y - x * z) * inv,
      -2.0F * (w * x + y * z) * inv,
      -(w * w - x * x - y * y + z * z) * inv,
  };
}

Vec termValues(const LocoLowerDeployConfig& config,
               const LocoLowerObservationTerm& term,
               const LowStateSample& low_state,
               const Vec& last_action,
               VelocityCommand command) {
  Vec values;
  if (term.name == "base_ang_vel") {
    values = {low_state.gyro[0], low_state.gyro[1], low_state.gyro[2]};
  } else if (term.name == "projected_gravity") {
    values = projectedGravity(low_state);
  } else if (term.name == "keyboard_velocity_commands") {
    values = {command.vx, command.vy, command.yaw_rate};
  } else if (term.name == "joint_pos_rel") {
    values.reserve(kLocoLowerPolicyJointDim);
    for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
      const std::size_t slot = sdkSlot(config, i);
      values.push_back(low_state.motors.at(slot).q -
                       static_cast<float>(config.default_joint_pos.at(i)));
    }
  } else if (term.name == "joint_vel_rel") {
    values.reserve(kLocoLowerPolicyJointDim);
    for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
      values.push_back(low_state.motors.at(sdkSlot(config, i)).dq);
    }
  } else if (term.name == "last_action") {
    values = last_action;
  } else {
    throw policyError("unknown observation term '" + term.name + "'");
  }

  requireSize("observation." + term.name, values.size(), term.width);
  requireSize("observation." + term.name + ".scale", term.scale.size(), term.width);
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] *= static_cast<float>(term.scale.at(i));
  }
  requireFinite("observation." + term.name, values);
  return values;
}

float clampToRange(float value, const LocoLowerRange& range, bool& clamped) {
  const float bounded =
      std::clamp(value, static_cast<float>(range.min), static_cast<float>(range.max));
  if (bounded != value) {
    clamped = true;
  }
  return bounded;
}

VelocityCommand clampCommand(const LocoLowerCommandRanges& ranges,
                             VelocityCommand command,
                             bool& clamped) {
  command.vx = clampToRange(command.vx, ranges.lin_vel_x, clamped);
  command.vy = clampToRange(command.vy, ranges.lin_vel_y, clamped);
  command.yaw_rate = clampToRange(command.yaw_rate, ranges.ang_vel_z, clamped);
  return command;
}

Vec processedAction(const LocoLowerDeployConfig& config, const Vec& raw_action) {
  Vec processed;
  processed.reserve(kLocoLowerPolicyJointDim);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    processed.push_back(raw_action.at(i) * static_cast<float>(config.action_scale.at(i)) +
                        static_cast<float>(config.action_offset.at(i)));
  }
  return processed;
}

Vec applyActionClip(const LocoLowerDeployConfig& config,
                    const Vec& raw_action,
                    bool& clamped) {
  Vec action = raw_action;
  if (!config.action_clip.has_value()) {
    return action;
  }
  for (std::size_t i = 0; i < action.size(); ++i) {
    float& value = action.at(i);
    const auto& range = config.action_clip->at(i);
    const float bounded =
        std::clamp(value, static_cast<float>(range.min), static_cast<float>(range.max));
    if (bounded != value) {
      clamped = true;
      value = bounded;
    }
  }
  return action;
}

using Sha256State = std::array<std::uint32_t, 8>;

constexpr std::array<std::uint32_t, 64> kSha256RoundConstants{{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
}};

std::uint32_t rotr(std::uint32_t value, int bits) {
  return (value >> bits) | (value << (32 - bits));
}

std::uint32_t readBigEndian32(const unsigned char* data) {
  return (static_cast<std::uint32_t>(data[0]) << 24) |
         (static_cast<std::uint32_t>(data[1]) << 16) |
         (static_cast<std::uint32_t>(data[2]) << 8) |
         static_cast<std::uint32_t>(data[3]);
}

void processSha256Block(Sha256State& state, const unsigned char* block) {
  std::array<std::uint32_t, 64> w{};
  for (std::size_t i = 0; i < 16; ++i) {
    w[i] = readBigEndian32(block + i * 4);
  }
  for (std::size_t i = 16; i < 64; ++i) {
    const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }

  std::uint32_t a = state[0];
  std::uint32_t b = state[1];
  std::uint32_t c = state[2];
  std::uint32_t d = state[3];
  std::uint32_t e = state[4];
  std::uint32_t f = state[5];
  std::uint32_t g = state[6];
  std::uint32_t h = state[7];

  for (std::size_t i = 0; i < 64; ++i) {
    const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
    const std::uint32_t ch = (e & f) ^ ((~e) & g);
    const std::uint32_t temp1 = h + s1 + ch + kSha256RoundConstants[i] + w[i];
    const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
    const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const std::uint32_t temp2 = s0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + temp1;
    d = c;
    c = b;
    b = a;
    a = temp1 + temp2;
  }

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;
}

}  // namespace

LocoLowerDeployConfig loadLocoLowerDeployConfig(const std::filesystem::path& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root || !root.IsMap()) {
      throw configError("root must be a map");
    }
    const YAML::Node observations = requiredMap(root, "observations", "observations");
    const YAML::Node obs = requiredMap(observations, "obs", "observations.obs");

    LocoLowerDeployConfig config;
    config.policy_decimation = readPolicyDecimation(root);
    config.step_dt = readStepDt(root);
    config.joint_ids_map = readIntVector(root, "joint_ids_map", kLocoLowerPolicyJointDim);
    config.sdk_joint_ids_map = readOptionalIntVector(root,
                                                     "sdk_joint_ids_map",
                                                     kLocoLowerPolicyJointDim,
                                                     lowerIdentityMap());
    config.stiffness = readRootDoubleVector(root, "stiffness", true, false);
    config.damping = readRootDoubleVector(root, "damping", true, false);
    config.default_joint_pos =
        readRootDoubleVector(root, "default_joint_pos", false, false);
    config.joint_min_q = readRootDoubleVector(root, "joint_min_q", false, false);
    config.joint_max_q = readRootDoubleVector(root, "joint_max_q", false, false);
    config.command_ranges = readCommandRanges(root);

    const YAML::Node actions = requiredMap(root, "actions", "actions");
    const YAML::Node action =
        requiredMap(actions, "JointPositionAction", "actions.JointPositionAction");
    config.action_clip = readOptionalActionClip(action);
    config.action_scale =
        readDoubleVector("actions.JointPositionAction.scale",
                         requiredNode(action, "scale", "actions.JointPositionAction.scale"),
                         kLocoLowerPolicyJointDim,
                         false,
                         true);
    config.action_offset =
        readDoubleVector("actions.JointPositionAction.offset",
                         requiredNode(action, "offset", "actions.JointPositionAction.offset"),
                         kLocoLowerPolicyJointDim,
                         false,
                         false);
    validateJointIds(requiredNode(action, "joint_ids", "actions.JointPositionAction.joint_ids"),
                     "actions.JointPositionAction.joint_ids",
                     config.joint_ids_map);

    std::size_t offset = 0;
    for (const ExpectedTerm& expected : kTerms) {
      const std::string name(expected.name);
      const YAML::Node term = requiredMap(obs, name.c_str(), "observations.obs." + name);
      readHistoryLength(term, "observations.obs." + name);
      if (name == "joint_pos_rel" || name == "joint_vel_rel") {
        const YAML::Node params =
            requiredMap(term, "params", "observations.obs." + name + ".params");
        const YAML::Node asset_cfg =
            requiredMap(params, "asset_cfg", "observations.obs." + name + ".params.asset_cfg");
        validateJointIds(requiredNode(asset_cfg, "joint_ids",
                                      "observations.obs." + name +
                                          ".params.asset_cfg.joint_ids"),
                         "observations.obs." + name + ".params.asset_cfg.joint_ids",
                         config.joint_ids_map);
      }

      LocoLowerObservationTerm out;
      out.name = name;
      out.width = expected.width;
      out.offset = offset;
      out.scale = readScale(term, "observations.obs." + name, expected.width);
      config.observation_terms.push_back(std::move(out));
      offset += expected.width;
    }
    if (offset != kLocoLowerPolicyObsRowWidth) {
      throw configError("observation row width mismatch");
    }
    config.obs_row_width = offset;
    config.obs_history_length = kLocoLowerPolicyHistoryLength;
    config.obs_dim = kLocoLowerPolicyObsDim;
    validateDeployConfig(config);
    return config;
  } catch (const LocoLowerDeployConfigError&) {
    throw;
  } catch (const YAML::Exception& err) {
    throw configError(err.what());
  }
}

VelocityPolicyInputs makeLocoLowerPolicyInputs(const LocoLowerDeployConfig& config,
                                               const LowStateSample& low_state,
                                               const Vec& last_action,
                                               VelocityCommand command) {
  validateConfig(config);
  requireSize("last_action", last_action.size(), kLocoLowerPolicyJointDim);

  VelocityPolicyInputs inputs;
  inputs.obs.reserve(kLocoLowerPolicyObsDim);
  std::size_t offset = 0;
  for (const LocoLowerObservationTerm& term : config.observation_terms) {
    if (term.offset != offset) {
      throw policyError("observation term offsets must be contiguous");
    }
    Vec values = termValues(config, term, low_state, last_action, command);
    inputs.obs.insert(inputs.obs.end(), values.begin(), values.end());
    offset += values.size();
  }
  requireSize("obs", inputs.obs.size(), kLocoLowerPolicyObsDim);
  return inputs;
}

LowCmdFrame makeLocoLowerLowCmdFrame(const LocoLowerDeployConfig& config,
                                     const Vec& raw_action,
                                     std::uint8_t expected_mode_machine,
                                     const LowCmdFrame* base_frame) {
  validateConfig(config);
  requireSize("raw_action", raw_action.size(), kLocoLowerPolicyJointDim);
  requireFinite("raw_action", raw_action);
  bool raw_action_clamped = false;
  const Vec safe_action = applyActionClip(config, raw_action, raw_action_clamped);
  const Vec processed_action = processedAction(config, safe_action);

  LowCmdFrame frame = base_frame == nullptr ? LowCmdFrame{} : *base_frame;
  frame.mode_machine = expected_mode_machine;
  frame.mode_pr = 0;
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    MotorCommand& motor = frame.motors.at(sdkSlot(config, i));
    motor.mode = 1;
    motor.q = processed_action.at(i);
    motor.dq = 0.0F;
    motor.kp = static_cast<float>(config.stiffness.at(i));
    motor.kd = static_cast<float>(config.damping.at(i));
    motor.tau = 0.0F;
  }
  return frame;
}

LocoLowerStepRunner::LocoLowerStepRunner(LocoLowerDeployConfig config,
                                         std::uint8_t expected_mode_machine)
    : config_(std::move(config)), expected_mode_machine_(expected_mode_machine) {
  reset();
}

void LocoLowerStepRunner::reset() {
  last_action_ = Vec(kLocoLowerPolicyJointDim, 0.0F);
  held_raw_action_ = Vec(kLocoLowerPolicyJointDim, 0.0F);
  tick_ = 0;
}

LocoLowerStepResult LocoLowerStepRunner::step(const LowStateSample& low_state,
                                              VelocityCommand command,
                                              VelocityPolicyInference& policy,
                                              const LowCmdFrame* base_frame) {
  validateConfig(config_);
  LocoLowerStepResult result;
  command = clampCommand(config_.command_ranges, command, result.command_clamped);
  result.inputs = makeLocoLowerPolicyInputs(config_, low_state, last_action_, command);

  ++tick_;
  if ((tick_ % config_.policy_decimation) != 0) {
    result.raw_action = held_raw_action_;
  } else {
    Vec inferred_action = policy.infer(result.inputs);
    requireSize("raw_action", inferred_action.size(), kLocoLowerPolicyJointDim);
    requireFinite("raw_action", inferred_action);
    result.raw_action = applyActionClip(config_, inferred_action, result.raw_action_clamped);
    result.action_clamped = result.raw_action_clamped;
    held_raw_action_ = result.raw_action;
    result.policy_evaluated = true;
  }

  result.processed_action = processedAction(config_, result.raw_action);
  result.low_cmd =
      makeLocoLowerLowCmdFrame(config_, result.raw_action, expected_mode_machine_, base_frame);
  result.action_clamped = result.raw_action_clamped;
  last_action_ = result.raw_action;
  return result;
}

std::string sha256File(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw LocoLowerPolicyError("failed to open file for sha256: " + path.string());
  }
  std::vector<unsigned char> data;
  for (std::istreambuf_iterator<char> it(in), end; it != end; ++it) {
    data.push_back(static_cast<unsigned char>(*it));
  }

  const std::uint64_t bit_length = static_cast<std::uint64_t>(data.size()) * 8U;
  data.push_back(0x80U);
  while ((data.size() % 64U) != 56U) {
    data.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    data.push_back(static_cast<unsigned char>((bit_length >> shift) & 0xffU));
  }

  Sha256State state{{
      0x6a09e667U,
      0xbb67ae85U,
      0x3c6ef372U,
      0xa54ff53aU,
      0x510e527fU,
      0x9b05688cU,
      0x1f83d9abU,
      0x5be0cd19U,
  }};
  for (std::size_t offset = 0; offset < data.size(); offset += 64) {
    processSha256Block(state, data.data() + offset);
  }

  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (std::uint32_t word : state) {
    out << std::setw(8) << word;
  }
  return out.str();
}

}  // namespace agentic_et1_tracker
