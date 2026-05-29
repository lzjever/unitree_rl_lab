#include "agentic_et1_tracker/policy/policy_math.hpp"

#include <sstream>
#include <string>

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointDim = 26;
constexpr std::size_t kObsCurrentDim = 131;
constexpr std::size_t kObsHistoryWidth = 105;
constexpr std::size_t kObsHistoryLength = 25;

PolicyMathError error(const std::string& message) {
  return PolicyMathError("policy math error: " + message);
}

std::string sizeMessage(const std::string& field,
                        std::size_t expected,
                        std::size_t actual) {
  std::ostringstream out;
  out << field << " must contain " << expected << " entries, got " << actual;
  return out.str();
}

void requireSize(const std::string& field,
                 std::size_t actual,
                 std::size_t expected) {
  if (actual != expected) {
    throw error(sizeMessage(field, expected, actual));
  }
}

const Vec& partByName(const PolicyObservationParts& parts, const std::string& name) {
  if (name == "command_root_ori_b") {
    return parts.command_root_ori_b;
  }
  if (name == "command_xy_yaw_vel") {
    return parts.command_xy_yaw_vel;
  }
  if (name == "command_jnt_pos") {
    return parts.command_jnt_pos;
  }
  if (name == "projected_gravity") {
    return parts.projected_gravity;
  }
  if (name == "base_ang_vel") {
    return parts.base_ang_vel;
  }
  if (name == "joint_pos_rel") {
    return parts.joint_pos_rel;
  }
  if (name == "joint_vel_rel") {
    return parts.joint_vel_rel;
  }
  if (name == "last_action") {
    return parts.last_action;
  }
  if (name == "command_foot_support_state") {
    return parts.command_foot_support_state;
  }
  if (name == "ref_com_rel_navi") {
    return parts.ref_com_rel_navi;
  }
  if (name == "ref_com_vel_navi") {
    return parts.ref_com_vel_navi;
  }

  throw error("unknown observation term '" + name + "'");
}

void validateContiguousTerm(const ObservationTerm& term,
                            std::size_t expected_offset,
                            const std::string& group) {
  if (term.offset != expected_offset) {
    std::ostringstream out;
    out << group << "." << term.name << " offset must be " << expected_offset
        << ", got " << term.offset;
    throw error(out.str());
  }
}

Vec buildRow(const std::vector<ObservationTerm>& terms,
             std::size_t expected_width,
             const PolicyObservationParts& parts,
             bool allow_last_action,
             const std::string& group) {
  Vec row;
  row.reserve(expected_width);

  std::size_t offset = 0;
  for (const ObservationTerm& term : terms) {
    validateContiguousTerm(term, offset, group);
    if (!allow_last_action && term.name == "last_action") {
      throw error(group + " must not include last_action");
    }

    const Vec& values = partByName(parts, term.name);
    requireSize(group + "." + term.name, values.size(), term.width);
    row.insert(row.end(), values.begin(), values.end());
    offset += term.width;
  }

  requireSize(group + " row", row.size(), expected_width);
  return row;
}

void validatePolicyConfig(const DeployConfig& config) {
  requireSize("DeployConfig.obs_current_dim", config.obs_current_dim, kObsCurrentDim);
  requireSize("DeployConfig.obs_history_width", config.obs_history_width, kObsHistoryWidth);
  requireSize("DeployConfig.obs_history_length", config.obs_history_length, kObsHistoryLength);
}

void validateActionConfig(const DeployConfig& config) {
  requireSize("DeployConfig.joint_dim", config.joint_dim, kJointDim);
  requireSize("DeployConfig.action_scale", config.action_scale.size(), kJointDim);
  requireSize("DeployConfig.action_offset", config.action_offset.size(), kJointDim);
  requireSize("DeployConfig.policy_kp", config.policy_kp.size(), kJointDim);
  requireSize("DeployConfig.policy_kd", config.policy_kd.size(), kJointDim);
}

Vec copyDoublesAsFloats(const std::vector<double>& values) {
  Vec out;
  out.reserve(values.size());
  for (const double value : values) {
    out.push_back(static_cast<float>(value));
  }
  return out;
}

}  // namespace

Vec buildObsCurrent(const DeployConfig& config, const PolicyObservationParts& parts) {
  validatePolicyConfig(config);
  return buildRow(config.obs_current_terms, config.obs_current_dim, parts, true,
                  "obs_current");
}

HistoryBuffer::HistoryBuffer(const DeployConfig& config)
    : terms_(config.obs_history_terms),
      row_width_(config.obs_history_width),
      length_(config.obs_history_length) {
  validatePolicyConfig(config);
}

void HistoryBuffer::reset(const PolicyObservationParts& parts) {
  const Vec row = buildRow(terms_, row_width_, parts, false, "obs_history");
  rows_.assign(length_, row);
}

void HistoryBuffer::push(const PolicyObservationParts& parts) {
  if (rows_.empty()) {
    throw error("HistoryBuffer must be reset before push");
  }

  const Vec row = buildRow(terms_, row_width_, parts, false, "obs_history");
  rows_.erase(rows_.begin());
  rows_.push_back(row);
}

Vec HistoryBuffer::flatten() const {
  if (rows_.empty()) {
    throw error("HistoryBuffer must be reset before flatten");
  }

  Vec out;
  out.reserve(length_ * row_width_);
  for (const Vec& row : rows_) {
    requireSize("obs_history stored row", row.size(), row_width_);
    out.insert(out.end(), row.begin(), row.end());
  }
  requireSize("obs_history flattened", out.size(), length_ * row_width_);
  return out;
}

PolicyInputs buildPolicyInputs(const DeployConfig& config,
                               const PolicyObservationParts& current,
                               const HistoryBuffer& history) {
  validatePolicyConfig(config);

  PolicyInputs inputs;
  inputs.obs_current = buildObsCurrent(config, current);
  inputs.obs_history = history.flatten();
  requireSize("PolicyInputs.obs_history", inputs.obs_history.size(),
              config.obs_history_length * config.obs_history_width);
  return inputs;
}

PolicyOutput scaleAction(const DeployConfig& config, const Vec& raw_action) {
  validateActionConfig(config);
  requireSize("raw_action", raw_action.size(), kJointDim);

  PolicyOutput output;
  output.raw_action = raw_action;
  output.target_q.reserve(kJointDim);
  for (std::size_t i = 0; i < kJointDim; ++i) {
    output.target_q.push_back(raw_action[i] * static_cast<float>(config.action_scale[i]) +
                              static_cast<float>(config.action_offset[i]));
  }
  output.kp = copyDoublesAsFloats(config.policy_kp);
  output.kd = copyDoublesAsFloats(config.policy_kd);
  return output;
}

}  // namespace agentic_et1_tracker
