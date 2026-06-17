#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"

#include <cmath>
#include <deque>
#include <sstream>
#include <string>
#include <utility>

namespace agentic_et1_tracker {
namespace {

VelocityPolicyError error(const std::string& message) {
  return VelocityPolicyError("velocity policy error: " + message);
}

void requireSize(const std::string& field, std::size_t actual, std::size_t expected) {
  if (actual != expected) {
    std::ostringstream out;
    out << field << " must contain " << expected << " entries, got " << actual;
    throw error(out.str());
  }
}

void requireFinite(const std::string& field, const Vec& values) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      throw error(field + "[" + std::to_string(i) + "] must be finite");
    }
  }
}

void validateConfig(const VelocityDeployConfig& config) {
  requireSize("joint_dim", config.joint_dim, kVelocityPolicyJointDim);
  requireSize("joint_ids_map", config.joint_ids_map.size(), kVelocityPolicyJointDim);
  requireSize("stiffness", config.stiffness.size(), kVelocityPolicyJointDim);
  requireSize("damping", config.damping.size(), kVelocityPolicyJointDim);
  requireSize("default_joint_pos", config.default_joint_pos.size(),
              kVelocityPolicyJointDim);
  requireSize("action_scale", config.action_scale.size(), kVelocityPolicyJointDim);
  requireSize("action_offset", config.action_offset.size(), kVelocityPolicyJointDim);
  requireSize("observation_terms", config.observation_terms.size(), 6);
  requireSize("obs_row_width", config.obs_row_width, kVelocityPolicyObsRowWidth);
  requireSize("obs_history_length", config.obs_history_length,
              kVelocityPolicyHistoryLength);
  requireSize("obs_dim", config.obs_dim, kVelocityPolicyObsDim);

  for (std::size_t policy_index = 0; policy_index < config.joint_ids_map.size();
       ++policy_index) {
    const int logical = config.joint_ids_map.at(policy_index);
    if (logical < 0 || logical >= static_cast<int>(kSdkMotorCount)) {
      throw error("joint_ids_map logical joint out of range");
    }
    if (!config.sdk_joint_ids_map.empty() &&
        logical >= static_cast<int>(config.sdk_joint_ids_map.size())) {
      throw error("sdk_joint_ids_map missing logical joint slot");
    }
  }
}

std::size_t sdkSlot(const VelocityDeployConfig& config, std::size_t policy_index) {
  const int logical = config.joint_ids_map.at(policy_index);
  const int slot = config.sdk_joint_ids_map.empty()
                       ? logical
                       : config.sdk_joint_ids_map.at(static_cast<std::size_t>(logical));
  if (slot < 0 || slot >= static_cast<int>(kSdkMotorCount)) {
    throw error("sdk joint slot out of range");
  }
  return static_cast<std::size_t>(slot);
}

Vec projectedGravity(const LowStateSample& low_state) {
  const float w = low_state.quat_wxyz[0];
  const float x = low_state.quat_wxyz[1];
  const float y = low_state.quat_wxyz[2];
  const float z = low_state.quat_wxyz[3];
  const float norm_sq = w * w + x * x + y * y + z * z;
  if (!std::isfinite(norm_sq) || norm_sq <= kMinSafeQuaternionNormSquared) {
    throw error("low_state.quat_wxyz must be a finite non-zero quaternion");
  }
  const float inv = 1.0F / norm_sq;
  return {
      2.0F * (w * y - x * z) * inv,
      -2.0F * (w * x + y * z) * inv,
      -(w * w - x * x - y * y + z * z) * inv,
  };
}

Vec termValues(const VelocityDeployConfig& config,
               const VelocityObservationTerm& term,
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
    values.reserve(kVelocityPolicyJointDim);
    for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
      const auto slot = sdkSlot(config, i);
      values.push_back(low_state.motors.at(slot).q -
                       static_cast<float>(config.default_joint_pos.at(i)));
    }
  } else if (term.name == "joint_vel_rel") {
    values.reserve(kVelocityPolicyJointDim);
    for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
      const auto slot = sdkSlot(config, i);
      values.push_back(low_state.motors.at(slot).dq);
    }
  } else if (term.name == "last_action") {
    values = last_action;
  } else {
    throw error("unknown observation term '" + term.name + "'");
  }

  requireSize("observation." + term.name, values.size(), term.width);
  requireSize("observation." + term.name + ".scale", term.scale.size(), term.width);
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] *= static_cast<float>(term.scale.at(i));
  }
  requireFinite("observation." + term.name, values);
  return values;
}

Vec flattenTermMajorHistory(const std::vector<std::deque<Vec>>& term_history) {
  Vec obs;
  obs.reserve(kVelocityPolicyObsDim);
  for (const auto& history : term_history) {
    requireSize("observation term history", history.size(),
                kVelocityPolicyHistoryLength);
    for (const Vec& frame : history) {
      obs.insert(obs.end(), frame.begin(), frame.end());
    }
  }
  requireSize("obs", obs.size(), kVelocityPolicyObsDim);
  return obs;
}

std::vector<Vec> currentTermValues(const VelocityDeployConfig& config,
                                   const LowStateSample& low_state,
                                   const Vec& last_action,
                                   VelocityCommand command) {
  std::vector<Vec> values;
  values.reserve(config.observation_terms.size());
  std::size_t offset = 0;
  for (const VelocityObservationTerm& term : config.observation_terms) {
    if (term.offset != offset) {
      throw error("observation term offsets must be contiguous");
    }
    values.push_back(termValues(config, term, low_state, last_action, command));
    offset += values.back().size();
  }
  requireSize("velocity obs row", offset, kVelocityPolicyObsRowWidth);
  return values;
}

Vec processedAction(const VelocityDeployConfig& config, const Vec& raw_action) {
  Vec processed;
  processed.reserve(kVelocityPolicyJointDim);
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    processed.push_back(raw_action.at(i) * static_cast<float>(config.action_scale.at(i)) +
                        static_cast<float>(config.action_offset.at(i)));
  }
  return processed;
}

}  // namespace

VelocityPolicyInputs makeVelocityPolicyInputs(const VelocityDeployConfig& config,
                                              const LowStateSample& low_state,
                                              const Vec& last_action) {
  return makeVelocityPolicyInputs(config, low_state, last_action, VelocityCommand{});
}

VelocityPolicyInputs makeVelocityPolicyInputs(const VelocityDeployConfig& config,
                                              const LowStateSample& low_state,
                                              const Vec& last_action,
                                              VelocityCommand command) {
  validateConfig(config);
  requireSize("last_action", last_action.size(), kVelocityPolicyJointDim);

  const std::vector<Vec> values =
      currentTermValues(config, low_state, last_action, command);
  std::vector<std::deque<Vec>> term_history;
  term_history.reserve(values.size());
  for (const Vec& term : values) {
    term_history.emplace_back(kVelocityPolicyHistoryLength, term);
  }

  VelocityPolicyInputs inputs;
  inputs.obs = flattenTermMajorHistory(term_history);
  return inputs;
}

LowCmdFrame makeVelocityLowCmdFrame(const VelocityDeployConfig& config,
                                    const Vec& raw_action,
                                    std::uint8_t expected_mode_machine,
                                    const LowCmdFrame* base_frame) {
  validateConfig(config);
  requireSize("raw_action", raw_action.size(), kVelocityPolicyJointDim);
  requireFinite("raw_action", raw_action);
  const Vec processed_action = processedAction(config, raw_action);

  LowCmdFrame frame = base_frame == nullptr ? LowCmdFrame{} : *base_frame;
  frame.mode_machine = expected_mode_machine;
  frame.mode_pr = 0;
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const std::size_t slot = sdkSlot(config, i);
    MotorCommand& motor = frame.motors.at(slot);
    motor.mode = 1;
    motor.q = processed_action.at(i);
    motor.dq = 0.0F;
    motor.kp = static_cast<float>(config.stiffness.at(i));
    motor.kd = static_cast<float>(config.damping.at(i));
    motor.tau = 0.0F;
  }
  return frame;
}

VelocityStepRunner::VelocityStepRunner(VelocityDeployConfig config,
                                       std::uint8_t expected_mode_machine)
    : config_(std::move(config)), expected_mode_machine_(expected_mode_machine) {
  reset();
}

void VelocityStepRunner::reset() {
  last_action_ = Vec(kVelocityPolicyJointDim, 0.0F);
  term_history_.clear();
}

VelocityStepResult VelocityStepRunner::step(const LowStateSample& low_state,
                                            VelocityPolicyInference& policy,
                                            const LowCmdFrame* base_frame) {
  return step(low_state, policy, VelocityCommand{}, base_frame);
}

VelocityStepResult VelocityStepRunner::step(const LowStateSample& low_state,
                                            VelocityPolicyInference& policy,
                                            VelocityCommand command,
                                            const LowCmdFrame* base_frame) {
  VelocityStepResult result;
  validateConfig(config_);
  requireSize("last_action", last_action_.size(), kVelocityPolicyJointDim);
  const std::vector<Vec> values =
      currentTermValues(config_, low_state, last_action_, command);
  if (term_history_.empty()) {
    term_history_.reserve(values.size());
    for (const Vec& term : values) {
      term_history_.emplace_back(kVelocityPolicyHistoryLength, term);
    }
  } else {
    requireSize("observation term history count", term_history_.size(), values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
      auto& history = term_history_.at(i);
      requireSize("observation term history", history.size(),
                  kVelocityPolicyHistoryLength);
      history.push_back(values.at(i));
      while (history.size() > kVelocityPolicyHistoryLength) {
        history.pop_front();
      }
    }
  }
  result.inputs.obs = flattenTermMajorHistory(term_history_);
  result.raw_action = policy.infer(result.inputs);
  requireSize("raw_action", result.raw_action.size(), kVelocityPolicyJointDim);
  requireFinite("raw_action", result.raw_action);
  result.processed_action = processedAction(config_, result.raw_action);
  result.low_cmd = makeVelocityLowCmdFrame(config_, result.raw_action,
                                           expected_mode_machine_,
                                           base_frame);
  last_action_ = result.raw_action;
  return result;
}

}  // namespace agentic_et1_tracker
