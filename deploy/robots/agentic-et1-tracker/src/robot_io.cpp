#include "agentic_et1_tracker/robot/robot_io.hpp"

#include <cmath>
#include <sstream>

namespace agentic_et1_tracker {
namespace {

RobotIOError error(const std::string& message) {
  return RobotIOError("robot io error: " + message);
}

void requireSize(const std::string& field, std::size_t actual, std::size_t expected) {
  if (actual == expected) {
    return;
  }

  std::ostringstream out;
  out << field << " must contain " << expected << " entries, got " << actual;
  throw error(out.str());
}

void requireFinite(const std::string& field, const std::vector<float>& values) {
  for (float value : values) {
    if (!std::isfinite(value)) {
      throw error(field + " must be finite");
    }
  }
}

void validatePolicyOutput(const PolicyOutput& output) {
  requireSize("PolicyOutput.target_q", output.target_q.size(), kPolicyJointCount);
  requireSize("PolicyOutput.kp", output.kp.size(), kPolicyJointCount);
  requireSize("PolicyOutput.kd", output.kd.size(), kPolicyJointCount);
  requireFinite("PolicyOutput.target_q", output.target_q);
  requireFinite("PolicyOutput.kp", output.kp);
  requireFinite("PolicyOutput.kd", output.kd);
}

void validateSdkMap(const std::vector<int>& sdk_joint_ids_map) {
  requireSize("sdk_joint_ids_map", sdk_joint_ids_map.size(), kPolicyJointCount);

  for (std::size_t i = 0; i < sdk_joint_ids_map.size(); ++i) {
    const int slot = sdk_joint_ids_map[i];
    if (slot < 0 || slot >= static_cast<int>(kSdkMotorCount)) {
      std::ostringstream out;
      out << "sdk_joint_ids_map[" << i << "] out of range: " << slot;
      throw error(out.str());
    }
  }
}

}  // namespace

ModeMachineCheck checkModeMachine(const std::optional<LowStateSample>& low_state,
                                  std::uint8_t expected_mode_machine) {
  ModeMachineCheck result;
  result.expected = expected_mode_machine;
  if (!low_state.has_value()) {
    return result;
  }

  result.connected = true;
  result.observed = low_state->mode_machine;
  result.ok = low_state->mode_machine == 0 ||
              low_state->mode_machine == expected_mode_machine;
  return result;
}

bool hasSafeBodyOrientation(const LowStateSample& low_state) {
  const float w = low_state.quat_wxyz[0];
  const float x = low_state.quat_wxyz[1];
  const float y = low_state.quat_wxyz[2];
  const float z = low_state.quat_wxyz[3];
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z)) {
    return false;
  }

  const float norm_sq = w * w + x * x + y * y + z * z;
  if (!std::isfinite(norm_sq) || norm_sq <= kMinSafeQuaternionNormSquared) {
    return false;
  }

  const float inv_norm_sq = 1.0F / norm_sq;
  const float body_z_world_z = 1.0F - 2.0F * (x * x + y * y) * inv_norm_sq;
  return std::isfinite(body_z_world_z) &&
         body_z_world_z >= kMinSafeBodyZProjection;
}

RobotReadinessStatus mapRobotReadiness(const std::optional<LowStateSample>& low_state,
                                       LowCmdOccupancy occupancy,
                                       std::uint8_t expected_mode_machine) {
  RobotReadinessStatus status;

  if (occupancy.occupied) {
    status.robot = RobotState::NotReady;
    status.err = ErrorCode::RobotNotReady;
    status.block = "lowcmd_occupied";
    if (low_state.has_value()) {
      status.low_ms = low_state->age_ms;
    }
    return status;
  }

  if (!low_state.has_value()) {
    status.robot = RobotState::Disconnected;
    status.err = ErrorCode::RobotDisconnected;
    status.block = "lowstate_missing";
    status.low_ms = std::nullopt;
    return status;
  }

  status.low_ms = low_state->age_ms;

  if (!low_state->fresh) {
    status.robot = RobotState::NotReady;
    status.err = ErrorCode::RobotNotReady;
    status.block = "lowstate_timeout";
    return status;
  }

  const ModeMachineCheck mode = checkModeMachine(low_state, expected_mode_machine);
  if (!mode.ok) {
    status.robot = RobotState::NotReady;
    status.err = ErrorCode::RobotNotReady;
    status.block = "mode_machine_mismatch";
    return status;
  }

  if (!hasSafeBodyOrientation(*low_state)) {
    status.robot = RobotState::Fault;
    status.err = ErrorCode::RobotBadOrientation;
    status.block = "bad_orientation";
    return status;
  }

  status.robot = RobotState::Idle;
  status.err = ErrorCode::Ok;
  status.block.clear();
  return status;
}

LowCmdFrame makeLowCmdFrame(const DeployConfig& config,
                            const PolicyOutput& output,
                            std::uint8_t expected_mode_machine,
                            const LowCmdFrame* base_frame) {
  return makeLowCmdFrame(config.sdk_joint_ids_map,
                         output,
                         expected_mode_machine,
                         base_frame);
}

LowCmdFrame makeLowCmdFrame(const std::vector<int>& sdk_joint_ids_map,
                            const PolicyOutput& output,
                            std::uint8_t expected_mode_machine,
                            const LowCmdFrame* base_frame) {
  validateSdkMap(sdk_joint_ids_map);
  validatePolicyOutput(output);

  LowCmdFrame frame = base_frame == nullptr ? LowCmdFrame{} : *base_frame;
  frame.mode_machine = expected_mode_machine;
  frame.mode_pr = 0;

  for (std::size_t policy_joint = 0; policy_joint < kPolicyJointCount; ++policy_joint) {
    const auto sdk_slot = static_cast<std::size_t>(sdk_joint_ids_map[policy_joint]);
    MotorCommand& motor = frame.motors[sdk_slot];
    motor.mode = 1;
    motor.q = output.target_q[policy_joint];
    motor.dq = 0.0F;
    motor.kp = output.kp[policy_joint];
    motor.kd = output.kd[policy_joint];
    motor.tau = 0.0F;
  }

  return frame;
}

LowCmdFrame makeHoldLowCmdFrame(const DeployConfig& config,
                                const LowStateSample& low_state,
                                std::uint8_t expected_mode_machine) {
  validateSdkMap(config.sdk_joint_ids_map);
  requireSize("DeployConfig.policy_kp", config.policy_kp.size(), kPolicyJointCount);
  requireSize("DeployConfig.policy_kd", config.policy_kd.size(), kPolicyJointCount);

  PolicyOutput output;
  output.target_q.reserve(kPolicyJointCount);
  output.kp.reserve(kPolicyJointCount);
  output.kd.reserve(kPolicyJointCount);
  for (std::size_t policy_joint = 0; policy_joint < kPolicyJointCount; ++policy_joint) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map[policy_joint]);
    output.target_q.push_back(low_state.motors[sdk_slot].q);
    output.kp.push_back(static_cast<float>(config.policy_kp[policy_joint]));
    output.kd.push_back(static_cast<float>(config.policy_kd[policy_joint]));
  }

  return makeLowCmdFrame(config.sdk_joint_ids_map, output, expected_mode_machine);
}

}  // namespace agentic_et1_tracker
