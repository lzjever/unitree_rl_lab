#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

struct LocoUpperSafeCommand {
  std::uint8_t mode{1};
  float q{0.0F};
  float kp{0.0F};
  float kd{0.0F};
};

struct LocoUpperLowCmdComposerConfig {
  std::size_t upper_start_joint{12};
  std::size_t upper_end_joint_exclusive{26};
  std::vector<int> logical_to_sdk;
  std::vector<float> upper_kp;
  std::vector<float> upper_kd;
  std::vector<float> lower_min_q;
  std::vector<float> lower_max_q;
  std::vector<float> upper_min_q;
  std::vector<float> upper_max_q;
  std::vector<float> upper_max_vel_radps;
  std::vector<float> upper_max_accel_radps2;
  LocoUpperSafeCommand unowned_safe;
  std::uint8_t expected_mode_machine{1};
};

struct LocoUpperLowCmdComposeResult {
  LowCmdFrame frame;
  bool lower_q_limited{false};
  bool lower_action_clamped{false};
};

class LocoUpperLowCmdComposerError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

[[nodiscard]] LocoUpperLowCmdComposeResult composeLocoUpperLowCmd(
    const LocoUpperLowCmdComposerConfig& config,
    const LowCmdFrame& lower_frame,
    const std::vector<float>& upper_joint_targets);

[[nodiscard]] LocoUpperLowCmdComposeResult composeLocoUpperLowCmd(
    const LocoUpperLowCmdComposerConfig& config,
    const LowCmdFrame& lower_frame,
    const TrkFrameView& upper_frame);

[[nodiscard]] LocoUpperLowCmdComposerConfig loadLocoUpperLowCmdComposerConfig(
    const std::filesystem::path& limits_path,
    const std::filesystem::path& joint_map_path,
    const DeployConfig& deploy_config,
    const LocoLowerDeployConfig& lower_deploy_config);

}  // namespace agentic_et1_tracker
