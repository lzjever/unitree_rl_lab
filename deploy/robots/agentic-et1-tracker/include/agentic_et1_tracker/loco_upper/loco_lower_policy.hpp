#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"

namespace agentic_et1_tracker {

inline constexpr std::size_t kLocoLowerPolicyJointDim = 12;
inline constexpr std::size_t kLocoLowerPolicyObsRowWidth = 45;
inline constexpr std::size_t kLocoLowerPolicyHistoryLength = 1;
inline constexpr std::size_t kLocoLowerPolicyObsDim =
    kLocoLowerPolicyObsRowWidth * kLocoLowerPolicyHistoryLength;

struct LocoLowerRange {
  double min{0.0};
  double max{0.0};
};

struct LocoLowerCommandRanges {
  LocoLowerRange lin_vel_x;
  LocoLowerRange lin_vel_y;
  LocoLowerRange ang_vel_z;
};

struct LocoLowerObservationTerm {
  std::string name;
  std::size_t width{0};
  std::size_t offset{0};
  std::vector<double> scale;
};

struct LocoLowerDeployConfig {
  std::size_t joint_dim{kLocoLowerPolicyJointDim};
  std::vector<int> joint_ids_map;
  std::vector<int> sdk_joint_ids_map;
  std::vector<double> stiffness;
  std::vector<double> damping;
  std::vector<double> default_joint_pos;
  std::vector<double> joint_min_q;
  std::vector<double> joint_max_q;
  std::optional<std::vector<LocoLowerRange>> action_clip;
  std::vector<double> action_scale;
  std::vector<double> action_offset;
  std::vector<LocoLowerObservationTerm> observation_terms;
  LocoLowerCommandRanges command_ranges;
  std::size_t obs_row_width{kLocoLowerPolicyObsRowWidth};
  std::size_t obs_history_length{kLocoLowerPolicyHistoryLength};
  std::size_t obs_dim{kLocoLowerPolicyObsDim};
  std::size_t policy_decimation{10};
  double step_dt{0.02};
};

class LocoLowerDeployConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class LocoLowerPolicyError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct LocoLowerStepResult {
  VelocityPolicyInputs inputs;
  Vec raw_action;
  Vec processed_action;
  LowCmdFrame low_cmd;
  bool command_clamped{false};
  bool raw_action_clamped{false};
  bool action_clamped{false};
  bool policy_evaluated{false};
};

class LocoLowerStepRunner {
 public:
  LocoLowerStepRunner(LocoLowerDeployConfig config,
                      std::uint8_t expected_mode_machine);

  void reset();
  LocoLowerStepResult step(const LowStateSample& low_state,
                           VelocityCommand command,
                           VelocityPolicyInference& policy,
                           const LowCmdFrame* base_frame = nullptr,
                           bool evaluate_policy = true);

 private:
  LocoLowerDeployConfig config_;
  std::uint8_t expected_mode_machine_{0};
  Vec last_action_;
  Vec held_raw_action_;
};

[[nodiscard]] LocoLowerDeployConfig loadLocoLowerDeployConfig(
    const std::filesystem::path& path);

[[nodiscard]] VelocityPolicyInputs makeLocoLowerPolicyInputs(
    const LocoLowerDeployConfig& config,
    const LowStateSample& low_state,
    const Vec& last_action,
    VelocityCommand command);

[[nodiscard]] LowCmdFrame makeLocoLowerLowCmdFrame(
    const LocoLowerDeployConfig& config,
    const Vec& raw_action,
    std::uint8_t expected_mode_machine,
    const LowCmdFrame* base_frame = nullptr);

[[nodiscard]] std::string sha256File(const std::filesystem::path& path);

}  // namespace agentic_et1_tracker
