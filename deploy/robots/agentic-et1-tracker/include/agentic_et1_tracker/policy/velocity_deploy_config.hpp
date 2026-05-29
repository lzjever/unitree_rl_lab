#pragma once

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace agentic_et1_tracker {

inline constexpr std::size_t kVelocityPolicyJointDim = 12;
inline constexpr std::size_t kVelocityPolicyObsRowWidth = 45;
inline constexpr std::size_t kVelocityPolicyHistoryLength = 5;
inline constexpr std::size_t kVelocityPolicyObsDim =
    kVelocityPolicyObsRowWidth * kVelocityPolicyHistoryLength;

struct VelocityObservationTerm {
  std::string name;
  std::size_t width{0};
  std::size_t offset{0};
  std::vector<double> scale;
};

struct VelocityDeployConfig {
  std::size_t joint_dim{kVelocityPolicyJointDim};
  std::vector<int> joint_ids_map;
  std::vector<int> sdk_joint_ids_map;
  std::vector<double> stiffness;
  std::vector<double> damping;
  std::vector<double> default_joint_pos;
  std::vector<double> action_scale;
  std::vector<double> action_offset;
  std::vector<VelocityObservationTerm> observation_terms;
  std::size_t obs_row_width{kVelocityPolicyObsRowWidth};
  std::size_t obs_history_length{kVelocityPolicyHistoryLength};
  std::size_t obs_dim{kVelocityPolicyObsDim};
  double step_dt{0.02};
};

class VelocityDeployConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

VelocityDeployConfig loadVelocityDeployConfig(const std::filesystem::path& path);

}  // namespace agentic_et1_tracker
