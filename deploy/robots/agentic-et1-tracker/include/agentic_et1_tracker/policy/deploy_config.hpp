#pragma once

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace agentic_et1_tracker {

struct ObservationTerm {
  std::string name;
  std::size_t width{0};
  std::size_t offset{0};
};

enum class ObservationContract {
  GeneralTracker,
  GeneralTrackerCLN,
};

struct DeployConfig {
  std::size_t joint_dim{26};
  ObservationContract observation_contract{ObservationContract::GeneralTracker};
  std::vector<int> joint_ids_map;
  std::vector<int> sdk_joint_ids_map;
  std::vector<int> override_joint_ids;
  std::vector<double> policy_kp;
  std::vector<double> policy_kd;
  std::vector<double> default_joint_pos;
  std::vector<double> action_scale;
  std::vector<double> action_offset;
  std::vector<std::string> obs_current;
  std::vector<std::string> obs_history;
  std::vector<ObservationTerm> obs_current_terms;
  std::vector<ObservationTerm> obs_history_terms;
  std::size_t obs_current_dim{0};
  std::size_t obs_history_width{0};
  std::size_t obs_history_length{0};
  double step_dt{0.0};
};

class DeployConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

DeployConfig loadDeployConfig(const std::filesystem::path& path);

}  // namespace agentic_et1_tracker
