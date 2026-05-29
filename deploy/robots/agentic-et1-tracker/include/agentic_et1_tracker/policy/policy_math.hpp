#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "agentic_et1_tracker/policy/deploy_config.hpp"

namespace agentic_et1_tracker {

using Vec = std::vector<float>;

struct PolicyObservationParts {
  Vec command_root_ori_b;
  Vec command_xy_yaw_vel;
  Vec command_jnt_pos;
  Vec projected_gravity;
  Vec base_ang_vel;
  Vec joint_pos_rel;
  Vec joint_vel_rel;
  Vec last_action;
  Vec command_foot_support_state;
  Vec ref_com_rel_navi;
  Vec ref_com_vel_navi;
};

struct PolicyInputs {
  Vec obs_current;
  Vec obs_history;
};

struct PolicyOutput {
  Vec raw_action;
  Vec target_q;
  Vec kp;
  Vec kd;
};

class PolicyMathError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

Vec buildObsCurrent(const DeployConfig& config, const PolicyObservationParts& parts);

class HistoryBuffer {
 public:
  explicit HistoryBuffer(const DeployConfig& config);

  void reset(const PolicyObservationParts& parts);
  void push(const PolicyObservationParts& parts);
  Vec flatten() const;

 private:
  std::vector<ObservationTerm> terms_;
  std::size_t row_width_{0};
  std::size_t length_{0};
  std::vector<Vec> rows_;
};

PolicyInputs buildPolicyInputs(const DeployConfig& config,
                               const PolicyObservationParts& current,
                               const HistoryBuffer& history);

PolicyOutput scaleAction(const DeployConfig& config, const Vec& raw_action);

}  // namespace agentic_et1_tracker
