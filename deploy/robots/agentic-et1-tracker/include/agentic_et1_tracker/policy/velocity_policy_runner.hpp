#pragma once

#include <cstdint>
#include <deque>
#include <stdexcept>
#include <vector>

#include "agentic_et1_tracker/policy/policy_math.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"

namespace agentic_et1_tracker {

struct VelocityPolicyInputs {
  Vec obs;
};

class VelocityPolicyInference {
 public:
  virtual ~VelocityPolicyInference() = default;
  virtual Vec infer(const VelocityPolicyInputs& inputs) = 0;
};

struct VelocityStepResult {
  VelocityPolicyInputs inputs;
  Vec raw_action;
  LowCmdFrame low_cmd;
};

class VelocityPolicyError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class VelocityStepRunner {
 public:
  VelocityStepRunner(VelocityDeployConfig config, std::uint8_t expected_mode_machine);

  void reset();
  VelocityStepResult step(const LowStateSample& low_state,
                          VelocityPolicyInference& policy);

 private:
  VelocityDeployConfig config_;
  std::uint8_t expected_mode_machine_{0};
  Vec last_action_;
  std::vector<std::deque<Vec>> term_history_;
};

VelocityPolicyInputs makeVelocityPolicyInputs(const VelocityDeployConfig& config,
                                              const LowStateSample& low_state,
                                              const Vec& last_action);
LowCmdFrame makeVelocityLowCmdFrame(const VelocityDeployConfig& config,
                                    const Vec& raw_action,
                                    std::uint8_t expected_mode_machine);

}  // namespace agentic_et1_tracker
