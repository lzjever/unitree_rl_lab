#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "agentic_et1_tracker/policy/observation_builder.hpp"
#include "agentic_et1_tracker/policy/policy_math.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

class PolicyInference {
 public:
  virtual ~PolicyInference() = default;

  virtual Vec infer(const PolicyInputs& inputs) = 0;
};

struct PolicyStepResult {
  std::size_t frame{0};
  PolicyInputs inputs;
  PolicyOutput output;
  LowCmdFrame low_cmd;
};

class PolicyStepError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class PolicyStepRunner {
 public:
  PolicyStepRunner(const DeployConfig& config,
                   TrkTrack track,
                   const LowStateSample& entry_low_state,
                   std::uint8_t expected_mode_machine,
                   ObservationBuilderConfig builder_config = {});

  void reset(const LowStateSample& entry_low_state);

  PolicyStepResult step(std::size_t frame_index,
                        const LowStateSample& low_state,
                        PolicyInference& policy);

 private:
  DeployConfig config_;
  TrkTrack track_;
  std::uint8_t expected_mode_machine_{0};
  ObservationBuilderConfig builder_config_;
  ObservationBuilderState builder_state_;
  HistoryBuffer history_;
  bool history_ready_{false};
  Vec last_action_;
};

}  // namespace agentic_et1_tracker
