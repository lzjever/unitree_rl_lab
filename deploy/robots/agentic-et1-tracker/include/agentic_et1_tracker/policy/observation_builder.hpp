#pragma once

#include <cstddef>
#include <stdexcept>

#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/policy/policy_math.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

struct ObservationBuilderConfig {
  bool no_global_mode{true};
};

struct ObservationBuilderState {
  double first_ref_root_yaw{0.0};
  double entry_robot_yaw{0.0};
};

class ObservationBuilderError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

ObservationBuilderState makeObservationBuilderState(
    const TrkFrameView& first_frame,
    const LowStateSample& entry_low_state,
    const ObservationBuilderConfig& config = {});

PolicyObservationParts buildObservationParts(
    const DeployConfig& config,
    const TrkFrameView& frame,
    const LowStateSample& low_state,
    const Vec& last_action,
    const ObservationBuilderState& state,
    const ObservationBuilderConfig& builder_config = {});

std::size_t referenceFrameIndex(double elapsed_s, double fps, std::size_t frame_count);

}  // namespace agentic_et1_tracker
