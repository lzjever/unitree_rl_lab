#include "agentic_et1_tracker/policy/policy_step_runner.hpp"

#include <optional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace agentic_et1_tracker {
namespace {

PolicyStepError error(const std::string& message) {
  return PolicyStepError("policy step error: " + message);
}

Vec zeroLastAction() {
  return Vec(kPolicyJointCount, 0.0F);
}

bool usesTemporalHistory(const DeployConfig& config) {
  return config.observation_contract == ObservationContract::GeneralTracker;
}

}  // namespace

PolicyStepRunner::PolicyStepRunner(const DeployConfig& config,
                                   TrkTrack track,
                                   const LowStateSample& entry_low_state,
                                   std::uint8_t expected_mode_machine,
                                   ObservationBuilderConfig builder_config)
    : PolicyStepRunner(config,
                       std::make_shared<TrkTrack>(std::move(track)),
                       entry_low_state,
                       expected_mode_machine,
                       builder_config) {}

PolicyStepRunner::PolicyStepRunner(const DeployConfig& config,
                                   std::shared_ptr<const TrkTrack> track,
                                   const LowStateSample& entry_low_state,
                                   std::uint8_t expected_mode_machine,
                                   ObservationBuilderConfig builder_config) try
    : config_(config),
      track_(std::move(track)),
      expected_mode_machine_(expected_mode_machine),
      builder_config_(builder_config),
      history_(config_) {
  if (!track_) {
    throw error("missing track");
  }
  reset(entry_low_state);
} catch (const PolicyStepError&) {
  throw;
} catch (const std::exception& err) {
  throw error(err.what());
}

void PolicyStepRunner::reset(const LowStateSample& entry_low_state) {
  try {
    const std::optional<TrkFrameView> first_frame = track_->frame(0);
    if (!first_frame.has_value()) {
      throw error("missing first frame");
    }

    builder_state_ =
        makeObservationBuilderState(*first_frame, entry_low_state, builder_config_);
    history_ = HistoryBuffer(config_);
    history_ready_ = false;
    last_action_ = zeroLastAction();
  } catch (const PolicyStepError&) {
    throw;
  } catch (const std::exception& err) {
    throw error(err.what());
  }
}

PolicyStepResult PolicyStepRunner::step(std::size_t frame_index,
                                        const LowStateSample& low_state,
                                        PolicyInference& policy,
                                        const LowCmdFrame* base_frame) {
  try {
    const PolicyObservationParts parts =
        buildObservationParts(config_, *track_, frame_index, low_state, last_action_, builder_state_,
                              builder_config_);

    HistoryBuffer next_history = history_;
    if (usesTemporalHistory(config_)) {
      if (history_ready_) {
        next_history.push(parts);
      } else {
        next_history.reset(parts);
      }
    }

    PolicyStepResult result;
    result.frame = frame_index;
    result.inputs = buildPolicyInputs(config_, parts, next_history);
    const Vec raw_action = policy.infer(result.inputs);
    result.output = scaleAction(config_, raw_action);
    result.low_cmd =
        makeLowCmdFrame(config_, result.output, expected_mode_machine_, base_frame);

    history_ = std::move(next_history);
    history_ready_ = usesTemporalHistory(config_) ? true : history_ready_;
    last_action_ = raw_action;
    return result;
  } catch (const PolicyStepError&) {
    throw;
  } catch (const std::exception& err) {
    throw error(err.what());
  }
}

}  // namespace agentic_et1_tracker
