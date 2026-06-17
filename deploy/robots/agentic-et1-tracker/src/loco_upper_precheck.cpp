#include "agentic_et1_tracker/loco_upper/precheck.hpp"

#include <cmath>
#include <string>
#include <utility>

#include "agentic_et1_tracker/loco_upper/planner.hpp"
#include "agentic_et1_tracker/loco_upper/validator.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {
namespace {

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

LocoUpperPrecheckResult fail(ErrorCode code, std::string message) {
  LocoUpperPrecheckResult result;
  result.code = code;
  result.message = std::move(message);
  return result;
}

LocoUpperPrecheckResult validateOptions(const LocoUpperPrecheckOptions& options) {
  if (!finitePositive(options.max_radius_m)) {
    return fail(ErrorCode::RequestInvalid, "loco upper max radius must be positive");
  }
  return {};
}

}  // namespace

LocoUpperPrecheckResult precheckLocoUpperTrack(
    const TrkTrack& track,
    const LocoUpperPrecheckOptions& options) {
  const LocoUpperPrecheckResult option_check = validateOptions(options);
  if (!option_check.ok()) {
    return option_check;
  }

  const LocoUpperRootPlanResult root = extractRootPlanarPath(track);
  if (!root.ok()) {
    return fail(ErrorCode::TrkValidationFailed, root.message);
  }

  const LocoUpperJointValidationResult upper =
      extractAndValidateUpperJointTargets(track,
                                          options.upper_joint_limits.value_or(
                                              LocoUpperJointValidationOptions{}));
  if (!upper.ok()) {
    return fail(ErrorCode::TrkValidationFailed, upper.message);
  }

  if (root.plan.samples.size() != upper.plan.frames.size()) {
    return fail(ErrorCode::TrkValidationFailed,
                "loco upper root and joint plans have mismatched frame counts");
  }

  const LocoUpperRootPlan aligned = alignRootPlanToStart(root.plan);
  const LocoUpperProjectionResult projected =
      projectRootPlanToRadius(aligned, options.max_radius_m);
  if (options.strict_pose && projected.radius_clamped) {
    return fail(ErrorCode::TrkValidationFailed,
                "loco upper root path exceeds max radius");
  }

  return {};
}

LocoUpperPrecheckResult precheckLocoUpperTrackFile(
    const TrkLoader& loader,
    const std::filesystem::path& path,
    const LocoUpperPrecheckOptions& options) {
  const LocoUpperPrecheckResult option_check = validateOptions(options);
  if (!option_check.ok()) {
    return option_check;
  }

  const TrkLoadResult load = loader.load(path);
  if (!load.ok()) {
    return fail(toCoreErrorCode(load.code), load.message);
  }

  return precheckLocoUpperTrack(*load.track, options);
}

}  // namespace agentic_et1_tracker
