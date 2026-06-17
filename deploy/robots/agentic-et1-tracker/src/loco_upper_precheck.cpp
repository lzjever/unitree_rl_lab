#include "agentic_et1_tracker/loco_upper/precheck.hpp"

#include <cmath>
#include <string>
#include <utility>

#include "agentic_et1_tracker/loco_upper/compiler.hpp"
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

LocoUpperCompileOptions compileOptionsFromPrecheck(
    const LocoUpperPrecheckOptions& options) {
  LocoUpperCompileOptions compile_options;
  compile_options.max_radius_m = options.max_radius_m;
  compile_options.root_options = options.root_options;
  compile_options.command_limits = options.command_limits;
  compile_options.upper_joint_limits =
      options.upper_joint_limits.value_or(LocoUpperJointValidationOptions{});
  compile_options.probe_only = true;
  return compile_options;
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

  const LocoUpperCompileResult compiled =
      compileLocoUpperPlan(track, compileOptionsFromPrecheck(options));
  if (!compiled.ok()) {
    const ErrorCode code =
        compiled.failure_kind == LocoUpperCompileFailureKind::InvalidOptions ||
                compiled.failure_kind == LocoUpperCompileFailureKind::InvalidConfig
            ? ErrorCode::RequestInvalid
            : ErrorCode::TrkValidationFailed;
    return fail(code, compiled.message);
  }

  LocoUpperPrecheckResult result;
  result.flags = compiled.flags;
  return result;
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
