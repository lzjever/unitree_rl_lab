#include "agentic_et1_tracker/loco_upper/compiler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kLogicalJointCount =
    static_cast<std::size_t>(TrkSchema::kJointDim);

LocoUpperCompileResult fail(LocoUpperCompileFailureKind kind,
                            std::string message) {
  LocoUpperCompileResult result;
  result.failure_kind = kind;
  result.message = std::move(message);
  return result;
}

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool validFullJointArrayForFrames(const TrkFloatArray& array,
                                  std::size_t frames) {
  if (frames == 0 || array.frame_size < kLogicalJointCount) {
    return false;
  }
  if (array.frame_size != 0 &&
      frames > std::numeric_limits<std::size_t>::max() / array.frame_size) {
    return false;
  }
  return array.values.size() >= frames * array.frame_size;
}

bool validOptionalLimitVector(const std::vector<double>& values) {
  return values.empty() || values.size() == kLocoUpperJointCount;
}

double lowerPositionLimit(const LocoUpperJointValidationOptions& options,
                          std::size_t upper_joint) {
  if (options.min_positions.size() == kLocoUpperJointCount) {
    return options.min_positions.at(upper_joint);
  }
  return options.min_position;
}

double upperPositionLimit(const LocoUpperJointValidationOptions& options,
                          std::size_t upper_joint) {
  if (options.max_positions.size() == kLocoUpperJointCount) {
    return options.max_positions.at(upper_joint);
  }
  return options.max_position;
}

double velocityLimit(const LocoUpperJointValidationOptions& options,
                     std::size_t upper_joint) {
  if (options.max_velocities.size() == kLocoUpperJointCount) {
    return options.max_velocities.at(upper_joint);
  }
  return options.max_velocity;
}

double accelerationLimit(const LocoUpperJointValidationOptions& options,
                         std::size_t upper_joint) {
  if (options.max_accelerations.size() == kLocoUpperJointCount) {
    return options.max_accelerations.at(upper_joint);
  }
  return options.max_acceleration;
}

bool validNonnegativeLimit(double value) {
  if (std::isnan(value)) {
    return false;
  }
  if (std::isinf(value)) {
    return value > 0.0;
  }
  return value >= 0.0;
}

bool validPositionLimit(double value) {
  return !std::isnan(value);
}

LocoUpperCompileResult validateCompileOptions(
    const LocoUpperCompileOptions& options) {
  if (!finitePositive(options.max_radius_m)) {
    return fail(LocoUpperCompileFailureKind::InvalidOptions,
                "loco upper max radius must be positive");
  }
  if (!validOptionalLimitVector(options.upper_joint_limits.min_positions) ||
      !validOptionalLimitVector(options.upper_joint_limits.max_positions) ||
      !validOptionalLimitVector(options.upper_joint_limits.max_velocities) ||
      !validOptionalLimitVector(options.upper_joint_limits.max_accelerations)) {
    return fail(LocoUpperCompileFailureKind::InvalidConfig,
                "loco upper joint validation vectors must contain 14 entries");
  }

  for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
    const double min_position = lowerPositionLimit(options.upper_joint_limits, joint);
    const double max_position = upperPositionLimit(options.upper_joint_limits, joint);
    if (!validPositionLimit(min_position) || !validPositionLimit(max_position)) {
      return fail(LocoUpperCompileFailureKind::InvalidConfig,
                  "loco upper joint position limits are invalid");
    }
    if (std::isfinite(min_position) && std::isfinite(max_position) &&
        min_position > max_position) {
      return fail(LocoUpperCompileFailureKind::InvalidConfig,
                  "loco upper joint position limits are invalid");
    }
    if (!validNonnegativeLimit(velocityLimit(options.upper_joint_limits, joint))) {
      return fail(LocoUpperCompileFailureKind::InvalidConfig,
                  "loco upper joint velocity limits are invalid");
    }
    if (!validNonnegativeLimit(accelerationLimit(options.upper_joint_limits, joint))) {
      return fail(LocoUpperCompileFailureKind::InvalidConfig,
                  "loco upper joint acceleration limits are invalid");
    }
  }
  if (!validNonnegativeLimit(options.command_limits.max_linear_velocity) ||
      !validNonnegativeLimit(options.command_limits.max_yaw_rate) ||
      !validNonnegativeLimit(options.command_limits.max_linear_acceleration) ||
      !validNonnegativeLimit(options.command_limits.max_yaw_acceleration)) {
    return fail(LocoUpperCompileFailureKind::InvalidConfig,
                "loco upper command limits are invalid");
  }

  return {};
}

bool clampUpperPosition(LocoUpperLogicalJointFrame& frame,
                        const LocoUpperJointValidationOptions& limits,
                        LocoUpperCompileFlags& flags) {
  bool changed = false;
  for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
    const std::size_t logical_joint = kLocoUpperJointFirst + joint;
    const double min_position = lowerPositionLimit(limits, joint);
    const double max_position = upperPositionLimit(limits, joint);
    const double original = frame.at(logical_joint);
    double value = original;
    if (std::isfinite(min_position)) {
      value = std::max(value, min_position);
    }
    if (std::isfinite(max_position)) {
      value = std::min(value, max_position);
    }
    if (value != original) {
      frame.at(logical_joint) = value;
      changed = true;
    }
  }
  flags.upper_clamped = flags.upper_clamped || changed;
  return changed;
}

double clampToUpperPositionBounds(
    double value,
    const LocoUpperJointValidationOptions& limits,
    std::size_t upper_joint) {
  const double min_position = lowerPositionLimit(limits, upper_joint);
  const double max_position = upperPositionLimit(limits, upper_joint);
  if (std::isfinite(min_position)) {
    value = std::max(value, min_position);
  }
  if (std::isfinite(max_position)) {
    value = std::min(value, max_position);
  }
  return value;
}

double clampToBoundedInterval(double value,
                              double interval_min,
                              double interval_max,
                              const LocoUpperJointValidationOptions& limits,
                              std::size_t upper_joint) {
  const double min_position = lowerPositionLimit(limits, upper_joint);
  const double max_position = upperPositionLimit(limits, upper_joint);
  double bounded_min = interval_min;
  double bounded_max = interval_max;
  if (std::isfinite(min_position)) {
    bounded_min = std::max(bounded_min, min_position);
  }
  if (std::isfinite(max_position)) {
    bounded_max = std::min(bounded_max, max_position);
  }
  if (bounded_min <= bounded_max) {
    return std::clamp(value, bounded_min, bounded_max);
  }

  const double rate_projected = std::clamp(value, interval_min, interval_max);
  return clampToUpperPositionBounds(rate_projected, limits, upper_joint);
}

void limitUpperVelocity(std::vector<LocoUpperLogicalJointFrame>& frames,
                        double dt_s,
                        const LocoUpperJointValidationOptions& limits,
                        LocoUpperCompileFlags& flags) {
  for (std::size_t frame = 1; frame < frames.size(); ++frame) {
    for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
      const double max_velocity = velocityLimit(limits, joint);
      if (!std::isfinite(max_velocity)) {
        continue;
      }
      const double max_delta = std::max(0.0, max_velocity) * dt_s;
      const std::size_t logical_joint = kLocoUpperJointFirst + joint;
      const double previous = frames.at(frame - 1).at(logical_joint);
      const double original = frames.at(frame).at(logical_joint);
      const double limited = clampToBoundedInterval(original,
                                                    previous - max_delta,
                                                    previous + max_delta,
                                                    limits,
                                                    joint);
      if (limited != original) {
        frames.at(frame).at(logical_joint) = limited;
        flags.upper_velocity_limited = true;
        flags.upper_rate_limited = true;
      }
    }
  }
}

void limitUpperAcceleration(std::vector<LocoUpperLogicalJointFrame>& frames,
                            double dt_s,
                            const LocoUpperJointValidationOptions& limits,
                            LocoUpperCompileFlags& flags) {
  for (std::size_t frame = 2; frame < frames.size(); ++frame) {
    for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
      const double max_acceleration = accelerationLimit(limits, joint);
      if (!std::isfinite(max_acceleration)) {
        continue;
      }
      const std::size_t logical_joint = kLocoUpperJointFirst + joint;
      const double previous_previous = frames.at(frame - 2).at(logical_joint);
      const double previous = frames.at(frame - 1).at(logical_joint);
      const double original = frames.at(frame).at(logical_joint);
      const double previous_velocity = (previous - previous_previous) / dt_s;
      const double velocity = (original - previous) / dt_s;
      const double max_velocity_delta = std::max(0.0, max_acceleration) * dt_s;
      const double min_velocity = previous_velocity - max_velocity_delta;
      const double max_velocity = previous_velocity + max_velocity_delta;
      const double min_position = previous + min_velocity * dt_s;
      const double max_position = previous + max_velocity * dt_s;
      const double limited = clampToBoundedInterval(original,
                                                    min_position,
                                                    max_position,
                                                    limits,
                                                    joint);
      const bool acceleration_limited =
          velocity < min_velocity || velocity > max_velocity;
      if (limited != original) {
        frames.at(frame).at(logical_joint) = limited;
      }
      if (acceleration_limited) {
        flags.upper_accel_limited = true;
        flags.upper_rate_limited = true;
      }
    }
  }
}

std::vector<LocoUpperVelocityCommand> expandIntervalCommandsToFrames(
    const std::vector<LocoUpperVelocityCommand>& interval_commands,
    std::size_t frame_count) {
  std::vector<LocoUpperVelocityCommand> frame_commands;
  frame_commands.reserve(frame_count);
  const std::size_t interval_count =
      frame_count == 0 ? 0 : std::min(interval_commands.size(), frame_count - 1);
  frame_commands.insert(frame_commands.end(),
                        interval_commands.begin(),
                        interval_commands.begin() + interval_count);
  while (frame_commands.size() < frame_count) {
    frame_commands.push_back(LocoUpperVelocityCommand{});
  }
  return frame_commands;
}

LocoUpperCompileResult buildJointFrames(
    const TrkTrack& track,
    const LocoUpperJointValidationOptions& limits,
    double dt_s,
    CompiledLocoUpperPlan& plan,
    LocoUpperCompileFlags& flags) {
  const std::size_t frames = track.metadata.frames;
  if (!validFullJointArrayForFrames(track.joint_pos, frames)) {
    return fail(LocoUpperCompileFailureKind::InvalidTrack,
                "loco upper joint_pos array has invalid shape");
  }

  plan.joint_pos_frames.reserve(frames);
  for (std::size_t frame = 0; frame < frames; ++frame) {
    LocoUpperLogicalJointFrame out{};
    for (std::size_t joint = 0; joint < kLogicalJointCount; ++joint) {
      const float raw =
          track.joint_pos.values.at(frame * track.joint_pos.frame_size + joint);
      if (!std::isfinite(raw)) {
        return fail(LocoUpperCompileFailureKind::InvalidTrack,
                    "loco upper joint_pos values must be finite");
      }
      out.at(joint) = static_cast<double>(raw);
    }
    clampUpperPosition(out, limits, flags);
    plan.joint_pos_frames.push_back(out);
  }

  limitUpperVelocity(plan.joint_pos_frames, dt_s, limits, flags);
  limitUpperAcceleration(plan.joint_pos_frames, dt_s, limits, flags);
  return {};
}

}  // namespace

LocoUpperCompileResult LocoUpperPlanCompiler::compile(
    const TrkTrack& track,
    const LocoUpperCompileOptions& options) const {
  LocoUpperCompileResult option_check = validateCompileOptions(options);
  if (!option_check.message.empty()) {
    return option_check;
  }

  const LocoUpperRootPlanResult root =
      extractRootPlanarPath(track, options.root_options);
  if (!root.ok()) {
    return fail(LocoUpperCompileFailureKind::InvalidTrack, root.message);
  }

  if (root.plan.samples.size() != track.metadata.frames) {
    return fail(LocoUpperCompileFailureKind::InvalidTrack,
                "loco upper root and joint plans have mismatched frame counts");
  }

  LocoUpperCompileResult result;
  result.plan.fps = track.metadata.fps;
  result.plan.frame_count = track.metadata.frames;
  result.plan.duration_s =
      static_cast<double>(track.metadata.frames - 1) / track.metadata.fps;

  const LocoUpperRootPlan aligned = alignRootPlanToStart(root.plan);
  const LocoUpperProjectionResult projected =
      projectRootPlanToRadius(aligned, options.max_radius_m);
  result.flags.radius_clamped = projected.radius_clamped;
  result.plan.root_plan = projected.plan;

  const LocoUpperVelocityCommandPlan command_plan =
      rootPlanToVelocityCommandPlan(result.plan.root_plan, options.command_limits);
  result.plan.root_velocity_commands =
      expandIntervalCommandsToFrames(command_plan.commands, result.plan.frame_count);
  result.flags.envelope_clamped = command_plan.limits_clamped;

  LocoUpperCompileResult joint_result =
      buildJointFrames(track,
                       options.upper_joint_limits,
                       root.plan.dt_s,
                       result.plan,
                       result.flags);
  if (!joint_result.message.empty()) {
    return joint_result;
  }

  result.valid = true;
  result.failure_kind = LocoUpperCompileFailureKind::None;
  return result;
}

LocoUpperCompileResult compileLocoUpperPlan(
    const TrkTrack& track,
    const LocoUpperCompileOptions& options) {
  return LocoUpperPlanCompiler{}.compile(track, options);
}

}  // namespace agentic_et1_tracker
