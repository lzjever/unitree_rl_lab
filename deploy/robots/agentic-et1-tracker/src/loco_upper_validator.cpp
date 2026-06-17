#include "agentic_et1_tracker/loco_upper/validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "agentic_et1_tracker/loco_upper/lowcmd_composer.hpp"

namespace agentic_et1_tracker {
namespace {

LocoUpperLowCmdComposerError loaderError(const std::string& message) {
  return LocoUpperLowCmdComposerError("loco upper joint validation error: " + message);
}

LocoUpperJointValidationResult fail(
    std::string message,
    std::size_t frame_index = 0,
    std::size_t joint_index = kLocoUpperJointFirst,
    LocoUpperJointValidationFailureKind failure_kind =
        LocoUpperJointValidationFailureKind::InvalidTrack) {
  LocoUpperJointValidationResult result;
  result.message = std::move(message);
  result.frame_index = frame_index;
  result.joint_index = joint_index;
  result.failure_kind = failure_kind;
  return result;
}

bool validJointArrayForFrames(const TrkFloatArray& array, std::size_t frames) {
  if (frames == 0 || array.frame_size < kLocoUpperJointLastExclusive) {
    return false;
  }
  if (array.frame_size != 0 &&
      frames > std::numeric_limits<std::size_t>::max() / array.frame_size) {
    return false;
  }
  return array.values.size() >= frames * array.frame_size;
}

double finiteNonnegativeLimitOrInfinity(double value) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, value);
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& err) {
    throw loaderError(field + " has invalid value: " + err.what());
  }
}

YAML::Node loadYamlMap(const std::filesystem::path& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception& err) {
    throw loaderError(std::string("limits failed to load: ") + err.what());
  }
  if (!root || !root.IsMap()) {
    throw loaderError("limits must be a map");
  }
  return root;
}

YAML::Node requiredNode(const YAML::Node& root,
                        const char* key,
                        const std::string& field) {
  const YAML::Node node = root[key];
  if (!node) {
    throw loaderError(field + " is required");
  }
  return node;
}

std::vector<double> readLimitVector(const YAML::Node& root,
                                    const char* key,
                                    const std::string& field) {
  const YAML::Node node = requiredNode(root, key, field);
  if (!node.IsSequence() || node.size() != kLocoUpperJointCount) {
    std::ostringstream out;
    out << field << " must contain " << kLocoUpperJointCount << " entries";
    throw loaderError(out.str());
  }

  std::vector<double> values;
  values.reserve(kLocoUpperJointCount);
  for (std::size_t i = 0; i < kLocoUpperJointCount; ++i) {
    const double value =
        scalarAs<double>(node[i], field + "[" + std::to_string(i) + "]");
    if (!std::isfinite(value)) {
      throw loaderError(field + "[" + std::to_string(i) + "] must be finite");
    }
    values.push_back(value);
  }
  return values;
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
    return finiteNonnegativeLimitOrInfinity(options.max_velocities.at(upper_joint));
  }
  return finiteNonnegativeLimitOrInfinity(options.max_velocity);
}

double accelerationLimit(const LocoUpperJointValidationOptions& options,
                         std::size_t upper_joint) {
  if (options.max_accelerations.size() == kLocoUpperJointCount) {
    return finiteNonnegativeLimitOrInfinity(
        options.max_accelerations.at(upper_joint));
  }
  return finiteNonnegativeLimitOrInfinity(options.max_acceleration);
}

LocoUpperJointValidationResult validatePerJointConfiguration(
    const LocoUpperJointValidationOptions& options) {
  const auto expect_size = [](const std::vector<double>& values) {
    return values.empty() || values.size() == kLocoUpperJointCount;
  };
  if (!expect_size(options.min_positions) || !expect_size(options.max_positions) ||
      !expect_size(options.max_velocities) ||
      !expect_size(options.max_accelerations)) {
    return fail("loco upper joint validation vectors must contain 14 entries",
                0,
                kLocoUpperJointFirst,
                LocoUpperJointValidationFailureKind::InvalidConfig);
  }

  for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
    const double min_position = lowerPositionLimit(options, joint);
    const double max_position = upperPositionLimit(options, joint);
    if (std::isfinite(min_position) && std::isfinite(max_position) &&
        min_position > max_position) {
      return fail("loco upper joint position limits are invalid",
                  0,
                  kLocoUpperJointFirst + joint,
                  LocoUpperJointValidationFailureKind::InvalidConfig);
    }
    const double max_velocity = velocityLimit(options, joint);
    const double max_acceleration = accelerationLimit(options, joint);
    if (std::isfinite(max_velocity) && max_velocity < 0.0) {
      return fail("loco upper joint velocity limits are invalid",
                  0,
                  kLocoUpperJointFirst + joint,
                  LocoUpperJointValidationFailureKind::InvalidConfig);
    }
    if (std::isfinite(max_acceleration) && max_acceleration < 0.0) {
      return fail("loco upper joint acceleration limits are invalid",
                  0,
                  kLocoUpperJointFirst + joint,
                  LocoUpperJointValidationFailureKind::InvalidConfig);
    }
  }

  return {};
}

}  // namespace

LocoUpperJointValidationOptions loadLocoUpperJointValidationOptions(
    const std::filesystem::path& limits_path) {
  const YAML::Node limits = loadYamlMap(limits_path);

  LocoUpperJointValidationOptions options;
  options.min_positions = readLimitVector(limits, "upper_min_q", "upper_min_q");
  options.max_positions = readLimitVector(limits, "upper_max_q", "upper_max_q");
  options.max_velocities = readLimitVector(limits, "max_vel_radps", "max_vel_radps");
  options.max_accelerations =
      readLimitVector(limits, "max_accel_radps2", "max_accel_radps2");
  return options;
}

LocoUpperJointValidationResult extractAndValidateUpperJointTargets(
    const TrkTrack& track,
    const LocoUpperJointValidationOptions& options) {
  const std::size_t frames = track.metadata.frames;
  if (frames == 0) {
    return fail("loco upper joint targets require at least one frame");
  }
  if (!std::isfinite(track.metadata.fps) || track.metadata.fps <= 0.0) {
    return fail("loco upper joint targets require finite positive fps");
  }
  if (!validJointArrayForFrames(track.joint_pos, frames)) {
    return fail("loco upper joint_pos array has invalid shape");
  }
  const LocoUpperJointValidationResult config_check =
      validatePerJointConfiguration(options);
  if (!config_check.ok() && !config_check.message.empty()) {
    return config_check;
  }

  LocoUpperJointTargetPlan plan;
  plan.dt_s = 1.0 / track.metadata.fps;
  plan.frames.reserve(frames);
  for (std::size_t frame = 0; frame < frames; ++frame) {
    LocoUpperJointFrame target{};
    for (std::size_t joint = kLocoUpperJointFirst;
         joint < kLocoUpperJointLastExclusive;
         ++joint) {
      const float raw =
          track.joint_pos.values.at(frame * track.joint_pos.frame_size + joint);
      if (!std::isfinite(raw)) {
        return fail("loco upper joint_pos values must be finite",
                    frame,
                    joint,
                    LocoUpperJointValidationFailureKind::InvalidTrack);
      }
      const double value = static_cast<double>(raw);
      const std::size_t upper_joint = joint - kLocoUpperJointFirst;
      const double min_position = lowerPositionLimit(options, upper_joint);
      const double max_position = upperPositionLimit(options, upper_joint);
      if (std::isfinite(min_position) && value < min_position) {
        return fail("loco upper joint_pos is below limit",
                    frame,
                    joint,
                    LocoUpperJointValidationFailureKind::Position);
      }
      if (std::isfinite(max_position) && value > max_position) {
        return fail("loco upper joint_pos is above limit",
                    frame,
                    joint,
                    LocoUpperJointValidationFailureKind::Position);
      }
      target.at(joint - kLocoUpperJointFirst) = value;
    }
    plan.frames.push_back(target);
  }

  for (std::size_t frame = 1; frame < plan.frames.size(); ++frame) {
    for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
      const double max_velocity = velocityLimit(options, joint);
      if (std::isfinite(max_velocity)) {
        const double velocity =
            (plan.frames.at(frame).at(joint) - plan.frames.at(frame - 1).at(joint)) /
            plan.dt_s;
        if (std::abs(velocity) > max_velocity) {
          return fail("loco upper joint velocity exceeds limit",
                      frame,
                      kLocoUpperJointFirst + joint,
                      LocoUpperJointValidationFailureKind::Velocity);
        }
      }
    }
  }

  for (std::size_t frame = 2; frame < plan.frames.size(); ++frame) {
    for (std::size_t joint = 0; joint < kLocoUpperJointCount; ++joint) {
      const double max_acceleration = accelerationLimit(options, joint);
      if (std::isfinite(max_acceleration)) {
        const double previous_velocity =
            (plan.frames.at(frame - 1).at(joint) -
             plan.frames.at(frame - 2).at(joint)) /
            plan.dt_s;
        const double velocity =
            (plan.frames.at(frame).at(joint) -
             plan.frames.at(frame - 1).at(joint)) /
            plan.dt_s;
        const double acceleration = (velocity - previous_velocity) / plan.dt_s;
        if (std::abs(acceleration) > max_acceleration) {
          return fail("loco upper joint acceleration exceeds limit",
                      frame,
                      kLocoUpperJointFirst + joint,
                      LocoUpperJointValidationFailureKind::Acceleration);
        }
      }
    }
  }

  LocoUpperJointValidationResult result;
  result.valid = true;
  result.plan = std::move(plan);
  return result;
}

}  // namespace agentic_et1_tracker
