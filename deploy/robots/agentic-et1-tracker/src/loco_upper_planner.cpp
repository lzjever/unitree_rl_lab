#include "agentic_et1_tracker/loco_upper/planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kBodyPositionDimensions = 3;
constexpr std::size_t kBodyQuatDimensions = 4;
constexpr std::size_t kMaxRootFrameStride = kBodyQuatDimensions;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

LocoUpperRootPlanResult fail(std::string message) {
  LocoUpperRootPlanResult result;
  result.message = std::move(message);
  return result;
}

bool validFloatArrayForFrames(const TrkFloatArray& array,
                              std::size_t frames,
                              std::size_t min_frame_size) {
  if (frames == 0 || array.frame_size < min_frame_size) {
    return false;
  }
  if (array.frame_size != 0 &&
      frames > std::numeric_limits<std::size_t>::max() / array.frame_size) {
    return false;
  }
  return array.values.size() >= frames * array.frame_size;
}

double finitePositiveLimitOrInfinity(double value) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, value);
}

double clampAbs(double value, double max_abs) {
  if (!std::isfinite(max_abs)) {
    return value;
  }
  return std::clamp(value, -max_abs, max_abs);
}

bool clampLinearVelocity(double max_linear_velocity,
                         LocoUpperVelocityCommand& command) {
  const double max_speed = finitePositiveLimitOrInfinity(max_linear_velocity);
  const double speed = std::hypot(command.vx, command.vy);
  if (!std::isfinite(speed) || !std::isfinite(max_speed) || speed <= max_speed) {
    return false;
  }
  if (speed <= std::numeric_limits<double>::epsilon() || max_speed == 0.0) {
    command.vx = 0.0;
    command.vy = 0.0;
    return true;
  }
  const double scale = max_speed / speed;
  command.vx *= scale;
  command.vy *= scale;
  return true;
}

bool clampLinearAcceleration(const LocoUpperVelocityCommand& previous,
                             double max_linear_acceleration,
                             double dt_s,
                             LocoUpperVelocityCommand& command) {
  const double max_accel = finitePositiveLimitOrInfinity(max_linear_acceleration);
  if (!std::isfinite(max_accel)) {
    return false;
  }

  const double max_delta = max_accel * dt_s;
  const double dvx = command.vx - previous.vx;
  const double dvy = command.vy - previous.vy;
  const double delta = std::hypot(dvx, dvy);
  if (!std::isfinite(delta) || delta <= max_delta) {
    return false;
  }
  if (delta <= std::numeric_limits<double>::epsilon() || max_delta == 0.0) {
    command.vx = previous.vx;
    command.vy = previous.vy;
    return true;
  }

  const double scale = max_delta / delta;
  command.vx = previous.vx + dvx * scale;
  command.vy = previous.vy + dvy * scale;
  return true;
}

bool clampYawAcceleration(const LocoUpperVelocityCommand& previous,
                          double max_yaw_acceleration,
                          double dt_s,
                          LocoUpperVelocityCommand& command) {
  const double max_accel = finitePositiveLimitOrInfinity(max_yaw_acceleration);
  if (!std::isfinite(max_accel)) {
    return false;
  }
  const double max_delta = max_accel * dt_s;
  const double delta = command.yaw_rate - previous.yaw_rate;
  const double clamped_delta = clampAbs(delta, max_delta);
  command.yaw_rate = previous.yaw_rate + clamped_delta;
  return clamped_delta != delta;
}

LocoUpperVelocityCommand smoothedCommand(
    const std::vector<LocoUpperVelocityCommand>& raw_commands,
    std::size_t index,
    std::size_t window_frames) {
  const std::size_t window = std::max<std::size_t>(1, window_frames);
  const std::size_t begin = index + 1 > window ? index + 1 - window : 0;
  LocoUpperVelocityCommand out;
  std::size_t count = 0;
  for (std::size_t i = begin; i <= index; ++i) {
    out.vx += raw_commands.at(i).vx;
    out.vy += raw_commands.at(i).vy;
    out.yaw_rate += raw_commands.at(i).yaw_rate;
    ++count;
  }
  if (count > 0) {
    out.vx /= static_cast<double>(count);
    out.vy /= static_cast<double>(count);
    out.yaw_rate /= static_cast<double>(count);
  }
  return out;
}

double normalizedYawFromQuat(const float* q) {
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < kBodyQuatDimensions; ++i) {
    norm_sq += static_cast<double>(q[i]) * static_cast<double>(q[i]);
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  const double w = static_cast<double>(q[0]) * inv_norm;
  const double x = static_cast<double>(q[1]) * inv_norm;
  const double y = static_cast<double>(q[2]) * inv_norm;
  const double z = static_cast<double>(q[3]) * inv_norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

double unwrapNear(double yaw, double previous_yaw) {
  double unwrapped = yaw;
  while (unwrapped - previous_yaw > kPi) {
    unwrapped -= kTwoPi;
  }
  while (unwrapped - previous_yaw < -kPi) {
    unwrapped += kTwoPi;
  }
  return unwrapped;
}

bool validRootQuaternion(const TrkFloatArray& body_quat_w,
                         std::size_t frame,
                         std::size_t root_body_index,
                         double& yaw_out) {
  const std::size_t offset =
      frame * body_quat_w.frame_size + root_body_index * kBodyQuatDimensions;
  const float* q = body_quat_w.values.data() + offset;
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < kBodyQuatDimensions; ++i) {
    if (!std::isfinite(q[i])) {
      return false;
    }
    norm_sq += static_cast<double>(q[i]) * static_cast<double>(q[i]);
  }

  const double min_norm_sq =
      static_cast<double>(std::numeric_limits<float>::epsilon()) *
      static_cast<double>(std::numeric_limits<float>::epsilon());
  if (norm_sq <= min_norm_sq) {
    return false;
  }

  yaw_out = normalizedYawFromQuat(q);
  return std::isfinite(yaw_out);
}

}  // namespace

LocoUpperRootPlanResult extractRootPlanarPath(
    const TrkTrack& track,
    const LocoUpperPlannerOptions& options) {
  const std::size_t frames = track.metadata.frames;
  if (frames < 2) {
    return fail("loco upper planner requires at least two frames");
  }
  if (!std::isfinite(track.metadata.fps) || track.metadata.fps <= 0.0) {
    return fail("loco upper planner requires finite positive fps");
  }

  const double dt_s = 1.0 / track.metadata.fps;
  const double duration_s = static_cast<double>(frames - 1) * dt_s;
  if (std::isfinite(options.min_duration_s) &&
      duration_s < std::max(0.0, options.min_duration_s)) {
    return fail("loco upper planner track is too short");
  }

  if (options.root_body_index >
      std::numeric_limits<std::size_t>::max() / kMaxRootFrameStride - 1) {
    return fail("loco upper planner root body index is invalid");
  }
  const std::size_t min_pos_frame_size =
      (options.root_body_index + 1) * kBodyPositionDimensions;
  const std::size_t min_quat_frame_size =
      (options.root_body_index + 1) * kBodyQuatDimensions;
  if (!validFloatArrayForFrames(track.body_pos_w, frames, min_pos_frame_size) ||
      !validFloatArrayForFrames(track.body_quat_w, frames, min_quat_frame_size)) {
    return fail("loco upper planner root arrays have invalid shape");
  }

  LocoUpperRootPlan plan;
  plan.dt_s = dt_s;
  plan.samples.reserve(frames);
  for (std::size_t frame = 0; frame < frames; ++frame) {
    const std::size_t pos_offset =
        frame * track.body_pos_w.frame_size +
        options.root_body_index * kBodyPositionDimensions;
    const float x = track.body_pos_w.values.at(pos_offset + 0);
    const float y = track.body_pos_w.values.at(pos_offset + 1);
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return fail("loco upper planner root XY must be finite");
    }

    double yaw = 0.0;
    if (!validRootQuaternion(track.body_quat_w,
                             frame,
                             options.root_body_index,
                             yaw)) {
      return fail("loco upper planner root quaternion must be finite and nonzero");
    }
    if (!plan.samples.empty()) {
      yaw = unwrapNear(yaw, plan.samples.back().yaw);
    }

    plan.samples.push_back(
        {static_cast<double>(x), static_cast<double>(y), yaw});
  }

  LocoUpperRootPlanResult result;
  result.valid = true;
  result.plan = std::move(plan);
  return result;
}

LocoUpperRootPlan alignRootPlanToStart(const LocoUpperRootPlan& plan,
                                       const LocoUpperPlanarSample& start) {
  LocoUpperRootPlan aligned;
  aligned.dt_s = plan.dt_s;
  aligned.samples.reserve(plan.samples.size());
  if (plan.samples.empty()) {
    return aligned;
  }

  const LocoUpperPlanarSample& first = plan.samples.front();
  const double yaw_delta = start.yaw - first.yaw;
  const double cos_yaw = std::cos(yaw_delta);
  const double sin_yaw = std::sin(yaw_delta);
  for (const LocoUpperPlanarSample& sample : plan.samples) {
    const double dx = sample.x - first.x;
    const double dy = sample.y - first.y;
    aligned.samples.push_back({start.x + cos_yaw * dx - sin_yaw * dy,
                               start.y + sin_yaw * dx + cos_yaw * dy,
                               sample.yaw + yaw_delta});
  }
  return aligned;
}

LocoUpperProjectionResult projectRootPlanToRadius(const LocoUpperRootPlan& plan,
                                                  double max_radius,
                                                  const LocoUpperPlanarSample& center) {
  LocoUpperProjectionResult result;
  result.plan = plan;
  if (std::isnan(max_radius) || max_radius < 0.0) {
    return result;
  }
  if (!std::isfinite(max_radius)) {
    return result;
  }

  for (LocoUpperPlanarSample& sample : result.plan.samples) {
    const double dx = sample.x - center.x;
    const double dy = sample.y - center.y;
    const double radius = std::hypot(dx, dy);
    if (!std::isfinite(radius) || radius <= max_radius) {
      continue;
    }
    result.radius_clamped = true;
    if (radius <= std::numeric_limits<double>::epsilon() || max_radius == 0.0) {
      sample.x = center.x;
      sample.y = center.y;
      continue;
    }
    const double scale = max_radius / radius;
    sample.x = center.x + dx * scale;
    sample.y = center.y + dy * scale;
  }
  return result;
}

LocoUpperVelocityCommandPlan rootPlanToVelocityCommandPlan(
    const LocoUpperRootPlan& plan,
    const LocoUpperCommandLimits& limits) {
  LocoUpperVelocityCommandPlan plan_result;
  std::vector<LocoUpperVelocityCommand>& commands = plan_result.commands;
  if (plan.samples.size() < 2 || !std::isfinite(plan.dt_s) || plan.dt_s <= 0.0) {
    return plan_result;
  }

  std::vector<LocoUpperVelocityCommand> raw_commands;
  raw_commands.reserve(plan.samples.size() - 1);
  for (std::size_t i = 1; i < plan.samples.size(); ++i) {
    const LocoUpperPlanarSample& previous_sample = plan.samples.at(i - 1);
    const LocoUpperPlanarSample& sample = plan.samples.at(i);
    LocoUpperVelocityCommand command;
    command.vx = (sample.x - previous_sample.x) / plan.dt_s;
    command.vy = (sample.y - previous_sample.y) / plan.dt_s;
    command.yaw_rate = (sample.yaw - previous_sample.yaw) / plan.dt_s;
    raw_commands.push_back(command);
  }

  commands.reserve(raw_commands.size());
  const double max_yaw_rate = finitePositiveLimitOrInfinity(limits.max_yaw_rate);
  for (std::size_t i = 0; i < raw_commands.size(); ++i) {
    LocoUpperVelocityCommand command =
        smoothedCommand(raw_commands, i, limits.smoothing_window_frames);
    plan_result.limits_clamped =
        clampLinearVelocity(limits.max_linear_velocity, command) ||
        plan_result.limits_clamped;
    const double unclamped_yaw_rate = command.yaw_rate;
    command.yaw_rate = clampAbs(command.yaw_rate, max_yaw_rate);
    if (command.yaw_rate != unclamped_yaw_rate) {
      plan_result.limits_clamped = true;
    }
    if (!commands.empty()) {
      const LocoUpperVelocityCommand& previous_command = commands.back();
      plan_result.limits_clamped =
          clampLinearAcceleration(previous_command,
                                  limits.max_linear_acceleration,
                                  plan.dt_s,
                                  command) ||
          plan_result.limits_clamped;
      plan_result.limits_clamped =
          clampYawAcceleration(previous_command,
                               limits.max_yaw_acceleration,
                               plan.dt_s,
                               command) ||
          plan_result.limits_clamped;
    }
    commands.push_back(command);
  }
  return plan_result;
}

std::vector<LocoUpperVelocityCommand> rootPlanToVelocityCommands(
    const LocoUpperRootPlan& plan,
    const LocoUpperCommandLimits& limits) {
  return rootPlanToVelocityCommandPlan(plan, limits).commands;
}

LocoUpperVelocityCommand suppressOutwardRadialVelocityNearRadius(
    const LocoUpperPlanarSample& current_root,
    const LocoUpperVelocityCommand& command,
    double max_radius,
    double margin,
    const LocoUpperPlanarSample& center) {
  LocoUpperVelocityCommand limited = command;
  limited.radius_limit_reached = false;
  if (std::isnan(max_radius) || max_radius < 0.0 || !std::isfinite(max_radius)) {
    return limited;
  }

  const double nonnegative_margin =
      std::isfinite(margin) ? std::max(0.0, margin) : 0.0;
  const double dx = current_root.x - center.x;
  const double dy = current_root.y - center.y;
  const double radius = std::hypot(dx, dy);
  if (!std::isfinite(radius) || radius < max_radius - nonnegative_margin) {
    return limited;
  }

  if (radius <= std::numeric_limits<double>::epsilon()) {
    return limited;
  }

  const double ux = dx / radius;
  const double uy = dy / radius;
  const double outward_velocity = limited.vx * ux + limited.vy * uy;
  if (std::isfinite(outward_velocity) && outward_velocity > 0.0) {
    limited.vx -= outward_velocity * ux;
    limited.vy -= outward_velocity * uy;
    limited.radius_limit_reached = true;
  }
  return limited;
}

LocoUpperVelocityCommand worldVelocityToBodyCommand(
    const LocoUpperVelocityCommand& command_world,
    double robot_yaw) {
  LocoUpperVelocityCommand command_body = command_world;
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  command_body.vx = cos_yaw * command_world.vx + sin_yaw * command_world.vy;
  command_body.vy = -sin_yaw * command_world.vx + cos_yaw * command_world.vy;
  return command_body;
}

}  // namespace agentic_et1_tracker
