#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

struct LocoUpperPlanarSample {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct LocoUpperRootPlan {
  std::vector<LocoUpperPlanarSample> samples;
  double dt_s{0.0};
};

struct LocoUpperPlannerOptions {
  std::size_t root_body_index{0};
  double min_duration_s{0.02};
};

struct LocoUpperRootPlanResult {
  bool valid{false};
  LocoUpperRootPlan plan;
  std::string message;

  bool ok() const { return valid; }
};

struct LocoUpperProjectionResult {
  LocoUpperRootPlan plan;
  bool radius_clamped{false};
};

struct LocoUpperVelocityCommand {
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  bool radius_limit_reached{false};
};

struct LocoUpperCommandLimits {
  double max_linear_velocity{std::numeric_limits<double>::infinity()};
  double max_yaw_rate{std::numeric_limits<double>::infinity()};
  double max_linear_acceleration{std::numeric_limits<double>::infinity()};
  double max_yaw_acceleration{std::numeric_limits<double>::infinity()};
  std::size_t smoothing_window_frames{1};
};

struct LocoUpperVelocityCommandPlan {
  std::vector<LocoUpperVelocityCommand> commands;
  bool limits_clamped{false};
};

[[nodiscard]] LocoUpperRootPlanResult extractRootPlanarPath(
    const TrkTrack& track,
    const LocoUpperPlannerOptions& options = {});

[[nodiscard]] LocoUpperRootPlan alignRootPlanToStart(
    const LocoUpperRootPlan& plan,
    const LocoUpperPlanarSample& start = {});

[[nodiscard]] LocoUpperProjectionResult projectRootPlanToRadius(
    const LocoUpperRootPlan& plan,
    double max_radius,
    const LocoUpperPlanarSample& center = {});

[[nodiscard]] std::vector<LocoUpperVelocityCommand> rootPlanToVelocityCommands(
    const LocoUpperRootPlan& plan,
    const LocoUpperCommandLimits& limits = {});

[[nodiscard]] LocoUpperVelocityCommandPlan rootPlanToVelocityCommandPlan(
    const LocoUpperRootPlan& plan,
    const LocoUpperCommandLimits& limits = {});

[[nodiscard]] LocoUpperVelocityCommand suppressOutwardRadialVelocityNearRadius(
    const LocoUpperPlanarSample& current_root,
    const LocoUpperVelocityCommand& command,
    double max_radius,
    double margin,
    const LocoUpperPlanarSample& center = {});

[[nodiscard]] LocoUpperVelocityCommand worldVelocityToBodyCommand(
    const LocoUpperVelocityCommand& command_world,
    double robot_yaw);

}  // namespace agentic_et1_tracker
