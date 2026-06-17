#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "agentic_et1_tracker/loco_upper/planner.hpp"
#include "agentic_et1_tracker/loco_upper/validator.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {

using LocoUpperLogicalJointFrame =
    std::array<double, static_cast<std::size_t>(TrkSchema::kJointDim)>;

struct LocoUpperCompileFlags {
  bool radius_clamped{false};
  bool envelope_clamped{false};
  bool upper_clamped{false};
  bool upper_velocity_limited{false};
  bool upper_accel_limited{false};
  bool upper_rate_limited{false};
};

struct CompiledLocoUpperPlan {
  double fps{0.0};
  std::size_t frame_count{0};
  double duration_s{0.0};
  LocoUpperRootPlan root_plan;
  std::vector<LocoUpperVelocityCommand> root_velocity_commands;
  std::vector<LocoUpperLogicalJointFrame> joint_pos_frames;
};

struct LocoUpperCompileOptions {
  double max_radius_m{0.0};
  LocoUpperPlannerOptions root_options;
  LocoUpperCommandLimits command_limits;
  LocoUpperJointValidationOptions upper_joint_limits;
  bool probe_only{false};
};

enum class LocoUpperCompileFailureKind {
  None,
  InvalidOptions,
  InvalidConfig,
  InvalidTrack,
};

struct LocoUpperCompileResult {
  bool valid{false};
  CompiledLocoUpperPlan plan;
  LocoUpperCompileFlags flags;
  std::string message;
  LocoUpperCompileFailureKind failure_kind{LocoUpperCompileFailureKind::None};

  bool ok() const { return valid; }
};

class LocoUpperPlanCompiler {
 public:
  [[nodiscard]] LocoUpperCompileResult compile(
      const TrkTrack& track,
      const LocoUpperCompileOptions& options) const;
};

[[nodiscard]] LocoUpperCompileResult compileLocoUpperPlan(
    const TrkTrack& track,
    const LocoUpperCompileOptions& options);

}  // namespace agentic_et1_tracker
