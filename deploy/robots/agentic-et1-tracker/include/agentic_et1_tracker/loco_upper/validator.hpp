#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "agentic_et1_tracker/trk/loader.hpp"

namespace agentic_et1_tracker {

constexpr std::size_t kLocoUpperJointFirst = 12;
constexpr std::size_t kLocoUpperJointLastExclusive = 26;
constexpr std::size_t kLocoUpperJointCount =
    kLocoUpperJointLastExclusive - kLocoUpperJointFirst;

using LocoUpperJointFrame = std::array<double, kLocoUpperJointCount>;

struct LocoUpperJointTargetPlan {
  std::vector<LocoUpperJointFrame> frames;
  double dt_s{0.0};
};

struct LocoUpperJointValidationOptions {
  std::vector<double> min_positions;
  std::vector<double> max_positions;
  std::vector<double> max_velocities;
  std::vector<double> max_accelerations;
  double min_position{-std::numeric_limits<double>::infinity()};
  double max_position{std::numeric_limits<double>::infinity()};
  double max_velocity{std::numeric_limits<double>::infinity()};
  double max_acceleration{std::numeric_limits<double>::infinity()};
};

enum class LocoUpperJointValidationFailureKind {
  None,
  Position,
  Velocity,
  Acceleration,
  InvalidConfig,
  InvalidTrack,
};

struct LocoUpperJointValidationResult {
  bool valid{false};
  LocoUpperJointTargetPlan plan;
  std::string message;
  std::size_t frame_index{0};
  std::size_t joint_index{kLocoUpperJointFirst};
  LocoUpperJointValidationFailureKind failure_kind{
      LocoUpperJointValidationFailureKind::None};

  bool ok() const { return valid; }
};

[[nodiscard]] LocoUpperJointValidationOptions loadLocoUpperJointValidationOptions(
    const std::filesystem::path& limits_path);

[[nodiscard]] LocoUpperJointValidationResult extractAndValidateUpperJointTargets(
    const TrkTrack& track,
    const LocoUpperJointValidationOptions& options = {});

}  // namespace agentic_et1_tracker
