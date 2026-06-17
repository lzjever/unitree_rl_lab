#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/loco_upper/validator.hpp"

namespace agentic_et1_tracker {

class TrkLoader;
struct TrkTrack;

struct LocoUpperPrecheckOptions {
  double max_radius_m{0.0};
  bool strict_pose{false};
  bool hold{false};
  std::optional<LocoUpperJointValidationOptions> upper_joint_limits;
};

struct LocoUpperPrecheckResult {
  ErrorCode code{ErrorCode::Ok};
  std::string message;

  bool ok() const { return code == ErrorCode::Ok; }
};

class LocoUpperPrecheckPort {
 public:
  virtual ~LocoUpperPrecheckPort() = default;
  virtual LocoUpperPrecheckResult precheck(
      const std::string& canonical_path,
      const LocoUpperPrecheckOptions& options) = 0;
};

[[nodiscard]] LocoUpperPrecheckResult precheckLocoUpperTrack(
    const TrkTrack& track,
    const LocoUpperPrecheckOptions& options);

[[nodiscard]] LocoUpperPrecheckResult precheckLocoUpperTrackFile(
    const TrkLoader& loader,
    const std::filesystem::path& path,
    const LocoUpperPrecheckOptions& options);

}  // namespace agentic_et1_tracker
