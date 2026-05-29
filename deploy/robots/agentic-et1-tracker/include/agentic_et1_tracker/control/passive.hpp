#pragma once

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "agentic_et1_tracker/control/fixstand.hpp"

namespace agentic_et1_tracker {

struct PassiveConfig {
  std::vector<int> mode;
  std::vector<double> kd;
};

class PassiveConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

PassiveConfig loadPassiveConfig(const std::filesystem::path& path);

LowCmdFrame makePassiveLowCmdFrame(const PassiveConfig& config,
                                   const LowStateSample& low_state,
                                   std::uint8_t expected_mode_machine);

}  // namespace agentic_et1_tracker
