#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "agentic_et1_tracker/robot/robot_io.hpp"

namespace agentic_et1_tracker {

inline constexpr std::size_t kFixStandMotorCount = 33;

struct FixStandConfig {
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<double> target_q;
  double duration_s{3.0};
};

class FixStandConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

FixStandConfig loadFixStandConfig(const std::filesystem::path& path);

class FixStandRunner {
 public:
  FixStandRunner(FixStandConfig config,
                 std::uint8_t expected_mode_machine,
                 double hz);

  void reset();
  LowCmdFrame step(const LowStateSample& low_state,
                   const LowCmdFrame* base_frame = nullptr);

 private:
  FixStandConfig config_;
  std::uint8_t expected_mode_machine_{0};
  double hz_{50.0};
  std::vector<float> q0_;
  std::size_t tick_{0};
  bool initialized_{false};
};

LowCmdFrame makeFixStandLowCmdFrame(const FixStandConfig& config,
                                    const std::vector<float>& q,
                                    std::uint8_t expected_mode_machine,
                                    const LowCmdFrame* base_frame = nullptr);

}  // namespace agentic_et1_tracker
