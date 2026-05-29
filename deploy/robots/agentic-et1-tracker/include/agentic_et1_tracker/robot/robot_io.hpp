#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/policy/policy_math.hpp"

namespace agentic_et1_tracker {

constexpr std::size_t kSdkMotorCount = 35;
constexpr std::size_t kPolicyJointCount = 26;

struct MotorStateSample {
  std::uint8_t mode{0};
  float q{0.0F};
  float dq{0.0F};
  float tau_est{0.0F};
};

struct MotorCommand {
  std::uint8_t mode{1};
  float q{0.0F};
  float dq{0.0F};
  float kp{0.0F};
  float kd{0.0F};
  float tau{0.0F};
};

struct LowStateSample {
  bool fresh{false};
  std::size_t age_ms{0};
  std::uint8_t mode_machine{0};
  std::uint8_t mode_pr{0};
  std::array<float, 4> quat_wxyz{{1.0F, 0.0F, 0.0F, 0.0F}};
  std::array<float, 3> gyro{{0.0F, 0.0F, 0.0F}};
  std::array<MotorStateSample, kSdkMotorCount> motors{};
};

struct HighStateSample {
  bool fresh{false};
  std::size_t age_ms{0};
  std::array<float, 3> position{{0.0F, 0.0F, 0.0F}};
  std::array<float, 4> quat_wxyz{{1.0F, 0.0F, 0.0F, 0.0F}};
  std::array<float, 3> linear_velocity{{0.0F, 0.0F, 0.0F}};
  std::array<float, 3> angular_velocity{{0.0F, 0.0F, 0.0F}};
};

struct LowCmdFrame {
  std::uint8_t mode_machine{1};
  std::uint8_t mode_pr{0};
  std::array<MotorCommand, kSdkMotorCount> motors{};
};

struct LowCmdOccupancy {
  bool occupied{false};
  std::size_t sample_age_ms{0};
};

enum class OrientationSafety {
  Enforce,
  Skip,
};

struct ModeMachineCheck {
  bool connected{false};
  bool ok{false};
  std::optional<std::uint8_t> observed;
  std::uint8_t expected{0};
};

struct RobotReadinessStatus {
  RobotState robot{RobotState::Disconnected};
  ErrorCode err{ErrorCode::RobotDisconnected};
  std::string block;
  std::optional<std::size_t> low_ms;
};

constexpr float kMinSafeBodyZProjection = 0.5403023058681398F;  // cos(1 rad)
constexpr float kMinSafeQuaternionNormSquared = 1.0e-12F;

class RobotIO {
 public:
  virtual ~RobotIO() = default;

  virtual std::optional<LowStateSample> readLowState() const = 0;
  virtual std::optional<HighStateSample> readHighState() const = 0;
  virtual LowCmdOccupancy lowCmdOccupancy() const = 0;
  virtual void writeLowCmd(const LowCmdFrame& frame) = 0;
};

class RobotIOError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

ModeMachineCheck checkModeMachine(const std::optional<LowStateSample>& low_state,
                                  std::uint8_t expected_mode_machine);

bool hasSafeBodyOrientation(const LowStateSample& low_state);

RobotReadinessStatus mapRobotReadiness(const std::optional<LowStateSample>& low_state,
                                       LowCmdOccupancy occupancy,
                                       std::uint8_t expected_mode_machine,
                                       OrientationSafety orientation_safety =
                                           OrientationSafety::Enforce);

LowCmdFrame makeLowCmdFrame(const DeployConfig& config,
                            const PolicyOutput& output,
                            std::uint8_t expected_mode_machine,
                            const LowCmdFrame* base_frame = nullptr);

LowCmdFrame makeLowCmdFrame(const std::vector<int>& sdk_joint_ids_map,
                            const PolicyOutput& output,
                            std::uint8_t expected_mode_machine,
                            const LowCmdFrame* base_frame = nullptr);

LowCmdFrame makeHoldLowCmdFrame(const DeployConfig& config,
                                const LowStateSample& low_state,
                                std::uint8_t expected_mode_machine);

}  // namespace agentic_et1_tracker
