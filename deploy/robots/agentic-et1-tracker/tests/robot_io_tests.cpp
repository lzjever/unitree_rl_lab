#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/robot/robot_io.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr float kPi = 3.14159265358979323846F;

LowStateSample lowState(std::uint8_t mode_machine, bool fresh = true, std::size_t age_ms = 7) {
  LowStateSample sample;
  sample.fresh = fresh;
  sample.age_ms = age_ms;
  sample.mode_machine = mode_machine;
  return sample;
}

std::array<float, 4> rollQuat(float radians) {
  return {std::cos(radians * 0.5F), std::sin(radians * 0.5F), 0.0F, 0.0F};
}

std::array<float, 4> yawQuat(float radians) {
  return {std::cos(radians * 0.5F), 0.0F, 0.0F, std::sin(radians * 0.5F)};
}

PolicyOutput policyOutput() {
  PolicyOutput output;
  output.target_q.reserve(kPolicyJointCount);
  output.kp.reserve(kPolicyJointCount);
  output.kd.reserve(kPolicyJointCount);
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    output.target_q.push_back(static_cast<float>(i + 1));
    output.kp.push_back(static_cast<float>(10 + i));
    output.kd.push_back(static_cast<float>(100 + i));
  }
  return output;
}

std::vector<int> frozenSdkMap() {
  return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
          13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30};
}

DeployConfig deployConfig() {
  DeployConfig config;
  config.sdk_joint_ids_map = frozenSdkMap();
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    config.policy_kp.push_back(20.0 + static_cast<double>(i));
    config.policy_kd.push_back(0.2 + static_cast<double>(i) * 0.01);
  }
  return config;
}

template <typename Fn>
void requireRobotIOError(Fn&& fn, const std::string& message) {
  try {
    fn();
    FAIL("expected RobotIOError");
  } catch (const RobotIOError& err) {
    REQUIRE_THAT(std::string(err.what()), ContainsSubstring(message));
  } catch (...) {
    FAIL("expected RobotIOError");
  }
}

LowStateSample holdLowState(const DeployConfig& config) {
  LowStateSample sample = lowState(1);
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
    sample.motors.at(sdk_slot).q = 5.0F + static_cast<float>(i);
  }
  return sample;
}

}  // namespace

TEST_CASE("RobotIO mode_machine check handles missing sim and mismatched lowstate") {
  const auto missing = checkModeMachine(std::nullopt, 1);
  REQUIRE_FALSE(missing.connected);
  REQUIRE_FALSE(missing.ok);
  REQUIRE_FALSE(missing.observed.has_value());
  REQUIRE(missing.expected == 1);

  const auto sim = checkModeMachine(lowState(0), 0);
  REQUIRE(sim.connected);
  REQUIRE(sim.ok);
  REQUIRE(sim.observed == 0);

  const auto sim_when_real_expected = checkModeMachine(lowState(0), 1);
  REQUIRE(sim_when_real_expected.connected);
  REQUIRE_FALSE(sim_when_real_expected.ok);
  REQUIRE(sim_when_real_expected.observed == 0);

  const auto mismatch = checkModeMachine(lowState(2), 1);
  REQUIRE(mismatch.connected);
  REQUIRE_FALSE(mismatch.ok);
  REQUIRE(mismatch.observed == 2);
}

TEST_CASE("RobotIO readiness maps lowstate and lowcmd occupancy to stable status blocks") {
  SECTION("no lowstate") {
    const RobotReadinessStatus status = mapRobotReadiness(std::nullopt, {}, 1);
    REQUIRE(status.robot == RobotState::Disconnected);
    REQUIRE(status.err == ErrorCode::RobotDisconnected);
    REQUIRE(status.block == "lowstate_missing");
    REQUIRE_FALSE(status.low_ms.has_value());
  }

  SECTION("stale lowstate") {
    const RobotReadinessStatus status = mapRobotReadiness(lowState(1, false, 123), {}, 1);
    REQUIRE(status.robot == RobotState::NotReady);
    REQUIRE(status.err == ErrorCode::RobotNotReady);
    REQUIRE(status.block == "lowstate_timeout");
    REQUIRE(status.low_ms == 123);
  }

  SECTION("simulator mode 0 passes only when simulator mode is expected") {
    const RobotReadinessStatus status = mapRobotReadiness(lowState(0, true, 8), {}, 0);
    REQUIRE(status.robot == RobotState::Idle);
    REQUIRE(status.err == ErrorCode::Ok);
    REQUIRE(status.block.empty());
    REQUIRE(status.low_ms == 8);
  }

  SECTION("simulator mode 0 mismatches when real mode is expected") {
    const RobotReadinessStatus status = mapRobotReadiness(lowState(0, true, 8), {}, 1);
    REQUIRE(status.robot == RobotState::NotReady);
    REQUIRE(status.err == ErrorCode::RobotNotReady);
    REQUIRE(status.block == "mode_machine_mismatch");
    REQUIRE(status.low_ms == 8);
  }

  SECTION("mode_machine mismatch") {
    const RobotReadinessStatus status = mapRobotReadiness(lowState(2, true, 9), {}, 1);
    REQUIRE(status.robot == RobotState::NotReady);
    REQUIRE(status.err == ErrorCode::RobotNotReady);
    REQUIRE(status.block == "mode_machine_mismatch");
    REQUIRE(status.low_ms == 9);
  }

  SECTION("lowcmd occupied") {
    const RobotReadinessStatus status =
        mapRobotReadiness(lowState(1, true, 10), {true, 3}, 1);
    REQUIRE(status.robot == RobotState::NotReady);
    REQUIRE(status.err == ErrorCode::RobotNotReady);
    REQUIRE(status.block == "lowcmd_occupied");
    REQUIRE(status.low_ms == 10);
  }

  SECTION("bad orientation") {
    LowStateSample low = lowState(1, true, 11);
    low.quat_wxyz = {0.70710677F, 0.70710677F, 0.0F, 0.0F};

    const RobotReadinessStatus status = mapRobotReadiness(low, {}, 1);

    REQUIRE(status.robot == RobotState::Fault);
    REQUIRE(status.err == ErrorCode::RobotBadOrientation);
    REQUIRE(status.block == "bad_orientation");
    REQUIRE(status.low_ms == 11);
  }

  SECTION("good lowstate") {
    const RobotReadinessStatus status = mapRobotReadiness(lowState(1, true, 42), {}, 1);
    REQUIRE(status.robot == RobotState::Idle);
    REQUIRE(status.err == ErrorCode::Ok);
    REQUIRE(status.block.empty());
    REQUIRE(status.low_ms == 42);
  }
}

TEST_CASE("RobotIO body orientation gate rejects tilt beyond one radian") {
  LowStateSample low = lowState(1);

  low.quat_wxyz = rollQuat(57.0F * kPi / 180.0F);
  REQUIRE(hasSafeBodyOrientation(low));

  low.quat_wxyz = rollQuat(58.5F * kPi / 180.0F);
  REQUIRE_FALSE(hasSafeBodyOrientation(low));

  low.quat_wxyz = yawQuat(179.0F * kPi / 180.0F);
  REQUIRE(hasSafeBodyOrientation(low));
}

TEST_CASE("RobotIO body orientation gate rejects invalid and near-zero quaternions") {
  LowStateSample low = lowState(1);

  low.quat_wxyz = {0.0F, 0.0F, 0.0F, 0.0F};
  REQUIRE_FALSE(hasSafeBodyOrientation(low));

  low.quat_wxyz = {1.0e-7F, 0.0F, 0.0F, 0.0F};
  REQUIRE_FALSE(hasSafeBodyOrientation(low));

  low.quat_wxyz = {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F};
  REQUIRE_FALSE(hasSafeBodyOrientation(low));
}

TEST_CASE("RobotIO maps 26 policy joints into the 35 motor SDK LowCmd frame") {
  const LowCmdFrame frame = makeLowCmdFrame(deployConfig(), policyOutput(), 1);

  REQUIRE(frame.mode_machine == 1);
  REQUIRE(frame.mode_pr == 0);

  REQUIRE(frame.motors[0].mode == 1);
  REQUIRE(frame.motors[0].q == 1.0F);
  REQUIRE(frame.motors[0].dq == 0.0F);
  REQUIRE(frame.motors[0].kp == 10.0F);
  REQUIRE(frame.motors[0].kd == 100.0F);
  REQUIRE(frame.motors[0].tau == 0.0F);

  REQUIRE(frame.motors[29].q == 25.0F);
  REQUIRE(frame.motors[29].kp == 34.0F);
  REQUIRE(frame.motors[29].kd == 124.0F);

  REQUIRE(frame.motors[30].q == 26.0F);
  REQUIRE(frame.motors[30].kp == 35.0F);
  REQUIRE(frame.motors[30].kd == 125.0F);

  REQUIRE(frame.motors[14].mode == 1);
  REQUIRE(frame.motors[14].q == 0.0F);
  REQUIRE(frame.motors[14].dq == 0.0F);
  REQUIRE(frame.motors[14].kp == 0.0F);
  REQUIRE(frame.motors[14].kd == 0.0F);
  REQUIRE(frame.motors[14].tau == 0.0F);
}

TEST_CASE("RobotIO builds hold-current LowCmd from current policy joint q") {
  const DeployConfig config = deployConfig();
  const LowStateSample low_state = holdLowState(config);

  const LowCmdFrame frame = makeHoldLowCmdFrame(config, low_state, 1);

  REQUIRE(frame.mode_machine == 1);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q == low_state.motors.at(sdk_slot).q);
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp == static_cast<float>(config.policy_kp.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).kd == static_cast<float>(config.policy_kd.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }

  REQUIRE(frame.motors[14].q == 0.0F);
  REQUIRE(frame.motors[14].kp == 0.0F);
  REQUIRE(frame.motors[14].kd == 0.0F);
}

TEST_CASE("RobotIO LowCmd mapping rejects non-finite policy output fields") {
  SECTION("target_q NaN") {
    PolicyOutput output = policyOutput();
    output.target_q[0] = std::numeric_limits<float>::quiet_NaN();

    requireRobotIOError([&] { (void)makeLowCmdFrame(deployConfig(), output, 1); },
                        "PolicyOutput.target_q");
  }

  SECTION("target_q Inf") {
    PolicyOutput output = policyOutput();
    output.target_q[0] = std::numeric_limits<float>::infinity();

    requireRobotIOError([&] { (void)makeLowCmdFrame(deployConfig(), output, 1); },
                        "PolicyOutput.target_q");
  }

  SECTION("kp NaN") {
    PolicyOutput output = policyOutput();
    output.kp[0] = std::numeric_limits<float>::quiet_NaN();

    requireRobotIOError([&] { (void)makeLowCmdFrame(deployConfig(), output, 1); },
                        "PolicyOutput.kp");
  }

  SECTION("kp Inf") {
    PolicyOutput output = policyOutput();
    output.kp[0] = std::numeric_limits<float>::infinity();

    requireRobotIOError([&] { (void)makeLowCmdFrame(deployConfig(), output, 1); },
                        "PolicyOutput.kp");
  }

  SECTION("kd NaN") {
    PolicyOutput output = policyOutput();
    output.kd[0] = std::numeric_limits<float>::quiet_NaN();

    requireRobotIOError([&] { (void)makeLowCmdFrame(deployConfig(), output, 1); },
                        "PolicyOutput.kd");
  }

  SECTION("kd Inf") {
    PolicyOutput output = policyOutput();
    output.kd[0] = std::numeric_limits<float>::infinity();

    requireRobotIOError([&] { (void)makeLowCmdFrame(deployConfig(), output, 1); },
                        "PolicyOutput.kd");
  }
}

TEST_CASE("RobotIO hold-current LowCmd rejects non-finite q and gains") {
  SECTION("low_state q NaN") {
    const DeployConfig config = deployConfig();
    LowStateSample low_state = holdLowState(config);
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(0));
    low_state.motors.at(sdk_slot).q = std::numeric_limits<float>::quiet_NaN();

    requireRobotIOError([&] { (void)makeHoldLowCmdFrame(config, low_state, 1); },
                        "PolicyOutput.target_q");
  }

  SECTION("low_state q Inf") {
    const DeployConfig config = deployConfig();
    LowStateSample low_state = holdLowState(config);
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(0));
    low_state.motors.at(sdk_slot).q = std::numeric_limits<float>::infinity();

    requireRobotIOError([&] { (void)makeHoldLowCmdFrame(config, low_state, 1); },
                        "PolicyOutput.target_q");
  }

  SECTION("policy_kp NaN") {
    DeployConfig config = deployConfig();
    const LowStateSample low_state = holdLowState(config);
    config.policy_kp[0] = std::numeric_limits<double>::quiet_NaN();

    requireRobotIOError([&] { (void)makeHoldLowCmdFrame(config, low_state, 1); },
                        "PolicyOutput.kp");
  }

  SECTION("policy_kp Inf") {
    DeployConfig config = deployConfig();
    const LowStateSample low_state = holdLowState(config);
    config.policy_kp[0] = std::numeric_limits<double>::infinity();

    requireRobotIOError([&] { (void)makeHoldLowCmdFrame(config, low_state, 1); },
                        "PolicyOutput.kp");
  }

  SECTION("policy_kd NaN") {
    DeployConfig config = deployConfig();
    const LowStateSample low_state = holdLowState(config);
    config.policy_kd[0] = std::numeric_limits<double>::quiet_NaN();

    requireRobotIOError([&] { (void)makeHoldLowCmdFrame(config, low_state, 1); },
                        "PolicyOutput.kd");
  }

  SECTION("policy_kd Inf") {
    DeployConfig config = deployConfig();
    const LowStateSample low_state = holdLowState(config);
    config.policy_kd[0] = std::numeric_limits<double>::infinity();

    requireRobotIOError([&] { (void)makeHoldLowCmdFrame(config, low_state, 1); },
                        "PolicyOutput.kd");
  }
}

TEST_CASE("RobotIO LowCmd mapping rejects incompatible policy output and SDK maps") {
  SECTION("policy output length mismatch") {
    PolicyOutput output = policyOutput();
    output.target_q.pop_back();

    REQUIRE_THROWS_WITH(makeLowCmdFrame(deployConfig(), output, 1),
                        ContainsSubstring("PolicyOutput.target_q"));
  }

  SECTION("sdk slot out of range") {
    std::vector<int> sdk_map = frozenSdkMap();
    sdk_map[1] = static_cast<int>(kSdkMotorCount);

    REQUIRE_THROWS_WITH(makeLowCmdFrame(sdk_map, policyOutput(), 1),
                        ContainsSubstring("sdk_joint_ids_map[1]"));
  }
}

}  // namespace agentic_et1_tracker
