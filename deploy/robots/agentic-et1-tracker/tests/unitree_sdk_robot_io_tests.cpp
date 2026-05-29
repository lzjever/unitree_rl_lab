#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "agentic_et1_tracker/robot/unitree_sdk_robot_io.hpp"
#include "unitree/dds_wrapper/common/crc.h"

namespace agentic_et1_tracker {
namespace {

LowCmdFrame lowCmdFrame() {
  LowCmdFrame frame;
  frame.mode_machine = 1;
  frame.mode_pr = 2;

  for (std::size_t i = 0; i < frame.motors.size(); ++i) {
    MotorCommand& motor = frame.motors[i];
    motor.mode = static_cast<std::uint8_t>(i % 4);
    motor.q = static_cast<float>(10.0 + i);
    motor.dq = static_cast<float>(20.0 + i);
    motor.kp = static_cast<float>(30.0 + i);
    motor.kd = static_cast<float>(40.0 + i);
    motor.tau = static_cast<float>(50.0 + i);
  }

  return frame;
}

}  // namespace

static_assert(std::is_nothrow_destructible_v<UnitreeSdkRobotIO>,
              "UnitreeSdkRobotIO teardown must not throw while closing SDK2 channels");

TEST_CASE("UnitreeSdkRobotIOConfig defaults to DDS domain 0") {
  const UnitreeSdkRobotIOConfig config;
  REQUIRE(config.domain_id == 0);
  REQUIRE(config.lowcmd_startup_preflight_ms == 200);
}

TEST_CASE("UnitreeSdkRobotIO maps SDK2 LowState into RobotIO low state samples") {
  unitree_hg::msg::dds_::LowState_ sdk;
  sdk.mode_machine(1);
  sdk.mode_pr(3);
  sdk.imu_state().quaternion({0.1F, 0.2F, 0.3F, 0.4F});
  sdk.imu_state().gyroscope({1.1F, 1.2F, 1.3F});

  for (std::size_t i = 0; i < sdk.motor_state().size(); ++i) {
    auto& motor = sdk.motor_state()[i];
    motor.mode(static_cast<std::uint8_t>(i % 7));
    motor.q(static_cast<float>(i + 0.25F));
    motor.dq(static_cast<float>(i + 0.5F));
    motor.tau_est(static_cast<float>(i + 0.75F));
  }

  const LowStateSample fresh = unitreeLowStateToSample(sdk, 199, 200);
  REQUIRE(fresh.fresh);
  REQUIRE(fresh.age_ms == 199);
  REQUIRE(fresh.mode_machine == 1);
  REQUIRE(fresh.mode_pr == 3);
  REQUIRE(fresh.quat_wxyz == std::array<float, 4>{{0.1F, 0.2F, 0.3F, 0.4F}});
  REQUIRE(fresh.gyro == std::array<float, 3>{{1.1F, 1.2F, 1.3F}});

  for (std::size_t i = 0; i < fresh.motors.size(); ++i) {
    REQUIRE(fresh.motors[i].mode == static_cast<std::uint8_t>(i % 7));
    REQUIRE(fresh.motors[i].q == static_cast<float>(i + 0.25F));
    REQUIRE(fresh.motors[i].dq == static_cast<float>(i + 0.5F));
    REQUIRE(fresh.motors[i].tau_est == static_cast<float>(i + 0.75F));
  }

  const LowStateSample stale = unitreeLowStateToSample(sdk, 201, 200);
  REQUIRE_FALSE(stale.fresh);
  REQUIRE(stale.age_ms == 201);
}

TEST_CASE("UnitreeSdkRobotIO maps SDK2 SportModeState into RobotIO high state samples") {
  unitree_go::msg::dds_::SportModeState_ sdk;
  sdk.position({1.0F, 2.0F, 3.0F});
  sdk.imu_state().quaternion({0.5F, 0.6F, 0.7F, 0.8F});
  sdk.velocity({4.0F, 5.0F, 6.0F});
  sdk.yaw_speed(7.0F);
  sdk.imu_state().gyroscope({8.0F, 9.0F, 10.0F});

  const HighStateSample fresh = unitreeHighStateToSample(sdk, 50, 50);
  REQUIRE(fresh.fresh);
  REQUIRE(fresh.age_ms == 50);
  REQUIRE(fresh.position == std::array<float, 3>{{1.0F, 2.0F, 3.0F}});
  REQUIRE(fresh.quat_wxyz == std::array<float, 4>{{0.5F, 0.6F, 0.7F, 0.8F}});
  REQUIRE(fresh.linear_velocity == std::array<float, 3>{{4.0F, 5.0F, 6.0F}});
  REQUIRE(fresh.angular_velocity == std::array<float, 3>{{8.0F, 9.0F, 10.0F}});

  const HighStateSample stale = unitreeHighStateToSample(sdk, 51, 50);
  REQUIRE_FALSE(stale.fresh);
  REQUIRE(stale.age_ms == 51);
}

TEST_CASE("UnitreeSdkRobotIO maps RobotIO low command frames into SDK2 LowCmd with CRC") {
  const LowCmdFrame frame = lowCmdFrame();
  unitree_hg::msg::dds_::LowCmd_ cmd = unitreeLowCmdFromFrame(frame);

  REQUIRE(cmd.mode_machine() == 1);
  REQUIRE(cmd.mode_pr() == 2);
  REQUIRE(cmd.reserve() == std::array<std::uint32_t, 4>{{0, 0, 0, 0}});

  for (std::size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
    const auto& motor = cmd.motor_cmd()[i];
    REQUIRE(motor.mode() == static_cast<std::uint8_t>(i % 4));
    REQUIRE(motor.q() == static_cast<float>(10.0 + i));
    REQUIRE(motor.dq() == static_cast<float>(20.0 + i));
    REQUIRE(motor.kp() == static_cast<float>(30.0 + i));
    REQUIRE(motor.kd() == static_cast<float>(40.0 + i));
    REQUIRE(motor.tau() == static_cast<float>(50.0 + i));
    REQUIRE(motor.reserve() == 0);
  }

  const std::uint32_t expected_crc =
      crc32_core(reinterpret_cast<std::uint32_t*>(&cmd), (sizeof(cmd) >> 2) - 1);
  REQUIRE(cmd.crc() == expected_crc);
  REQUIRE(cmd.crc() != 0);
}

TEST_CASE("Unitree LowCmd ownership accepts delayed echo of recent own writes") {
  using Clock = LowCmdOwnershipTracker::Clock;
  using namespace std::chrono_literals;

  LowCmdFrame first_frame = lowCmdFrame();
  LowCmdFrame second_frame = lowCmdFrame();
  second_frame.motors[0].q += 1.0F;
  const auto first = unitreeLowCmdFromFrame(first_frame);
  const auto second = unitreeLowCmdFromFrame(second_frame);
  const auto t0 = Clock::time_point(1000ms);

  LowCmdOwnershipTracker tracker(200, 8);
  tracker.recordOwnWrite(first, t0);
  tracker.observe(first, t0 + 1ms);
  REQUIRE_FALSE(tracker.occupancy(t0 + 1ms).occupied);

  tracker.recordOwnWrite(second, t0 + 2ms);
  tracker.observe(first, t0 + 3ms);

  const LowCmdOccupancy occupancy = tracker.occupancy(t0 + 3ms);
  REQUIRE_FALSE(occupancy.occupied);
  REQUIRE(occupancy.sample_age_ms == 0);
}

TEST_CASE("Unitree LowCmd ownership flags fresh commands outside own-write window") {
  using Clock = LowCmdOwnershipTracker::Clock;
  using namespace std::chrono_literals;

  const auto own = unitreeLowCmdFromFrame(lowCmdFrame());
  LowCmdFrame external_frame = lowCmdFrame();
  external_frame.motors[0].q += 5.0F;
  const auto external = unitreeLowCmdFromFrame(external_frame);
  const auto t0 = Clock::time_point(2000ms);

  LowCmdOwnershipTracker tracker(50, 4);
  tracker.observe(external, t0);
  REQUIRE(tracker.occupancy(t0).occupied);

  tracker.recordOwnWrite(own, t0);
  tracker.observe(own, t0 + 60ms);

  const LowCmdOccupancy occupancy = tracker.occupancy(t0 + 60ms);
  REQUIRE(occupancy.occupied);
  REQUIRE(occupancy.sample_age_ms == 0);
}

TEST_CASE("Unitree LowCmd startup preflight blocks only fresh external owners") {
  using Clock = LowCmdOwnershipTracker::Clock;
  using namespace std::chrono_literals;

  const auto own = unitreeLowCmdFromFrame(lowCmdFrame());
  LowCmdFrame external_frame = lowCmdFrame();
  external_frame.motors[0].q += 3.0F;
  const auto external = unitreeLowCmdFromFrame(external_frame);
  const auto t0 = Clock::time_point(3000ms);

  SECTION("no observed owner passes after the wait window") {
    LowCmdOwnershipTracker tracker(200, 4);
    const LowCmdStartupPreflightResult result =
        checkLowCmdStartupPreflight(tracker, t0 + 200ms);
    REQUIRE(result.ok);
    REQUIRE_FALSE(result.occupancy.occupied);
  }

  SECTION("fresh external owner is blocked") {
    LowCmdOwnershipTracker tracker(200, 4);
    tracker.observe(external, t0 + 50ms);

    const LowCmdStartupPreflightResult result =
        checkLowCmdStartupPreflight(tracker, t0 + 200ms);

    REQUIRE_FALSE(result.ok);
    REQUIRE(result.occupancy.occupied);
    REQUIRE(result.occupancy.sample_age_ms == 150);
  }

  SECTION("recent own echo is not blocked") {
    LowCmdOwnershipTracker tracker(200, 4);
    tracker.recordOwnWrite(own, t0 + 10ms);
    tracker.observe(own, t0 + 20ms);

    const LowCmdStartupPreflightResult result =
        checkLowCmdStartupPreflight(tracker, t0 + 200ms);

    REQUIRE(result.ok);
    REQUIRE_FALSE(result.occupancy.occupied);
    REQUIRE(result.occupancy.sample_age_ms == 180);
  }
}

}  // namespace agentic_et1_tracker
