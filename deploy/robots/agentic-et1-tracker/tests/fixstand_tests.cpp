#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "agentic_et1_tracker/control/fixstand.hpp"
#include "agentic_et1_tracker/control/passive.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr std::uint8_t kExpectedModeMachine = 7;

std::filesystem::path repoFile(const std::string& relative) {
  return (std::filesystem::path(__FILE__).parent_path().parent_path() / relative)
      .lexically_normal();
}

struct TempYaml {
  explicit TempYaml(const std::string& text) {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_fixstand_tests_" + std::to_string(suffix) + ".yaml");
    std::ofstream out(path);
    REQUIRE(out);
    out << text;
  }

  ~TempYaml() {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }

  std::filesystem::path path;
};

std::string repeated(const std::string& value, std::size_t count) {
  std::string out = "[";
  for (std::size_t i = 0; i < count; ++i) {
    if (i != 0) {
      out += ", ";
    }
    out += value;
  }
  out += "]";
  return out;
}

FixStandConfig minimalConfig() {
  FixStandConfig config;
  config.kp = std::vector<double>(kFixStandMotorCount, 20.0);
  config.kd = std::vector<double>(kFixStandMotorCount, 1.0);
  config.target_q.reserve(kFixStandMotorCount);
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    config.target_q.push_back(1.0 + static_cast<double>(i));
  }
  config.duration_s = 3.0;
  return config;
}

LowStateSample lowState() {
  LowStateSample low;
  low.fresh = true;
  low.age_ms = 4;
  low.mode_machine = kExpectedModeMachine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    low.motors.at(i).q = -1.0F - static_cast<float>(i);
  }
  return low;
}

}  // namespace

TEST_CASE("FixStandConfig parses app-owned finite 33-motor profile") {
  const FixStandConfig config = loadFixStandConfig(
      repoFile("config/posture/fixstand/v0/fixstand.yaml"));

  REQUIRE(config.kp.size() == kFixStandMotorCount);
  REQUIRE(config.kd.size() == kFixStandMotorCount);
  REQUIRE(config.target_q.size() == kFixStandMotorCount);
  REQUIRE(config.duration_s == 3.0);
  REQUIRE(config.target_q.at(0) == -0.15);
  REQUIRE(config.target_q.at(32) == 0.0);
}

TEST_CASE("FixStandConfig validates finite values and 33-length target profile") {
  SECTION("short target") {
    TempYaml yaml("kp: " + repeated("1.0", kFixStandMotorCount) + "\n" +
                  "kd: " + repeated("0.1", kFixStandMotorCount) + "\n" +
                  "ts: [0, 3]\n" +
                  "qs: [[], " + repeated("0.0", kFixStandMotorCount - 1) + "]\n");

    REQUIRE_THROWS_WITH(loadFixStandConfig(yaml.path), ContainsSubstring("qs[1]"));
  }

  SECTION("non-finite gain") {
    TempYaml yaml("kp: [.nan, " + repeated("1.0", kFixStandMotorCount - 1).substr(1) +
                  "\n" +
                  "kd: " + repeated("0.1", kFixStandMotorCount) + "\n" +
                  "ts: [0, 3]\n" +
                  "qs: [[], " + repeated("0.0", kFixStandMotorCount) + "]\n");

    REQUIRE_THROWS_WITH(loadFixStandConfig(yaml.path), ContainsSubstring("kp[0]"));
  }
}

TEST_CASE("FixStandRunner starts from current q, interpolates, then holds target") {
  const FixStandConfig config = minimalConfig();
  const LowStateSample low = lowState();
  FixStandRunner runner(config, kExpectedModeMachine, 50.0);

  const LowCmdFrame first = runner.step(low);
  REQUIRE(first.mode_machine == kExpectedModeMachine);
  REQUIRE(first.mode_pr == 0);
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    REQUIRE(first.motors.at(i).mode == 1);
    REQUIRE(first.motors.at(i).q == low.motors.at(i).q);
    REQUIRE(first.motors.at(i).kp == static_cast<float>(config.kp.at(i)));
    REQUIRE(first.motors.at(i).kd == static_cast<float>(config.kd.at(i)));
  }

  LowCmdFrame mid;
  for (int i = 0; i < 75; ++i) {
    mid = runner.step(low);
  }
  REQUIRE(mid.motors.at(0).q > low.motors.at(0).q);
  REQUIRE(mid.motors.at(0).q < static_cast<float>(config.target_q.at(0)));

  LowCmdFrame held;
  for (int i = 0; i < 100; ++i) {
    held = runner.step(low);
  }
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    REQUIRE(held.motors.at(i).q == static_cast<float>(config.target_q.at(i)));
  }
  REQUIRE(held.motors.at(33).kp == 0.0F);
  REQUIRE(held.motors.at(34).kp == 0.0F);
}

TEST_CASE("FixStandRunner starts from base LowCmd q and preserves unmapped motors") {
  const FixStandConfig config = minimalConfig();
  const LowStateSample low = lowState();
  LowCmdFrame base;
  for (std::size_t i = 0; i < base.motors.size(); ++i) {
    MotorCommand& motor = base.motors.at(i);
    motor.mode = static_cast<std::uint8_t>(20 + i);
    motor.q = 10.0F + static_cast<float>(i);
    motor.dq = 20.0F + static_cast<float>(i);
    motor.kp = 30.0F + static_cast<float>(i);
    motor.kd = 40.0F + static_cast<float>(i);
    motor.tau = 50.0F + static_cast<float>(i);
  }
  FixStandRunner runner(config, kExpectedModeMachine, 50.0);

  const LowCmdFrame first = runner.step(low, &base);

  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    REQUIRE(first.motors.at(i).q == base.motors.at(i).q);
    REQUIRE(first.motors.at(i).kp == static_cast<float>(config.kp.at(i)));
    REQUIRE(first.motors.at(i).kd == static_cast<float>(config.kd.at(i)));
  }
  REQUIRE(first.motors.at(33).mode == base.motors.at(33).mode);
  REQUIRE(first.motors.at(33).q == base.motors.at(33).q);
  REQUIRE(first.motors.at(33).kp == base.motors.at(33).kp);
  REQUIRE(first.motors.at(33).kd == base.motors.at(33).kd);
}

TEST_CASE("Passive LowCmd explicitly disables SDK slots without passive config entries") {
  PassiveConfig config;
  config.mode = std::vector<int>(kFixStandMotorCount, 1);
  config.kd = std::vector<double>(kFixStandMotorCount, 2.0);
  const LowStateSample low = lowState();

  const LowCmdFrame frame = makePassiveLowCmdFrame(config, low, kExpectedModeMachine);

  REQUIRE(frame.motors.at(32).mode == 1);
  REQUIRE(frame.motors.at(32).kd == 2.0F);
  for (const std::size_t sdk_slot : {33UL, 34UL}) {
    REQUIRE(frame.motors.at(sdk_slot).mode == 0);
    REQUIRE(frame.motors.at(sdk_slot).q == low.motors.at(sdk_slot).q);
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kd == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

}  // namespace agentic_et1_tracker
