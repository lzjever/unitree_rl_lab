#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "agentic_et1_tracker/loco_upper/lowcmd_composer.hpp"
#include "agentic_et1_tracker/loco_upper/validator.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

std::vector<int> frozenSdkMap() {
  return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
          13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30};
}

std::filesystem::path repoFile(const std::string& relative) {
  return (std::filesystem::path(__FILE__).parent_path().parent_path() / relative)
      .lexically_normal();
}

DeployConfig deployConfig() {
  DeployConfig config;
  config.sdk_joint_ids_map = frozenSdkMap();
  config.policy_kp.assign(kPolicyJointCount, 0.0);
  config.policy_kd.assign(kPolicyJointCount, 0.0);
  for (std::size_t logical = 0; logical < kPolicyJointCount; ++logical) {
    config.policy_kp.at(logical) = 20.0 + static_cast<double>(logical);
    config.policy_kd.at(logical) = 0.2 + 0.01 * static_cast<double>(logical);
  }
  return config;
}

LocoUpperLowCmdComposerConfig config() {
  LocoUpperLowCmdComposerConfig cfg;
  cfg.logical_to_sdk = frozenSdkMap();
  cfg.upper_kp.assign(kPolicyJointCount, 0.0F);
  cfg.upper_kd.assign(kPolicyJointCount, 0.0F);
  cfg.lower_min_q.assign(12, -1000.0F);
  cfg.lower_max_q.assign(12, 1000.0F);
  cfg.upper_min_q.assign(kPolicyJointCount, -10.0F);
  cfg.upper_max_q.assign(kPolicyJointCount, 10.0F);
  cfg.upper_max_vel_radps.assign(kPolicyJointCount, 0.0F);
  cfg.upper_max_accel_radps2.assign(kPolicyJointCount, 0.0F);
  for (std::size_t logical = cfg.upper_start_joint;
       logical < cfg.upper_end_joint_exclusive;
       ++logical) {
    cfg.upper_kp.at(logical) = 50.0F + static_cast<float>(logical);
    cfg.upper_kd.at(logical) = 1.0F + 0.1F * static_cast<float>(logical);
    cfg.upper_min_q.at(logical) = -2.0F;
    cfg.upper_max_q.at(logical) = 2.0F;
    cfg.upper_max_vel_radps.at(logical) = 4.0F;
    cfg.upper_max_accel_radps2.at(logical) = 20.0F;
  }
  cfg.unowned_safe.q = -0.25F;
  cfg.unowned_safe.kp = 3.0F;
  cfg.unowned_safe.kd = 0.4F;
  cfg.unowned_safe.mode = 7;
  cfg.expected_mode_machine = 5;
  return cfg;
}

LocoLowerDeployConfig lowerDeployConfig() {
  LocoLowerDeployConfig config;
  config.joint_ids_map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  config.sdk_joint_ids_map = config.joint_ids_map;
  config.stiffness.assign(12, 20.0);
  config.damping.assign(12, 1.0);
  config.default_joint_pos.assign(12, 0.0);
  config.joint_min_q.assign(12, -1.0);
  config.joint_max_q.assign(12, 1.0);
  config.action_scale.assign(12, 0.25);
  config.action_offset.assign(12, 0.0);
  config.observation_terms = {
      {"base_ang_vel", 3, 0, {1.0, 1.0, 1.0}},
      {"projected_gravity", 3, 3, {1.0, 1.0, 1.0}},
      {"keyboard_velocity_commands", 3, 6, {1.0, 1.0, 1.0}},
      {"joint_pos_rel", 12, 9, std::vector<double>(12, 1.0)},
      {"joint_vel_rel", 12, 21, std::vector<double>(12, 1.0)},
      {"last_action", 12, 33, std::vector<double>(12, 1.0)},
  };
  config.command_ranges.lin_vel_x = {-1.0, 1.0};
  config.command_ranges.lin_vel_y = {-1.0, 1.0};
  config.command_ranges.ang_vel_z = {-1.0, 1.0};
  config.policy_decimation = 1;
  config.obs_row_width = kLocoLowerPolicyObsRowWidth;
  config.obs_history_length = kLocoLowerPolicyHistoryLength;
  config.obs_dim = kLocoLowerPolicyObsDim;
  config.step_dt = 0.02;
  return config;
}

LowCmdFrame lowerFrame() {
  LowCmdFrame frame;
  frame.mode_machine = 9;
  frame.mode_pr = 8;
  for (std::size_t slot = 0; slot < frame.motors.size(); ++slot) {
    MotorCommand& motor = frame.motors.at(slot);
    motor.mode = static_cast<std::uint8_t>(20 + slot);
    motor.q = 100.0F + static_cast<float>(slot);
    motor.dq = 200.0F + static_cast<float>(slot);
    motor.kp = 300.0F + static_cast<float>(slot);
    motor.kd = 400.0F + static_cast<float>(slot);
    motor.tau = 500.0F + static_cast<float>(slot);
  }
  return frame;
}

std::vector<float> upperTargets() {
  std::vector<float> targets(kPolicyJointCount, 0.0F);
  for (std::size_t logical = 12; logical < 26; ++logical) {
    targets.at(logical) = -1.0F + 0.05F * static_cast<float>(logical);
  }
  return targets;
}

std::vector<float> fixstandLowerFixtureTargets() {
  return {
      -0.15F, 0.0F, 0.0F, 0.325F, -0.15F, 0.0F,
      -0.15F, 0.0F, 0.0F, 0.325F, -0.15F, 0.0F,
  };
}

template <typename Fn>
void requireComposerError(Fn&& fn, const std::string& message) {
  try {
    fn();
    FAIL("expected LocoUpperLowCmdComposerError");
  } catch (const LocoUpperLowCmdComposerError& err) {
    REQUIRE_THAT(std::string(err.what()), ContainsSubstring(message));
  } catch (...) {
    FAIL("expected LocoUpperLowCmdComposerError");
  }
}

struct TempTree {
  TempTree() {
    const auto suffix = std::to_string(
        std::filesystem::file_time_type::clock::now().time_since_epoch().count());
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_loco_upper_loader_tests_" + suffix);
    std::filesystem::create_directories(root);
  }

  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path root;
};

void writeText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out << text;
}

}  // namespace

TEST_CASE("loco upper LowCmd composer preserves lower joints and overlays upper joints") {
  const LocoUpperLowCmdComposerConfig cfg = config();
  const LowCmdFrame lower = lowerFrame();
  const std::vector<float> targets = upperTargets();

  const LocoUpperLowCmdComposeResult result =
      composeLocoUpperLowCmd(cfg, lower, targets);

  REQUIRE_FALSE(result.lower_q_limited);
  REQUIRE_FALSE(result.lower_action_clamped);
  REQUIRE(result.frame.mode_machine == cfg.expected_mode_machine);
  REQUIRE(result.frame.mode_pr == 0);

  for (std::size_t logical = 0; logical < 12; ++logical) {
    const std::size_t slot = static_cast<std::size_t>(cfg.logical_to_sdk.at(logical));
    REQUIRE(result.frame.motors.at(slot).mode == lower.motors.at(slot).mode);
    REQUIRE(result.frame.motors.at(slot).q == lower.motors.at(slot).q);
    REQUIRE(result.frame.motors.at(slot).dq == lower.motors.at(slot).dq);
    REQUIRE(result.frame.motors.at(slot).kp == lower.motors.at(slot).kp);
    REQUIRE(result.frame.motors.at(slot).kd == lower.motors.at(slot).kd);
    REQUIRE(result.frame.motors.at(slot).tau == lower.motors.at(slot).tau);
  }

  for (std::size_t logical = 12; logical < 26; ++logical) {
    const std::size_t slot = static_cast<std::size_t>(cfg.logical_to_sdk.at(logical));
    REQUIRE(result.frame.motors.at(slot).mode == 1);
    REQUIRE(result.frame.motors.at(slot).q == targets.at(logical));
    REQUIRE(result.frame.motors.at(slot).dq == 0.0F);
    REQUIRE(result.frame.motors.at(slot).kp == cfg.upper_kp.at(logical));
    REQUIRE(result.frame.motors.at(slot).kd == cfg.upper_kd.at(logical));
    REQUIRE(result.frame.motors.at(slot).tau == 0.0F);
  }

  for (const std::size_t slot : {14U, 20U, 21U, 27U, 28U, 31U, 32U, 33U, 34U}) {
    REQUIRE(result.frame.motors.at(slot).mode == cfg.unowned_safe.mode);
    REQUIRE(result.frame.motors.at(slot).q == cfg.unowned_safe.q);
    REQUIRE(result.frame.motors.at(slot).dq == 0.0F);
    REQUIRE(result.frame.motors.at(slot).kp == cfg.unowned_safe.kp);
    REQUIRE(result.frame.motors.at(slot).kd == cfg.unowned_safe.kd);
    REQUIRE(result.frame.motors.at(slot).tau == 0.0F);
  }
}

TEST_CASE("loco upper LowCmd composer rejects invalid upper targets") {
  SECTION("NaN target") {
    LocoUpperLowCmdComposerConfig cfg = config();
    std::vector<float> targets = upperTargets();
    targets.at(12) = std::numeric_limits<float>::quiet_NaN();

    requireComposerError([&] { (void)composeLocoUpperLowCmd(cfg, lowerFrame(), targets); },
                         "upper target");
  }

  SECTION("target outside configured limits") {
    LocoUpperLowCmdComposerConfig cfg = config();
    std::vector<float> targets = upperTargets();
    targets.at(13) = 2.1F;

    requireComposerError([&] { (void)composeLocoUpperLowCmd(cfg, lowerFrame(), targets); },
                         "upper target[13]");
  }
}

TEST_CASE("loco upper LowCmd composer rejects invalid maps and lower commands") {
  SECTION("wrong map size") {
    LocoUpperLowCmdComposerConfig cfg = config();
    cfg.logical_to_sdk.pop_back();

    requireComposerError([&] { (void)composeLocoUpperLowCmd(cfg, lowerFrame(), upperTargets()); },
                         "logical_to_sdk");
  }

  SECTION("SDK slot out of range") {
    LocoUpperLowCmdComposerConfig cfg = config();
    cfg.logical_to_sdk.at(3) = static_cast<int>(kSdkMotorCount);

    requireComposerError([&] { (void)composeLocoUpperLowCmd(cfg, lowerFrame(), upperTargets()); },
                         "logical_to_sdk[3]");
  }

  SECTION("duplicate SDK slot") {
    LocoUpperLowCmdComposerConfig cfg = config();
    cfg.logical_to_sdk.at(13) = cfg.logical_to_sdk.at(12);

    requireComposerError([&] { (void)composeLocoUpperLowCmd(cfg, lowerFrame(), upperTargets()); },
                         "duplicate");
  }

  SECTION("non-finite lower command") {
    LocoUpperLowCmdComposerConfig cfg = config();
    LowCmdFrame lower = lowerFrame();
    lower.motors.at(static_cast<std::size_t>(cfg.logical_to_sdk.at(4))).q =
        std::numeric_limits<float>::infinity();

    requireComposerError([&] { (void)composeLocoUpperLowCmd(cfg, lower, upperTargets()); },
                         "lower motor");
  }
}

TEST_CASE("loco upper LowCmd composer accepts TRK frame view upper target source") {
  const LocoUpperLowCmdComposerConfig cfg = config();
  const std::vector<float> targets = upperTargets();
  TrkFrameView frame;
  frame.joint_pos.ptr = targets.data();
  frame.joint_pos.size = targets.size();

  const LocoUpperLowCmdComposeResult result =
      composeLocoUpperLowCmd(cfg, lowerFrame(), frame);

  const std::size_t sdk_slot = static_cast<std::size_t>(cfg.logical_to_sdk.at(25));
  REQUIRE(result.frame.motors.at(sdk_slot).q == targets.at(25));
}

TEST_CASE("loco upper LowCmd composer loader builds app-owned composer config") {
  TempTree tmp;
  const auto limits = tmp.root / "limits.yaml";
  const auto joint_map = tmp.root / "joint_map.yaml";
  writeText(limits,
            "upper_min_q: [-1.2, -1.3, -1.4, -1.5, -1.6, -1.7, -1.8, "
            "-1.9, -2.0, -2.1, -2.2, -2.3, -2.4, -2.5]\n"
            "upper_max_q: [1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, "
            "1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5]\n"
            "max_vel_radps: [2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, "
            "2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3]\n"
            "max_accel_radps2: [10, 11, 12, 13, 14, 15, 16, "
            "17, 18, 19, 20, 21, 22, 23]\n");
  writeText(joint_map,
            "logical_to_sdk: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, "
            "17, 18, 19, 22, 23, 24, 25, 26, 29, 30]\n");

  const LocoUpperLowCmdComposerConfig cfg =
      loadLocoUpperLowCmdComposerConfig(limits, joint_map, deployConfig(), lowerDeployConfig());

  REQUIRE(cfg.logical_to_sdk == frozenSdkMap());
  REQUIRE(cfg.lower_min_q.at(0) == -1.0F);
  REQUIRE(cfg.lower_max_q.at(11) == 1.0F);
  REQUIRE(cfg.upper_kp.at(12) == 32.0F);
  REQUIRE(cfg.upper_kd.at(25) == 0.45F);
  REQUIRE(cfg.upper_min_q.at(12) == -1.2F);
  REQUIRE(cfg.upper_max_q.at(25) == 2.5F);
  REQUIRE(cfg.upper_max_vel_radps.at(12) == 2.0F);
  REQUIRE(cfg.upper_max_accel_radps2.at(25) == 23.0F);
  REQUIRE(cfg.unowned_safe.mode == 1);
  REQUIRE(cfg.unowned_safe.q == 0.0F);
  REQUIRE(cfg.unowned_safe.kp == 0.0F);
  REQUIRE(cfg.unowned_safe.kd == 0.0F);

  std::vector<float> targets = upperTargets();
  targets.at(12) = -1.2F;
  targets.at(25) = 2.5F;
  const auto composed = composeLocoUpperLowCmd(cfg, lowerFrame(), targets);
  REQUIRE(composed.frame.motors.at(static_cast<std::size_t>(cfg.logical_to_sdk.at(12))).q ==
          -1.2F);

  targets.at(25) = 2.6F;
  requireComposerError(
      [&] { (void)composeLocoUpperLowCmd(cfg, lowerFrame(), targets); },
      "upper target[25]");
}

TEST_CASE("loco upper LowCmd composer loader rejects lower slot mapping mismatch") {
  LocoLowerDeployConfig lower = lowerDeployConfig();
  lower.joint_ids_map = {6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5};

  REQUIRE_THROWS_WITH(
      loadLocoUpperLowCmdComposerConfig(
          repoFile("config/limits/et1_upper_body/v0/limits.yaml"),
          repoFile("config/limits/et1_upper_body/v0/joint_map.yaml"),
          deployConfig(),
          lower),
      ContainsSubstring("lower SDK slot map mismatch"));
}

TEST_CASE("loco upper LowCmd composer clamps lower joints to configured q limits") {
  LocoUpperLowCmdComposerConfig cfg = config();
  cfg.lower_min_q.assign(12, -0.2F);
  cfg.lower_max_q.assign(12, 0.2F);
  LowCmdFrame lower = lowerFrame();
  lower.motors.at(static_cast<std::size_t>(cfg.logical_to_sdk.at(0))).q = 0.35F;
  lower.motors.at(static_cast<std::size_t>(cfg.logical_to_sdk.at(1))).q = -0.4F;

  const LocoUpperLowCmdComposeResult result =
      composeLocoUpperLowCmd(cfg, lower, upperTargets());

  REQUIRE(result.lower_q_limited);
  REQUIRE(result.lower_action_clamped);
  REQUIRE(result.frame.motors.at(static_cast<std::size_t>(cfg.logical_to_sdk.at(0))).q == 0.2F);
  REQUIRE(result.frame.motors.at(static_cast<std::size_t>(cfg.logical_to_sdk.at(1))).q == -0.2F);
}

TEST_CASE("real loco lower zero action from fixstand-compatible state stays inside hard q limits") {
  const LocoLowerDeployConfig lower_deploy =
      loadLocoLowerDeployConfig(repoFile("config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"));
  const LocoUpperLowCmdComposerConfig composer =
      loadLocoUpperLowCmdComposerConfig(
          repoFile("config/limits/et1_upper_body/v0/limits.yaml"),
          repoFile("config/limits/et1_upper_body/v0/joint_map.yaml"),
          deployConfig(),
          lower_deploy);

  const std::vector<float> fixstand_lower = fixstandLowerFixtureTargets();
  REQUIRE(fixstand_lower.size() == kLocoLowerPolicyJointDim);
  const Vec zero_raw_action(kLocoLowerPolicyJointDim, 0.0F);
  for (std::size_t joint = 0; joint < kLocoLowerPolicyJointDim; ++joint) {
    const float current_q = fixstand_lower.at(joint);
    const float policy_target = static_cast<float>(lower_deploy.action_offset.at(joint));
    const float min_q = static_cast<float>(lower_deploy.joint_min_q.at(joint));
    const float max_q = static_cast<float>(lower_deploy.joint_max_q.at(joint));
    INFO("joint=" << joint << " current_q=" << current_q << " policy_target=" << policy_target
                  << " limits=[" << min_q << ", " << max_q << "]");
    REQUIRE(current_q >= min_q);
    REQUIRE(current_q <= max_q);
    REQUIRE(std::clamp(policy_target, min_q, max_q) == Catch::Approx(policy_target));
  }

  const LowCmdFrame lower_frame =
      makeLocoLowerLowCmdFrame(lower_deploy, zero_raw_action, composer.expected_mode_machine);
  std::vector<float> upper_targets(kPolicyJointCount, 0.0F);
  for (std::size_t logical = composer.upper_start_joint;
       logical < composer.upper_end_joint_exclusive;
       ++logical) {
    upper_targets.at(logical) = std::clamp(0.0F,
                                           composer.upper_min_q.at(logical),
                                           composer.upper_max_q.at(logical));
  }

  const LocoUpperLowCmdComposeResult result =
      composeLocoUpperLowCmd(composer, lower_frame, upper_targets);

  REQUIRE_FALSE(result.lower_q_limited);
  REQUIRE_FALSE(result.lower_action_clamped);
  for (std::size_t joint = 0; joint < kLocoLowerPolicyJointDim; ++joint) {
    const std::size_t sdk_slot = static_cast<std::size_t>(composer.logical_to_sdk.at(joint));
    INFO("joint=" << joint << " sdk_slot=" << sdk_slot
                  << " q=" << result.frame.motors.at(sdk_slot).q);
    REQUIRE(result.frame.motors.at(sdk_slot).q ==
            Catch::Approx(static_cast<float>(lower_deploy.action_offset.at(joint))));
  }
}

TEST_CASE("loco upper joint validation loader requires finite dynamic limits") {
  TempTree tmp;
  const auto limits = tmp.root / "limits.yaml";

  SECTION("missing max_vel_radps") {
    writeText(limits,
              "upper_min_q: [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]\n"
              "upper_max_q: [ 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1]\n"
              "max_accel_radps2: [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]\n");

    REQUIRE_THROWS_AS(loadLocoUpperJointValidationOptions(limits),
                      LocoUpperLowCmdComposerError);
  }

  SECTION("missing max_accel_radps2") {
    writeText(limits,
              "upper_min_q: [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]\n"
              "upper_max_q: [ 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1]\n"
              "max_vel_radps: [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]\n");

    REQUIRE_THROWS_AS(loadLocoUpperJointValidationOptions(limits),
                      LocoUpperLowCmdComposerError);
  }

  SECTION("non-finite max_vel_radps entry") {
    writeText(limits,
              "upper_min_q: [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]\n"
              "upper_max_q: [ 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1]\n"
              "max_vel_radps: [2, 2, 2, 2, .nan, 2, 2, 2, 2, 2, 2, 2, 2, 2]\n"
              "max_accel_radps2: [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]\n");

    REQUIRE_THROWS_AS(loadLocoUpperJointValidationOptions(limits),
                      LocoUpperLowCmdComposerError);
  }

  SECTION("valid file loads 14-entry dynamic vectors") {
    writeText(limits,
              "upper_min_q: [-1.2, -1.3, -1.4, -1.5, -1.6, -1.7, -1.8, "
              "-1.9, -2.0, -2.1, -2.2, -2.3, -2.4, -2.5]\n"
              "upper_max_q: [1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, "
              "1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5]\n"
              "max_vel_radps: [2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, "
              "2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3]\n"
              "max_accel_radps2: [10, 11, 12, 13, 14, 15, 16, "
              "17, 18, 19, 20, 21, 22, 23]\n");

    const LocoUpperJointValidationOptions options =
        loadLocoUpperJointValidationOptions(limits);

    REQUIRE(options.min_positions.size() == kLocoUpperJointCount);
    REQUIRE(options.max_positions.size() == kLocoUpperJointCount);
    REQUIRE(options.max_velocities.size() == kLocoUpperJointCount);
    REQUIRE(options.max_accelerations.size() == kLocoUpperJointCount);
    REQUIRE(options.min_positions.at(0) == Catch::Approx(-1.2));
    REQUIRE(options.max_positions.at(13) == Catch::Approx(2.5));
    REQUIRE(options.max_velocities.at(0) == Catch::Approx(2.0));
    REQUIRE(options.max_accelerations.at(13) == Catch::Approx(23.0));
  }
}

}  // namespace agentic_et1_tracker
