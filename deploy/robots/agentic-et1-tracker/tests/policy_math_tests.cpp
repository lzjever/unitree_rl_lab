#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/policy_math.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

using Vec = std::vector<float>;

constexpr std::size_t kJointDim = 26;
constexpr std::size_t kHistoryLength = 25;
constexpr std::size_t kCurrentDim = 131;
constexpr std::size_t kHistoryWidth = 105;
constexpr std::size_t kClnCurrentDim = 121;
constexpr std::size_t kClnHistoryWidth = 35;

Vec seq(float start, std::size_t count) {
  Vec values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(start + static_cast<float>(i));
  }
  return values;
}

std::vector<double> doubleSeq(double start, double step, std::size_t count) {
  std::vector<double> values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(start + step * static_cast<double>(i));
  }
  return values;
}

std::vector<ObservationTerm> terms(
    std::initializer_list<std::pair<const char*, std::size_t>> specs) {
  std::vector<ObservationTerm> out;
  std::size_t offset = 0;
  for (const auto& spec : specs) {
    out.push_back({spec.first, spec.second, offset});
    offset += spec.second;
  }
  return out;
}

DeployConfig validConfig() {
  DeployConfig config;
  config.joint_dim = kJointDim;
  config.policy_kp = doubleSeq(10.0, 1.0, kJointDim);
  config.policy_kd = doubleSeq(0.5, 0.05, kJointDim);
  config.action_scale = doubleSeq(0.25, 0.125, kJointDim);
  config.action_offset = doubleSeq(-1.0, 0.2, kJointDim);
  config.obs_current_terms = terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_history_terms = terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_current_dim = kCurrentDim;
  config.obs_history_width = kHistoryWidth;
  config.obs_history_length = kHistoryLength;
  return config;
}

DeployConfig validClnConfig() {
  DeployConfig config = validConfig();
  config.observation_contract = ObservationContract::GeneralTrackerCLN;
  config.obs_current_terms = terms({
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
  });
  config.obs_history_terms = terms({
      {"future_commands", kClnHistoryWidth},
  });
  config.obs_current_dim = kClnCurrentDim;
  config.obs_history_width = kClnHistoryWidth;
  config.obs_history_length = kHistoryLength;
  return config;
}

PolicyObservationParts parts(float base) {
  PolicyObservationParts p;
  p.command_yaw = seq(base - 10.0f, 2);
  p.command_root_ori_b = seq(base + 0.0f, 6);
  p.command_xy_yaw_vel = seq(base + 10.0f, 3);
  p.command_jnt_pos = seq(base + 20.0f, kJointDim);
  p.projected_gravity = seq(base + 60.0f, 3);
  p.base_ang_vel = seq(base + 70.0f, 3);
  p.joint_pos_rel = seq(base + 80.0f, kJointDim);
  p.joint_vel_rel = seq(base + 120.0f, kJointDim);
  p.last_action = seq(base + 200.0f, kJointDim);
  p.command_foot_support_state = seq(base + 300.0f, 6);
  p.ref_com_rel_navi = seq(base + 400.0f, 3);
  p.ref_com_vel_navi = seq(base + 500.0f, 3);
  p.future_commands = seq(base + 600.0f, kHistoryLength * kClnHistoryWidth);
  return p;
}

Vec expectedCurrentRow(const PolicyObservationParts& p) {
  Vec out;
  const auto append = [&out](const Vec& values) {
    out.insert(out.end(), values.begin(), values.end());
  };
  append(p.command_root_ori_b);
  append(p.command_xy_yaw_vel);
  append(p.command_jnt_pos);
  append(p.projected_gravity);
  append(p.base_ang_vel);
  append(p.joint_pos_rel);
  append(p.joint_vel_rel);
  append(p.last_action);
  append(p.command_foot_support_state);
  append(p.ref_com_rel_navi);
  append(p.ref_com_vel_navi);
  return out;
}

Vec expectedHistoryRow(const PolicyObservationParts& p) {
  Vec out;
  const auto append = [&out](const Vec& values) {
    out.insert(out.end(), values.begin(), values.end());
  };
  append(p.command_root_ori_b);
  append(p.command_xy_yaw_vel);
  append(p.command_jnt_pos);
  append(p.projected_gravity);
  append(p.base_ang_vel);
  append(p.joint_pos_rel);
  append(p.joint_vel_rel);
  append(p.command_foot_support_state);
  append(p.ref_com_rel_navi);
  append(p.ref_com_vel_navi);
  return out;
}

Vec expectedClnCurrentRow(const PolicyObservationParts& p) {
  Vec out;
  const auto append = [&out](const Vec& values) {
    out.insert(out.end(), values.begin(), values.end());
  };
  append(p.command_yaw);
  append(p.command_root_ori_b);
  append(p.command_xy_yaw_vel);
  append(p.command_jnt_pos);
  append(p.projected_gravity);
  append(p.base_ang_vel);
  append(p.joint_pos_rel);
  append(p.joint_vel_rel);
  append(p.last_action);
  return out;
}

void requireSliceEquals(const Vec& actual,
                        std::size_t offset,
                        const Vec& expected) {
  REQUIRE(offset + expected.size() <= actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    REQUIRE(actual[offset + i] == expected[i]);
  }
}

void requireRowEquals(const Vec& flat,
                      std::size_t row,
                      const Vec& expected_row) {
  requireSliceEquals(flat, row * kHistoryWidth, expected_row);
}

}  // namespace

TEST_CASE("buildObsCurrent follows DeployConfig current term offsets") {
  const DeployConfig config = validConfig();
  const PolicyObservationParts p = parts(1000.0f);

  const Vec obs = buildObsCurrent(config, p);

  REQUIRE(obs.size() == kCurrentDim);
  requireSliceEquals(obs, 0, p.command_root_ori_b);
  requireSliceEquals(obs, 6, p.command_xy_yaw_vel);
  requireSliceEquals(obs, 9, p.command_jnt_pos);
  requireSliceEquals(obs, 35, p.projected_gravity);
  requireSliceEquals(obs, 38, p.base_ang_vel);
  requireSliceEquals(obs, 41, p.joint_pos_rel);
  requireSliceEquals(obs, 67, p.joint_vel_rel);
  requireSliceEquals(obs, 93, p.last_action);
  requireSliceEquals(obs, 119, p.command_foot_support_state);
  requireSliceEquals(obs, 125, p.ref_com_rel_navi);
  requireSliceEquals(obs, 128, p.ref_com_vel_navi);
  REQUIRE(obs == expectedCurrentRow(p));
}

TEST_CASE("HistoryBuffer flattens history rows oldest to newest without last_action") {
  const DeployConfig config = validConfig();
  const PolicyObservationParts initial = parts(10.0f);
  const PolicyObservationParts newest = parts(2000.0f);

  HistoryBuffer history(config);
  history.reset(initial);
  history.push(newest);

  const Vec flat = history.flatten();

  REQUIRE(flat.size() == kHistoryLength * kHistoryWidth);
  requireRowEquals(flat, 0, expectedHistoryRow(initial));
  requireRowEquals(flat, 23, expectedHistoryRow(initial));
  requireRowEquals(flat, 24, expectedHistoryRow(newest));

  const std::size_t newest_row = 24 * kHistoryWidth;
  requireSliceEquals(flat, newest_row + 0, newest.command_root_ori_b);
  requireSliceEquals(flat, newest_row + 6, newest.command_xy_yaw_vel);
  requireSliceEquals(flat, newest_row + 9, newest.command_jnt_pos);
  requireSliceEquals(flat, newest_row + 35, newest.projected_gravity);
  requireSliceEquals(flat, newest_row + 38, newest.base_ang_vel);
  requireSliceEquals(flat, newest_row + 41, newest.joint_pos_rel);
  requireSliceEquals(flat, newest_row + 67, newest.joint_vel_rel);
  requireSliceEquals(flat, newest_row + 93, newest.command_foot_support_state);
  requireSliceEquals(flat, newest_row + 99, newest.ref_com_rel_navi);
  requireSliceEquals(flat, newest_row + 102, newest.ref_com_vel_navi);

  REQUIRE(std::find(flat.begin(), flat.end(), newest.last_action.front()) == flat.end());
}

TEST_CASE("HistoryBuffer reset fills all rows and push drops oldest rows") {
  const DeployConfig config = validConfig();
  const PolicyObservationParts initial = parts(50.0f);

  HistoryBuffer history(config);
  history.reset(initial);

  Vec flat = history.flatten();
  REQUIRE(flat.size() == kHistoryLength * kHistoryWidth);
  for (std::size_t row = 0; row < kHistoryLength; ++row) {
    requireRowEquals(flat, row, expectedHistoryRow(initial));
  }

  for (std::size_t i = 1; i <= kHistoryLength + 1; ++i) {
    history.push(parts(1000.0f * static_cast<float>(i)));
  }

  flat = history.flatten();
  requireRowEquals(flat, 0, expectedHistoryRow(parts(2000.0f)));
  requireRowEquals(flat, 23, expectedHistoryRow(parts(25000.0f)));
  requireRowEquals(flat, 24, expectedHistoryRow(parts(26000.0f)));
}

TEST_CASE("buildPolicyInputs combines current observation and flattened history") {
  const DeployConfig config = validConfig();
  const PolicyObservationParts current = parts(700.0f);
  const PolicyObservationParts old = parts(800.0f);
  const PolicyObservationParts newest = parts(900.0f);

  HistoryBuffer history(config);
  history.reset(old);
  history.push(newest);

  const PolicyInputs inputs = buildPolicyInputs(config, current, history);

  REQUIRE(inputs.obs_current == expectedCurrentRow(current));
  REQUIRE(inputs.obs_history.size() == kHistoryLength * kHistoryWidth);
  requireRowEquals(inputs.obs_history, 0, expectedHistoryRow(old));
  requireRowEquals(inputs.obs_history, 24, expectedHistoryRow(newest));
}

TEST_CASE("buildPolicyInputs uses CLN future_commands as obs_history") {
  const DeployConfig config = validClnConfig();
  const PolicyObservationParts current = parts(1200.0f);
  HistoryBuffer history(config);

  const PolicyInputs inputs = buildPolicyInputs(config, current, history);

  REQUIRE(inputs.obs_current == expectedClnCurrentRow(current));
  REQUIRE(inputs.obs_history == current.future_commands);
  REQUIRE(inputs.obs_current.size() == kClnCurrentDim);
  REQUIRE(inputs.obs_history.size() == kHistoryLength * kClnHistoryWidth);
}

TEST_CASE("scaleAction applies scale and offset and copies policy gains") {
  const DeployConfig config = validConfig();
  const Vec raw = seq(-2.0f, kJointDim);

  const PolicyOutput output = scaleAction(config, raw);

  REQUIRE(output.raw_action == raw);
  REQUIRE(output.target_q.size() == kJointDim);
  REQUIRE(output.kp.size() == kJointDim);
  REQUIRE(output.kd.size() == kJointDim);
  for (std::size_t i = 0; i < kJointDim; ++i) {
    const float expected_q =
        raw[i] * static_cast<float>(config.action_scale[i]) +
        static_cast<float>(config.action_offset[i]);
    REQUIRE(output.target_q[i] == expected_q);
    REQUIRE(output.kp[i] == static_cast<float>(config.policy_kp[i]));
    REQUIRE(output.kd[i] == static_cast<float>(config.policy_kd[i]));
  }
}

TEST_CASE("policy math validates observation and action widths") {
  SECTION("current observation part width") {
    const DeployConfig config = validConfig();
    PolicyObservationParts p = parts(0.0f);
    p.command_xy_yaw_vel.pop_back();

    REQUIRE_THROWS_WITH(buildObsCurrent(config, p), ContainsSubstring("command_xy_yaw_vel"));
    REQUIRE_THROWS_AS(buildObsCurrent(config, p), PolicyMathError);
  }

  SECTION("history observation part width") {
    const DeployConfig config = validConfig();
    PolicyObservationParts p = parts(0.0f);
    p.command_foot_support_state.pop_back();
    HistoryBuffer history(config);

    REQUIRE_THROWS_WITH(history.reset(p), ContainsSubstring("command_foot_support_state"));
    REQUIRE_THROWS_AS(history.reset(p), PolicyMathError);
  }

  SECTION("raw action length") {
    const DeployConfig config = validConfig();
    const Vec raw = seq(0.0f, kJointDim - 1);

    REQUIRE_THROWS_WITH(scaleAction(config, raw), ContainsSubstring("raw_action"));
    REQUIRE_THROWS_AS(scaleAction(config, raw), PolicyMathError);
  }

  SECTION("DeployConfig action vectors") {
    DeployConfig config = validConfig();
    config.policy_kd.pop_back();
    const Vec raw = seq(0.0f, kJointDim);

    REQUIRE_THROWS_WITH(scaleAction(config, raw), ContainsSubstring("policy_kd"));
    REQUIRE_THROWS_AS(scaleAction(config, raw), PolicyMathError);
  }

  SECTION("CLN future command length") {
    const DeployConfig config = validClnConfig();
    PolicyObservationParts p = parts(0.0f);
    p.future_commands.pop_back();
    HistoryBuffer history(config);

    REQUIRE_THROWS_WITH(buildPolicyInputs(config, p, history),
                        ContainsSubstring("PolicyInputs.obs_history"));
    REQUIRE_THROWS_AS(buildPolicyInputs(config, p, history), PolicyMathError);
  }
}

}  // namespace agentic_et1_tracker
