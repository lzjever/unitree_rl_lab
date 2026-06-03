#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/deploy_config.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr const char* kCurrentObs[] = {
    "command_root_ori_b",
    "command_xy_yaw_vel",
    "command_jnt_pos",
    "projected_gravity",
    "base_ang_vel",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
    "command_foot_support_state",
    "ref_com_rel_navi",
    "ref_com_vel_navi",
};

constexpr const char* kHistoryObs[] = {
    "command_root_ori_b",
    "command_xy_yaw_vel",
    "command_jnt_pos",
    "projected_gravity",
    "base_ang_vel",
    "joint_pos_rel",
    "joint_vel_rel",
    "command_foot_support_state",
    "ref_com_rel_navi",
    "ref_com_vel_navi",
};

constexpr const char* kClnCurrentObs[] = {
    "command_yaw",
    "command_root_ori_b",
    "command_xy_yaw_vel",
    "command_jnt_pos",
    "projected_gravity",
    "base_ang_vel",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
};

constexpr const char* kClnHistoryObs[] = {
    "future_commands",
};

struct TempDeploy {
  explicit TempDeploy(const std::string& yaml) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_deploy_config_tests_" + std::to_string(now) + ".yaml");

    std::ofstream out(path);
    REQUIRE(out);
    out << yaml;
  }

  ~TempDeploy() {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }

  DeployConfig load() const { return loadDeployConfig(path); }

  std::filesystem::path path;
};

std::string repeatValues(const std::string& value, int count) {
  std::ostringstream out;
  out << '[';
  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << value;
  }
  out << ']';
  return out.str();
}

std::string intRange(int begin, int end_exclusive) {
  std::ostringstream out;
  out << '[';
  for (int i = begin; i < end_exclusive; ++i) {
    if (i > begin) {
      out << ", ";
    }
    out << i;
  }
  out << ']';
  return out.str();
}

std::string obsBlock(const char* const* names, std::size_t count, int history_length) {
  std::ostringstream out;
  out << "    use_gym_history: true\n";
  for (std::size_t i = 0; i < count; ++i) {
    out << "    " << names[i] << ":\n";
    out << "      history_length: " << history_length << "\n";
  }
  return out.str();
}

std::string validDeployYaml() {
  std::ostringstream out;
  out << "joint_ids_map: " << intRange(0, 26) << "\n";
  out << "sdk_joint_ids_map: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, "
         "13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30]\n";
  out << "step_dt: 0.02\n";
  out << "default_joint_pos: " << repeatValues("0.25", 26) << "\n";
  out << "actions:\n";
  out << "  JointPositionAction:\n";
  out << "    scale: " << repeatValues("0.1", 26) << "\n";
  out << "    offset: " << repeatValues("0.0", 26) << "\n";
  out << "policy_kp: " << repeatValues("1.0", 26) << "\n";
  out << "policy_kd: " << repeatValues("0.1", 26) << "\n";
  out << "observations:\n";
  out << "  obs_current:\n";
  out << obsBlock(kCurrentObs, sizeof(kCurrentObs) / sizeof(kCurrentObs[0]), 1);
  out << "  obs_history:\n";
  out << obsBlock(kHistoryObs, sizeof(kHistoryObs) / sizeof(kHistoryObs[0]), 25);
  return out.str();
}

std::string validClnDeployYaml() {
  std::ostringstream out;
  out << "joint_ids_map: " << intRange(0, 26) << "\n";
  out << "sdk_joint_ids_map: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, "
         "13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30]\n";
  out << "step_dt: 0.02\n";
  out << "default_joint_pos: " << repeatValues("0.25", 26) << "\n";
  out << "actions:\n";
  out << "  JointPositionAction:\n";
  out << "    scale: " << repeatValues("0.1", 26) << "\n";
  out << "    offset: " << repeatValues("0.0", 26) << "\n";
  out << "policy_kp: " << repeatValues("1.0", 26) << "\n";
  out << "policy_kd: " << repeatValues("0.1", 26) << "\n";
  out << "observations:\n";
  out << "  obs_current:\n";
  out << obsBlock(kClnCurrentObs, sizeof(kClnCurrentObs) / sizeof(kClnCurrentObs[0]), 1);
  out << "  obs_history:\n";
  out << obsBlock(kClnHistoryObs, sizeof(kClnHistoryObs) / sizeof(kClnHistoryObs[0]), 1);
  return out.str();
}

std::string replaceOnce(std::string yaml,
                        const std::string& needle,
                        const std::string& replacement) {
  const std::size_t pos = yaml.find(needle);
  REQUIRE(pos != std::string::npos);
  yaml.replace(pos, needle.size(), replacement);
  return yaml;
}

DeployConfig loadYaml(const std::string& yaml) {
  TempDeploy tmp(yaml);
  return tmp.load();
}

void requireTerm(const std::vector<ObservationTerm>& terms,
                 const std::string& name,
                 std::size_t width,
                 std::size_t offset) {
  const auto it = std::find_if(terms.begin(), terms.end(), [&name](const ObservationTerm& term) {
    return term.name == name;
  });
  REQUIRE(it != terms.end());
  REQUIRE(it->width == width);
  REQUIRE(it->offset == offset);
}

}  // namespace

TEST_CASE("DeployConfig loads the frozen GeneralTracker deploy contract") {
  const DeployConfig config = loadYaml(validDeployYaml());

  REQUIRE(config.joint_dim == 26);
  REQUIRE(config.joint_ids_map.size() == 26);
  REQUIRE(config.sdk_joint_ids_map.size() == 26);
  REQUIRE(config.policy_kp.size() == 26);
  REQUIRE(config.policy_kd.size() == 26);
  REQUIRE(config.default_joint_pos.size() == 26);
  REQUIRE(config.action_scale.size() == 26);
  REQUIRE(config.action_offset.size() == 26);
  REQUIRE(config.obs_current.size() == 11);
  REQUIRE(config.obs_history.size() == 10);
  REQUIRE(config.obs_current_terms.size() == 11);
  REQUIRE(config.obs_history_terms.size() == 10);
  REQUIRE(config.obs_current_dim == 131);
  REQUIRE(config.obs_history_width == 105);
  REQUIRE(config.obs_history_length == 25);
  REQUIRE(config.step_dt == 0.02);
  REQUIRE(config.default_joint_pos[0] == 0.25);
  REQUIRE(config.default_joint_pos[25] == 0.25);
  REQUIRE(config.sdk_joint_ids_map[24] == 29);
  REQUIRE(config.sdk_joint_ids_map[25] == 30);
  REQUIRE(config.override_joint_ids.empty());

  requireTerm(config.obs_current_terms, "command_root_ori_b", 6, 0);
  requireTerm(config.obs_current_terms, "command_xy_yaw_vel", 3, 6);
  requireTerm(config.obs_current_terms, "command_foot_support_state", 6, 119);
  requireTerm(config.obs_history_terms, "command_root_ori_b", 6, 0);
  requireTerm(config.obs_history_terms, "command_xy_yaw_vel", 3, 6);
  requireTerm(config.obs_history_terms, "command_foot_support_state", 6, 93);
}

TEST_CASE("DeployConfig loads the GeneralTrackerCLN observation contract") {
  const DeployConfig config = loadYaml(validClnDeployYaml());

  REQUIRE(config.joint_dim == 26);
  REQUIRE(config.observation_contract == ObservationContract::GeneralTrackerCLN);
  REQUIRE(config.obs_current.size() == 9);
  REQUIRE(config.obs_history.size() == 1);
  REQUIRE(config.obs_current_terms.size() == 9);
  REQUIRE(config.obs_history_terms.size() == 1);
  REQUIRE(config.obs_current_dim == 121);
  REQUIRE(config.obs_history_width == 35);
  REQUIRE(config.obs_history_length == 25);

  requireTerm(config.obs_current_terms, "command_yaw", 2, 0);
  requireTerm(config.obs_current_terms, "command_root_ori_b", 6, 2);
  requireTerm(config.obs_current_terms, "last_action", 26, 95);
  requireTerm(config.obs_history_terms, "future_commands", 35, 0);
}

TEST_CASE("DeployConfig loads ET1 GeneralTracker override joint ids when configured") {
  const std::string yaml =
      replaceOnce(validDeployYaml(), "step_dt: 0.02\n",
                  "step_dt: 0.02\n"
                  "override_joint_ids: [24, 25]\n");

  const DeployConfig config = loadYaml(yaml);

  REQUIRE(config.override_joint_ids == std::vector<int>{24, 25});
}

TEST_CASE("DeployConfig parses and validates default joint positions") {
  SECTION("missing default_joint_pos") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "default_joint_pos: " + repeatValues("0.25", 26) + "\n",
                    "");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("default_joint_pos"));
  }

  SECTION("default_joint_pos length") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "default_joint_pos: " + repeatValues("0.25", 26),
                    "default_joint_pos: " + repeatValues("0.25", 25));

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("default_joint_pos"));
  }

  SECTION("default_joint_pos finite values") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "default_joint_pos: " + repeatValues("0.25", 26),
                    "default_joint_pos: [.nan, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, "
                    "0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, "
                    "0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("default_joint_pos"));
  }
}

TEST_CASE("DeployConfig rejects unsupported GeneralTracker override joint ids") {
  SECTION("unsupported id") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "step_dt: 0.02\n",
                    "step_dt: 0.02\n"
                    "override_joint_ids: [23]\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("override_joint_ids"));
  }

  SECTION("duplicate id") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "step_dt: 0.02\n",
                    "step_dt: 0.02\n"
                    "override_joint_ids: [24, 24]\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("override_joint_ids"));
  }
}

TEST_CASE("DeployConfig rejects missing required GeneralTracker observations") {
  const std::string yaml = replaceOnce(validDeployYaml(),
                                       "    command_foot_support_state:\n"
                                       "      history_length: 1\n",
                                       "");

  REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("command_foot_support_state"));
}

TEST_CASE("DeployConfig rejects unknown CLN legacy and dance observation keys") {
  SECTION("legacy current command_yaw") {
    const std::string yaml = replaceOnce(validDeployYaml(),
                                         "    use_gym_history: true\n",
                                         "    use_gym_history: true\n"
                                         "    command_yaw:\n"
                                         "      history_length: 1\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("command_yaw"));
  }

  SECTION("legacy history future_commands") {
    const std::string yaml = replaceOnce(validDeployYaml(),
                                         "  obs_history:\n"
                                         "    use_gym_history: true\n",
                                         "  obs_history:\n"
                                         "    use_gym_history: true\n"
                                         "    future_commands:\n"
                                         "      history_length: 25\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("future_commands"));
  }
}

TEST_CASE("DeployConfig rejects inconsistent history observation lengths") {
  const std::string yaml = replaceOnce(validDeployYaml(),
                                       "    ref_com_vel_navi:\n"
                                       "      history_length: 25\n",
                                       "    ref_com_vel_navi:\n"
                                       "      history_length: 24\n");

  REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("history_length"));
}

TEST_CASE("DeployConfig rejects observation order and metadata drift") {
  SECTION("obs_current order drift") {
    const std::string yaml = replaceOnce(validDeployYaml(),
                                         "    command_root_ori_b:\n"
                                         "      history_length: 1\n"
                                         "    command_xy_yaw_vel:\n"
                                         "      history_length: 1\n",
                                         "    command_xy_yaw_vel:\n"
                                         "      history_length: 1\n"
                                         "    command_root_ori_b:\n"
                                         "      history_length: 1\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("observations.obs_current"));
  }

  SECTION("use_gym_history false") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "    use_gym_history: true\n",
                    "    use_gym_history: false\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("use_gym_history"));
  }

  SECTION("non-null observation scale") {
    const std::string yaml = replaceOnce(validDeployYaml(),
                                         "    command_root_ori_b:\n"
                                         "      history_length: 1\n",
                                         "    command_root_ori_b:\n"
                                         "      scale: 1.0\n"
                                         "      history_length: 1\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("scale"));
  }
}

TEST_CASE("DeployConfig rejects bad fixed-width vectors") {
  SECTION("policy_kd length") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "policy_kd: " + repeatValues("0.1", 26),
                    "policy_kd: " + repeatValues("0.1", 25));

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("policy_kd"));
  }

  SECTION("action scale length") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "    scale: " + repeatValues("0.1", 26),
                    "    scale: " + repeatValues("0.1", 25));

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("actions.JointPositionAction.scale"));
  }
}

TEST_CASE("DeployConfig rejects non-frozen joint id maps") {
  SECTION("joint_ids_map duplicate") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "joint_ids_map: " + intRange(0, 26),
                    "joint_ids_map: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, "
                    "13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 24]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("joint_ids_map"));
  }

  SECTION("joint_ids_map reordered permutation") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "joint_ids_map: " + intRange(0, 26),
                    "joint_ids_map: [1, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, "
                    "13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("joint_ids_map"));
  }

  SECTION("sdk_joint_ids_map drift") {
    const std::string yaml = replaceOnce(
        validDeployYaml(),
        "sdk_joint_ids_map: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, "
        "15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30]",
        "sdk_joint_ids_map: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, "
        "15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 31]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("sdk_joint_ids_map"));
  }
}

TEST_CASE("DeployConfig rejects negative or NaN policy gains") {
  SECTION("negative kp") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "policy_kp: " + repeatValues("1.0", 26),
                    "policy_kp: [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "
                    "1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "
                    "1.0, 1.0, 1.0, 1.0, 1.0, 1.0]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("policy_kp"));
  }

  SECTION("zero kp") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "policy_kp: " + repeatValues("1.0", 26),
                    "policy_kp: [0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "
                    "1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "
                    "1.0, 1.0, 1.0, 1.0, 1.0, 1.0]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("policy_kp"));
  }

  SECTION("NaN kp") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "policy_kp: " + repeatValues("1.0", 26),
                    "policy_kp: [.nan, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "
                    "1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "
                    "1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("policy_kp"));
  }
}

TEST_CASE("DeployConfig rejects missing JointPositionAction scale or offset") {
  SECTION("missing scale") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "    scale: " + repeatValues("0.1", 26) + "\n", "");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("actions.JointPositionAction.scale"));
  }

  SECTION("missing offset") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "    offset: " + repeatValues("0.0", 26) + "\n", "");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("actions.JointPositionAction.offset"));
  }
}

TEST_CASE("DeployConfig rejects action contract drift") {
  SECTION("multiple actions") {
    const std::string yaml = replaceOnce(validDeployYaml(),
                                         "policy_kp: " + repeatValues("1.0", 26) + "\n",
                                         "  OtherAction:\n"
                                         "    scale: []\n"
                                         "policy_kp: " + repeatValues("1.0", 26) + "\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("actions"));
  }

  SECTION("JointPositionAction joint_ids non-null") {
    const std::string yaml =
        replaceOnce(validDeployYaml(), "    offset: " + repeatValues("0.0", 26) + "\n",
                    "    offset: " + repeatValues("0.0", 26) + "\n"
                    "    joint_ids: [0]\n");

    REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("joint_ids"));
  }
}

TEST_CASE("DeployConfig rejects frozen timing drift") {
  const std::string yaml = replaceOnce(validDeployYaml(), "step_dt: 0.02\n", "step_dt: 0.01\n");

  REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("step_dt"));
}

TEST_CASE("DeployConfig rejects CLN legacy-like deploy fixtures") {
  std::string yaml = replaceOnce(validDeployYaml(), "command_root_ori_b", "command_root_ori_b_unbiased");
  yaml = replaceOnce(yaml,
                     "    ref_com_rel_navi:\n"
                     "      history_length: 1\n"
                     "    ref_com_vel_navi:\n"
                     "      history_length: 1\n",
                     "");

  REQUIRE_THROWS_WITH(loadYaml(yaml), ContainsSubstring("command_root_ori_b_unbiased"));
}

}  // namespace agentic_et1_tracker
