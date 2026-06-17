#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "agentic_et1_tracker/app/app_runtime_factory.hpp"
#include "agentic_et1_tracker/policy/deploy_config.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"

#ifndef AGENTIC_ET1_TRACKER_REAL_FACTORY
#define AGENTIC_ET1_TRACKER_REAL_FACTORY 0
#endif

namespace agentic_et1_tracker {
namespace app_internal {

TrkValidationConfig internalStandbyTrkValidationConfig(
    const AppConfig& config,
    const std::filesystem::path& standby_reference);
bool referencesEt1RuntimeDependency(const std::filesystem::path& path);
void tryAttachLocoUpperDeps(const AppConfig& config, AppRuntimeDeps& deps);

}  // namespace app_internal
namespace {

struct TempTree {
  TempTree() {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_app_runtime_factory_tests_" + std::to_string(suffix));
    policy_dir = root / "policy";
    exported_dir = policy_dir / "exported";
    params_dir = policy_dir / "params";
    std::filesystem::create_directories(exported_dir);
    std::filesystem::create_directories(params_dir);
  }

  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path root;
  std::filesystem::path policy_dir;
  std::filesystem::path exported_dir;
  std::filesystem::path params_dir;
};

AppConfig factoryConfig(const TempTree& tmp) {
  AppConfig config;
  config.runtime.hz = 50.0;
  config.runtime.queue_limit = 3;
  config.policy.policy_dir = tmp.policy_dir.string();
  config.policy.policy_file = "missing.onnx";
  config.policy.deploy = (tmp.params_dir / "deploy.yaml").string();
  return config;
}

std::filesystem::path appRoot() {
  return std::filesystem::absolute(std::filesystem::path(__FILE__).parent_path().parent_path())
      .lexically_normal();
}

AppConfig realFactoryConfigWithRepoAssets() {
  const auto root = appRoot();
  AppConfig config;
  config.mode_machine = 0;
  config.release_motion_mode_on_startup = false;
  config.trk.allowlist_dirs = {"/tmp/user-motions"};
  config.policy.profile = "GeneralTrackerCLNFootstate";
  config.policy.policy_dir = (root / "config/policy/general_tracker_cln").string();
  config.policy.policy_file = "multi_policy_footstate3.onnx";
  config.policy.deploy =
      (root / "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml")
          .string();
  config.control.velocity_policy_dir = (root / "config/policy/velocity/v0").string();
  config.control.velocity_policy_file = "policy.onnx";
  config.control.velocity_deploy =
      (root / "config/policy/velocity/v0/params/deploy.yaml").string();
  config.control.fixstand_config =
      (root / "config/posture/fixstand/v0/fixstand.yaml").string();
  config.control.passive_config =
      (root / "config/posture/passive/v0/passive.yaml").string();
  return config;
}

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

void writeObsBlock(std::ostringstream& out,
                   const char* const* names,
                   std::size_t count,
                   int history_length) {
  out << "    use_gym_history: true\n";
  for (std::size_t i = 0; i < count; ++i) {
    out << "    " << names[i] << ":\n";
    out << "      history_length: " << history_length << "\n";
  }
}

std::string validDeployYaml() {
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
  writeObsBlock(out, kCurrentObs, sizeof(kCurrentObs) / sizeof(kCurrentObs[0]), 1);
  out << "  obs_history:\n";
  writeObsBlock(out, kHistoryObs, sizeof(kHistoryObs) / sizeof(kHistoryObs[0]), 25);
  return out.str();
}

void writeText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out << text;
}

void writeBadTrk(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  writeText(path, "not an ET1TRK1 file");
}

void copyRepoStandbyRefTo(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  const auto source = appRoot() / "config/reference/standby/v0/standby_ref.trk";
  REQUIRE(std::filesystem::is_regular_file(source));
  std::filesystem::copy_file(source, path, std::filesystem::copy_options::overwrite_existing);
}

void writeMinimalGaOnnx(const std::filesystem::path& path) {
  // Constant-zero GA policy model with obs_current/obs_history inputs and actions output.
  static constexpr unsigned char kModel[] = {
      0x08, 0x07, 0x12, 0x19, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f,
      0x65, 0x74, 0x31, 0x5f, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x5f,
      0x74, 0x65, 0x73, 0x74, 0x73, 0x3a, 0x9b, 0x02, 0x0a, 0xa1, 0x01, 0x12,
      0x07, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x22, 0x08, 0x43, 0x6f,
      0x6e, 0x73, 0x74, 0x61, 0x6e, 0x74, 0x2a, 0x8b, 0x01, 0x0a, 0x05, 0x76,
      0x61, 0x6c, 0x75, 0x65, 0x2a, 0x7f, 0x08, 0x01, 0x08, 0x1a, 0x10, 0x01,
      0x22, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x0d, 0x61, 0x63, 0x74, 0x69,
      0x6f, 0x6e, 0x73, 0x5f, 0x76, 0x61, 0x6c, 0x75, 0x65, 0xa0, 0x01, 0x04,
      0x12, 0x17, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f, 0x65, 0x74,
      0x31, 0x5f, 0x74, 0x65, 0x73, 0x74, 0x5f, 0x70, 0x6f, 0x6c, 0x69, 0x63,
      0x79, 0x5a, 0x1e, 0x0a, 0x0b, 0x6f, 0x62, 0x73, 0x5f, 0x63, 0x75, 0x72,
      0x72, 0x65, 0x6e, 0x74, 0x12, 0x0f, 0x0a, 0x0d, 0x08, 0x01, 0x12, 0x09,
      0x0a, 0x02, 0x08, 0x01, 0x0a, 0x03, 0x08, 0x82, 0x01, 0x5a, 0x21, 0x0a,
      0x0b, 0x6f, 0x62, 0x73, 0x5f, 0x68, 0x69, 0x73, 0x74, 0x6f, 0x72, 0x79,
      0x12, 0x12, 0x0a, 0x10, 0x08, 0x01, 0x12, 0x0c, 0x0a, 0x02, 0x08, 0x01,
      0x0a, 0x02, 0x08, 0x19, 0x0a, 0x02, 0x08, 0x68, 0x62, 0x19, 0x0a, 0x07,
      0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x12, 0x0e, 0x0a, 0x0c, 0x08,
      0x01, 0x12, 0x08, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x02, 0x08, 0x1a, 0x42,
      0x04, 0x0a, 0x00, 0x10, 0x0d};
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out.write(reinterpret_cast<const char*>(kModel), sizeof(kModel));
}

void writeMinimalLocoLowerOnnx(const std::filesystem::path& path) {
  static constexpr unsigned char kModel[] = {
      0x08, 0x08, 0x12, 0x11, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f,
      0x65, 0x74, 0x31, 0x5f, 0x74, 0x65, 0x73, 0x74, 0x73, 0x3a, 0xad, 0x01,
      0x0a, 0x60, 0x12, 0x07, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x22,
      0x08, 0x43, 0x6f, 0x6e, 0x73, 0x74, 0x61, 0x6e, 0x74, 0x2a, 0x4b, 0x0a,
      0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x2a, 0x3f, 0x08, 0x01, 0x08, 0x0c,
      0x10, 0x01, 0x22, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x42, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0xa0,
      0x01, 0x04, 0x12, 0x17, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f,
      0x65, 0x74, 0x31, 0x5f, 0x74, 0x65, 0x73, 0x74, 0x5f, 0x70, 0x6f, 0x6c,
      0x69, 0x63, 0x79, 0x5a, 0x15, 0x0a, 0x03, 0x6f, 0x62, 0x73, 0x12, 0x0e,
      0x0a, 0x0c, 0x08, 0x01, 0x12, 0x08, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x02,
      0x08, 0x2d, 0x62, 0x19, 0x0a, 0x07, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e,
      0x73, 0x12, 0x0e, 0x0a, 0x0c, 0x08, 0x01, 0x12, 0x08, 0x0a, 0x02, 0x08,
      0x01, 0x0a, 0x02, 0x08, 0x0c, 0x42, 0x04, 0x0a, 0x00, 0x10, 0x0d};
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out.write(reinterpret_cast<const char*>(kModel), sizeof(kModel));
}

void writeWrongLocoLowerInputShapeOnnx(const std::filesystem::path& path) {
  static constexpr unsigned char kModel[] = {
      0x08, 0x08, 0x12, 0x11, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f,
      0x65, 0x74, 0x31, 0x5f, 0x74, 0x65, 0x73, 0x74, 0x73, 0x3a, 0xad, 0x01,
      0x0a, 0x60, 0x12, 0x07, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x22,
      0x08, 0x43, 0x6f, 0x6e, 0x73, 0x74, 0x61, 0x6e, 0x74, 0x2a, 0x4b, 0x0a,
      0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x2a, 0x3f, 0x08, 0x01, 0x08, 0x0c,
      0x10, 0x01, 0x22, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x42, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0xa0,
      0x01, 0x04, 0x12, 0x17, 0x61, 0x67, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x5f,
      0x65, 0x74, 0x31, 0x5f, 0x74, 0x65, 0x73, 0x74, 0x5f, 0x70, 0x6f, 0x6c,
      0x69, 0x63, 0x79, 0x5a, 0x15, 0x0a, 0x03, 0x6f, 0x62, 0x73, 0x12, 0x0e,
      0x0a, 0x0c, 0x08, 0x01, 0x12, 0x08, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x02,
      0x08, 0x2e, 0x62, 0x19, 0x0a, 0x07, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e,
      0x73, 0x12, 0x0e, 0x0a, 0x0c, 0x08, 0x01, 0x12, 0x08, 0x0a, 0x02, 0x08,
      0x01, 0x0a, 0x02, 0x08, 0x0c, 0x42, 0x04, 0x0a, 0x00, 0x10, 0x0d};
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out.write(reinterpret_cast<const char*>(kModel), sizeof(kModel));
}

class DummyRobotIO final : public RobotIO {
 public:
  std::optional<LowStateSample> readLowState() const override { return std::nullopt; }
  std::optional<HighStateSample> readHighState() const override { return std::nullopt; }
  LowCmdOccupancy lowCmdOccupancy() const override { return {}; }
  void writeLowCmd(const LowCmdFrame&) override {}
};

class DummyPolicy final : public PolicyInference {
 public:
  Vec infer(const PolicyInputs&) override { return {}; }
};

class DummyVelocityPolicy final : public VelocityPolicyInference {
 public:
  Vec infer(const VelocityPolicyInputs&) override { return {}; }
};

AppRuntimeDeps baseDepsForLocoAttach() {
  AppRuntimeDeps deps;
  deps.robot_io = std::make_unique<DummyRobotIO>();
  deps.policy = std::make_unique<DummyPolicy>();
  deps.velocity_policy = std::make_unique<DummyVelocityPolicy>();
  deps.deploy_config = loadDeployConfig(
      appRoot() / "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml");
  return deps;
}

void requireModelNotReady(const AppRuntimeFactoryResult& result) {
  REQUIRE_FALSE(result.deps.has_value());
  REQUIRE(result.snapshot.ready == false);
  REQUIRE(result.snapshot.robot == RobotState::NotReady);
  REQUIRE(result.snapshot.ctrl == ControllerState::Starting);
  REQUIRE(result.snapshot.block == "policy_not_loaded");
  REQUIRE(result.snapshot.err == ErrorCode::ModelNotReady);
  REQUIRE(result.health.state == ServiceHealth::Starting);
  REQUIRE(result.health.err == ErrorCode::ModelNotReady);
  REQUIRE(result.health.block == "policy_not_loaded");
}

void requireMode(const AppRuntimeFactoryResult& result, RuntimeMode mode) {
  REQUIRE(result.snapshot.mode == mode);
  REQUIRE(result.health.mode == mode);
}

}  // namespace

TEST_CASE("AppRuntimeFactory stub reports policy not loaded when production deps are disabled") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  SUCCEED("stub factory is not compiled in this build");
#else
  TempTree tmp;
  AppConfig config = factoryConfig(tmp);
  config.mode_machine = 0;
  const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

  requireModelNotReady(result);
  requireMode(result, RuntimeMode::Sim);
  REQUIRE(result.snapshot.hz == 50.0);
  REQUIRE(result.snapshot.queue.limit == 3);
#endif
}

TEST_CASE("AppRuntimeFactory real factory fails before SDK when deploy file is missing") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  SECTION("sim mode_machine reports sim not-ready") {
    TempTree tmp;
    AppConfig config = factoryConfig(tmp);
    config.mode_machine = 0;
    config.policy.deploy = (tmp.params_dir / "missing-deploy.yaml").string();

    const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

    requireModelNotReady(result);
    requireMode(result, RuntimeMode::Sim);
  }

  SECTION("real mode_machine reports real not-ready") {
    TempTree tmp;
    AppConfig config = factoryConfig(tmp);
    config.mode_machine = 1;
    config.policy.deploy = (tmp.params_dir / "missing-deploy.yaml").string();

    const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

    requireModelNotReady(result);
    requireMode(result, RuntimeMode::Real);
  }
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory fails before SDK when model file is missing") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = factoryConfig(tmp);
  writeText(config.policy.deploy, validDeployYaml());

  const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

  requireModelNotReady(result);
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory fails before SDK on policy/deploy contract mismatch") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  const auto root = appRoot();
  const auto legacy_deploy = root / "config/policy/general_tracker/params/deploy.yaml";
  const auto cln_deploy = root / "config/policy/general_tracker_cln/params/deploy.yaml";
  const auto footstate_deploy =
      root / "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml";

  AppConfig config;
  config.mode_machine = 0;
  config.release_motion_mode_on_startup = false;
  config.policy.policy_dir = (root / "config/policy/general_tracker_cln").string();
  config.policy.policy_file = "missing.onnx";

  SECTION("legacy profile with footstate deploy") {
    config.policy.profile = "GeneralTracker";
    config.policy.deploy = footstate_deploy.string();
    requireModelNotReady(createAppRuntimeDeps(config));
  }

  SECTION("CLN profile with footstate deploy") {
    config.policy.profile = "GeneralTrackerCLN";
    config.policy.deploy = footstate_deploy.string();
    requireModelNotReady(createAppRuntimeDeps(config));
  }

  SECTION("footstate profile with legacy deploy") {
    config.policy.profile = "GeneralTrackerCLNFootstate";
    config.policy.deploy = legacy_deploy.string();
    requireModelNotReady(createAppRuntimeDeps(config));
  }

  SECTION("footstate profile with CLN deploy") {
    config.policy.profile = "GeneralTrackerCLNFootstate";
    config.policy.deploy = cln_deploy.string();
    requireModelNotReady(createAppRuntimeDeps(config));
  }
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory blocks final model symlink into ET1 policy tree") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = factoryConfig(tmp);
  config.policy.policy_file = "model.onnx";
  writeText(config.policy.deploy, validDeployYaml());

  const auto et1_exported =
      tmp.root / "unitree_rl_lab/deploy/robots/et1/config/policy/general_tracker/exported";
  std::filesystem::create_directories(et1_exported);
  const auto et1_model = et1_exported / "model.onnx";
  writeMinimalGaOnnx(et1_model);

  std::error_code ec;
  std::filesystem::create_symlink(et1_model, tmp.exported_dir / config.policy.policy_file, ec);
  if (ec) {
    SUCCEED("file symlinks are not supported on this platform");
    return;
  }

  const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

  requireModelNotReady(result);
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory blocks final model symlink into ET1 config file") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = factoryConfig(tmp);
  config.policy.policy_file = "model.onnx";
  writeText(config.policy.deploy, validDeployYaml());

  const auto et1_config = tmp.root / "unitree_rl_lab/deploy/robots/et1/config/config.yaml";
  std::filesystem::create_directories(et1_config.parent_path());
  writeMinimalGaOnnx(et1_config);

  std::error_code ec;
  std::filesystem::create_symlink(et1_config, tmp.exported_dir / config.policy.policy_file, ec);
  if (ec) {
    SUCCEED("file symlinks are not supported on this platform");
    return;
  }

  const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

  requireModelNotReady(result);
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory validates internal standby with fixed duration limit") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  AppConfig config;
  config.trk.fps = 50.0;
  config.trk.max_duration_s = 0.1;
  config.trk.allowlist_dirs = {"/tmp/user-motions"};
  const std::filesystem::path standby_reference =
      "/opt/agentic-et1/current/share/agentic-et1-tracker/config/reference/standby/v0/"
      "standby_ref.trk";

  const TrkValidationConfig standby_config =
      app_internal::internalStandbyTrkValidationConfig(config, standby_reference);

  REQUIRE(standby_config.fps == 50.0);
  REQUIRE(standby_config.max_duration_s == 5.0);
  REQUIRE(standby_config.allowlist_dirs ==
          std::vector<std::filesystem::path>{standby_reference.parent_path()});
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory reports not-ready when internal standby asset is "
          "missing without ET1 fallback") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = realFactoryConfigWithRepoAssets();
  config.control.standby_reference =
      (tmp.root / "deploy/robots/agentic-et1-tracker/config/reference/standby/v0/"
                  "standby_ref.trk")
          .string();

  const auto et1_fallback =
      tmp.root / "deploy/robots/et1/config/reference/standby/v0/standby_ref.trk";
  copyRepoStandbyRefTo(et1_fallback);

  const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

  requireModelNotReady(result);
  requireMode(result, RuntimeMode::Sim);
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory real factory reports not-ready when internal standby asset is "
          "damaged without ET1 fallback") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = realFactoryConfigWithRepoAssets();
  const auto standby_reference =
      tmp.root / "deploy/robots/agentic-et1-tracker/config/reference/standby/v0/"
                 "standby_ref.trk";
  writeBadTrk(standby_reference);
  config.control.standby_reference = standby_reference.string();

  const auto et1_fallback =
      tmp.root / "deploy/robots/et1/config/reference/standby/v0/standby_ref.trk";
  copyRepoStandbyRefTo(et1_fallback);

  const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

  requireModelNotReady(result);
  requireMode(result, RuntimeMode::Sim);
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory path guard rejects ET1 app reference tree but keeps app-owned assets") {
  REQUIRE(app_internal::referencesEt1RuntimeDependency(
      "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/reference/standby/v0/"
      "standby_ref.trk"));
  REQUIRE_FALSE(app_internal::referencesEt1RuntimeDependency(
      "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/config/"
      "reference/standby/v0/standby_ref.trk"));
}

TEST_CASE("AppRuntimeFactory releases Unitree motion mode only for real mode_machine") {
  AppConfig config;
  config.release_motion_mode_on_startup = true;

  config.mode_machine = 1;
  REQUIRE(shouldReleaseMotionModeOnStartup(config));

  config.mode_machine = 0;
  REQUIRE_FALSE(shouldReleaseMotionModeOnStartup(config));

  config.mode_machine = 1;
  config.release_motion_mode_on_startup = false;
  REQUIRE_FALSE(shouldReleaseMotionModeOnStartup(config));
}

TEST_CASE("AppRuntimeFactory loco attach keeps failures isolated from base deps") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = realFactoryConfigWithRepoAssets();
  config.loco_upper.enabled = true;
  config.loco_upper.policy_dir = tmp.policy_dir.string();
  config.loco_upper.policy_file = "loco.onnx";
  config.loco_upper.deploy =
      (appRoot() / "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml").string();
  config.loco_upper.limits =
      (appRoot() / "config/limits/et1_upper_body/v0/limits.yaml").string();
  config.loco_upper.joint_map =
      (appRoot() / "config/limits/et1_upper_body/v0/joint_map.yaml").string();

  SECTION("valid app-owned assets attach all loco deps") {
    writeMinimalLocoLowerOnnx(tmp.exported_dir / config.loco_upper.policy_file);
    AppRuntimeDeps deps = baseDepsForLocoAttach();

    app_internal::tryAttachLocoUpperDeps(config, deps);

    REQUIRE(deps.loco_lower_policy);
    REQUIRE(deps.loco_lower_deploy_config.has_value());
    REQUIRE(deps.loco_upper_composer_config.has_value());
  }

  SECTION("bad model path keeps baseline deps untouched") {
    AppRuntimeDeps deps = baseDepsForLocoAttach();
    RobotIO* const robot = deps.robot_io.get();
    PolicyInference* const policy = deps.policy.get();
    VelocityPolicyInference* const velocity_policy = deps.velocity_policy.get();

    app_internal::tryAttachLocoUpperDeps(config, deps);

    REQUIRE_FALSE(deps.loco_lower_policy);
    REQUIRE_FALSE(deps.loco_lower_deploy_config.has_value());
    REQUIRE_FALSE(deps.loco_upper_composer_config.has_value());
    REQUIRE(deps.robot_io.get() == robot);
    REQUIRE(deps.policy.get() == policy);
    REQUIRE(deps.velocity_policy.get() == velocity_policy);
  }

  SECTION("bad lower policy IO contract keeps baseline deps untouched") {
    writeWrongLocoLowerInputShapeOnnx(tmp.exported_dir / config.loco_upper.policy_file);
    AppRuntimeDeps deps = baseDepsForLocoAttach();
    RobotIO* const robot = deps.robot_io.get();
    PolicyInference* const policy = deps.policy.get();
    VelocityPolicyInference* const velocity_policy = deps.velocity_policy.get();

    app_internal::tryAttachLocoUpperDeps(config, deps);

    REQUIRE_FALSE(deps.loco_lower_policy);
    REQUIRE_FALSE(deps.loco_lower_deploy_config.has_value());
    REQUIRE_FALSE(deps.loco_upper_composer_config.has_value());
    REQUIRE(deps.robot_io.get() == robot);
    REQUIRE(deps.policy.get() == policy);
    REQUIRE(deps.velocity_policy.get() == velocity_policy);
  }

  SECTION("bad upper limits keep baseline deps untouched") {
    writeMinimalLocoLowerOnnx(tmp.exported_dir / config.loco_upper.policy_file);
    config.loco_upper.limits = (tmp.root / "missing_limits.yaml").string();
    AppRuntimeDeps deps = baseDepsForLocoAttach();
    RobotIO* const robot = deps.robot_io.get();
    PolicyInference* const policy = deps.policy.get();
    VelocityPolicyInference* const velocity_policy = deps.velocity_policy.get();

    app_internal::tryAttachLocoUpperDeps(config, deps);

    REQUIRE_FALSE(deps.loco_lower_policy);
    REQUIRE_FALSE(deps.loco_lower_deploy_config.has_value());
    REQUIRE_FALSE(deps.loco_upper_composer_config.has_value());
    REQUIRE(deps.robot_io.get() == robot);
    REQUIRE(deps.policy.get() == policy);
    REQUIRE(deps.velocity_policy.get() == velocity_policy);
  }
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory enabled loco_upper asset failures surface as model-not-ready") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  TempTree tmp;
  AppConfig config = realFactoryConfigWithRepoAssets();
  config.loco_upper.enabled = true;
  config.loco_upper.policy_dir = tmp.policy_dir.string();
  config.loco_upper.policy_file = "loco.onnx";
  config.loco_upper.deploy =
      (appRoot() / "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml").string();
  config.loco_upper.limits =
      (appRoot() / "config/limits/et1_upper_body/v0/limits.yaml").string();
  config.loco_upper.joint_map =
      (appRoot() / "config/limits/et1_upper_body/v0/joint_map.yaml").string();

  SECTION("missing lower model fails startup") {
    const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

    requireModelNotReady(result);
    requireMode(result, RuntimeMode::Sim);
  }

  SECTION("bad lower policy IO contract fails startup") {
    writeWrongLocoLowerInputShapeOnnx(tmp.exported_dir / config.loco_upper.policy_file);

    const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

    requireModelNotReady(result);
    requireMode(result, RuntimeMode::Sim);
  }

  SECTION("missing upper limits fail startup") {
    writeMinimalLocoLowerOnnx(tmp.exported_dir / config.loco_upper.policy_file);
    config.loco_upper.limits = (tmp.root / "missing_limits.yaml").string();

    const AppRuntimeFactoryResult result = createAppRuntimeDeps(config);

    requireModelNotReady(result);
    requireMode(result, RuntimeMode::Sim);
  }
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

TEST_CASE("AppRuntimeFactory loco attach skips disabled config without touching bogus assets") {
#if AGENTIC_ET1_TRACKER_REAL_FACTORY
  AppConfig config = realFactoryConfigWithRepoAssets();
  config.loco_upper.enabled = false;
  config.loco_upper.policy_dir = "/tmp/does-not-exist/policy";
  config.loco_upper.policy_file = "missing.onnx";
  config.loco_upper.deploy = "/tmp/does-not-exist/deploy.yaml";
  config.loco_upper.limits = "/tmp/does-not-exist/limits.yaml";
  config.loco_upper.joint_map = "/tmp/does-not-exist/joint_map.yaml";
  AppRuntimeDeps deps = baseDepsForLocoAttach();
  RobotIO* const robot = deps.robot_io.get();
  PolicyInference* const policy = deps.policy.get();
  VelocityPolicyInference* const velocity_policy = deps.velocity_policy.get();

  app_internal::tryAttachLocoUpperDeps(config, deps);

  REQUIRE_FALSE(deps.loco_lower_policy);
  REQUIRE_FALSE(deps.loco_lower_deploy_config.has_value());
  REQUIRE_FALSE(deps.loco_upper_composer_config.has_value());
  REQUIRE(deps.robot_io.get() == robot);
  REQUIRE(deps.policy.get() == policy);
  REQUIRE(deps.velocity_policy.get() == velocity_policy);
#else
  SUCCEED("real factory is not compiled in this build");
#endif
}

}  // namespace agentic_et1_tracker
