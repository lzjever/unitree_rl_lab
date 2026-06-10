#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "agentic_et1_tracker/app/app_config.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

struct TempConfig {
  explicit TempConfig(const std::string& yaml) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_app_config_tests_" + std::to_string(now) + ".yaml");

    std::ofstream out(path);
    REQUIRE(out);
    out << yaml;
  }

  ~TempConfig() {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }

  AppConfig load() const { return loadAppConfig(path); }

  std::filesystem::path path;
};

struct TempConfigTree {
  TempConfigTree() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_app_config_tree_tests_" + std::to_string(now));
    std::filesystem::create_directories(root);
  }

  ~TempConfigTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path writeConfig(const std::string& yaml) const {
    const auto path = root / "config.yaml";
    std::ofstream out(path);
    REQUIRE(out);
    out << yaml;
    return path;
  }

  AppConfig load(const std::string& yaml) const { return loadAppConfig(writeConfig(yaml)); }

  std::filesystem::path root;
};

AppConfig loadYaml(const std::string& yaml) {
  TempConfig tmp(yaml);
  return tmp.load();
}

std::string policyYaml(const std::string& profile,
                       const std::filesystem::path& policy_dir,
                       const std::filesystem::path& deploy) {
  return "agentic_et1_tracker:\n"
         "  motion_dirs: [\"/tmp/motions\"]\n"
         "  policy:\n"
         "    profile: \"" +
         profile + "\"\n"
         "    policy_dir: \"" +
         policy_dir.string() + "\"\n"
         "    deploy: \"" +
         deploy.string() + "\"\n";
}

std::filesystem::path appRoot() {
  return std::filesystem::absolute(std::filesystem::path(__FILE__).parent_path().parent_path())
      .lexically_normal();
}

bool pathIsAtOrWithin(const std::filesystem::path& child, const std::filesystem::path& parent) {
  const std::filesystem::path normalized_child = child.lexically_normal();
  const std::filesystem::path normalized_parent = parent.lexically_normal();
  if (normalized_child == normalized_parent) {
    return true;
  }
  auto child_it = normalized_child.begin();
  const auto child_end = normalized_child.end();
  auto parent_it = normalized_parent.begin();
  const auto parent_end = normalized_parent.end();

  for (; parent_it != parent_end; ++parent_it, ++child_it) {
    if (child_it == child_end || *child_it != *parent_it) {
      return false;
    }
  }
  return child_it != child_end;
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream in(path);
  REQUIRE(in);
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

std::string replaceAll(std::string value,
                       const std::string& needle,
                       const std::string& replacement) {
  std::size_t pos = 0;
  while ((pos = value.find(needle, pos)) != std::string::npos) {
    value.replace(pos, needle.size(), replacement);
    pos += replacement.size();
  }
  return value;
}

}  // namespace

TEST_CASE("AppConfig loads PRD defaults from agentic_et1_tracker section") {
  TempConfig tmp(R"yaml(
agentic_et1_tracker:
  motion_dirs:
    - "/home/galbot/motions"
)yaml");
  const auto config = tmp.load();
  const auto config_dir = tmp.path.parent_path();

  REQUIRE(config.http.host == "127.0.0.1");
  REQUIRE(config.http.port == 8083);
  REQUIRE(config.http.thread_pool_size == kHttpServerDefaultThreadPoolSize);
  REQUIRE(config.runtime.queue_limit == 8);
  REQUIRE(config.runtime.recent_limit == 32);
  REQUIRE(config.runtime.hz == 1000.0);
  REQUIRE(config.runtime.stop_hold_s == 0.0);
  REQUIRE(config.runtime.transition_duration_s == 0.30);
  REQUIRE(config.trk.fps == 50.0);
  REQUIRE(config.trk.max_duration_s == 120.0);
  REQUIRE(config.trk.allowlist_dirs == std::vector<std::filesystem::path>{"/home/galbot/motions"});
  REQUIRE(config.domain_id == 0);
  REQUIRE(config.lowcmd_startup_preflight_ms == 200);
  REQUIRE(config.release_motion_mode_on_startup);
  REQUIRE(config.release_motion_mode_timeout_s == 3.0);
  REQUIRE(config.release_motion_mode_max_attempts == 3);
  REQUIRE(config.release_motion_mode_retry_interval_ms == 500);
  REQUIRE(config.lock_path.empty());
  REQUIRE(config.passive_password == "galaxy");
  REQUIRE(config.policy.profile == "GeneralTrackerCLNFootstate");
  REQUIRE(config.policy.policy_dir ==
          (config_dir / "config/policy/general_tracker_cln").lexically_normal().string());
  REQUIRE(config.policy.policy_file == "multi_policy_footstate3.onnx");
  REQUIRE(config.policy.deploy ==
          (config_dir / "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.startup_control == "FixStand");
  REQUIRE(config.control.velocity_policy_dir ==
          (config_dir / "config/policy/velocity/v0").lexically_normal().string());
  REQUIRE(config.control.velocity_policy_file == "policy.onnx");
  REQUIRE(config.control.velocity_deploy ==
          (config_dir / "config/policy/velocity/v0/params/deploy.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.fixstand_config ==
          (config_dir / "config/posture/fixstand/v0/fixstand.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.passive_config ==
          (config_dir / "config/posture/passive/v0/passive.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.standby_reference ==
          (config_dir / "config/reference/standby/v0/standby_ref.trk")
              .lexically_normal()
              .string());
  REQUIRE_FALSE(config.reference.enabled);
}

TEST_CASE("AppConfig default file keeps StandbyVelocity and posture assets app-owned") {
  const auto root = appRoot();
  const auto config = loadAppConfig(root / "config.yaml");

  REQUIRE(config.policy.profile == "GeneralTrackerCLNFootstate");
  REQUIRE(pathIsAtOrWithin(config.policy.policy_dir,
                           root / "config/policy/general_tracker_cln"));
  REQUIRE(config.policy.policy_file == "multi_policy_footstate3.onnx");
  REQUIRE(config.policy.deploy ==
          (root / "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml")
              .lexically_normal()
              .string());
  REQUIRE(pathIsAtOrWithin(config.control.velocity_policy_dir,
                           root / "config/policy/velocity/v0"));
  REQUIRE(config.control.velocity_policy_file == "policy.onnx");
  REQUIRE(config.control.velocity_deploy ==
          (root / "config/policy/velocity/v0/params/deploy.yaml")
              .lexically_normal()
              .string());
  REQUIRE(pathIsAtOrWithin(config.control.fixstand_config,
                           root / "config/posture/fixstand/v0"));
  REQUIRE(pathIsAtOrWithin(config.control.passive_config,
                           root / "config/posture/passive/v0"));
  REQUIRE(config.control.standby_reference ==
          (root / "config/reference/standby/v0/standby_ref.trk")
              .lexically_normal()
              .string());
  REQUIRE(config.control.startup_control == "FixStand");
  REQUIRE(config.stop_hold_s == 0.0);
  REQUIRE(config.runtime.stop_hold_s == 0.0);
  REQUIRE(config.runtime.transition_duration_s == 0.30);
  REQUIRE(config.passive_password == "galaxy");
  REQUIRE_FALSE(config.reference.enabled);
}

TEST_CASE("AppConfig simulation example is ready for local MuJoCo acceptance") {
  const auto root = appRoot();
  const auto config = loadAppConfig(root / "config.sim.yaml.example");

  REQUIRE(config.http.host == "127.0.0.1");
  REQUIRE(config.network == "lo");
  REQUIRE(config.domain_id == 0);
  REQUIRE(config.mode_machine == 0);
  REQUIRE_FALSE(config.release_motion_mode_on_startup);
  REQUIRE(config.trk.allowlist_dirs ==
          std::vector<std::filesystem::path>{"/home/galbot/works/agent-test/generated"});
  REQUIRE(config.policy.profile == "GeneralTrackerCLNFootstate");
  REQUIRE(pathIsAtOrWithin(config.policy.policy_dir,
                           root / "config/policy/general_tracker_cln"));
  REQUIRE(config.policy.policy_file == "multi_policy_footstate3.onnx");
  REQUIRE(config.policy.deploy ==
          (root / "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml")
              .lexically_normal()
              .string());
  REQUIRE(pathIsAtOrWithin(config.control.velocity_policy_dir,
                           root / "config/policy/velocity/v0"));
  REQUIRE(pathIsAtOrWithin(config.control.fixstand_config,
                           root / "config/posture/fixstand/v0"));
  REQUIRE(pathIsAtOrWithin(config.control.passive_config,
                           root / "config/posture/passive/v0"));
  REQUIRE(config.control.standby_reference ==
          (root / "config/reference/standby/v0/standby_ref.trk")
              .lexically_normal()
              .string());
  REQUIRE(config.control.startup_control == "FixStand");
  REQUIRE(config.stop_hold_s == 0.0);
  REQUIRE(config.runtime.transition_duration_s == 0.7);
  REQUIRE(config.passive_password == "galaxy");
  REQUIRE(config.reference.enabled);
}

TEST_CASE("AppConfig release template resolves internal standby reference to release asset") {
  TempConfigTree tmp;
  const auto root = appRoot();
  const auto prefix = tmp.root / "prefix";
  const std::string templ = readText(root / "packaging/config.robot.yaml.template");
  const auto config = tmp.load(replaceAll(templ, "@PREFIX@", prefix.string()));

  REQUIRE(config.policy.profile == "GeneralTrackerCLNFootstate");
  REQUIRE(config.policy.policy_file == "multi_policy_footstate3.onnx");
  REQUIRE(config.policy.policy_dir ==
          (prefix / "current/share/agentic-et1-tracker/config/policy/general_tracker_cln")
              .lexically_normal()
              .string());
  REQUIRE(config.policy.deploy ==
          (prefix / "current/share/agentic-et1-tracker/config/policy/general_tracker_cln/params/"
                    "deploy_fut_multi_footstate.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.standby_reference ==
          (prefix / "current/share/agentic-et1-tracker/config/reference/standby/v0/standby_ref.trk")
              .lexically_normal()
              .string());
}

TEST_CASE("AppConfig preserved old release config derives missing standby reference from app "
          "assets") {
  TempConfigTree tmp;
  const auto prefix = tmp.root / "prefix";
  const auto app_config_root =
      prefix / "current/share/agentic-et1-tracker/config";
  const auto config = tmp.load("agentic_et1_tracker:\n"
                               "  bind: \"127.0.0.1\"\n"
                               "  port: 8083\n"
                               "  network: \"eth0\"\n"
                               "  domain_id: 0\n"
                               "  lock_path: \"" +
                               (prefix / "shared/run/agentic-et1-tracker.lock").string() +
                               "\"\n"
                               "  mode_machine: 1\n"
                               "  motion_dirs:\n"
                               "    - \"" +
                               (prefix / "shared/motions").string() +
                               "\"\n"
                               "  policy:\n"
                               "    profile: \"GeneralTrackerCLN\"\n"
                               "    policy_dir: \"" +
                               (app_config_root / "policy/general_tracker_cln").string() +
                               "\"\n"
                               "    policy_file: \"multi_policy_v17c2_70k.onnx\"\n"
                               "    deploy: \"" +
                               (app_config_root / "policy/general_tracker_cln/params/deploy.yaml")
                                   .string() +
                               "\"\n"
                               "    fps: 50\n"
                               "  control:\n"
                               "    startup_control: \"FixStand\"\n"
                               "    velocity_policy_dir: \"" +
                               (app_config_root / "policy/velocity/v0").string() +
                               "\"\n"
                               "    velocity_policy_file: \"policy.onnx\"\n"
                               "    velocity_deploy: \"" +
                               (app_config_root / "policy/velocity/v0/params/deploy.yaml")
                                   .string() +
                               "\"\n"
                               "    fixstand_config: \"" +
                               (app_config_root / "posture/fixstand/v0/fixstand.yaml")
                                   .string() +
                               "\"\n"
                               "    passive_config: \"" +
                               (app_config_root / "posture/passive/v0/passive.yaml").string() +
                               "\"\n");

  REQUIRE(config.policy.profile == "GeneralTrackerCLN");
  REQUIRE(config.policy.policy_file == "multi_policy_v17c2_70k.onnx");
  REQUIRE(config.control.standby_reference ==
          (app_config_root / "reference/standby/v0/standby_ref.trk")
              .lexically_normal()
              .string());
}

TEST_CASE("AppConfig maps complete PRD YAML into component configs") {
  TempConfig tmp(R"yaml(
agentic_et1_tracker:
  bind: "0.0.0.0"
  port: 18080
  network: "eth0"
  domain_id: 7
  lowcmd_startup_preflight_ms: 25
  release_motion_mode_on_startup: false
  release_motion_mode_timeout_s: 1.5
  release_motion_mode_max_attempts: 2
  release_motion_mode_retry_interval_ms: 10
  mode_machine: 0
  motion_dirs:
    - "/srv/motions/a"
    - "/srv/motions/b"
  queue_limit: 3
  recent_limit: 9
  max_track_duration_s: 42.5
  transition_duration_s: 1.0
  stop_hold_s: 0.0
  idle_mode: "hold_current"
  passive_password: "secret"
  lock_path: "/tmp/agentic-et1-tracker-test.lock"
  policy:
    profile: "GeneralTracker"
    policy_dir: "config/policy/custom"
    policy_file: "custom.onnx"
    deploy: "config/policy/custom/params/deploy.yaml"
    fps: 60
  control:
    startup_control: "StandbyVelocity"
    velocity_policy_dir: "config/policy/velocity/custom"
    velocity_policy_file: "standby.onnx"
    velocity_deploy: "config/policy/velocity/custom/params/deploy.yaml"
    fixstand_config: "config/posture/fixstand/custom/fixstand.yaml"
    passive_config: "config/posture/passive/custom/passive.yaml"
    standby_reference: "config/reference/standby/custom/standby_ref.trk"
  reference:
    enabled: true
)yaml");
  const auto config = tmp.load();
  const auto config_dir = tmp.path.parent_path();

  REQUIRE(config.http.host == "0.0.0.0");
  REQUIRE(config.http.port == 18080);
  REQUIRE(config.runtime.queue_limit == 3);
  REQUIRE(config.runtime.recent_limit == 9);
  REQUIRE(config.runtime.hz == 1000.0);
  REQUIRE(config.runtime.stop_hold_s == 0.0);
  REQUIRE(config.runtime.transition_duration_s == 1.0);
  REQUIRE(config.trk.allowlist_dirs ==
          std::vector<std::filesystem::path>{"/srv/motions/a", "/srv/motions/b"});
  REQUIRE(config.trk.max_duration_s == 42.5);
  REQUIRE(config.trk.fps == 60.0);
  REQUIRE(config.network == "eth0");
  REQUIRE(config.domain_id == 7);
  REQUIRE(config.lowcmd_startup_preflight_ms == 25);
  REQUIRE_FALSE(config.release_motion_mode_on_startup);
  REQUIRE(config.release_motion_mode_timeout_s == 1.5);
  REQUIRE(config.release_motion_mode_max_attempts == 2);
  REQUIRE(config.release_motion_mode_retry_interval_ms == 10);
  REQUIRE(config.mode_machine == 0);
  REQUIRE(config.stop_hold_s == 0.0);
  REQUIRE(config.idle_mode == "hold_current");
  REQUIRE(config.passive_password == "secret");
  REQUIRE(config.lock_path == "/tmp/agentic-et1-tracker-test.lock");
  REQUIRE(config.policy.policy_dir ==
          (config_dir / "config/policy/custom").lexically_normal().string());
  REQUIRE(config.policy.policy_file == "custom.onnx");
  REQUIRE(config.policy.deploy ==
          (config_dir / "config/policy/custom/params/deploy.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.startup_control == "StandbyVelocity");
  REQUIRE(config.control.velocity_policy_dir ==
          (config_dir / "config/policy/velocity/custom").lexically_normal().string());
  REQUIRE(config.control.velocity_policy_file == "standby.onnx");
  REQUIRE(config.control.velocity_deploy ==
          (config_dir / "config/policy/velocity/custom/params/deploy.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.fixstand_config ==
          (config_dir / "config/posture/fixstand/custom/fixstand.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.passive_config ==
          (config_dir / "config/posture/passive/custom/passive.yaml")
              .lexically_normal()
              .string());
  REQUIRE(config.control.standby_reference ==
          (config_dir / "config/reference/standby/custom/standby_ref.trk")
              .lexically_normal()
              .string());
  REQUIRE(config.reference.enabled);
}

TEST_CASE("AppConfig resolves relative policy paths from config file directory") {
  TempConfig tmp(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "./config/policy/../policy/custom"
    deploy: "config/policy/custom/params/../params/deploy.yaml"
)yaml");

  const auto config = tmp.load();
  const auto config_dir = tmp.path.parent_path();
  REQUIRE(config.policy.policy_dir ==
          (config_dir / "config/policy/custom").lexically_normal().string());
  REQUIRE(config.policy.deploy ==
          (config_dir / "config/policy/custom/params/deploy.yaml")
              .lexically_normal()
              .string());
}

TEST_CASE("AppConfig accepts external deploy files under policy params") {
  SECTION("absolute external policy dir") {
    const auto config = loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "/opt/agentic-assets/general_tracker"
    deploy: "/opt/agentic-assets/general_tracker/params/deploy.yaml"
)yaml");

    REQUIRE(config.policy.policy_dir == "/opt/agentic-assets/general_tracker");
    REQUIRE(config.policy.deploy == "/opt/agentic-assets/general_tracker/params/deploy.yaml");
  }

  SECTION("relative external policy dir resolves from config file directory") {
    TempConfigTree tmp;
    const auto config = tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "assets/general_tracker"
    deploy: "assets/general_tracker/params/deploy.yaml"
)yaml");

    REQUIRE(config.policy.policy_dir ==
            (tmp.root / "assets/general_tracker").lexically_normal().string());
    REQUIRE(config.policy.deploy ==
            (tmp.root / "assets/general_tracker/params/deploy.yaml")
                .lexically_normal()
                .string());
  }
}

TEST_CASE("AppConfig rejects deploy files outside policy params") {
  SECTION("different absolute asset roots") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "/opt/a"
    deploy: "/opt/b/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("deploy"));
  }

  SECTION("outside policy dir even when not ET1") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "/opt/agentic-assets/general_tracker"
    deploy: "/opt/agentic-assets/other_tracker/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("deploy"));
  }
}

TEST_CASE("AppConfig validates explicit lock_path") {
  SECTION("absolute lock path") {
    const auto config = loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  lock_path: "/tmp/agentic-et1-tracker-test.lock"
)yaml");

    REQUIRE(config.lock_path == "/tmp/agentic-et1-tracker-test.lock");
  }

  SECTION("relative lock path") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  lock_path: "tracker.lock"
)yaml"),
                        ContainsSubstring("lock_path"));
  }
}

TEST_CASE("AppConfig rejects invalid scalar bounds") {
  SECTION("port below range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  port: 0
  motion_dirs: ["/tmp/motions"]
)yaml"),
                        ContainsSubstring("port"));
  }

  SECTION("port above range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  port: 65536
  motion_dirs: ["/tmp/motions"]
)yaml"),
                        ContainsSubstring("port"));
  }

  SECTION("queue limit must be positive") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  queue_limit: 0
)yaml"),
                        ContainsSubstring("queue_limit"));
  }

  SECTION("recent limit must be positive") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  recent_limit: 0
)yaml"),
                        ContainsSubstring("recent_limit"));
  }

  SECTION("policy fps must be positive") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    fps: 0
)yaml"),
                        ContainsSubstring("fps"));
  }

  SECTION("hz below supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  hz: 0.5
)yaml"),
                        ContainsSubstring("hz"));
  }

  SECTION("hz above supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  hz: 1000.1
)yaml"),
                        ContainsSubstring("hz"));
  }

  SECTION("policy fps below supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    fps: 0.5
)yaml"),
                        ContainsSubstring("fps"));
  }

  SECTION("policy fps above supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    fps: 1000.1
)yaml"),
                        ContainsSubstring("fps"));
  }

  SECTION("release motion mode timeout must be positive") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  release_motion_mode_timeout_s: 0
)yaml"),
                        ContainsSubstring("release_motion_mode_timeout_s"));
  }

  SECTION("release motion mode max attempts must be positive") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  release_motion_mode_max_attempts: 0
)yaml"),
                        ContainsSubstring("release_motion_mode_max_attempts"));
  }

  SECTION("passive password must not be empty") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  passive_password: ""
)yaml"),
                        ContainsSubstring("passive_password"));
  }

  SECTION("reference max_rate_hz is unsupported because sim poller owns rate") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  reference:
    max_rate_hz: 60
)yaml"),
                        ContainsSubstring("max_rate_hz"));
  }

  SECTION("transition duration must be bounded positive") {
    for (const char* value : {"0.0", "-0.1", "6.0", ".nan"}) {
      CAPTURE(value);
      REQUIRE_THROWS_WITH(loadYaml(std::string("agentic_et1_tracker:\n"
                                              "  motion_dirs: [\"/tmp/motions\"]\n"
                                              "  transition_duration_s: ") +
                                   value + "\n"),
                          ContainsSubstring("transition_duration_s"));
    }
  }
}

TEST_CASE("AppConfig requires non-empty motion_dirs allowlist") {
  REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: []
)yaml"),
                      ContainsSubstring("motion_dirs"));
}

TEST_CASE("AppConfig requires absolute motion_dirs allowlist entries") {
  REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["motions"]
)yaml"),
                      ContainsSubstring("motion_dirs"));
}

TEST_CASE("AppConfig accepts supported policy profiles and rejects unknown profiles") {
  SECTION("GeneralTrackerCLNFootstate") {
    const auto config = loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    profile: "GeneralTrackerCLNFootstate"
)yaml");
    REQUIRE(config.policy.profile == "GeneralTrackerCLNFootstate");
  }

  SECTION("GeneralTrackerCLN") {
    const auto config = loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    profile: "GeneralTrackerCLN"
)yaml");
    REQUIRE(config.policy.profile == "GeneralTrackerCLN");
  }

  SECTION("legacy GeneralTracker") {
    const auto config = loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    profile: "GeneralTracker"
    policy_dir: "config/policy/general_tracker"
    deploy: "config/policy/general_tracker/params/deploy.yaml"
)yaml");
    REQUIRE(config.policy.profile == "GeneralTracker");
  }

  SECTION("arbitrary profile") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    profile: "CustomTracker"
)yaml"),
                        ContainsSubstring("policy.profile"));
  }
}

TEST_CASE("AppConfig rejects policy profile and deploy observation contract mismatches") {
  const auto root = appRoot();
  const auto legacy_policy_dir = root / "config/policy/general_tracker";
  const auto legacy_deploy = legacy_policy_dir / "params/deploy.yaml";
  const auto cln_policy_dir = root / "config/policy/general_tracker_cln";
  const auto cln_deploy = cln_policy_dir / "params/deploy.yaml";
  const auto footstate_deploy = cln_policy_dir / "params/deploy_fut_multi_footstate.yaml";

  SECTION("legacy profile with CLN deploy") {
    REQUIRE_THROWS_WITH(loadYaml(policyYaml("GeneralTracker", cln_policy_dir, cln_deploy)),
                        ContainsSubstring("policy.profile"));
  }

  SECTION("legacy profile with footstate deploy") {
    REQUIRE_THROWS_WITH(loadYaml(policyYaml("GeneralTracker", cln_policy_dir, footstate_deploy)),
                        ContainsSubstring("policy.profile"));
  }

  SECTION("CLN profile with legacy deploy") {
    REQUIRE_THROWS_WITH(loadYaml(policyYaml("GeneralTrackerCLN",
                                            legacy_policy_dir,
                                            legacy_deploy)),
                        ContainsSubstring("policy.profile"));
  }

  SECTION("CLN profile with footstate deploy") {
    REQUIRE_THROWS_WITH(loadYaml(policyYaml("GeneralTrackerCLN",
                                            cln_policy_dir,
                                            footstate_deploy)),
                        ContainsSubstring("policy.profile"));
  }

  SECTION("footstate profile with legacy deploy") {
    REQUIRE_THROWS_WITH(loadYaml(policyYaml("GeneralTrackerCLNFootstate",
                                            legacy_policy_dir,
                                            legacy_deploy)),
                        ContainsSubstring("policy.profile"));
  }

  SECTION("footstate profile with CLN deploy") {
    REQUIRE_THROWS_WITH(loadYaml(policyYaml("GeneralTrackerCLNFootstate",
                                            cln_policy_dir,
                                            cln_deploy)),
                        ContainsSubstring("policy.profile"));
  }
}

TEST_CASE("AppConfig rejects unsafe policy_file paths") {
  SECTION("absolute path") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_file: "/tmp/model.onnx"
)yaml"),
                        ContainsSubstring("policy_file"));
  }

  SECTION("path with directory separator") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_file: "params/model.onnx"
)yaml"),
                        ContainsSubstring("policy_file"));
  }

  SECTION("ET1 app policy file") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_file: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/policy/general_tracker/self_collision_footmesh_15k.onnx"
)yaml"),
                        ContainsSubstring("policy_file"));
  }
}

TEST_CASE("AppConfig rejects ET1 app policy and deploy paths") {
  SECTION("policy_dir") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "unitree_rl_lab/deploy/robots/et1/config/policy/general_tracker"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("deploy") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    deploy: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/policy/general_tracker/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("deploy to ET1 config") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    deploy: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/config.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }
}

TEST_CASE("AppConfig rejects ET1 app policy and config paths for standby control") {
  SECTION("velocity_policy_dir") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    velocity_policy_dir: "unitree_rl_lab/deploy/robots/et1/config/policy/velocity/v0"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("velocity model") {
    TempConfigTree tmp;
    const auto et1_model =
        tmp.root / "unitree_rl_lab/deploy/robots/et1/config/policy/velocity/v0/exported/policy.onnx";
    std::filesystem::create_directories(et1_model.parent_path());
    std::ofstream(et1_model).put('\0');

    const auto velocity_dir = tmp.root / "config/policy/velocity/v0";
    std::filesystem::create_directories(velocity_dir / "exported");
    std::filesystem::create_directories(velocity_dir / "params");
    std::error_code ec;
    std::filesystem::create_symlink(et1_model, velocity_dir / "exported/policy.onnx", ec);
    if (ec) {
      SUCCEED("file symlinks are not supported on this platform");
      return;
    }

    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("velocity_deploy") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    velocity_deploy: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/policy/velocity/v0/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("fixstand_config") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    fixstand_config: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/config.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("passive_config") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    passive_config: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/config.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("standby_reference under ET1 app config tree") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    standby_reference: "/home/galbot/works/et1/unitree_rl_lab/deploy/robots/et1/config/reference/standby/v0/standby_ref.trk"
)yaml"),
                        ContainsSubstring("ET1"));
  }
}

TEST_CASE("AppConfig rejects policy paths that symlink into the ET1 app policy tree") {
  TempConfigTree tmp;
  const auto app_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
  auto et1_policy = app_dir.parent_path() / "et1/config/policy/general_tracker";
  if (!std::filesystem::exists(et1_policy)) {
    et1_policy = tmp.root / "unitree_rl_lab/deploy/robots/et1/config/policy/general_tracker";
    std::filesystem::create_directories(et1_policy / "params");
  }

  const auto link = tmp.root / "policy_link";
  std::error_code ec;
  std::filesystem::create_directory_symlink(et1_policy, link, ec);
  if (ec) {
    SUCCEED("directory symlinks are not supported on this platform");
    return;
  }

  SECTION("policy_dir") {
    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "policy_link"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("deploy") {
    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    deploy: "policy_link/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }
}

TEST_CASE("AppConfig rejects deploy paths that symlink into the ET1 app config file") {
  TempConfigTree tmp;
  const auto app_dir = appRoot();
  auto et1_config = app_dir.parent_path() / "et1/config/config.yaml";
  if (!std::filesystem::exists(et1_config)) {
    et1_config = tmp.root / "unitree_rl_lab/deploy/robots/et1/config/config.yaml";
    std::filesystem::create_directories(et1_config.parent_path());
    std::ofstream(et1_config).put('\n');
  }

  const auto params_dir = tmp.root / "policy/params";
  std::filesystem::create_directories(params_dir);
  std::error_code ec;
  std::filesystem::create_symlink(et1_config, params_dir / "deploy.yaml", ec);
  if (ec) {
    SUCCEED("file symlinks are not supported on this platform");
    return;
  }

  REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  policy:
    policy_dir: "policy"
    deploy: "policy/params/deploy.yaml"
)yaml"),
                      ContainsSubstring("ET1"));
}

TEST_CASE("AppConfig rejects control paths that symlink into ET1 app assets") {
  TempConfigTree tmp;
  const auto app_dir = appRoot();
  auto et1_policy = app_dir.parent_path() / "et1/config/policy/velocity/v0";
  if (!std::filesystem::exists(et1_policy)) {
    et1_policy = tmp.root / "unitree_rl_lab/deploy/robots/et1/config/policy/velocity/v0";
    std::filesystem::create_directories(et1_policy / "params");
  }

  auto et1_config = app_dir.parent_path() / "et1/config/config.yaml";
  if (!std::filesystem::exists(et1_config)) {
    et1_config = tmp.root / "unitree_rl_lab/deploy/robots/et1/config/config.yaml";
    std::filesystem::create_directories(et1_config.parent_path());
    std::ofstream(et1_config).put('\n');
  }

  const auto policy_link = tmp.root / "velocity_link";
  const auto posture_link = tmp.root / "fixstand_link.yaml";
  std::error_code ec;
  std::filesystem::create_directory_symlink(et1_policy, policy_link, ec);
  if (ec) {
    SUCCEED("directory symlinks are not supported on this platform");
    return;
  }
  std::filesystem::create_symlink(et1_config, posture_link, ec);
  if (ec) {
    SUCCEED("file symlinks are not supported on this platform");
    return;
  }

  SECTION("velocity_policy_dir") {
    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    velocity_policy_dir: "velocity_link"
    velocity_deploy: "velocity_link/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("velocity_deploy") {
    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    velocity_deploy: "velocity_link/params/deploy.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("fixstand_config") {
    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    fixstand_config: "fixstand_link.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }

  SECTION("passive_config") {
    REQUIRE_THROWS_WITH(tmp.load(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  control:
    passive_config: "fixstand_link.yaml"
)yaml"),
                        ContainsSubstring("ET1"));
  }
}

TEST_CASE("AppConfig rejects unsupported GA mode fields") {
  SECTION("domain_id below supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  domain_id: -1
)yaml"),
                        ContainsSubstring("domain_id"));
  }

  SECTION("domain_id above supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  domain_id: 233
)yaml"),
                        ContainsSubstring("domain_id"));
  }

  SECTION("mode_machine below supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  mode_machine: -1
)yaml"),
                        ContainsSubstring("mode_machine"));
  }

  SECTION("mode_machine above supported range") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  mode_machine: 2
)yaml"),
                        ContainsSubstring("mode_machine"));
  }

  SECTION("idle_mode must be hold_current") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  idle_mode: "zero_torque"
)yaml"),
                        ContainsSubstring("idle_mode"));
  }

  SECTION("stop_hold_s must stay disabled") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  motion_dirs: ["/tmp/motions"]
  stop_hold_s: 0.01
)yaml"),
                        ContainsSubstring("stop_hold_s"));
  }

  SECTION("reference endpoint is sim-only") {
    REQUIRE_THROWS_WITH(loadYaml(R"yaml(
agentic_et1_tracker:
  mode_machine: 1
  motion_dirs: ["/tmp/motions"]
  reference:
    enabled: true
)yaml"),
                        ContainsSubstring("reference.enabled"));
  }
}

}  // namespace agentic_et1_tracker
