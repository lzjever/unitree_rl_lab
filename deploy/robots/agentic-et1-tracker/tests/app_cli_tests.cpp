#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>

#ifndef AGENTIC_ET1_TRACKER_BINARY_PATH
#error "AGENTIC_ET1_TRACKER_BINARY_PATH must be defined by CMake"
#endif

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

struct TempDir {
  TempDir() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_app_cli_tests_" + std::to_string(now));
    std::filesystem::create_directories(root);
  }

  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path writeText(const std::string& name, const std::string& text) const {
    const auto path = root / name;
    std::ofstream out(path);
    REQUIRE(out);
    out << text;
    return path;
  }

  std::filesystem::path root;
};

struct CliResult {
  int exit_code;
  std::string stdout_text;
  std::string stderr_text;
};

std::string shellQuote(const std::string& value) {
  std::string out = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out.push_back('\'');
  return out;
}

std::string shellQuote(const std::filesystem::path& path) {
  return shellQuote(path.string());
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream in(path);
  REQUIRE(in);
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

int decodeExitCode(int raw_status) {
  if (raw_status == -1) {
    return -1;
  }
  if (WIFEXITED(raw_status)) {
    return WEXITSTATUS(raw_status);
  }
  if (WIFSIGNALED(raw_status)) {
    return 128 + WTERMSIG(raw_status);
  }
  return -1;
}

CliResult runCheckConfig(const TempDir& tmp, const std::filesystem::path& config_path) {
  const auto stdout_path = tmp.root / "stdout.txt";
  const auto stderr_path = tmp.root / "stderr.txt";
  const std::string command =
      shellQuote(std::string(AGENTIC_ET1_TRACKER_BINARY_PATH)) + " --check-config --config " +
      shellQuote(config_path) + " >" + shellQuote(stdout_path) + " 2>" +
      shellQuote(stderr_path);

  const int raw_status = std::system(command.c_str());
  return CliResult{
      decodeExitCode(raw_status),
      readText(stdout_path),
      readText(stderr_path),
  };
}

std::string parseOnlyConfigYaml(const std::filesystem::path& lock_path) {
  return "agentic_et1_tracker:\n"
         "  lock_path: \"" +
         lock_path.string() + "\"\n"
         "  motion_dirs: [\"/tmp/motions\"]\n"
         "  transition_duration_s: 0.30\n"
         "  transition_min_frames: 2\n"
         "  transition_duration_dt_tolerance_s: 1.0e-9\n"
         "  user_bridge_reduced_startup_hold_s: 0.10\n"
         "  transition_root_yaw_residual_limit_rad: 0.05\n"
         "  transition_contact_guard: \"same_nonzero_contact\"\n"
         "  transition_max_velocity: 250.0\n"
         "  transition_max_acceleration: 10000.0\n"
         "  transition_max_jerk: 1000000.0\n";
}

std::string removeTopLevelKey(std::string yaml, const std::string& key) {
  std::istringstream in(yaml);
  std::ostringstream out;
  const std::string prefix = "  " + key + ":";
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind(prefix, 0) != 0) {
      out << line << '\n';
    }
  }
  return out.str();
}

}  // namespace

TEST_CASE("agentic-et1-tracker --check-config validates config without starting runtime") {
  TempDir tmp;
  const auto runtime_failing_lock = tmp.root / "missing-parent" / "tracker.lock";
  const auto config_path =
      tmp.writeText("config.yaml", parseOnlyConfigYaml(runtime_failing_lock));

  const CliResult result = runCheckConfig(tmp, config_path);

  REQUIRE(result.exit_code == 0);
  REQUIRE(result.stdout_text.empty());
  REQUIRE(result.stderr_text.find("failed to start agentic-et1-tracker HTTP server") ==
          std::string::npos);
  REQUIRE(result.stderr_text.find("agentic-et1-tracker listening on") == std::string::npos);
}

TEST_CASE("agentic-et1-tracker --check-config rejects missing required config key") {
  TempDir tmp;
  const auto runtime_failing_lock = tmp.root / "missing-parent" / "tracker.lock";
  const std::string yaml = parseOnlyConfigYaml(runtime_failing_lock);
  const auto config_path =
      tmp.writeText("missing-transition-limit.yaml",
                    removeTopLevelKey(yaml, "transition_max_velocity"));

  const CliResult result = runCheckConfig(tmp, config_path);

  REQUIRE(result.exit_code == 2);
  REQUIRE_THAT(result.stderr_text, ContainsSubstring("transition_max_velocity"));
  REQUIRE(result.stderr_text.find("failed to start agentic-et1-tracker HTTP server") ==
          std::string::npos);
  REQUIRE(result.stderr_text.find("agentic-et1-tracker listening on") == std::string::npos);
}

}  // namespace agentic_et1_tracker
