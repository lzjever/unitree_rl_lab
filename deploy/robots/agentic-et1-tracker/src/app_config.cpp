#include "agentic_et1_tracker/app/app_config.hpp"

#include <cmath>
#include <optional>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "agentic_et1_tracker/policy/deploy_config.hpp"

#include "app_policy_path_guard.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr const char* kRootKey = "agentic_et1_tracker";
constexpr const char* kPolicyProfile = "GeneralTrackerCLN";
constexpr const char* kLegacyPolicyProfile = "GeneralTracker";
constexpr const char* kStartupFixStand = "FixStand";
constexpr const char* kStartupStandbyVelocity = "StandbyVelocity";
constexpr const char* kIdleModeHoldCurrent = "hold_current";
constexpr int kMinPort = 1;
constexpr int kMaxPort = 65535;
constexpr int kMinDomainId = 0;
constexpr int kMaxDomainId = 232;
constexpr double kMinRuntimeRate = 1.0;
constexpr double kMaxRuntimeRate = 1000.0;
constexpr const char* kInternalStandbyReference = "reference/standby/v0/standby_ref.trk";

ConfigError error(const std::string& message) { return ConfigError("config error: " + message); }

YAML::Node requiredRoot(const YAML::Node& root) {
  const YAML::Node section = root[kRootKey];
  if (!section || !section.IsMap()) {
    throw error("missing agentic_et1_tracker section");
  }
  return section;
}

template <typename T>
T scalarAs(const YAML::Node& node, const char* name) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& e) {
    std::ostringstream out;
    out << name << " has invalid value: " << e.what();
    throw error(out.str());
  }
}

std::string optionalString(const YAML::Node& section,
                           const char* key,
                           std::string current,
                           bool reject_empty = true) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  std::string value = scalarAs<std::string>(node, key);
  if (reject_empty && value.empty()) {
    throw error(std::string(key) + " must not be empty");
  }
  return value;
}

int optionalPort(const YAML::Node& section, const char* key, int current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const int value = scalarAs<int>(node, key);
  if (value < kMinPort || value > kMaxPort) {
    throw error(std::string(key) + " must be in the range 1..65535");
  }
  return value;
}

int optionalDomainId(const YAML::Node& section, const char* key, int current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const int value = scalarAs<int>(node, key);
  if (value < kMinDomainId || value > kMaxDomainId) {
    throw error(std::string(key) + " must be in the range 0..232");
  }
  return value;
}

std::size_t optionalPositiveSize(const YAML::Node& section,
                                 const char* key,
                                 std::size_t current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const long long value = scalarAs<long long>(node, key);
  if (value <= 0) {
    throw error(std::string(key) + " must be positive");
  }
  return static_cast<std::size_t>(value);
}

std::size_t optionalNonNegativeSize(const YAML::Node& section,
                                    const char* key,
                                    std::size_t current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const long long value = scalarAs<long long>(node, key);
  if (value < 0) {
    throw error(std::string(key) + " must be non-negative");
  }
  return static_cast<std::size_t>(value);
}

double optionalPositiveDouble(const YAML::Node& section, const char* key, double current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const double value = scalarAs<double>(node, key);
  if (!std::isfinite(value) || value <= 0.0) {
    throw error(std::string(key) + " must be positive");
  }
  return value;
}

double optionalRuntimeRate(const YAML::Node& section, const char* key, double current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const double value = scalarAs<double>(node, key);
  if (!std::isfinite(value) || value < kMinRuntimeRate || value > kMaxRuntimeRate) {
    std::ostringstream out;
    out << key << " must be in the range " << kMinRuntimeRate << ".." << kMaxRuntimeRate;
    throw error(out.str());
  }
  return value;
}

bool optionalBool(const YAML::Node& section, const char* key, bool current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  return scalarAs<bool>(node, key);
}

double optionalNonNegativeDouble(const YAML::Node& section, const char* key, double current) {
  const YAML::Node node = section[key];
  if (!node) {
    return current;
  }
  const double value = scalarAs<double>(node, key);
  if (!std::isfinite(value) || value < 0.0) {
    throw error(std::string(key) + " must be non-negative");
  }
  return value;
}

std::vector<std::filesystem::path> requiredMotionDirs(const YAML::Node& section) {
  const YAML::Node node = section["motion_dirs"];
  if (!node || !node.IsSequence() || node.size() == 0) {
    throw error("motion_dirs must contain at least one directory");
  }

  std::vector<std::filesystem::path> dirs;
  dirs.reserve(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string value = scalarAs<std::string>(node[i], "motion_dirs");
    if (value.empty()) {
      throw error("motion_dirs entries must not be empty");
    }
    std::filesystem::path dir(value);
    if (!dir.is_absolute()) {
      throw error("motion_dirs entries must be absolute");
    }
    dirs.emplace_back(std::move(dir));
  }
  return dirs;
}

std::filesystem::path configDirectory(const std::filesystem::path& config_path) {
  std::filesystem::path dir = config_path.parent_path();
  if (dir.empty()) {
    dir = ".";
  }
  if (dir.is_relative()) {
    dir = std::filesystem::absolute(dir);
  }
  return dir.lexically_normal();
}

void rejectEt1RuntimeDependency(const std::string& key, const std::filesystem::path& path) {
  if (app_internal::referencesEt1RuntimeDependency(path)) {
    throw error(std::string(key) +
                " must not point at the ET1 app deploy/robots/et1 config tree");
  }
}

bool pathIsWithin(const std::filesystem::path& child, const std::filesystem::path& parent) {
  const std::filesystem::path normalized_child = child.lexically_normal();
  const std::filesystem::path normalized_parent = parent.lexically_normal();
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

std::string resolveConfigPath(const char* key,
                              const std::string& value,
                              const std::filesystem::path& config_dir) {
  const std::filesystem::path path(value);
  const std::filesystem::path resolved =
      (path.is_absolute() ? path : config_dir / path).lexically_normal();
  rejectEt1RuntimeDependency(key, resolved);
  return resolved.string();
}

std::optional<std::filesystem::path> appConfigRootFromPolicyDir(
    const std::filesystem::path& policy_dir) {
  const std::filesystem::path normalized = policy_dir.lexically_normal();
  std::filesystem::path candidate;
  bool previous_was_config = false;
  for (const auto& part : normalized) {
    const std::string value = part.generic_string();
    if (previous_was_config && value == "policy") {
      return candidate.lexically_normal();
    }
    candidate /= part;
    previous_was_config = value == "config";
  }
  return std::nullopt;
}

std::string defaultInternalStandbyReference(const std::filesystem::path& policy_dir,
                                            const std::filesystem::path& config_dir) {
  if (const auto app_config_root = appConfigRootFromPolicyDir(policy_dir)) {
    return (*app_config_root / kInternalStandbyReference).lexically_normal().string();
  }
  return (config_dir / ControlConfig{}.standby_reference).lexically_normal().string();
}

void validateDeployPath(const std::string& policy_dir, const std::string& deploy) {
  const std::filesystem::path params_dir = std::filesystem::path(policy_dir) / "params";
  if (!pathIsWithin(deploy, params_dir)) {
    throw error("deploy must be under policy_dir/params");
  }
}

void validatePolicyModelPath(const std::string& key,
                             const std::string& policy_dir,
                             const std::string& policy_file) {
  rejectEt1RuntimeDependency(key,
                             std::filesystem::path(policy_dir) / "exported" / policy_file);
}

void validateLockPath(const std::string& value) {
  if (!value.empty() && !std::filesystem::path(value).is_absolute()) {
    throw error("lock_path must be absolute when set");
  }
}

void validatePolicyFile(const std::string& value) {
  const std::filesystem::path path(value);
  if (path.is_absolute() || path.has_parent_path() || value.find('/') != std::string::npos ||
      value.find('\\') != std::string::npos || value == "." || value == "..") {
    throw error("policy_file must be a file name only");
  }
}

void validatePolicyProfileDeployContract(const PolicyConfig& policy) {
  std::error_code ec;
  if (!std::filesystem::is_regular_file(policy.deploy, ec)) {
    return;
  }

  DeployConfig deploy_config;
  try {
    deploy_config = loadDeployConfig(policy.deploy);
  } catch (const DeployConfigError& err) {
    throw error(std::string("policy.deploy invalid: ") + err.what());
  }

  const bool profile_is_cln = policy.profile == kPolicyProfile;
  const bool deploy_is_cln =
      deploy_config.observation_contract == ObservationContract::GeneralTrackerCLN;
  if (profile_is_cln != deploy_is_cln) {
    throw error("policy.profile must match deploy observation contract");
  }
}

void validateReferenceKeys(const YAML::Node& reference) {
  for (const auto& item : reference) {
    const std::string key = scalarAs<std::string>(item.first, "reference key");
    if (key != "enabled") {
      throw error("reference." + key + " is unsupported");
    }
  }
}

}  // namespace

AppConfig loadAppConfig(const std::filesystem::path& path) {
  try {
    const std::filesystem::path config_dir = configDirectory(path);
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node section = requiredRoot(root);

    AppConfig config;
    config.http.host = optionalString(section, "bind", config.http.host);
    config.http.port = optionalPort(section, "port", config.http.port);
    config.http.thread_pool_size =
        optionalPositiveSize(section, "thread_pool_size", config.http.thread_pool_size);
    config.http = normalizeHttpServerConfig(std::move(config.http));

    config.network = optionalString(section, "network", config.network);
    config.domain_id = optionalDomainId(section, "domain_id", config.domain_id);
    config.lowcmd_startup_preflight_ms =
        optionalNonNegativeSize(section,
                                "lowcmd_startup_preflight_ms",
                                config.lowcmd_startup_preflight_ms);
    config.release_motion_mode_on_startup =
        optionalBool(section,
                     "release_motion_mode_on_startup",
                     config.release_motion_mode_on_startup);
    config.release_motion_mode_timeout_s =
        optionalPositiveDouble(section,
                               "release_motion_mode_timeout_s",
                               config.release_motion_mode_timeout_s);
    config.release_motion_mode_max_attempts =
        optionalPositiveSize(section,
                             "release_motion_mode_max_attempts",
                             config.release_motion_mode_max_attempts);
    config.release_motion_mode_retry_interval_ms =
        optionalNonNegativeSize(section,
                                "release_motion_mode_retry_interval_ms",
                                config.release_motion_mode_retry_interval_ms);
    if (const YAML::Node mode_machine = section["mode_machine"]) {
      config.mode_machine = scalarAs<int>(mode_machine, "mode_machine");
    }
    if (config.mode_machine != 0 && config.mode_machine != 1) {
      throw error("mode_machine must be 0 or 1");
    }

    const YAML::Node reference = section["reference"];
    if (reference) {
      if (!reference.IsMap()) {
        throw error("reference must be a map");
      }
      validateReferenceKeys(reference);
      config.reference.enabled =
          optionalBool(reference, "enabled", config.reference.enabled);
    }
    if (config.reference.enabled && config.mode_machine != 0) {
      throw error("reference.enabled requires mode_machine 0");
    }

    config.stop_hold_s = optionalNonNegativeDouble(section, "stop_hold_s", config.stop_hold_s);
    if (config.stop_hold_s != 0.0) {
      throw error("stop_hold_s must be 0.0");
    }
    config.runtime.stop_hold_s = config.stop_hold_s;
    config.idle_mode = optionalString(section, "idle_mode", config.idle_mode);
    if (config.idle_mode != kIdleModeHoldCurrent) {
      throw error("idle_mode must be hold_current");
    }
    config.passive_password =
        optionalString(section, "passive_password", config.passive_password);
    config.lock_path = optionalString(section, "lock_path", config.lock_path, false);
    validateLockPath(config.lock_path);

    config.runtime.queue_limit =
        optionalPositiveSize(section, "queue_limit", config.runtime.queue_limit);
    config.runtime.recent_limit =
        optionalPositiveSize(section, "recent_limit", config.runtime.recent_limit);
    config.runtime.hz = optionalRuntimeRate(section, "hz", config.runtime.hz);

    config.trk.allowlist_dirs = requiredMotionDirs(section);
    config.trk.max_duration_s =
        optionalPositiveDouble(section, "max_track_duration_s", config.trk.max_duration_s);

    const YAML::Node policy = section["policy"];
    if (policy) {
      if (!policy.IsMap()) {
        throw error("policy must be a map");
      }
      config.policy.profile = optionalString(policy, "profile", config.policy.profile);
      if (config.policy.profile != kPolicyProfile &&
          config.policy.profile != kLegacyPolicyProfile) {
        throw error("policy.profile must be GeneralTrackerCLN or GeneralTracker");
      }
      config.policy.policy_dir =
          optionalString(policy, "policy_dir", config.policy.policy_dir);
      config.policy.policy_file =
          optionalString(policy, "policy_file", config.policy.policy_file);
      config.policy.deploy = optionalString(policy, "deploy", config.policy.deploy);
      config.policy.fps = optionalRuntimeRate(policy, "fps", config.policy.fps);
    }

    const YAML::Node control = section["control"];
    bool standby_reference_explicit = false;
    if (control) {
      if (!control.IsMap()) {
        throw error("control must be a map");
      }
      config.control.startup_control =
          optionalString(control, "startup_control", config.control.startup_control);
      config.control.velocity_policy_dir =
          optionalString(control, "velocity_policy_dir", config.control.velocity_policy_dir);
      config.control.velocity_policy_file = optionalString(
          control, "velocity_policy_file", config.control.velocity_policy_file);
      config.control.velocity_deploy =
          optionalString(control, "velocity_deploy", config.control.velocity_deploy);
      config.control.fixstand_config =
          optionalString(control, "fixstand_config", config.control.fixstand_config);
      config.control.passive_config =
          optionalString(control, "passive_config", config.control.passive_config);
      standby_reference_explicit = static_cast<bool>(control["standby_reference"]);
      config.control.standby_reference =
          optionalString(control, "standby_reference", config.control.standby_reference);
    }
    if (config.control.startup_control != kStartupFixStand &&
        config.control.startup_control != kStartupStandbyVelocity) {
      throw error("control.startup_control must be FixStand or StandbyVelocity");
    }

    validatePolicyFile(config.policy.policy_file);
    validatePolicyFile(config.control.velocity_policy_file);
    config.policy.policy_dir =
        resolveConfigPath("policy_dir", config.policy.policy_dir, config_dir);
    config.policy.deploy = resolveConfigPath("deploy", config.policy.deploy, config_dir);
    validateDeployPath(config.policy.policy_dir, config.policy.deploy);
    validatePolicyProfileDeployContract(config.policy);
    validatePolicyModelPath("policy model", config.policy.policy_dir,
                            config.policy.policy_file);

    config.control.velocity_policy_dir = resolveConfigPath(
        "control.velocity_policy_dir", config.control.velocity_policy_dir, config_dir);
    config.control.velocity_deploy =
        resolveConfigPath("control.velocity_deploy", config.control.velocity_deploy,
                          config_dir);
    config.control.fixstand_config =
        resolveConfigPath("control.fixstand_config", config.control.fixstand_config,
                          config_dir);
    config.control.passive_config =
        resolveConfigPath("control.passive_config", config.control.passive_config,
                          config_dir);
    if (!standby_reference_explicit) {
      config.control.standby_reference =
          defaultInternalStandbyReference(config.policy.policy_dir, config_dir);
      rejectEt1RuntimeDependency("control.standby_reference",
                                 config.control.standby_reference);
    }
    config.control.standby_reference =
        resolveConfigPath("control.standby_reference", config.control.standby_reference,
                          config_dir);
    validateDeployPath(config.control.velocity_policy_dir, config.control.velocity_deploy);
    validatePolicyModelPath("control velocity model", config.control.velocity_policy_dir,
                            config.control.velocity_policy_file);

    config.trk.fps = config.policy.fps;
    return config;
  } catch (const ConfigError&) {
    throw;
  } catch (const YAML::Exception& e) {
    throw error(e.what());
  }
}

}  // namespace agentic_et1_tracker
