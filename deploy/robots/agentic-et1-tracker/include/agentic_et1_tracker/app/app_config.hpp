#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <cstddef>

#include "agentic_et1_tracker/http/server.hpp"
#include "agentic_et1_tracker/runtime/runtime_config.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {

struct PolicyConfig {
  std::string profile{"GeneralTracker"};
  std::string policy_dir{"config/policy/general_tracker"};
  std::string policy_file{"self_collision_footmesh_15k.onnx"};
  std::string deploy{"config/policy/general_tracker/params/deploy.yaml"};
  double fps{50.0};
};

struct ControlConfig {
  std::string startup_control{"FixStand"};
  std::string velocity_policy_dir{"config/policy/velocity/v0"};
  std::string velocity_policy_file{"policy.onnx"};
  std::string velocity_deploy{"config/policy/velocity/v0/params/deploy.yaml"};
  std::string fixstand_config{"config/posture/fixstand/v0/fixstand.yaml"};
  std::string passive_config{"config/posture/passive/v0/passive.yaml"};
};

struct ReferenceConfig {
  bool enabled{false};
};

struct AppConfig {
  HttpServerConfig http;
  RuntimeConfig runtime;
  TrkValidationConfig trk;
  std::string network{"lo"};
  int domain_id{0};
  std::size_t lowcmd_startup_preflight_ms{200};
  int mode_machine{1};
  double stop_hold_s{0.0};
  std::string idle_mode{"hold_current"};
  std::string lock_path;
  PolicyConfig policy;
  ControlConfig control;
  ReferenceConfig reference;
};

class ConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

AppConfig loadAppConfig(const std::filesystem::path& path);

}  // namespace agentic_et1_tracker
