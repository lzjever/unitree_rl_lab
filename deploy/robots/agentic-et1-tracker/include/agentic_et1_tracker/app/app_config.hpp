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
  std::string profile{"GeneralTrackerDR3"};
  std::string policy_dir{"config/policy/general_tracker_dr3"};
  std::string policy_file{"DR3-all.onnx"};
  std::string deploy{"config/policy/general_tracker_dr3/params/deploy_fut_obs.yaml"};
  double fps{50.0};
};

struct ControlConfig {
  std::string startup_control{"FixStand"};
  std::string velocity_policy_dir{"config/policy/velocity/v0"};
  std::string velocity_policy_file{"policy.onnx"};
  std::string velocity_deploy{"config/policy/velocity/v0/params/deploy.yaml"};
  std::string fixstand_config{"config/posture/fixstand/v0/fixstand.yaml"};
  std::string passive_config{"config/posture/passive/v0/passive.yaml"};
  std::string standby_reference{"config/reference/standby/v0/standby_ref.trk"};
};

struct ReferenceConfig {
  bool enabled{false};
};

struct LocoUpperConfig {
  bool enabled{false};
  std::string policy_dir{"config/policy/loco_lower/et1_low"};
  std::string policy_file{"policy.onnx"};
  std::string deploy{"config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"};
  double default_radius_m{0.8};
  double max_radius_m{2.0};
  double radius_tolerance_m{0.05};
  double max_hold_s{10.0};
  bool strict_pose{false};
  std::size_t pose_fresh_timeout_ms{100};
  double pose_jump_reject_m{0.25};
  double max_lin_accel_mps2{0.4};
  double max_yaw_accel_radps2{0.5};
  std::size_t smoothing_window_frames{5};
  std::string limits{"config/limits/et1_upper_body/v0/limits.yaml"};
  std::string joint_map{"config/limits/et1_upper_body/v0/joint_map.yaml"};
};

struct AppConfig {
  HttpServerConfig http;
  RuntimeConfig runtime;
  TrkValidationConfig trk;
  std::string network{"lo"};
  int domain_id{0};
  std::size_t lowcmd_startup_preflight_ms{200};
  bool release_motion_mode_on_startup{true};
  double release_motion_mode_timeout_s{3.0};
  std::size_t release_motion_mode_max_attempts{3};
  std::size_t release_motion_mode_retry_interval_ms{500};
  int mode_machine{1};
  double stop_hold_s{0.0};
  double transition_duration_s{0.30};
  std::size_t transition_min_frames{2};
  double transition_duration_dt_tolerance_s{1.0e-9};
  double user_bridge_reduced_startup_hold_s{0.10};
  double transition_root_yaw_residual_limit_rad{0.05};
  std::string transition_contact_guard{"same_nonzero_contact"};
  double transition_max_velocity{250.0};
  double transition_max_acceleration{10000.0};
  double transition_max_jerk{1000000.0};
  std::string idle_mode{"hold_current"};
  std::string passive_password{"galaxy"};
  std::string lock_path;
  PolicyConfig policy;
  ControlConfig control;
  ReferenceConfig reference;
  LocoUpperConfig loco_upper;
};

class ConfigError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

AppConfig loadAppConfig(const std::filesystem::path& path);

}  // namespace agentic_et1_tracker
