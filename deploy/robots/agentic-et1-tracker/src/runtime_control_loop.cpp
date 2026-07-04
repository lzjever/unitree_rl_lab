#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include "agentic_et1_tracker/loco_upper/command_limits.hpp"
#include "agentic_et1_tracker/policy/observation_builder.hpp"
#include "agentic_et1_tracker/loco_upper/validator.hpp"
#include "agentic_et1_tracker/trk/reference_alignment.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/synthetic_transition.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr double kMaxTransitionDurationS = 5.0;
constexpr double kPolicyStartupHoldDurationS = 0.5;
constexpr std::size_t kStartupUpperBodyFirstPolicyJoint = 14;
constexpr std::size_t kStartupUpperBodyLastPolicyJointExclusive = 26;
constexpr double kLocoUpperRadiusSuppressMarginM = 0.02;
constexpr double kRawStartFallbackZeroEpsilon = 1.0e-4;
constexpr double kTransitionFrameCountEpsilon = 1.0e-9;
constexpr std::size_t kRootBodyIndex = 0;
constexpr std::size_t kBodyQuatDimensions = 4;
constexpr std::size_t kBodyPositionDimensions = 3;
constexpr std::size_t kBodyCount = TrkSchema::kBodyCount;
constexpr std::size_t kBodyPosFrameSize = kBodyCount * kBodyPositionDimensions;
constexpr std::size_t kBodyQuatFrameSize = kBodyCount * kBodyQuatDimensions;

bool isStartupOrWaitingError(ErrorCode error) {
  switch (error) {
    case ErrorCode::Ok:
    case ErrorCode::ServiceNotReady:
    case ErrorCode::RobotNotReady:
    case ErrorCode::RobotDisconnected:
    case ErrorCode::ModelNotReady:
      return true;
    default:
      return false;
  }
}

bool shouldLatchPassiveReason(const RobotReadinessStatus& readiness) {
  return readiness.err == ErrorCode::RobotBadOrientation &&
         readiness.block == "bad_orientation";
}

bool isGeneralTrackerRequest(const MotionRequest& request) {
  return request.executor == MotionExecutor::GeneralTracker;
}

bool isNonFinitePolicyMessage(const char* message) {
  if (message == nullptr) {
    return false;
  }
  return std::strstr(message, "non-finite") != nullptr ||
         std::strstr(message, "not finite") != nullptr ||
         std::strstr(message, "must be finite") != nullptr;
}

bool isFinitePlanarPosition(const std::array<float, 3>& position) {
  return std::isfinite(position[0]) && std::isfinite(position[1]);
}

bool isFiniteVec3(const std::array<float, 3>& values) {
  return std::isfinite(values[0]) && std::isfinite(values[1]) &&
         std::isfinite(values[2]);
}

bool isFiniteQuat(const std::array<float, 4>& values) {
  double norm_sq = 0.0;
  for (const float value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
    norm_sq += static_cast<double>(value) * static_cast<double>(value);
  }
  return norm_sq > static_cast<double>(std::numeric_limits<float>::epsilon()) *
                       static_cast<double>(std::numeric_limits<float>::epsilon());
}

std::optional<double> maxAbsDeltaFinite(const TrkArrayView<float>& source,
                                        const TrkArrayView<float>& target) {
  if (source.ptr == nullptr || target.ptr == nullptr || source.size == 0 ||
      source.size != target.size) {
    return std::nullopt;
  }
  double max_abs_delta = 0.0;
  for (std::size_t i = 0; i < source.size; ++i) {
    const double source_value = static_cast<double>(source.ptr[i]);
    const double target_value = static_cast<double>(target.ptr[i]);
    if (!std::isfinite(source_value) || !std::isfinite(target_value)) {
      return std::nullopt;
    }
    max_abs_delta = std::max(max_abs_delta, std::abs(target_value - source_value));
  }
  return max_abs_delta;
}

std::optional<double> maxAbsDeltaPrefixFinite(const TrkArrayView<float>& source,
                                              const TrkArrayView<float>& target,
                                              std::size_t count) {
  if (source.ptr == nullptr || target.ptr == nullptr || source.size < count ||
      target.size < count || count == 0) {
    return std::nullopt;
  }
  double max_abs_delta = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double source_value = static_cast<double>(source.ptr[i]);
    const double target_value = static_cast<double>(target.ptr[i]);
    if (!std::isfinite(source_value) || !std::isfinite(target_value)) {
      return std::nullopt;
    }
    max_abs_delta = std::max(max_abs_delta, std::abs(target_value - source_value));
  }
  return max_abs_delta;
}

std::optional<double> rootYawFromFrame(const TrkFrameView& frame) {
  if (frame.body_quat_w.ptr == nullptr ||
      frame.body_quat_w.size < (kRootBodyIndex + 1) * kBodyQuatDimensions) {
    return std::nullopt;
  }
  const float* q = frame.body_quat_w.ptr + kRootBodyIndex * kBodyQuatDimensions;
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < kBodyQuatDimensions; ++i) {
    const double value = static_cast<double>(q[i]);
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    norm_sq += value * value;
  }
  if (!std::isfinite(norm_sq) ||
      norm_sq <= static_cast<double>(std::numeric_limits<float>::epsilon()) *
                     static_cast<double>(std::numeric_limits<float>::epsilon())) {
    return std::nullopt;
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  const double w = static_cast<double>(q[0]) * inv_norm;
  const double x = static_cast<double>(q[1]) * inv_norm;
  const double y = static_cast<double>(q[2]) * inv_norm;
  const double z = static_cast<double>(q[3]) * inv_norm;
  const double yaw = std::atan2(2.0 * (w * z + x * y),
                               1.0 - 2.0 * (y * y + z * z));
  if (!std::isfinite(yaw)) {
    return std::nullopt;
  }
  return yaw;
}

double shortestAngleAbs(double lhs, double rhs) {
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

std::optional<bool> rootYawResidualWithinLimit(const TrkFrameView& source,
                                               const TrkFrameView& target,
                                               double limit_rad) {
  if (!std::isfinite(limit_rad) || limit_rad < 0.0) {
    return std::nullopt;
  }
  const std::optional<double> source_yaw = rootYawFromFrame(source);
  const std::optional<double> target_yaw = rootYawFromFrame(target);
  if (!source_yaw || !target_yaw) {
    return std::nullopt;
  }
  return shortestAngleAbs(*source_yaw, *target_yaw) <= limit_rad;
}

bool sameContactValue(const TrkArrayView<std::int64_t>& source,
                      const TrkArrayView<std::int64_t>& target) {
  return source.ptr != nullptr && target.ptr != nullptr && source.size == 1 &&
         target.size == 1 && source.ptr[0] == target.ptr[0];
}

bool rawStartGapNearZero(const TrkFrameView& source,
                         const TrkFrameView& target) {
  const std::optional<bool> yaw_zero =
      rootYawResidualWithinLimit(source, target, kRawStartFallbackZeroEpsilon);
  const std::optional<double> joint_gap =
      maxAbsDeltaFinite(source.joint_pos, target.joint_pos);
  const std::optional<double> joint_vel_gap =
      maxAbsDeltaFinite(source.joint_vel, target.joint_vel);
  const std::optional<double> root_gap =
      maxAbsDeltaPrefixFinite(source.body_pos_w, target.body_pos_w, 3);
  const std::optional<double> root_lin_vel_gap =
      maxAbsDeltaPrefixFinite(source.body_lin_vel_w, target.body_lin_vel_w, 3);
  const std::optional<double> root_ang_vel_gap =
      maxAbsDeltaPrefixFinite(source.body_ang_vel_w, target.body_ang_vel_w, 3);
  const std::optional<double> com_gap =
      maxAbsDeltaFinite(source.ref_com_rel_navi, target.ref_com_rel_navi);
  const std::optional<double> com_vel_gap =
      maxAbsDeltaFinite(source.ref_com_vel_navi, target.ref_com_vel_navi);
  return yaw_zero.has_value() && *yaw_zero &&
         joint_gap.has_value() && *joint_gap <= kRawStartFallbackZeroEpsilon &&
         joint_vel_gap.has_value() &&
         *joint_vel_gap <= kRawStartFallbackZeroEpsilon &&
         root_gap.has_value() && *root_gap <= kRawStartFallbackZeroEpsilon &&
         root_lin_vel_gap.has_value() &&
         *root_lin_vel_gap <= kRawStartFallbackZeroEpsilon &&
         root_ang_vel_gap.has_value() &&
         *root_ang_vel_gap <= kRawStartFallbackZeroEpsilon &&
         com_gap.has_value() && *com_gap <= kRawStartFallbackZeroEpsilon &&
         com_vel_gap.has_value() && *com_vel_gap <= kRawStartFallbackZeroEpsilon &&
         sameContactValue(source.left_foot_contact_state,
                          target.left_foot_contact_state) &&
         sameContactValue(source.right_foot_contact_state,
                          target.right_foot_contact_state);
}

bool allowStandbyRawStartAfterTransitionBuildFailure(const TrkFrameView& source,
                                                     const TrkFrameView& target,
                                                     double transition_duration_s,
                                                     double target_fps,
                                                     double root_yaw_residual_limit_rad,
                                                     const SyntheticTransitionLimits& limits) {
  (void)transition_duration_s;
  (void)target_fps;
  (void)root_yaw_residual_limit_rad;
  (void)limits;
  return rawStartGapNearZero(source, target);
}

bool copyFloatFrame(TrkFloatArray& out,
                    const TrkArrayView<float>& view,
                    std::vector<std::uint64_t> shape,
                    std::size_t frame_size) {
  if (view.ptr == nullptr || view.size != frame_size) {
    return false;
  }
  out.shape = std::move(shape);
  out.frame_size = frame_size;
  out.values.assign(view.ptr, view.ptr + frame_size);
  return true;
}

bool copyContactFrame(TrkContactArray& out,
                      const TrkArrayView<std::int64_t>& view,
                      std::vector<std::uint64_t> shape,
                      std::size_t frame_size) {
  if (view.ptr == nullptr || view.size != frame_size) {
    return false;
  }
  out.shape = std::move(shape);
  out.frame_size = frame_size;
  out.values.assign(view.ptr, view.ptr + frame_size);
  return true;
}

std::optional<TrkTrack> singleFrameTrackFromFrame(const TrkFrameView& frame,
                                                  double fps) {
  if (!std::isfinite(fps) || fps <= 0.0) {
    return std::nullopt;
  }

  TrkTrack track;
  track.metadata.frames = 1;
  track.metadata.duration_s = 0.0;
  track.metadata.fps = fps;
  track.metadata.version = TrkSchema::kVersion;
  track.metadata.array_count = TrkSchema::kRequiredArrays.size();

  if (!copyFloatFrame(track.joint_pos,
                      frame.joint_pos,
                      {1, TrkSchema::kJointDim},
                      TrkSchema::kJointDim) ||
      !copyFloatFrame(track.joint_vel,
                      frame.joint_vel,
                      {1, TrkSchema::kJointDim},
                      TrkSchema::kJointDim) ||
      !copyFloatFrame(track.body_pos_w,
                      frame.body_pos_w,
                      {1, TrkSchema::kBodyCount, 3},
                      kBodyPosFrameSize) ||
      !copyFloatFrame(track.body_quat_w,
                      frame.body_quat_w,
                      {1, TrkSchema::kBodyCount, 4},
                      kBodyQuatFrameSize) ||
      !copyFloatFrame(track.body_lin_vel_w,
                      frame.body_lin_vel_w,
                      {1, TrkSchema::kBodyCount, 3},
                      kBodyPosFrameSize) ||
      !copyFloatFrame(track.body_ang_vel_w,
                      frame.body_ang_vel_w,
                      {1, TrkSchema::kBodyCount, 3},
                      kBodyPosFrameSize) ||
      !copyContactFrame(track.left_foot_contact_state,
                        frame.left_foot_contact_state,
                        {1},
                        1) ||
      !copyContactFrame(track.right_foot_contact_state,
                        frame.right_foot_contact_state,
                        {1},
                        1) ||
      !copyFloatFrame(track.ref_com_rel_navi,
                      frame.ref_com_rel_navi,
                      {1, 3},
                      3) ||
      !copyFloatFrame(track.ref_com_vel_navi,
                      frame.ref_com_vel_navi,
                      {1, 3},
                      3)) {
    return std::nullopt;
  }

  return track;
}

std::optional<std::array<double, 2>> highstatePlanarPosition(
    const std::optional<HighStateSample>& high_state) {
  if (!high_state || !high_state->fresh || !isFinitePlanarPosition(high_state->position)) {
    return std::nullopt;
  }
  return std::array<double, 2>{
      static_cast<double>(high_state->position[0]),
      static_cast<double>(high_state->position[1]),
  };
}

std::optional<std::array<double, 2>> freshHighstatePlanarPosition(
    const std::optional<HighStateSample>& high_state,
    std::size_t max_age_ms) {
  if (!high_state || !high_state->fresh || high_state->age_ms > max_age_ms ||
      !isFinitePlanarPosition(high_state->position)) {
    return std::nullopt;
  }
  return std::array<double, 2>{
      static_cast<double>(high_state->position[0]),
      static_cast<double>(high_state->position[1]),
  };
}

double locoCommandDtS(double root_plan_dt_s, double active_fps, double runtime_hz) {
  if (std::isfinite(root_plan_dt_s) && root_plan_dt_s > 0.0) {
    return root_plan_dt_s;
  }
  if (std::isfinite(active_fps) && active_fps > 0.0) {
    return 1.0 / active_fps;
  }
  return 1.0 / std::max(1.0, runtime_hz);
}

LocoUpperPlanarSample planarSampleFromXY(const std::array<double, 2>& xy) {
  LocoUpperPlanarSample sample;
  sample.x = xy[0];
  sample.y = xy[1];
  return sample;
}

VelocityCommand clampBodyCommand(const LocoLowerCommandRanges& ranges,
                                 VelocityCommand command,
                                 bool& clamped) {
  const auto clamp_axis = [&clamped](float value, const LocoLowerRange& range) {
    const float bounded =
        std::clamp(value, static_cast<float>(range.min), static_cast<float>(range.max));
    if (bounded != value) {
      clamped = true;
    }
    return bounded;
  };
  command.vx = clamp_axis(command.vx, ranges.lin_vel_x);
  command.vy = clamp_axis(command.vy, ranges.lin_vel_y);
  command.yaw_rate = clamp_axis(command.yaw_rate, ranges.ang_vel_z);
  return command;
}

LocoUpperVelocityCommand bodyVelocityToWorldCommand(
    const LocoUpperVelocityCommand& command_body,
    double robot_yaw) {
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  LocoUpperVelocityCommand command_world = command_body;
  command_world.vx = cos_yaw * command_body.vx - sin_yaw * command_body.vy;
  command_world.vy = sin_yaw * command_body.vx + cos_yaw * command_body.vy;
  return command_world;
}

LocoUpperVelocityCommand bodyVelocityToWorldCommand(
    const VelocityCommand& command_body,
    double robot_yaw) {
  LocoUpperVelocityCommand command;
  command.vx = static_cast<double>(command_body.vx);
  command.vy = static_cast<double>(command_body.vy);
  command.yaw_rate = static_cast<double>(command_body.yaw_rate);
  return bodyVelocityToWorldCommand(command, robot_yaw);
}

ServiceHealth healthStateForSnapshot(const StatusSnapshot& snapshot) {
  if (snapshot.ready && snapshot.err == ErrorCode::Ok && snapshot.block.empty()) {
    return ServiceHealth::Ready;
  }
  if (snapshot.ctrl == ControllerState::Fault || snapshot.robot == RobotState::Fault) {
    return ServiceHealth::Error;
  }
  if (!snapshot.ready && isStartupOrWaitingError(snapshot.err)) {
    return ServiceHealth::Starting;
  }
  return ServiceHealth::Error;
}

HealthSnapshot healthFromSnapshot(const StatusSnapshot& snapshot) {
  HealthSnapshot health;
  health.state = healthStateForSnapshot(snapshot);
  health.mode = snapshot.mode;
  health.err = snapshot.err;
  health.block = snapshot.block;
  health.loco_upper = snapshot.loco_upper;
  return health;
}

LocoUpperJointValidationOptions jointValidationOptionsFromComposerConfig(
    const LocoUpperLowCmdComposerConfig& config) {
  LocoUpperJointValidationOptions options;
  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive;
       ++logical) {
    options.min_positions.push_back(static_cast<double>(config.upper_min_q.at(logical)));
    options.max_positions.push_back(static_cast<double>(config.upper_max_q.at(logical)));
    options.max_velocities.push_back(
        static_cast<double>(config.upper_max_vel_radps.at(logical)));
    options.max_accelerations.push_back(
        static_cast<double>(config.upper_max_accel_radps2.at(logical)));
  }
  return options;
}

LocoReason locoReasonFromValidationFailure(
    LocoUpperJointValidationFailureKind failure_kind) {
  switch (failure_kind) {
    case LocoUpperJointValidationFailureKind::Position:
      return LocoReason::UpperLimit;
    case LocoUpperJointValidationFailureKind::Velocity:
    case LocoUpperJointValidationFailureKind::Acceleration:
      return LocoReason::UpperDynamic;
    case LocoUpperJointValidationFailureKind::None:
    case LocoUpperJointValidationFailureKind::InvalidConfig:
    case LocoUpperJointValidationFailureKind::InvalidTrack:
      break;
  }
  return LocoReason::UpperLimit;
}

double yawFromLowState(const LowStateSample& low_state) {
  const float w = low_state.quat_wxyz[0];
  const float x = low_state.quat_wxyz[1];
  const float y = low_state.quat_wxyz[2];
  const float z = low_state.quat_wxyz[3];
  return std::atan2(2.0 * (static_cast<double>(w) * z +
                           static_cast<double>(x) * y),
                    1.0 - 2.0 * (static_cast<double>(y) * y +
                                  static_cast<double>(z) * z));
}

std::vector<float> upperTargetsFromFrame(const TrkFrameView& frame) {
  if (frame.joint_pos.ptr == nullptr || frame.joint_pos.size < kPolicyJointCount) {
    return {};
  }
  return std::vector<float>(frame.joint_pos.ptr,
                            frame.joint_pos.ptr + frame.joint_pos.size);
}

std::vector<float> upperTargetsFromCompiledFrame(
    const LocoUpperLogicalJointFrame& frame) {
  std::vector<float> out;
  out.reserve(kPolicyJointCount);
  for (const double value : frame) {
    out.push_back(static_cast<float>(value));
  }
  return out;
}

std::vector<float> upperTargetsFromLowState(
    const LocoUpperLowCmdComposerConfig& config,
    const LowStateSample& low_state,
    const std::vector<float>& fallback) {
  std::vector<float> targets = fallback;
  if (targets.size() < kPolicyJointCount) {
    targets.assign(kPolicyJointCount, 0.0F);
  }
  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive &&
       logical < config.logical_to_sdk.size();
       ++logical) {
    const int sdk_slot_raw = config.logical_to_sdk.at(logical);
    if (sdk_slot_raw >= 0 && sdk_slot_raw < static_cast<int>(kSdkMotorCount)) {
      targets.at(logical) =
          low_state.motors.at(static_cast<std::size_t>(sdk_slot_raw)).q;
    }
  }
  return targets;
}

std::size_t locoLowerSdkSlot(const LocoLowerDeployConfig& config,
                             std::size_t policy_index) {
  const int logical = config.joint_ids_map.at(policy_index);
  return static_cast<std::size_t>(
      config.sdk_joint_ids_map.at(static_cast<std::size_t>(logical)));
}

std::vector<float> lowerEntryTargetsFromLowState(
    const LocoLowerDeployConfig& config,
    const LowStateSample& low_state) {
  std::vector<float> targets;
  targets.reserve(kLocoLowerPolicyJointDim);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    targets.push_back(low_state.motors.at(locoLowerSdkSlot(config, i)).q);
  }
  return targets;
}

void applyLocoUpperLowerEntryInterpolation(const LocoLowerDeployConfig& config,
                                           const std::vector<float>& start_q,
                                           const Vec& policy_target_q,
                                           float alpha,
                                           LowCmdFrame& lower_frame) {
  if (start_q.size() != kLocoLowerPolicyJointDim ||
      policy_target_q.size() != kLocoLowerPolicyJointDim) {
    throw std::runtime_error("loco upper lower entry handoff size mismatch");
  }

  const float bounded_alpha = std::clamp(alpha, 0.0F, 1.0F);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const std::size_t sdk_slot = locoLowerSdkSlot(config, i);
    MotorCommand& motor = lower_frame.motors.at(sdk_slot);
    motor.q = start_q.at(i) +
              bounded_alpha * (policy_target_q.at(i) - start_q.at(i));
  }
}

std::vector<float> interpolateUpperTargets(const std::vector<float>& start,
                                           const std::vector<float>& target,
                                           float alpha) {
  std::vector<float> out = target;
  if (out.size() < kPolicyJointCount) {
    out.resize(kPolicyJointCount, 0.0F);
  }
  for (std::size_t i = 0; i < std::min(start.size(), out.size()); ++i) {
    out.at(i) = start.at(i) + alpha * (target.at(i) - start.at(i));
  }
  return out;
}

}  // namespace

RuntimeControlLoop::RuntimeControlLoop(RuntimeConfig config,
                                       RuntimeBridge& bridge,
                                       RuntimeStatusStore& status,
                                       TrkLoader loader,
                                       ReferenceFrameSink* reference_sink,
                                       std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink) {
  clearReference();
  publishSnapshot();
}

RuntimeControlLoop::RuntimeControlLoop(
    RuntimeConfig config,
    RuntimeBridge& bridge,
    RuntimeStatusStore& status,
    TrkLoader loader,
    RobotIO& robot_io,
    PolicyInference& policy,
    DeployConfig deploy_config,
    VelocityPolicyInference& velocity_policy,
    VelocityDeployConfig velocity_deploy_config,
    FixStandConfig fixstand_config,
    PassiveConfig passive_config,
    ControlMode startup_control,
    std::uint8_t expected_mode_machine,
    VelocityPolicyInference& loco_lower_policy,
    LocoLowerDeployConfig loco_lower_deploy_config,
    LocoUpperLowCmdComposerConfig loco_upper_composer_config,
    RuntimeMode mode,
    ReferenceFrameSink* reference_sink,
    std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink),
      robot_io_(&robot_io),
      policy_(&policy),
      deploy_config_(std::move(deploy_config)),
      velocity_policy_(&velocity_policy),
      velocity_deploy_config_(std::move(velocity_deploy_config)),
      loco_lower_policy_(&loco_lower_policy),
      loco_lower_deploy_config_(std::move(loco_lower_deploy_config)),
      loco_upper_composer_config_(std::move(loco_upper_composer_config)),
      fixstand_config_(std::move(fixstand_config)),
      passive_config_(std::move(passive_config)),
      expected_mode_machine_(expected_mode_machine),
      mode_(mode),
      post_stop_control_(startup_control) {
  runtime_state_.ready = false;
  runtime_state_.robot = RobotState::Disconnected;
  runtime_state_.err = ErrorCode::ServiceNotReady;
  runtime_state_.block = "runtime_not_started";
  fixstand_runner_.emplace(*fixstand_config_, expected_mode_machine_, config_.hz);
  velocity_runner_.emplace(*velocity_deploy_config_, expected_mode_machine_);
  if (startup_control == ControlMode::FixStand) {
    enterFixStandState();
  } else {
    enterVelocityState();
  }
  clearReference();
  publishSnapshot();
}

RuntimeControlLoop::RuntimeControlLoop(RuntimeConfig config,
                                       RuntimeBridge& bridge,
                                       RuntimeStatusStore& status,
                                       TrkLoader loader,
                                       RobotIO& robot_io,
                                       PolicyInference& policy,
                                       DeployConfig deploy_config,
                                       PassiveConfig passive_config,
                                       std::uint8_t expected_mode_machine,
                                       RuntimeMode mode,
                                       ReferenceFrameSink* reference_sink,
                                       std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink),
      robot_io_(&robot_io),
      policy_(&policy),
      deploy_config_(std::move(deploy_config)),
      passive_config_(std::move(passive_config)),
      expected_mode_machine_(expected_mode_machine),
      mode_(mode) {
  runtime_state_.ready = false;
  runtime_state_.robot = RobotState::Disconnected;
  runtime_state_.err = ErrorCode::ServiceNotReady;
  runtime_state_.block = "runtime_not_started";
  clearReference();
  publishSnapshot();
}

RuntimeControlLoop::RuntimeControlLoop(RuntimeConfig config,
                                       RuntimeBridge& bridge,
                                       RuntimeStatusStore& status,
                                       TrkLoader loader,
                                       RobotIO& robot_io,
                                       PolicyInference& policy,
                                       DeployConfig deploy_config,
                                       VelocityPolicyInference& velocity_policy,
                                       VelocityDeployConfig velocity_deploy_config,
                                       FixStandConfig fixstand_config,
                                       PassiveConfig passive_config,
                                       ControlMode startup_control,
                                       std::uint8_t expected_mode_machine,
                                       RuntimeMode mode,
                                       ReferenceFrameSink* reference_sink,
                                       std::shared_ptr<const TrkTrack> standby_track)
    : config_(config),
      bridge_(bridge),
      status_(status),
      loader_(std::move(loader)),
      standby_track_(std::move(standby_track)),
      reference_sink_(reference_sink),
      robot_io_(&robot_io),
      policy_(&policy),
      deploy_config_(std::move(deploy_config)),
      velocity_policy_(&velocity_policy),
      velocity_deploy_config_(std::move(velocity_deploy_config)),
      fixstand_config_(std::move(fixstand_config)),
      passive_config_(std::move(passive_config)),
      expected_mode_machine_(expected_mode_machine),
      mode_(mode),
      post_stop_control_(startup_control) {
  runtime_state_.ready = false;
  runtime_state_.robot = RobotState::Disconnected;
  runtime_state_.err = ErrorCode::ServiceNotReady;
  runtime_state_.block = "runtime_not_started";
  fixstand_runner_.emplace(*fixstand_config_, expected_mode_machine_, config_.hz);
  velocity_runner_.emplace(*velocity_deploy_config_, expected_mode_machine_);
  if (startup_control == ControlMode::FixStand) {
    enterFixStandState();
  } else {
    enterVelocityState();
  }
  clearReference();
  publishSnapshot();
}

RuntimeInternalState RuntimeControlLoop::internalStateForTest() const {
  return fsm_state_;
}

void RuntimeControlLoop::failNextTransitionStartForTest() {
  fail_next_transition_start_for_test_ = true;
}

void RuntimeControlLoop::faultNextTransitionStartForTest() {
  fault_next_transition_start_for_test_ = true;
}

void RuntimeControlLoop::forceNextRootYawResidualForTest(double residual_rad) {
  forced_next_root_yaw_residual_for_test_ = residual_rad;
}

std::optional<bool> RuntimeControlLoop::rootYawResidualAllowsBridge(
    const TrkFrameView& source,
    const TrkFrameView& target) {
  if (forced_next_root_yaw_residual_for_test_) {
    const double residual = *forced_next_root_yaw_residual_for_test_;
    forced_next_root_yaw_residual_for_test_.reset();
    if (!std::isfinite(residual)) {
      return std::nullopt;
    }
    if (!std::isfinite(config_.transition_root_yaw_residual_limit_rad) ||
        config_.transition_root_yaw_residual_limit_rad < 0.0) {
      return std::nullopt;
    }
    return std::abs(residual) <= config_.transition_root_yaw_residual_limit_rad;
  }
  return rootYawResidualWithinLimit(source,
                                    target,
                                    config_.transition_root_yaw_residual_limit_rad);
}

std::optional<TrkTrack> RuntimeControlLoop::controllableSourceTrack(
    const TrkFrameView& reference_frame,
    double fps,
    const LowStateSample& low_state,
    const std::optional<HighStateSample>& high_state,
    const TrkFrameView* contact_frame) const {
  if (!deploy_config_ ||
      deploy_config_->sdk_joint_ids_map.size() < kPolicyJointCount ||
      !isFiniteQuat(low_state.quat_wxyz) || !isFiniteVec3(low_state.gyro)) {
    return std::nullopt;
  }

  std::optional<TrkTrack> track = singleFrameTrackFromFrame(reference_frame, fps);
  if (!track || track->joint_pos.values.size() < kPolicyJointCount ||
      track->joint_vel.values.size() < kPolicyJointCount ||
      track->body_quat_w.values.size() < kBodyQuatDimensions ||
      track->body_ang_vel_w.values.size() < kBodyPositionDimensions) {
    return std::nullopt;
  }

  for (std::size_t policy_index = 0; policy_index < kPolicyJointCount;
       ++policy_index) {
    const int sdk_slot_raw = deploy_config_->sdk_joint_ids_map.at(policy_index);
    if (sdk_slot_raw < 0 ||
        sdk_slot_raw >= static_cast<int>(low_state.motors.size())) {
      return std::nullopt;
    }
    const MotorStateSample& motor =
        low_state.motors.at(static_cast<std::size_t>(sdk_slot_raw));
    if (!std::isfinite(motor.q) || !std::isfinite(motor.dq)) {
      return std::nullopt;
    }
    track->joint_pos.values.at(policy_index) = motor.q;
    track->joint_vel.values.at(policy_index) = motor.dq;
  }

  for (std::size_t i = 0; i < low_state.quat_wxyz.size(); ++i) {
    track->body_quat_w.values.at(kRootBodyIndex * kBodyQuatDimensions + i) =
        low_state.quat_wxyz.at(i);
  }
  for (std::size_t i = 0; i < low_state.gyro.size(); ++i) {
    track->body_ang_vel_w.values.at(kRootBodyIndex * kBodyPositionDimensions + i) =
        low_state.gyro.at(i);
  }

  if (high_state && high_state->fresh &&
      isFiniteVec3(high_state->position) &&
      isFiniteVec3(high_state->linear_velocity) &&
      isFiniteVec3(high_state->angular_velocity)) {
    if (track->body_pos_w.values.size() < kBodyPositionDimensions ||
        track->body_lin_vel_w.values.size() < kBodyPositionDimensions) {
      return std::nullopt;
    }
    for (std::size_t i = 0; i < high_state->position.size(); ++i) {
      track->body_pos_w.values.at(kRootBodyIndex * kBodyPositionDimensions + i) =
          high_state->position.at(i);
      track->body_lin_vel_w.values.at(kRootBodyIndex * kBodyPositionDimensions + i) =
          high_state->linear_velocity.at(i);
      track->body_ang_vel_w.values.at(kRootBodyIndex * kBodyPositionDimensions + i) =
          high_state->angular_velocity.at(i);
    }
  }

  if (contact_frame != nullptr &&
      (!copyContactFrame(track->left_foot_contact_state,
                         contact_frame->left_foot_contact_state,
                         {1},
                         1) ||
       !copyContactFrame(track->right_foot_contact_state,
                         contact_frame->right_foot_contact_state,
                         {1},
                         1))) {
    return std::nullopt;
  }

  return track;
}

std::optional<TrkTrack> RuntimeControlLoop::controllableStandbySourceTrack(
    const TrkFrameView& reference_frame,
    double fps,
    const TrkFrameView& standby_target_frame) {
  const std::optional<LowStateSample> entry_low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(entry_low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    failActiveReadiness(readiness);
    return std::nullopt;
  }
  applyReadiness(readiness);

  const std::optional<HighStateSample> high_state = readHighStateForStatus();
  return controllableSourceTrack(reference_frame,
                                 fps,
                                 *entry_low_state,
                                 high_state,
                                 &standby_target_frame);
}

void RuntimeControlLoop::tick() {
  if (fsm_state_ == RuntimeInternalState::Stopping) {
    consumeStoppingCommands();
    if (hasPolicyRuntime() && stopping_hold_ticks_remaining_ > 0) {
      if (!writeStoppingHold()) {
        publishSnapshot();
        return;
      }
      --stopping_hold_ticks_remaining_;
    }
    if (stop_to_idle_pending_ && stopping_hold_ticks_remaining_ == 0) {
      completeStoppingActive(MotionState::Stopped, ErrorCode::Ok);
      stop_to_idle_pending_ = false;
      stop_reason_ = StopReason::None;
      if (!hasControlRuntime()) {
        enterGeneralTrackerIdleState();
      } else if (post_stop_control_ == ControlMode::FixStand) {
        enterFixStandState();
      } else {
        enterGeneralTrackerIdleState();
      }
      acknowledgeDeferredPostStopControl();
    }
    publishSnapshot();
    return;
  }

  if (consumePendingCommands()) {
    publishSnapshot();
    return;
  }

  if (fsm_state_ == RuntimeInternalState::LocoUpperActive) {
    advanceLocoUpperActive();
    publishSnapshot();
    return;
  }

  if (fsm_state_ == RuntimeInternalState::GeneralTrackerActive ||
      fsm_state_ == RuntimeInternalState::GeneralTrackerTransition) {
    if (ctrl_ == ControllerState::Preparing) {
      completePreparing();
    } else {
      advanceActive();
    }
    publishSnapshot();
    return;
  }

  if (fsm_state_ == RuntimeInternalState::Passive) {
    runPassiveState();
  } else if (isMotionAcceptingState() && !waiting_.empty()) {
    startNext();
    publishSnapshot();
    return;
  } else if (hasIdleStartCandidate()) {
    refreshReadinessForPolicyRuntime();
    if (canStartIdle()) {
      startIdle();
    }
    publishSnapshot();
    return;
  } else if (hasControlRuntime() && isControlPublishingState() && waiting_.empty()) {
    publishControlIfReady();
  } else if (fsm_state_ == RuntimeInternalState::GeneralTrackerIdle && waiting_.empty()) {
    refreshReadinessForPolicyRuntime();
    publishIdleHoldIfReady();
  }
  publishSnapshot();
}

bool RuntimeControlLoop::consumePendingCommands() {
  while (auto command = bridge_.consumeNextCommand()) {
    switch (command->kind) {
      case CommandKind::Stop:
        handleStop(command->sequence, command->stop_requires_stopping);
        return true;
      case CommandKind::UrgentStop:
        handleUrgentStop(command->sequence);
        return true;
      case CommandKind::Passive:
        handleControl(ControlMode::Passive);
        acknowledgeConsumedControl(ControlMode::Passive, command->sequence);
        return true;
      case CommandKind::FixStand:
        handleControl(ControlMode::FixStand);
        acknowledgeConsumedControl(ControlMode::FixStand, command->sequence);
        if (fsm_state_ == RuntimeInternalState::Stopping ||
            (active_ && active_->executor == MotionExecutor::LocoUpper &&
             active_->state == MotionState::Stopping)) {
          return true;
        }
        break;
      case CommandKind::StandbyVelocity:
        handleControl(ControlMode::StandbyVelocity);
        acknowledgeConsumedControl(ControlMode::StandbyVelocity, command->sequence);
        if (fsm_state_ == RuntimeInternalState::Stopping ||
            (active_ && active_->executor == MotionExecutor::LocoUpper &&
             active_->state == MotionState::Stopping)) {
          return true;
        }
        break;
      case CommandKind::IdleConfig:
        handleIdleConfig(std::move(command->idle_motions));
        return true;
      case CommandKind::Interrupt:
        handleInterrupt(std::move(command->request));
        return fsm_state_ == RuntimeInternalState::Stopping ||
               (active_ && active_->executor == MotionExecutor::LocoUpper &&
                active_->state == MotionState::Stopping);
      case CommandKind::Queue:
        if (active_kind_ == ActiveKind::Idle) {
          MotionRequest request = std::move(command->request);
          if (isGeneralTrackerRequest(request) &&
              startTransitionFromIdleToUser(request)) {
            return true;
          }
          stopIdleActive();
          waiting_.push_back(std::move(request));
          break;
        }
        if (isBackgroundOwnedTransitionOrPlayback()) {
          if (isGeneralTrackerRequest(command->request)) {
            startTransitionFromCurrentReferenceToUser(std::move(command->request),
                                                      StopReason::None);
          } else {
            abortTransition(MotionState::Canceled, StopReason::None, ErrorCode::Ok);
            enterGeneralTrackerIdleState();
            waiting_.push_back(std::move(command->request));
          }
          return true;
        }
        waiting_.push_back(std::move(command->request));
        break;
    }
  }
  return false;
}

void RuntimeControlLoop::consumeStoppingCommands() {
  while (auto command = bridge_.consumeNextCommand()) {
    switch (command->kind) {
      case CommandKind::Stop:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::StandbyVelocity;
        clearDeferredPostStopControlAck();
        break;
      case CommandKind::UrgentStop:
        handleUrgentStop(command->sequence);
        break;
      case CommandKind::Passive:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::Passive;
        handleControl(ControlMode::Passive);
        acknowledgeConsumedControl(ControlMode::Passive, command->sequence);
        break;
      case CommandKind::FixStand:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::FixStand;
        deferPostStopControlAck(ControlMode::FixStand, command->sequence);
        break;
      case CommandKind::StandbyVelocity:
        cancelWaiting(StopReason::Stop, command->sequence);
        post_stop_control_ = ControlMode::StandbyVelocity;
        deferPostStopControlAck(ControlMode::StandbyVelocity, command->sequence);
        break;
      case CommandKind::IdleConfig:
        handleIdleConfig(std::move(command->idle_motions));
        break;
      case CommandKind::Interrupt:
        cancelWaiting(StopReason::Interrupt, command->sequence);
        waiting_.push_back(std::move(command->request));
        break;
      case CommandKind::Queue:
        waiting_.push_back(std::move(command->request));
        break;
    }
  }
}

void RuntimeControlLoop::acknowledgeConsumedControl(ControlMode mode,
                                                    std::uint64_t sequence) {
  consumed_control_acks_.push_back(ConsumedControlAck{mode, sequence});
}

void RuntimeControlLoop::deferPostStopControlAck(ControlMode mode,
                                                 std::uint64_t sequence) {
  deferred_post_stop_control_ack_ = ConsumedControlAck{mode, sequence};
}

void RuntimeControlLoop::clearDeferredPostStopControlAck() {
  if (!deferred_post_stop_control_ack_) {
    return;
  }
  status_.clearPendingControl(deferred_post_stop_control_ack_->mode,
                              deferred_post_stop_control_ack_->sequence);
  deferred_post_stop_control_ack_.reset();
}

void RuntimeControlLoop::acknowledgeDeferredPostStopControl() {
  if (!deferred_post_stop_control_ack_) {
    return;
  }
  acknowledgeConsumedControl(deferred_post_stop_control_ack_->mode,
                             deferred_post_stop_control_ack_->sequence);
  deferred_post_stop_control_ack_.reset();
}

void RuntimeControlLoop::handleStop(std::uint64_t sequence, bool requires_stopping) {
  cancelWaiting(StopReason::Stop, sequence);
  clearIdleConfig();
  if (fsm_state_ == RuntimeInternalState::Fault) {
    return;
  }
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    if (hasControlRuntime()) {
      enterGeneralTrackerIdleState();
    }
    return;
  }
  post_stop_control_ = ControlMode::StandbyVelocity;
  if (active_kind_ == ActiveKind::Transition ||
      (active_kind_ == ActiveKind::User && active_ &&
       active_->executor == MotionExecutor::GeneralTracker)) {
    const StandbyTransitionResult transition_result =
        startTransitionFromActiveToStandbyCancellation();
    if (transition_result == StandbyTransitionResult::Started ||
        transition_result == StandbyTransitionResult::SafetyTerminal) {
      return;
    }
  }
  if (active_kind_ == ActiveKind::Transition) {
    abortTransition();
    stop_reason_ = StopReason::Stop;
    enterStopping(StopReason::Stop);
    return;
  }
  if (fsm_state_ == RuntimeInternalState::Passive) {
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }
  if (active_ && active_kind_ == ActiveKind::User &&
      active_->executor == MotionExecutor::LocoUpper) {
    post_stop_control_ = ControlMode::StandbyVelocity;
    if (loco_upper_) {
      beginLocoUpperStopping(StopReason::Stop);
      return;
    }
    markActiveStopping(StopReason::Stop);
    stop_reason_ = StopReason::Stop;
    enterStopping(StopReason::Stop);
    return;
  }
  if (active_) {
    markActiveStopping(StopReason::Stop);
  } else if (fsm_state_ == RuntimeInternalState::FixStand && waiting_.empty()) {
    policy_runner_.reset();
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  } else if (!requires_stopping && waiting_.empty() &&
             (fsm_state_ == RuntimeInternalState::Velocity ||
              fsm_state_ == RuntimeInternalState::GeneralTrackerIdle)) {
    policy_runner_.reset();
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }
  stop_reason_ = StopReason::Stop;
  enterStopping(StopReason::Stop);
}

void RuntimeControlLoop::handleUrgentStop(std::uint64_t sequence) {
  cancelWaiting(StopReason::UrgentStop, sequence);
  clearDeferredPostStopControlAck();
  clearIdleConfig();
  const bool keep_fixstand =
      fsm_state_ == RuntimeInternalState::FixStand && active_kind_ == ActiveKind::None &&
      !active_ && waiting_.empty();
  post_stop_control_ =
      keep_fixstand ? ControlMode::FixStand : ControlMode::StandbyVelocity;

  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
  } else if (active_kind_ == ActiveKind::Transition) {
    abortTransition(MotionState::Canceled,
                    StopReason::UrgentStop,
                    ErrorCode::Ok);
  } else if (active_) {
    finishActive(MotionState::Canceled,
                 StopReason::UrgentStop,
                 ErrorCode::Ok);
  }

  active_.reset();
  active_kind_ = ActiveKind::None;
  active_track_.reset();
  loco_upper_.reset();
  policy_runner_.reset();
  transition_.reset();
  clearReference();

  if (fsm_state_ == RuntimeInternalState::Passive ||
      fsm_state_ == RuntimeInternalState::Fault) {
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }
  if (keep_fixstand) {
    enterFixStandState();
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }

  enterUrgentStopping();
}

void RuntimeControlLoop::handleControl(ControlMode mode) {
  if (mode == ControlMode::Passive) {
    cancelWaiting(StopReason::Stop);
    clearIdleConfig();
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    } else if (active_kind_ == ActiveKind::Transition) {
      abortTransition(MotionState::Canceled, StopReason::Stop, ErrorCode::Ok);
    } else if (active_) {
      finishActive(MotionState::Stopped, StopReason::Stop, ErrorCode::Ok);
    }
    active_.reset();
    active_track_.reset();
    loco_upper_.reset();
    policy_runner_.reset();
    clearReference();
    post_stop_control_ = ControlMode::Passive;
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    handleInternalEvent(RuntimeInternalEvent::SafetyPassive);
    return;
  }
  if (mode == ControlMode::StandbyVelocity && active_kind_ == ActiveKind::User &&
      active_ && active_->executor == MotionExecutor::LocoUpper) {
    cancelWaiting(StopReason::Stop);
    post_stop_control_ = ControlMode::StandbyVelocity;
    beginLocoUpperStopping(StopReason::Stop);
    return;
  }
  if (mode == ControlMode::FixStand && active_kind_ == ActiveKind::User &&
      active_ && active_->executor == MotionExecutor::LocoUpper) {
    cancelWaiting(StopReason::Stop);
    active_->loco.phase = LocoPhase::Stopped;
    finishActive(MotionState::Stopped, StopReason::Stop, ErrorCode::Ok);
    post_stop_control_ = ControlMode::FixStand;
    enterFixStandState();
    return;
  }
  if (mode == ControlMode::StandbyVelocity && active_kind_ == ActiveKind::User &&
      active_ && active_->state == MotionState::Holding) {
    cancelWaiting(StopReason::Stop);
    if (startTransitionFromHoldingToStandby()) {
      return;
    }
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "standby_transition_failed",
                        runtime_state_.low_ms);
    return;
  }
  if (fsm_state_ == RuntimeInternalState::Fault && mode != ControlMode::FixStand) {
    return;
  }
  if (fsm_state_ == RuntimeInternalState::Passive &&
      mode != ControlMode::FixStand) {
    return;
  }

  if (mode == ControlMode::FixStand || mode == ControlMode::StandbyVelocity) {
    cancelWaiting(StopReason::Stop);
  }
  post_stop_control_ = mode;
  if (mode == ControlMode::StandbyVelocity) {
    const StandbyTransitionResult transition_result =
        startTransitionFromActiveToStandbyCancellation();
    if (transition_result == StandbyTransitionResult::Started ||
        transition_result == StandbyTransitionResult::SafetyTerminal) {
      return;
    }
  }
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
  } else if (active_kind_ == ActiveKind::Transition) {
    abortTransition();
  }
  if (fsm_state_ == RuntimeInternalState::Fault) {
    active_.reset();
    active_track_.reset();
    loco_upper_.reset();
    policy_runner_.reset();
    clearReference();
    handleInternalEvent(RuntimeInternalEvent::FixStand);
    stop_reason_ = StopReason::None;
    stop_to_idle_pending_ = false;
    stopping_hold_ticks_remaining_ = 0;
    return;
  }

  if (active_) {
    markActiveStopping(StopReason::Stop);
    enterStopping(StopReason::Stop);
    return;
  }

  policy_runner_.reset();
  if (mode == ControlMode::FixStand) {
    handleInternalEvent(RuntimeInternalEvent::FixStand);
  } else {
    handleInternalEvent(RuntimeInternalEvent::Velocity);
  }
  stop_reason_ = StopReason::None;
  stop_to_idle_pending_ = false;
}

void RuntimeControlLoop::handleIdleConfig(std::vector<IdleMotion> motions) {
  if (motions.empty()) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    } else if (active_kind_ == ActiveKind::Transition && transition_ &&
               transition_->target_kind == TransitionTargetKind::Idle) {
      abortTransition(MotionState::Canceled, StopReason::None, ErrorCode::Ok);
      enterGeneralTrackerIdleState();
    }
    clearIdleConfig();
    return;
  }
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
  }
  idle_config_ = std::move(motions);
  idle_next_index_ = 0;
  if (isMotionAcceptingState()) {
    refreshReadinessForPolicyRuntime();
  }
}

void RuntimeControlLoop::handleInterrupt(MotionRequest request) {
  cancelWaiting(StopReason::Interrupt);
  if (active_kind_ == ActiveKind::Idle) {
    if (isGeneralTrackerRequest(request) && startTransitionFromIdleToUser(request)) {
      return;
    }
    stopIdleActive();
  }
  if (active_kind_ == ActiveKind::Transition) {
    const StopReason replaced_reason =
        isUserOwnedTransition() ? StopReason::Interrupt : StopReason::None;
    if (isGeneralTrackerRequest(request)) {
      startTransitionFromCurrentReferenceToUser(std::move(request), replaced_reason);
    } else {
      abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
      enterGeneralTrackerIdleState();
      waiting_.push_back(std::move(request));
    }
    return;
  }
  if (active_kind_ == ActiveKind::User && active_ &&
      active_->state == MotionState::Running &&
      active_->executor == MotionExecutor::GeneralTracker &&
      isGeneralTrackerRequest(request)) {
    const RunningInterruptHandoffResult handoff_result =
        tryStartRunningUserInterruptHandoff(request);
    if (handoff_result == RunningInterruptHandoffResult::Started ||
        handoff_result == RunningInterruptHandoffResult::SafetyTerminal) {
      return;
    }
  }
  if (active_kind_ == ActiveKind::User && active_ &&
      active_->state == MotionState::Holding &&
      active_->executor == MotionExecutor::GeneralTracker &&
      isGeneralTrackerRequest(request)) {
    const std::optional<TrkFrameView> source_frame =
        active_track_ ? active_track_->frame(active_->frame) : std::nullopt;
    const UserHandoffResult handoff_result =
        source_frame
            ? tryStartUserHandoffFromFrame(*source_frame,
                                           request,
                                           MotionState::Stopped,
                                           StopReason::Interrupt,
                                           ErrorCode::Ok,
                                           true)
            : UserHandoffResult::NoTransition;
    if (handoff_result == UserHandoffResult::Started ||
        handoff_result == UserHandoffResult::SafetyTerminal ||
        handoff_result == UserHandoffResult::TargetFailed) {
      return;
    }

    waiting_.push_back(std::move(request));
    markActiveStopping(StopReason::Interrupt);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterStopping(StopReason::Interrupt);
    return;
  }
  if (active_kind_ == ActiveKind::User && active_ &&
      active_->state == MotionState::Holding &&
      active_->executor == MotionExecutor::GeneralTracker &&
      !isGeneralTrackerRequest(request)) {
    finishActive(MotionState::Stopped, StopReason::Interrupt, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    waiting_.push_back(std::move(request));
    return;
  }
  waiting_.push_back(std::move(request));
  if (active_kind_ == ActiveKind::User && active_ &&
      active_->executor == MotionExecutor::LocoUpper) {
    post_stop_control_ = ControlMode::StandbyVelocity;
    beginLocoUpperStopping(StopReason::Interrupt);
    return;
  }
  if (active_kind_ == ActiveKind::User && active_ &&
      active_->state == MotionState::Holding) {
    return;
  }
  if (active_) {
    markActiveStopping(StopReason::Interrupt);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterStopping(StopReason::Interrupt);
  }
}

RuntimeControlLoop::RunningInterruptHandoffResult
RuntimeControlLoop::tryStartRunningUserInterruptHandoff(
    const MotionRequest& request) {
  if (active_kind_ != ActiveKind::User || !active_ ||
      active_->state != MotionState::Running ||
      active_->executor != MotionExecutor::GeneralTracker ||
      !active_track_ || !isGeneralTrackerRequest(request)) {
    return RunningInterruptHandoffResult::Fallback;
  }

  const std::optional<TrkFrameView> source_frame =
      active_track_->frame(active_->frame);
  if (!source_frame) {
    return RunningInterruptHandoffResult::Fallback;
  }

  const UserHandoffResult handoff_result =
      tryStartUserHandoffFromFrame(*source_frame,
                                   request,
                                   MotionState::Stopped,
                                   StopReason::Interrupt,
                                   ErrorCode::Ok,
                                   false);
  switch (handoff_result) {
    case UserHandoffResult::Started:
      return RunningInterruptHandoffResult::Started;
    case UserHandoffResult::SafetyTerminal:
      return RunningInterruptHandoffResult::SafetyTerminal;
    case UserHandoffResult::TargetFailed:
    case UserHandoffResult::NoTransition:
      return RunningInterruptHandoffResult::Fallback;
  }
  return RunningInterruptHandoffResult::Fallback;
}

RuntimeControlLoop::UserHandoffResult
RuntimeControlLoop::tryStartUserHandoffFromFrame(
    const TrkFrameView& source_frame,
    const MotionRequest& request,
    MotionState source_completion_state,
    StopReason source_completion_reason,
    ErrorCode source_completion_error,
    bool publish_target_failure) {
  auto publishTargetFailed = [this](MotionRequest failed, ErrorCode error) {
    failed.state = MotionState::Failed;
    failed.frame = 0;
    failed.err = error;
    failed.stop_reason = StopReason::None;
    failed.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(failed));
  };

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    applyReadiness(readiness);
    if (readiness.err != ErrorCode::Ok) {
      publishTargetFailed(request, readiness.err);
      failActiveReadiness(readiness);
      return UserHandoffResult::SafetyTerminal;
    }
  }

  MotionRequest target_request = request;
  target_request.state = MotionState::Queued;
  target_request.frame = 0;
  target_request.err = ErrorCode::Ok;
  target_request.stop_reason = StopReason::None;

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    if (publish_target_failure) {
      publishTargetFailed(std::move(target_request), toCoreErrorCode(loaded.code));
      return UserHandoffResult::TargetFailed;
    }
    return UserHandoffResult::NoTransition;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<const TrkTrack>(std::move(*loaded.track));

  ErrorCode transition_error = ErrorCode::InternalError;
  bool transition_build_rejected = false;
  std::optional<UserTransitionTracks> tracks =
      makeUserTransitionTracks(source_frame,
                               target_track,
                               transition_error,
                               transition_build_rejected);

  if (!tracks) {
    tracks = makeUserTransitionTracksFromControllableSource(source_frame,
                                                            target_track,
                                                            entry_low_state,
                                                            transition_error,
                                                            transition_build_rejected);
  }

  if (!tracks) {
    if (hasPolicyRuntime() && transition_error != ErrorCode::Ok &&
        transition_error != ErrorCode::InternalError) {
      RobotReadinessStatus readiness;
      readiness.robot = runtime_state_.robot;
      readiness.low_ms = runtime_state_.low_ms;
      readiness.block = runtime_state_.block;
      readiness.err = transition_error;
      publishTargetFailed(std::move(target_request), transition_error);
      failActiveReadiness(readiness);
      return UserHandoffResult::SafetyTerminal;
    }
    if (isSafetyTerminalState()) {
      publishTargetFailed(std::move(target_request), transition_error);
      return UserHandoffResult::SafetyTerminal;
    }
    return UserHandoffResult::NoTransition;
  }

  const MotionRequest source_to_complete = *active_;
  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(tracks->target_track);
  target.target_request = std::move(target_request);
  target.source_completion_state = source_completion_state;
  target.source_completion_reason = source_completion_reason;
  target.source_completion_error = source_completion_error;

  const bool started = startInternalTransition(std::move(tracks->transition_track),
                                               std::move(target),
                                               std::move(entry_low_state),
                                               publish_target_failure);
  if (!started) {
    if (last_transition_start_fatal_) {
      const TransitionStartFatal fatal = std::move(*last_transition_start_fatal_);
      last_transition_start_fatal_.reset();
      publishTargetFailed(request, fatal.error);
      failActiveWithFault(fatal.error, fatal.robot, fatal.block, fatal.low_ms);
      return UserHandoffResult::SafetyTerminal;
    }
    if (isSafetyTerminalState()) {
      publishTargetFailed(request, runtime_state_.err);
      return UserHandoffResult::SafetyTerminal;
    }
    return publish_target_failure ? UserHandoffResult::TargetFailed
                                  : UserHandoffResult::NoTransition;
  }

  MotionRequest completed = source_to_complete;
  completed.state = source_completion_state;
  completed.err = source_completion_error;
  completed.stop_reason = source_completion_reason;
  completed.ended_at = std::chrono::steady_clock::now();
  status_.publishRunStatus(toStatus(completed));
  post_stop_control_ = ControlMode::StandbyVelocity;
  return UserHandoffResult::Started;
}

void RuntimeControlLoop::cancelWaiting(StopReason reason) {
  for (auto& request : waiting_) {
    request.state = MotionState::Canceled;
    request.stop_reason = reason;
    request.err = ErrorCode::Ok;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
  }
  waiting_.clear();
}

void RuntimeControlLoop::cancelWaiting(StopReason reason, std::uint64_t sequence) {
  std::deque<MotionRequest> remaining;
  for (auto& request : waiting_) {
    if (request.sequence > sequence) {
      remaining.push_back(std::move(request));
      continue;
    }

    request.state = MotionState::Canceled;
    request.stop_reason = reason;
    request.err = ErrorCode::Ok;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
  }
  waiting_ = std::move(remaining);
}

void RuntimeControlLoop::failWaiting(ErrorCode error) {
  for (auto& request : waiting_) {
    request.state = MotionState::Failed;
    request.stop_reason = StopReason::None;
    request.err = error;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
  }
  waiting_.clear();
}

ControllerState RuntimeControlLoop::controllerStateForInternal(
    RuntimeInternalState state) const {
  switch (state) {
    case RuntimeInternalState::Passive:
      return ControllerState::Passive;
    case RuntimeInternalState::FixStand:
      return ControllerState::FixStand;
    case RuntimeInternalState::Velocity:
      return ControllerState::StandbyVelocity;
    case RuntimeInternalState::GeneralTrackerIdle:
      return hasControlRuntime() ? ControllerState::StandbyVelocity : ControllerState::Idle;
    case RuntimeInternalState::GeneralTrackerActive:
    case RuntimeInternalState::GeneralTrackerTransition:
    case RuntimeInternalState::LocoUpperActive:
      return ControllerState::Running;
    case RuntimeInternalState::Stopping:
      return ControllerState::Stopping;
    case RuntimeInternalState::Fault:
      return ControllerState::Fault;
  }
  return ControllerState::Fault;
}

void RuntimeControlLoop::enterInternalState(RuntimeInternalState state) {
  fsm_state_ = state;
  ctrl_ = controllerStateForInternal(state);
  switch (state) {
    case RuntimeInternalState::Passive:
      active_policy_ticks_until_next_ = 0;
      resetGeneralTrackerPolicyTiming();
      clearPolicyStartupHold();
      velocity_policy_ticks_until_next_ = 0;
      break;
    case RuntimeInternalState::FixStand:
      active_policy_ticks_until_next_ = 0;
      resetGeneralTrackerPolicyTiming();
      clearPolicyStartupHold();
      velocity_policy_ticks_until_next_ = 0;
      if (fixstand_runner_) {
        fixstand_runner_->reset();
      }
      break;
    case RuntimeInternalState::Velocity:
    case RuntimeInternalState::GeneralTrackerIdle:
      active_policy_ticks_until_next_ = 0;
      resetGeneralTrackerPolicyTiming();
      clearPolicyStartupHold();
      velocity_policy_ticks_until_next_ = 0;
      if (velocity_runner_) {
        velocity_runner_->reset();
      }
      break;
    case RuntimeInternalState::GeneralTrackerActive:
      clearPolicyStartupHold();
      active_policy_ticks_until_next_ = 0;
      resetGeneralTrackerPolicyTiming();
      active_first_advance_ = true;
      break;
    case RuntimeInternalState::GeneralTrackerTransition:
      clearPolicyStartupHold();
      active_policy_ticks_until_next_ = 0;
      resetGeneralTrackerPolicyTiming();
      active_first_advance_ = true;
      break;
    case RuntimeInternalState::LocoUpperActive:
      clearPolicyStartupHold();
      active_policy_ticks_until_next_ = 0;
      active_first_advance_ = true;
      break;
    case RuntimeInternalState::Stopping:
      clearPolicyStartupHold();
      break;
    case RuntimeInternalState::Fault:
      active_policy_ticks_until_next_ = 0;
      resetGeneralTrackerPolicyTiming();
      clearPolicyStartupHold();
      velocity_policy_ticks_until_next_ = 0;
      break;
  }
}

void RuntimeControlLoop::handleInternalEvent(RuntimeInternalEvent event) {
  switch (event) {
    case RuntimeInternalEvent::FixStand:
      enterFixStandState();
      break;
    case RuntimeInternalEvent::Velocity:
      enterVelocityState();
      break;
    case RuntimeInternalEvent::MotionRequest:
      enterTrackPreparingState();
      break;
    case RuntimeInternalEvent::SafetyPassive:
      enterInternalState(RuntimeInternalState::Passive);
      break;
    case RuntimeInternalEvent::Fault:
      enterInternalState(RuntimeInternalState::Fault);
      break;
  }
}

void RuntimeControlLoop::enterPassiveState(const RobotReadinessStatus& readiness) {
  if (readiness.err == ErrorCode::RobotBadOrientation) {
    failWaiting(readiness.err);
    clearIdleConfig();
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    } else if (active_kind_ == ActiveKind::Transition) {
      abortTransition(MotionState::Failed, StopReason::None, readiness.err);
    } else if (active_) {
      finishActive(MotionState::Failed, StopReason::None, readiness.err);
    }
    active_.reset();
    active_track_.reset();
    loco_upper_.reset();
    policy_runner_.reset();
    clearReference();
  }
  handleInternalEvent(RuntimeInternalEvent::SafetyPassive);
  if (shouldLatchPassiveReason(readiness)) {
    passive_reason_ = PassiveReason{readiness.err, readiness.block};
  }
  applyReadiness(readiness);
}

void RuntimeControlLoop::enterFixStandState() {
  passive_reason_.reset();
  enterInternalState(RuntimeInternalState::FixStand);
}

void RuntimeControlLoop::enterVelocityState() {
  enterInternalState(RuntimeInternalState::Velocity);
}

void RuntimeControlLoop::enterGeneralTrackerIdleState() {
  enterInternalState(RuntimeInternalState::GeneralTrackerIdle);
}

void RuntimeControlLoop::enterTrackPreparingState() {
  enterInternalState(RuntimeInternalState::GeneralTrackerActive);
  ctrl_ = ControllerState::Preparing;
}

void RuntimeControlLoop::enterTrackActiveState(
    const std::optional<LowStateSample>& entry_low_state,
    StartupHoldMode startup_hold_mode) {
  enterInternalState(RuntimeInternalState::GeneralTrackerActive);
  resetPolicyStartupHoldForActiveUser(entry_low_state, startup_hold_mode);
}

void RuntimeControlLoop::resetPolicyStartupHoldForActiveUser(
    const std::optional<LowStateSample>& entry_low_state,
    StartupHoldMode startup_hold_mode) {
  clearPolicyStartupHold();
  if (!hasPolicyRuntime() || active_kind_ != ActiveKind::User) {
    return;
  }

  const double duration_s =
      startup_hold_mode == StartupHoldMode::Reduced
          ? config_.user_bridge_reduced_startup_hold_s
          : kPolicyStartupHoldDurationS;
  policy_startup_hold_total_steps_ = policyStartupHoldPolicySteps(duration_s);
  policy_startup_hold_steps_remaining_ = policy_startup_hold_total_steps_;

  const std::optional<LowStateSample>& hold_low_state =
      entry_low_state ? entry_low_state : latest_low_state_;
  if (!hold_low_state || !active_track_ || !deploy_config_ ||
      deploy_config_->sdk_joint_ids_map.size() <
          kStartupUpperBodyLastPolicyJointExclusive) {
    return;
  }

  const std::optional<TrkFrameView> first_frame = active_track_->frame(0);
  if (!first_frame ||
      first_frame->joint_pos.size < kStartupUpperBodyLastPolicyJointExclusive) {
    return;
  }

  const std::size_t upper_body_count =
      std::min(kStartupUpperBodyLastPolicyJointExclusive, kPolicyJointCount) -
      kStartupUpperBodyFirstPolicyJoint;
  policy_startup_upper_body_start_q_.assign(upper_body_count, 0.0F);
  policy_startup_upper_body_target_q_.assign(upper_body_count, 0.0F);
  for (std::size_t i = 0; i < upper_body_count; ++i) {
    const std::size_t policy_joint = kStartupUpperBodyFirstPolicyJoint + i;
    const int sdk_slot_raw = deploy_config_->sdk_joint_ids_map.at(policy_joint);
    if (sdk_slot_raw < 0 || sdk_slot_raw >= static_cast<int>(kSdkMotorCount)) {
      continue;
    }
    const auto sdk_slot = static_cast<std::size_t>(sdk_slot_raw);
    policy_startup_upper_body_start_q_.at(i) =
        hold_low_state->motors.at(sdk_slot).q;
    policy_startup_upper_body_target_q_.at(i) =
        first_frame->joint_pos.ptr[policy_joint];
  }
}

void RuntimeControlLoop::clearPolicyStartupHold() {
  policy_startup_hold_total_steps_ = 0;
  policy_startup_hold_steps_remaining_ = 0;
  policy_startup_upper_body_start_q_.clear();
  policy_startup_upper_body_target_q_.clear();
}

void RuntimeControlLoop::applyPolicyStartupUpperBodyInterpolation(
    LowCmdFrame& frame) const {
  if (!deploy_config_ || active_kind_ != ActiveKind::User ||
      policy_startup_hold_steps_remaining_ == 0 ||
      policy_startup_hold_total_steps_ == 0 ||
      policy_startup_upper_body_start_q_.empty() ||
      policy_startup_upper_body_start_q_.size() !=
          policy_startup_upper_body_target_q_.size()) {
    return;
  }

  const std::size_t completed_steps =
      policy_startup_hold_total_steps_ - policy_startup_hold_steps_remaining_;
  const float alpha =
      policy_startup_hold_total_steps_ <= 1
          ? 1.0F
          : std::clamp(static_cast<float>(completed_steps) /
                           static_cast<float>(policy_startup_hold_total_steps_ - 1),
                       0.0F,
                       1.0F);

  for (std::size_t i = 0; i < policy_startup_upper_body_start_q_.size(); ++i) {
    const std::size_t policy_joint = kStartupUpperBodyFirstPolicyJoint + i;
    if (policy_joint >= deploy_config_->sdk_joint_ids_map.size()) {
      continue;
    }
    const int sdk_slot_raw = deploy_config_->sdk_joint_ids_map.at(policy_joint);
    if (sdk_slot_raw < 0 || sdk_slot_raw >= static_cast<int>(kSdkMotorCount)) {
      continue;
    }
    const float start_q = policy_startup_upper_body_start_q_.at(i);
    const float target_q = policy_startup_upper_body_target_q_.at(i);
    frame.motors.at(static_cast<std::size_t>(sdk_slot_raw)).q =
        start_q + alpha * (target_q - start_q);
  }
}

bool RuntimeControlLoop::isMotionAcceptingState() const {
  return fsm_state_ == RuntimeInternalState::Velocity ||
         fsm_state_ == RuntimeInternalState::GeneralTrackerIdle;
}

bool RuntimeControlLoop::isControlPublishingState() const {
  return fsm_state_ == RuntimeInternalState::FixStand ||
         fsm_state_ == RuntimeInternalState::Velocity ||
         fsm_state_ == RuntimeInternalState::GeneralTrackerIdle;
}

bool RuntimeControlLoop::isUserOwnedTransition() const {
  return active_kind_ == ActiveKind::Transition && transition_ &&
         transition_->target_kind == TransitionTargetKind::User;
}

bool RuntimeControlLoop::isBackgroundOwnedTransitionOrPlayback() const {
  return active_kind_ == ActiveKind::Transition && transition_ &&
         (transition_->target_kind == TransitionTargetKind::Idle ||
          transition_->target_kind == TransitionTargetKind::Standby);
}

void RuntimeControlLoop::runPassiveState() {
  if (!hasPolicyRuntime()) {
    return;
  }

  writePassiveDamping();
}

bool RuntimeControlLoop::writePassiveDamping() {
  if (!passive_config_) {
    return false;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const LowCmdOccupancy occupancy = robot_io_->lowCmdOccupancy();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, occupancy, expected_mode_machine_);
  if (readinessRequiresFault(readiness)) {
    enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    return false;
  }
  applyReadiness(readiness);

  if (!low_state.has_value()) {
    return false;
  }

  try {
    const LowCmdFrame frame =
        makePassiveLowCmdFrame(*passive_config_, *low_state, expected_mode_machine_);
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }
  return true;
}

void RuntimeControlLoop::startNext() {
  if (!waiting_.empty() && waiting_.front().executor == MotionExecutor::LocoUpper) {
    MotionRequest request = std::move(waiting_.front());
    waiting_.pop_front();
    startLocoUpper(std::move(request));
    return;
  }

  std::optional<LowStateSample> entry_low_state;
  std::optional<RobotReadinessStatus> readiness;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    applyReadiness(*readiness);
    if (readiness->err != ErrorCode::Ok) {
      if (readinessRequiresFault(*readiness)) {
        enterFault(readiness->err,
                   readiness->robot,
                   readiness->block,
                   readiness->low_ms);
      } else {
        enterPassiveState(*readiness);
      }
      return;
    }
  }

  MotionRequest request = std::move(waiting_.front());
  waiting_.pop_front();
  request.state = MotionState::Queued;
  request.frame = 0;
  request.err = ErrorCode::Ok;
  request.stop_reason = StopReason::None;
  if (shouldStartTransitionFromStandbyToUser()) {
    startTransitionFromStandbyToUser(std::move(request), std::move(entry_low_state));
    return;
  }

  active_ = std::move(request);
  active_kind_ = ActiveKind::User;
  idle_current_index_.reset();
  handleInternalEvent(RuntimeInternalEvent::MotionRequest);
  publishActive();
}

void RuntimeControlLoop::startLocoUpper(MotionRequest request) {
  auto failRequest = [this](MotionRequest failed,
                            ErrorCode error,
                            std::optional<RobotReadinessStatus> readiness = std::nullopt) {
    active_track_.reset();
    loco_upper_.reset();
    failed.state = MotionState::Failed;
    failed.frame = 0;
    failed.err = error;
    failed.stop_reason = StopReason::None;
    failed.loco.phase = LocoPhase::Failed;
    failed.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(failed));
    if (readiness) {
      if (readinessRequiresFault(*readiness)) {
        enterFault(readiness->err,
                   readiness->robot,
                   readiness->block,
                   readiness->low_ms);
      } else {
        enterPassiveState(*readiness);
      }
    }
  };

  request.state = MotionState::Queued;
  request.frame = 0;
  request.err = ErrorCode::Ok;
  request.stop_reason = StopReason::None;
  request.loco.max_radius_m = request.loco_options.max_radius_m;
  request.loco.phase = LocoPhase::Queued;

  if (!hasLocoUpperRuntime()) {
    failRequest(std::move(request), ErrorCode::ModelNotReady);
    return;
  }

  const std::optional<LowStateSample> entry_low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(entry_low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  applyReadiness(readiness);
  if (readiness.err != ErrorCode::Ok) {
    failRequest(std::move(request), readiness.err, readiness);
    return;
  }

  if (!prepareLocoUpperTrack(request)) {
    failRequest(std::move(request), request.err);
    return;
  }

  if (!loco_upper_ || !active_track_) {
    request.err = ErrorCode::InternalError;
    failRequest(std::move(request), request.err);
    return;
  }

  const std::optional<HighStateSample> start_high_state = readHighStateForStatus();
  const auto start_xy =
      config_.loco_upper_strict_pose
          ? freshHighstatePlanarPosition(start_high_state,
                                         config_.loco_upper_pose_fresh_timeout_ms)
          : highstatePlanarPosition(start_high_state);
  if (config_.loco_upper_strict_pose && !start_xy) {
    request.loco.reason = LocoReason::PoseMissing;
    failRequest(std::move(request), ErrorCode::SafetyLimitTriggered);
    return;
  }
  if (start_xy) {
    loco_upper_->highstate_start_xy = *start_xy;
    loco_upper_->last_highstate_xy = *start_xy;
    loco_upper_->has_highstate_start = true;
    loco_upper_->has_last_highstate_xy = true;
  }

  loco_upper_->entry_start_upper =
      upperTargetsFromLowState(*loco_upper_composer_config_,
                               *entry_low_state,
                               loco_upper_->first_upper);
  loco_upper_->lower_entry_start_q =
      lowerEntryTargetsFromLowState(*loco_lower_deploy_config_, *entry_low_state);
  loco_upper_->phase_total_ticks = ticksForPeriod(
      transitionDurationForUse().value_or(1.0 / std::max(1.0, config_.hz)));
  loco_upper_->phase_ticks_remaining = loco_upper_->phase_total_ticks;
  request.state = MotionState::Running;
  request.started_at = std::chrono::steady_clock::now();
  request.loco.phase = LocoPhase::Entry;
  active_ = std::move(request);
  active_kind_ = ActiveKind::User;
  idle_current_index_.reset();
  enterInternalState(RuntimeInternalState::LocoUpperActive);
  publishReferenceActive();
  publishActive();
}

bool RuntimeControlLoop::prepareLocoUpperTrack(MotionRequest& request) {
  TrkLoadResult loaded = loader_.load(request.path);
  if (!loaded.ok()) {
    request.err = toCoreErrorCode(loaded.code);
    return false;
  }

  request.frames = loaded.track->metadata.frames;
  request.fps = loaded.track->metadata.fps;
  request.duration_s = loaded.track->metadata.duration_s;
  active_track_ = std::make_shared<TrkTrack>(std::move(*loaded.track));

  LocoUpperCompileOptions compile_options;
  compile_options.max_radius_m =
      request.loco_options.max_radius_m > 0.0
          ? request.loco_options.max_radius_m
          : std::numeric_limits<double>::max();
  compile_options.root_options = LocoUpperPlannerOptions{};
  compile_options.command_limits =
      locoUpperCommandLimitsFromConfig(config_, *loco_lower_deploy_config_);
  compile_options.upper_joint_limits =
      jointValidationOptionsFromComposerConfig(*loco_upper_composer_config_);

  const LocoUpperCompileResult compiled =
      compileLocoUpperPlan(*active_track_, compile_options);
  if (!compiled.ok()) {
    request.err = compiled.failure_kind == LocoUpperCompileFailureKind::InvalidConfig ||
                          compiled.failure_kind ==
                              LocoUpperCompileFailureKind::InvalidOptions
                      ? ErrorCode::InternalError
                      : ErrorCode::TrkValidationFailed;
    request.loco.reason = compiled.failure_kind == LocoUpperCompileFailureKind::InvalidTrack
                              ? LocoReason::RootInvalid
                              : LocoReason::UpperLimit;
    return false;
  }

  LocoUpperRuntimeState runtime;
  runtime.root_plan = compiled.plan.root_plan;
  runtime.commands_body = compiled.plan.root_velocity_commands;
  runtime.upper_frames = compiled.plan.joint_pos_frames;
  runtime.radius_clamped = compiled.flags.radius_clamped;
  runtime.envelope_clamped = compiled.flags.envelope_clamped;
  runtime.upper_clamped = compiled.flags.upper_clamped;
  runtime.upper_rate_limited = compiled.flags.upper_rate_limited;
  runtime.lower_runner.emplace(*loco_lower_deploy_config_, expected_mode_machine_);
  resetLocoLowerPolicyTiming();
  if (compiled.plan.frame_count == 0 ||
      runtime.commands_body.size() != compiled.plan.frame_count ||
      runtime.upper_frames.size() != compiled.plan.frame_count) {
    request.err = ErrorCode::InternalError;
    return false;
  }
  runtime.first_upper = upperTargetsFromCompiledFrame(runtime.upper_frames.front());
  runtime.final_upper = upperTargetsFromCompiledFrame(runtime.upper_frames.back());
  if (runtime.first_upper.size() < kPolicyJointCount ||
      runtime.final_upper.size() < kPolicyJointCount) {
    request.err = ErrorCode::InternalError;
    return false;
  }
  request.loco.radius_clamped = runtime.radius_clamped;
  request.loco.envelope_clamped = runtime.envelope_clamped;
  request.loco.upper_clamped = runtime.upper_clamped;
  request.loco.upper_rate_limited = runtime.upper_rate_limited;
  request.loco_options.radius_clamped = runtime.radius_clamped;
  request.loco_options.envelope_clamped = runtime.envelope_clamped;
  request.loco_options.upper_clamped = runtime.upper_clamped;
  request.loco_options.upper_rate_limited = runtime.upper_rate_limited;
  request.loco.distance_m = 0.0;
  loco_upper_ = std::move(runtime);
  return true;
}

bool RuntimeControlLoop::canStartIdle() const {
  if (idle_config_.empty() || active_ || !waiting_.empty()) {
    return false;
  }
  if (!isMotionAcceptingState()) {
    return false;
  }
  if (hasPolicyRuntime()) {
    return runtime_state_.ready && runtime_state_.err == ErrorCode::Ok;
  }
  return true;
}

bool RuntimeControlLoop::hasIdleStartCandidate() const {
  return !idle_config_.empty() && !active_ && waiting_.empty() &&
         isMotionAcceptingState();
}

void RuntimeControlLoop::startIdle() {
  if (!canStartIdle()) {
    return;
  }

  if (idle_next_index_ >= idle_config_.size()) {
    idle_next_index_ = 0;
  }
  const std::size_t index = idle_next_index_;
  idle_next_index_ = (idle_next_index_ + 1) % idle_config_.size();
  const IdleMotion& motion = idle_config_.at(index);

  MotionRequest request;
  request.id.clear();
  request.path = motion.path;
  request.state = MotionState::Queued;
  request.frame = 0;
  request.frames = motion.track.frames;
  request.fps = motion.track.fps;
  request.duration_s = motion.track.duration_s;
  request.err = ErrorCode::Ok;
  request.stop_reason = StopReason::None;
  active_ = std::move(request);
  active_kind_ = ActiveKind::Idle;
  idle_current_index_ = index;
  handleInternalEvent(RuntimeInternalEvent::MotionRequest);
}

bool RuntimeControlLoop::startTransitionFromIdleToUser(MotionRequest target_request) {
  if (!isGeneralTrackerRequest(target_request)) {
    return false;
  }
  if (active_kind_ != ActiveKind::Idle || !active_ || !active_track_) {
    return false;
  }

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    return false;
  }

  auto failTarget = [this](MotionRequest request, ErrorCode error) {
    request.state = MotionState::Failed;
    request.frame = 0;
    request.err = error;
    request.stop_reason = StopReason::None;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    }
  };

  target_request.state = MotionState::Queued;
  target_request.frame = 0;
  target_request.err = ErrorCode::Ok;
  target_request.stop_reason = StopReason::None;

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    failTarget(std::move(target_request), toCoreErrorCode(loaded.code));
    return true;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<const TrkTrack>(std::move(*loaded.track));

  ErrorCode transition_error = ErrorCode::InternalError;
  bool transition_build_rejected = false;
  std::optional<UserTransitionTracks> tracks =
      makeUserTransitionTracks(*source_frame,
                               target_track,
                               transition_error,
                               transition_build_rejected);

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = latest_low_state_;
    if (!entry_low_state) {
      entry_low_state = readLowStateForStatus();
    }
  }

  if (!tracks) {
    tracks = makeUserTransitionTracksFromControllableSource(*source_frame,
                                                            target_track,
                                                            entry_low_state,
                                                            transition_error,
                                                            transition_build_rejected);
  }

  if (!tracks) {
    failTarget(std::move(target_request), transition_error);
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(tracks->target_track);
  target.target_request = std::move(target_request);

  const bool started = startInternalTransition(std::move(tracks->transition_track),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (!started) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    }
  }
  return true;
}

bool RuntimeControlLoop::shouldStartTransitionFromStandbyToUser() const {
  return hasControlRuntime() && ctrl_ == ControllerState::StandbyVelocity &&
         active_kind_ == ActiveKind::None && !active_ && !active_track_ &&
         !transition_;
}

void RuntimeControlLoop::startTransitionFromStandbyToUser(
    MotionRequest target_request,
    std::optional<LowStateSample> entry_low_state) {
  if (!isGeneralTrackerRequest(target_request)) {
    waiting_.push_front(std::move(target_request));
    return;
  }

  auto failTarget = [this](MotionRequest request, ErrorCode error) {
    request.state = MotionState::Failed;
    request.frame = 0;
    request.err = error;
    request.stop_reason = StopReason::None;
    request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(request));
    active_.reset();
    active_kind_ = ActiveKind::None;
    active_track_.reset();
    transition_.reset();
    policy_runner_.reset();
    idle_current_index_.reset();
    clearReference();
  };
  auto startWithoutTransition = [this](MotionRequest request) {
    active_ = std::move(request);
    active_kind_ = ActiveKind::User;
    idle_current_index_.reset();
    handleInternalEvent(RuntimeInternalEvent::MotionRequest);
    publishActive();
  };

  if (!standby_track_ || standby_track_->metadata.frames == 0) {
    failTarget(std::move(target_request), ErrorCode::InternalError);
    return;
  }

  const std::optional<TrkFrameView> source_frame =
      standby_track_->frame(standby_track_->metadata.frames - 1);
  if (!source_frame) {
    failTarget(std::move(target_request), ErrorCode::InternalError);
    return;
  }

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    failTarget(std::move(target_request), toCoreErrorCode(loaded.code));
    return;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  auto original_target_track =
      std::make_shared<const TrkTrack>(*target_track);
  ErrorCode controllable_fallback_error = ErrorCode::InternalError;
  auto tryControllableFallback = [this,
                                  &source_frame,
                                  &original_target_track,
                                  &entry_low_state,
                                  &target_request,
                                  &controllable_fallback_error]() -> bool {
    controllable_fallback_error = ErrorCode::InternalError;
    bool fallback_transition_build_rejected = false;
    std::optional<UserTransitionTracks> fallback_tracks =
        makeUserTransitionTracksFromControllableSource(
            *source_frame,
            original_target_track,
            entry_low_state,
            controllable_fallback_error,
            fallback_transition_build_rejected);
    if (!fallback_tracks) {
      return false;
    }

    PendingTransition target;
    target.target_kind = TransitionTargetKind::User;
    target.target_id = target_request.id;
    target.target_state = MotionState::Queued;
    target.target_track = std::move(fallback_tracks->target_track);
    target.target_request = std::move(target_request);

    const bool started =
        startInternalTransition(std::move(fallback_tracks->transition_track),
                                std::move(target),
                                std::move(entry_low_state));
    if (!started) {
      clearReference();
    }
    return true;
  };
  std::optional<TrkTrack> aligned_target_track =
      alignTrackRootPlanarPose(*target_track, *source_frame);
  if (!aligned_target_track) {
    if (tryControllableFallback()) {
      return;
    }
    failTarget(std::move(target_request), controllable_fallback_error);
    return;
  }
  target_track = std::make_shared<TrkTrack>(std::move(*aligned_target_track));

  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!target_frame) {
    if (tryControllableFallback()) {
      return;
    }
    failTarget(std::move(target_request), controllable_fallback_error);
    return;
  }
  const std::optional<bool> yaw_residual_ok =
      rootYawResidualAllowsBridge(*source_frame, *target_frame);
  if (!yaw_residual_ok || !*yaw_residual_ok) {
    if (tryControllableFallback()) {
      return;
    }
    failTarget(std::move(target_request), controllable_fallback_error);
    return;
  }

  const std::optional<double> transition_duration_s = transitionDurationForUse();
  if (!transition_duration_s) {
    if (tryControllableFallback()) {
      return;
    }
    failTarget(std::move(target_request), controllable_fallback_error);
    return;
  }

  const double transition_fps = transitionSampleFpsForTarget(*target_track);
  std::optional<TrkTrack> transition_track =
      makeSyntheticTransitionTrk(*source_frame,
                                 *target_frame,
                                 transition_fps,
                                 transitionOptionsForTarget(*target_track));
  if (!transition_track) {
    if (allowStandbyRawStartAfterTransitionBuildFailure(*source_frame,
                                                        *target_frame,
                                                        *transition_duration_s,
                                                        transition_fps,
                                                        config_.transition_root_yaw_residual_limit_rad,
                                                        transitionLimitsForUse())) {
      startWithoutTransition(std::move(target_request));
      return;
    }

    if (!tryControllableFallback()) {
      failTarget(std::move(target_request), controllable_fallback_error);
      return;
    }
    return;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(target_track);
  target.target_request = std::move(target_request);

  auto transition_track_ptr =
      std::make_shared<TrkTrack>(std::move(*transition_track));
  const bool started = startInternalTransition(std::move(transition_track_ptr),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (!started) {
    clearReference();
  }
}

std::optional<RuntimeControlLoop::UserTransitionTracks>
RuntimeControlLoop::makeUserTransitionTracks(
    const TrkFrameView& source_frame,
    std::shared_ptr<const TrkTrack> target_track,
    ErrorCode& error,
    bool& transition_build_rejected,
    bool allow_same_valid_contact) {
  transition_build_rejected = false;
  error = ErrorCode::InternalError;
  if (!target_track) {
    return std::nullopt;
  }

  std::optional<TrkTrack> aligned_target_track =
      alignTrackRootPlanarPose(*target_track, source_frame);
  if (!aligned_target_track) {
    return std::nullopt;
  }
  auto aligned_target_track_ptr =
      std::make_shared<const TrkTrack>(std::move(*aligned_target_track));
  const std::optional<TrkFrameView> target_frame =
      aligned_target_track_ptr->frame(0);
  if (!target_frame) {
    return std::nullopt;
  }

  const std::optional<bool> yaw_residual_ok =
      rootYawResidualAllowsBridge(source_frame, *target_frame);
  if (!yaw_residual_ok || !*yaw_residual_ok) {
    return std::nullopt;
  }

  const std::optional<double> transition_duration_s = transitionDurationForUse();
  if (!transition_duration_s) {
    return std::nullopt;
  }

  const double transition_fps =
      transitionSampleFpsForTarget(*aligned_target_track_ptr);
  SyntheticTransitionOptions options =
      allow_same_valid_contact
          ? controlledTransitionOptionsForTarget(*aligned_target_track_ptr)
          : transitionOptionsForTarget(*aligned_target_track_ptr);
  std::optional<TrkTrack> transition_track =
      makeSyntheticTransitionTrk(source_frame,
                                 *target_frame,
                                 transition_fps,
                                 options);
  if (!transition_track) {
    transition_build_rejected = true;
    return std::nullopt;
  }

  UserTransitionTracks tracks;
  tracks.transition_track =
      std::make_shared<const TrkTrack>(std::move(*transition_track));
  tracks.target_track = std::move(aligned_target_track_ptr);
  error = ErrorCode::Ok;
  return tracks;
}

std::optional<RuntimeControlLoop::UserTransitionTracks>
RuntimeControlLoop::makeUserTransitionTracksFromControllableSource(
    const TrkFrameView& reference_frame,
    std::shared_ptr<const TrkTrack> target_track,
    std::optional<LowStateSample>& entry_low_state,
    ErrorCode& error,
    bool& transition_build_rejected) {
  transition_build_rejected = false;
  if (!hasPolicyRuntime() || !target_track) {
    return std::nullopt;
  }

  std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  applyReadiness(readiness);
  if (readiness.err != ErrorCode::Ok || !low_state) {
    error = readiness.err == ErrorCode::Ok ? ErrorCode::RobotNotReady
                                           : readiness.err;
    return std::nullopt;
  }

  const std::optional<TrkFrameView> target_contact_frame = target_track->frame(0);
  if (!target_contact_frame) {
    error = ErrorCode::InternalError;
    return std::nullopt;
  }

  const std::optional<HighStateSample> high_state = readHighStateForStatus();
  std::optional<TrkTrack> source_track =
      controllableSourceTrack(reference_frame,
                              transitionSampleFpsForTarget(*target_track),
                              *low_state,
                              high_state,
                              &*target_contact_frame);
  if (!source_track) {
    error = ErrorCode::InternalError;
    return std::nullopt;
  }

  const std::optional<TrkFrameView> source_frame = source_track->frame(0);
  if (!source_frame) {
    error = ErrorCode::InternalError;
    return std::nullopt;
  }

  std::optional<UserTransitionTracks> tracks =
      makeUserTransitionTracks(*source_frame,
                               std::move(target_track),
                               error,
                               transition_build_rejected,
                               true);
  if (tracks) {
    entry_low_state = std::move(low_state);
  }
  return tracks;
}

void RuntimeControlLoop::completePreparing() {
  if (!active_) {
    active_track_.reset();
    clearReference();
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  std::optional<LowStateSample> entry_low_state;
  std::optional<RobotReadinessStatus> readiness;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    applyReadiness(*readiness);
    if (readiness->err != ErrorCode::Ok) {
      if (active_kind_ == ActiveKind::Idle) {
        stopIdleActive();
        if (readinessRequiresFault(*readiness)) {
          enterFault(readiness->err,
                     readiness->robot,
                     readiness->block,
                     readiness->low_ms);
        } else {
          enterPassiveState(*readiness);
        }
        return;
      }
      if (readinessRequiresFault(*readiness)) {
        failActiveWithFault(readiness->err,
                            readiness->robot,
                            readiness->block,
                            readiness->low_ms);
        return;
      }

      MotionRequest request = std::move(*active_);
      request.state = MotionState::Queued;
      request.frame = 0;
      request.err = ErrorCode::Ok;
      request.stop_reason = StopReason::None;
      active_.reset();
      active_track_.reset();
      loco_upper_.reset();
      policy_runner_.reset();
      clearReference();
      waiting_.push_front(std::move(request));
      status_.publishRunStatus(toStatus(waiting_.front()));
      enterPassiveState(*readiness);
      return;
    }
  }

  TrkLoadResult loaded = loader_.load(active_->path);
  if (!loaded.ok()) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
      return;
    }
    active_->state = MotionState::Failed;
    active_->frame = 0;
    active_->err = toCoreErrorCode(loaded.code);
    active_->stop_reason = StopReason::None;
    active_->ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(*active_));
    active_.reset();
    active_track_.reset();
    loco_upper_.reset();
    clearReference();
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  const std::size_t loaded_frames = loaded.track->metadata.frames;
  const double loaded_duration_s = loaded.track->metadata.duration_s;
  active_->frames = loaded_frames;
  active_->fps = loaded.track->metadata.fps;
  active_->duration_s = loaded_duration_s;
  auto track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  active_track_ = track;
  if (hasPolicyRuntime()) {
    try {
      policy_runner_.emplace(*deploy_config_,
                             track,
                             *entry_low_state,
                             expected_mode_machine_);
    } catch (const std::exception&) {
      if (active_kind_ == ActiveKind::Idle) {
        stopIdleActive();
        post_stop_control_ = ControlMode::StandbyVelocity;
        enterFault(ErrorCode::ModelInferenceFailed,
                   RobotState::Fault,
                   "policy_inference_failed",
                   readiness ? readiness->low_ms : std::nullopt);
        return;
      }
      active_->state = MotionState::Failed;
      active_->frame = 0;
      active_->err = ErrorCode::ModelInferenceFailed;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 readiness ? readiness->low_ms : std::nullopt);
      return;
    }
  }

  active_->state = MotionState::Running;
  active_->frame = 0;
  active_->err = ErrorCode::Ok;
  active_->stop_reason = StopReason::None;
  active_->started_at = std::chrono::steady_clock::now();
  enterTrackActiveState(entry_low_state);
  publishReferenceActive();
  publishActive();
}

void RuntimeControlLoop::advanceActive() {
  if (active_kind_ == ActiveKind::Transition) {
    if (hasPolicyRuntime()) {
      advanceTransitionWithPolicy();
    } else {
      advanceTransition();
    }
    return;
  }

  if (hasPolicyRuntime()) {
    advanceActiveWithPolicy();
    return;
  }

  if (!active_) {
    active_track_.reset();
    clearReference();
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    transition_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  if (active_kind_ == ActiveKind::User &&
      active_->state == MotionState::Holding) {
    advanceHolding();
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_first_advance_ = false;
    active_->frame = 0;
  } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
    ++active_->frame;
  }

  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    const std::size_t last_frame = active_->frames == 0 ? 0 : active_->frames - 1;
    if (active_->frame < last_frame) {
      active_->frame = last_frame;
      publishReferenceActive();
      publishActive();
      return;
    }
    active_->frame = last_frame;
    publishReferenceActive();
    if (active_kind_ == ActiveKind::User && active_->hold) {
      active_->state = MotionState::Holding;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      publishActive();
      return;
    }
    if (active_kind_ == ActiveKind::User &&
        startTransitionFromCompletedUserToNextUser()) {
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToIdle()) {
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToStandby()) {
      return;
    }
    if (active_kind_ == ActiveKind::Idle && startTransitionFromCompletedIdleToIdle()) {
      return;
    }
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  publishActive();
  publishReferenceActive();
}

void RuntimeControlLoop::advanceLocoUpperActive() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->executor != MotionExecutor::LocoUpper || !loco_upper_) {
    finishActive(MotionState::Failed, StopReason::None, ErrorCode::InternalError);
    enterGeneralTrackerIdleState();
    return;
  }

  switch (active_->loco.phase) {
    case LocoPhase::Entry:
      advanceLocoUpperEntry();
      break;
    case LocoPhase::Motion:
      advanceLocoUpperMotion();
      break;
    case LocoPhase::Exit:
      advanceLocoUpperExit();
      break;
    case LocoPhase::Holding:
      advanceLocoUpperHolding();
      break;
    case LocoPhase::Stopping:
      advanceLocoUpperStopping();
      break;
    default:
      finishActive(MotionState::Failed, StopReason::None, ErrorCode::InternalError);
      enterGeneralTrackerIdleState();
      break;
  }
}

void RuntimeControlLoop::advanceLocoUpperEntry() {
  if (!active_ || !loco_upper_) {
    return;
  }

  const std::size_t total = std::max<std::size_t>(1, loco_upper_->phase_total_ticks);
  const std::size_t remaining = loco_upper_->phase_ticks_remaining;
  const std::size_t completed = total > remaining ? total - remaining : 0;
  const float alpha =
      total <= 1 ? 1.0F
                 : std::clamp(static_cast<float>(completed) /
                                  static_cast<float>(total - 1),
                              0.0F,
                              1.0F);
  if (!writeLocoUpperStep({0.0F, 0.0F, 0.0F},
                          interpolateUpperTargets(loco_upper_->entry_start_upper,
                                                   loco_upper_->first_upper,
                                                   alpha),
                          alpha)) {
    return;
  }
  publishActive();
  publishReferenceActive();
  if (loco_upper_->phase_ticks_remaining > 0) {
    --loco_upper_->phase_ticks_remaining;
  }
  if (loco_upper_->phase_ticks_remaining == 0) {
    active_->frame = 0;
    setLocoPhase(LocoPhase::Motion);
  }
}

void RuntimeControlLoop::advanceLocoUpperMotion() {
  if (!active_ || !loco_upper_) {
    return;
  }
  const bool frame_due =
      consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks());
  if (frame_due) {
    if (active_first_advance_) {
      active_->started_at = std::chrono::steady_clock::now();
      active_->frame = 0;
      active_first_advance_ = false;
    } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
      ++active_->frame;
    }

    if (!prepareLocoUpperFrameStep(active_->frame, false)) {
      return;
    }
  } else if (loco_upper_->current_upper.size() < kPolicyJointCount &&
             !prepareLocoUpperFrameStep(active_->frame, false)) {
    return;
  }

  if (!writeLocoUpperStep(loco_upper_->current_command,
                          loco_upper_->current_upper)) {
    return;
  }
  publishReferenceActive();
  publishActive();

  if (frame_due &&
      (active_->frames == 0 || active_->frame + 1 >= active_->frames)) {
    active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
    publishReferenceActive();
    if (active_->hold) {
      active_->state = MotionState::Holding;
      loco_upper_->hold_ticks_remaining = ticksForPeriod(config_.loco_upper_max_hold_s);
      setLocoPhase(LocoPhase::Holding);
      publishActive();
      return;
    }
    beginLocoUpperExit(LocoReason::None);
    publishActive();
  }
}

void RuntimeControlLoop::advanceLocoUpperExit() {
  if (!active_ || !loco_upper_) {
    return;
  }
  if (!writeLocoUpperUpperTransitionStep()) {
    return;
  }
  publishReferenceActive();
  publishActive();
  if (loco_upper_->phase_ticks_remaining > 0) {
    --loco_upper_->phase_ticks_remaining;
  }
  if (loco_upper_->phase_ticks_remaining == 0) {
    finishLocoUpperDone();
  }
}

void RuntimeControlLoop::advanceLocoUpperHolding() {
  if (!active_ || !loco_upper_) {
    return;
  }
  if (!waiting_.empty()) {
    beginLocoUpperExit(LocoReason::None);
    publishActive();
    return;
  }
  if (loco_upper_->hold_ticks_remaining == 0) {
    beginLocoUpperExit(LocoReason::HoldTimeout);
    publishActive();
    return;
  }
  if (!writeLocoUpperFinalHold(true)) {
    return;
  }
  if (loco_upper_->hold_ticks_remaining > 0) {
    --loco_upper_->hold_ticks_remaining;
  }
  active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
  publishReferenceActive();
  publishActive();
}

void RuntimeControlLoop::advanceLocoUpperStopping() {
  if (!active_ || !loco_upper_) {
    return;
  }
  if (!writeLocoUpperUpperTransitionStep()) {
    return;
  }
  publishReferenceActive();
  publishActive();
  if (loco_upper_->phase_ticks_remaining > 0) {
    --loco_upper_->phase_ticks_remaining;
  }
  if (loco_upper_->phase_ticks_remaining == 0) {
    completeLocoUpperStopped();
  }
}

void RuntimeControlLoop::beginLocoUpperExit(LocoReason reason) {
  if (!active_ || !loco_upper_) {
    return;
  }
  loco_upper_->phase_total_ticks = ticksForPeriod(
      transitionDurationForUse().value_or(1.0 / std::max(1.0, config_.hz)));
  loco_upper_->phase_ticks_remaining = loco_upper_->phase_total_ticks;
  loco_upper_->transition_start_upper = currentLocoUpperTargets();
  loco_upper_->transition_target_upper = locoUpperStandbyTargets();
  active_->loco.reason = reason;
  setLocoPhase(LocoPhase::Exit);
}

bool RuntimeControlLoop::requireStrictPoseForLocoUpper(
    std::array<double, 2>& estimate_xy) {
  if (!active_ || !loco_upper_) {
    return false;
  }
  const auto high_xy =
      freshHighstatePlanarPosition(readHighStateForStatus(),
                                   config_.loco_upper_pose_fresh_timeout_ms);
  if (!high_xy) {
    failLocoUpperSafety(LocoReason::PoseMissing, "pose_missing");
    return false;
  }
  if (loco_upper_->has_last_highstate_xy) {
    const double dx = (*high_xy)[0] - loco_upper_->last_highstate_xy[0];
    const double dy = (*high_xy)[1] - loco_upper_->last_highstate_xy[1];
    const double jump = std::hypot(dx, dy);
    if (!std::isfinite(jump) ||
        jump > config_.loco_upper_pose_jump_reject_m) {
      failLocoUpperSafety(LocoReason::PoseJump, "pose_jump");
      return false;
    }
  }

  if (!loco_upper_->has_highstate_start) {
    loco_upper_->highstate_start_xy = *high_xy;
    loco_upper_->has_highstate_start = true;
  }
  loco_upper_->last_highstate_xy = *high_xy;
  loco_upper_->has_last_highstate_xy = true;
  estimate_xy[0] = (*high_xy)[0] - loco_upper_->highstate_start_xy[0];
  estimate_xy[1] = (*high_xy)[1] - loco_upper_->highstate_start_xy[1];
  loco_upper_->integrated_xy = estimate_xy;
  active_->loco.radius_source = "highstate";
  active_->loco.distance_m = std::hypot(estimate_xy[0], estimate_xy[1]);
  return true;
}

void RuntimeControlLoop::failLocoUpperSafety(LocoReason reason, const char* block) {
  if (!active_) {
    return;
  }
  active_->loco.reason = reason;
  active_->loco.phase = LocoPhase::Failed;
  finishActive(MotionState::Failed,
               StopReason::None,
               ErrorCode::SafetyLimitTriggered);
  RobotReadinessStatus readiness;
  readiness.robot = RobotState::NotReady;
  readiness.err = ErrorCode::SafetyLimitTriggered;
  readiness.block = block;
  enterPassiveState(readiness);
}

bool RuntimeControlLoop::updateLocoUpperRadiusState(
    std::array<double, 2>& estimate_xy) {
  if (!active_ || !loco_upper_) {
    return false;
  }
  if (loco_upper_->has_highstate_start) {
    if (const auto high_xy = highstatePlanarPosition(readHighStateForStatus())) {
      estimate_xy[0] = (*high_xy)[0] - loco_upper_->highstate_start_xy[0];
      estimate_xy[1] = (*high_xy)[1] - loco_upper_->highstate_start_xy[1];
      loco_upper_->integrated_xy = estimate_xy;
      loco_upper_->last_highstate_xy = *high_xy;
      loco_upper_->has_last_highstate_xy = true;
      active_->loco.radius_source = "highstate";
      active_->loco.distance_m = std::hypot(estimate_xy[0], estimate_xy[1]);
      return true;
    }
  }

  active_->loco.radius_source = "integrated";
  active_->loco.distance_m = std::hypot(estimate_xy[0], estimate_xy[1]);
  return true;
}

bool RuntimeControlLoop::enforceLocoUpperRadiusLimit(
    const std::array<double, 2>& estimate_xy) {
  if (!active_ || active_->loco_options.max_radius_m <= 0.0) {
    return true;
  }

  const double distance = std::hypot(estimate_xy[0], estimate_xy[1]);
  active_->loco.distance_m = distance;
  if (loco_upper_ && loco_upper_->radius_limit_reached) {
    active_->loco.radius_limit_reached = true;
  }
  if (distance <= active_->loco_options.max_radius_m + config_.radius_tolerance_m) {
    return true;
  }

  active_->loco.radius_limit_reached = true;
  if (loco_upper_) {
    loco_upper_->radius_limit_reached = true;
  }
  return true;
}

void RuntimeControlLoop::failLocoUpperPolicy(
    LocoReason reason,
    const RobotReadinessStatus& readiness) {
  if (active_) {
    active_->loco.reason = reason;
  }
  failActiveWithFault(ErrorCode::ModelInferenceFailed,
                      RobotState::Fault,
                      "policy_inference_failed",
                      readiness.low_ms);
}

bool RuntimeControlLoop::writeLocoUpperStep(
    VelocityCommand command,
    const std::vector<float>& upper_targets,
    std::optional<float> lower_entry_alpha) {
  if (!active_ || !loco_upper_ || !loco_upper_->lower_runner ||
      !hasLocoUpperRuntime()) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed");
    return false;
  }

  if (config_.loco_upper_strict_pose) {
    std::array<double, 2> estimate_xy = loco_upper_->integrated_xy;
    if (!requireStrictPoseForLocoUpper(estimate_xy)) {
      return false;
    }
    if (!enforceLocoUpperRadiusLimit(estimate_xy)) {
      return false;
    }
  } else {
    std::array<double, 2> estimate_xy = loco_upper_->integrated_xy;
    if (!updateLocoUpperRadiusState(estimate_xy) ||
        !enforceLocoUpperRadiusLimit(estimate_xy)) {
      return false;
    }
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    failActiveReadiness(readiness);
    return false;
  }
  applyReadiness(readiness);

  LocoLowerStepResult lower;
  LocoUpperLowCmdComposeResult composed;
  try {
    const bool lower_policy_due = consumeLocoLowerPolicyDue();
    lower = loco_upper_->lower_runner->step(*low_state,
                                            command,
                                            *loco_lower_policy_,
                                            lowCmdBaseFrame(),
                                            lower_policy_due);
    LowCmdFrame lower_frame = lower.low_cmd;
    if (lower_entry_alpha.has_value()) {
      applyLocoUpperLowerEntryInterpolation(*loco_lower_deploy_config_,
                                            loco_upper_->lower_entry_start_q,
                                            lower.processed_action,
                                            *lower_entry_alpha,
                                            lower_frame);
    }
    composed = composeLocoUpperLowCmd(*loco_upper_composer_config_,
                                      lower_frame,
                                      upper_targets);
  } catch (const LocoLowerPolicyError& err) {
    const LocoReason reason = isNonFinitePolicyMessage(err.what())
                                  ? LocoReason::PolicyNan
                                  : LocoReason::PolicyInfer;
    failLocoUpperPolicy(reason, readiness);
    return false;
  } catch (const std::exception& err) {
    failLocoUpperPolicy(isNonFinitePolicyMessage(err.what())
                            ? LocoReason::PolicyNan
                            : LocoReason::PolicyInfer,
                        readiness);
    return false;
  }

  try {
    writeLowCmdFrame(composed.frame);
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "lowcmd_write_failed",
                        readiness.low_ms);
    return false;
  }
  loco_upper_->last_upper = upper_targets;
  loco_upper_->envelope_clamped =
      loco_upper_->envelope_clamped || lower.command_clamped;
  active_->loco.envelope_clamped = loco_upper_->envelope_clamped;
  active_->loco.upper_clamped = loco_upper_->upper_clamped;
  active_->loco.upper_rate_limited = loco_upper_->upper_rate_limited;
  loco_upper_->raw_action_clamped =
      loco_upper_->raw_action_clamped || lower.raw_action_clamped;
  active_->loco.raw_action_clamped = loco_upper_->raw_action_clamped;
  loco_upper_->lower_q_limited =
      loco_upper_->lower_q_limited || composed.lower_q_limited;
  active_->loco.lower_q_limited = loco_upper_->lower_q_limited;
  loco_upper_->lower_action_clamped =
      loco_upper_->lower_action_clamped || lower.raw_action_clamped ||
      composed.lower_q_limited;
  active_->loco.lower_action_clamped = loco_upper_->lower_action_clamped;
  return true;
}

bool RuntimeControlLoop::prepareLocoUpperFrameStep(std::size_t frame,
                                                   bool zero_lower_command) {
  if (!active_ || !loco_upper_) {
    return false;
  }
  if (frame >= loco_upper_->upper_frames.size()) {
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "policy_inference_failed");
    return false;
  }

  VelocityCommand command{};
  double robot_yaw = 0.0;
  if (const std::optional<LowStateSample> low_state =
          latest_low_state_ ? latest_low_state_ : readLowStateForStatus()) {
    robot_yaw = yawFromLowState(*low_state);
  }

  if (!zero_lower_command && !loco_upper_->commands_body.empty()) {
    const std::size_t command_index =
        std::min(frame, loco_upper_->commands_body.size() - 1);
    LocoUpperVelocityCommand command_body =
        loco_upper_->commands_body.at(command_index);

    std::array<double, 2> estimate_xy = loco_upper_->integrated_xy;
    std::string radius_source;
    if (config_.loco_upper_strict_pose) {
      if (!requireStrictPoseForLocoUpper(estimate_xy)) {
        return false;
      }
      radius_source = "highstate";
    } else if (loco_upper_->has_highstate_start) {
      if (const auto high_xy = highstatePlanarPosition(readHighStateForStatus())) {
        estimate_xy[0] = (*high_xy)[0] - loco_upper_->highstate_start_xy[0];
        estimate_xy[1] = (*high_xy)[1] - loco_upper_->highstate_start_xy[1];
        radius_source = "highstate";
        loco_upper_->integrated_xy = estimate_xy;
      }
    }
    if (radius_source.empty()) {
      radius_source = "integrated";
    }

    if (active_->loco_options.max_radius_m > 0.0) {
      LocoUpperVelocityCommand command_world =
          bodyVelocityToWorldCommand(command_body, robot_yaw);
      command_world = suppressOutwardRadialVelocityNearRadius(
          planarSampleFromXY(estimate_xy),
          command_world,
          active_->loco_options.max_radius_m,
          kLocoUpperRadiusSuppressMarginM);
      loco_upper_->radius_limit_reached =
          loco_upper_->radius_limit_reached || command_world.radius_limit_reached;
      command_body = worldVelocityToBodyCommand(command_world, robot_yaw);
    }

    command.vx = static_cast<float>(command_body.vx);
    command.vy = static_cast<float>(command_body.vy);
    command.yaw_rate = static_cast<float>(command_body.yaw_rate);
    bool command_clamped = false;
    command = clampBodyCommand(loco_lower_deploy_config_->command_ranges,
                               command,
                               command_clamped);
    loco_upper_->envelope_clamped =
        loco_upper_->envelope_clamped || command_clamped;
    active_->loco.radius_source = radius_source;
    if (radius_source == "integrated") {
      const LocoUpperVelocityCommand integrated_world =
          bodyVelocityToWorldCommand(command, robot_yaw);
      const double dt_s = locoCommandDtS(loco_upper_->root_plan.dt_s,
                                         active_->fps,
                                         config_.hz);
      loco_upper_->integrated_xy[0] += integrated_world.vx * dt_s;
      loco_upper_->integrated_xy[1] += integrated_world.vy * dt_s;
      estimate_xy = loco_upper_->integrated_xy;
    }
    active_->loco.distance_m = std::hypot(estimate_xy[0], estimate_xy[1]);
    if (!enforceLocoUpperRadiusLimit(estimate_xy)) {
      return false;
    }
  }

  active_->loco.radius_limit_reached = loco_upper_->radius_limit_reached;
  active_->loco.envelope_clamped = loco_upper_->envelope_clamped;
  active_->loco.upper_clamped = loco_upper_->upper_clamped;
  active_->loco.upper_rate_limited = loco_upper_->upper_rate_limited;
  loco_upper_->current_command = command;
  loco_upper_->current_upper =
      upperTargetsFromCompiledFrame(loco_upper_->upper_frames.at(frame));
  return true;
}

bool RuntimeControlLoop::writeLocoUpperFrame(std::size_t frame,
                                             bool zero_lower_command) {
  if (!prepareLocoUpperFrameStep(frame, zero_lower_command)) {
    return false;
  }
  return writeLocoUpperStep(loco_upper_->current_command,
                            loco_upper_->current_upper);
}

bool RuntimeControlLoop::writeLocoUpperFinalHold(bool zero_lower_command) {
  if (!active_ || !loco_upper_) {
    return false;
  }
  if (active_->frames > 0) {
    active_->frame = active_->frames - 1;
  }
  if (!zero_lower_command) {
    return writeLocoUpperFrame(active_->frame, false);
  }
  return writeLocoUpperStep({0.0F, 0.0F, 0.0F}, loco_upper_->final_upper);
}

bool RuntimeControlLoop::writeLocoUpperUpperTransitionStep() {
  if (!active_ || !loco_upper_) {
    return false;
  }
  if (active_->frames > 0) {
    active_->frame = active_->frames - 1;
  }
  const std::size_t total = std::max<std::size_t>(1, loco_upper_->phase_total_ticks);
  const std::size_t remaining = loco_upper_->phase_ticks_remaining;
  const std::size_t completed = total > remaining ? total - remaining : 0;
  const float alpha =
      total <= 1 ? 1.0F
                 : std::clamp(static_cast<float>(completed) /
                                  static_cast<float>(total - 1),
                              0.0F,
                              1.0F);
  return writeLocoUpperStep(
      {0.0F, 0.0F, 0.0F},
      interpolateUpperTargets(loco_upper_->transition_start_upper,
                              loco_upper_->transition_target_upper,
                              alpha));
}

std::vector<float> RuntimeControlLoop::fixstandUpperTargets() const {
  std::vector<float> targets(kPolicyJointCount, 0.0F);
  if (!loco_upper_composer_config_ || !fixstand_config_) {
    return targets;
  }
  for (std::size_t logical = loco_upper_composer_config_->upper_start_joint;
       logical < loco_upper_composer_config_->upper_end_joint_exclusive &&
       logical < targets.size() &&
       logical < loco_upper_composer_config_->logical_to_sdk.size();
       ++logical) {
    const int sdk_slot_raw = loco_upper_composer_config_->logical_to_sdk.at(logical);
    if (sdk_slot_raw < 0) {
      continue;
    }
    const auto sdk_slot = static_cast<std::size_t>(sdk_slot_raw);
    if (sdk_slot >= fixstand_config_->target_q.size()) {
      continue;
    }
    targets.at(logical) =
        static_cast<float>(fixstand_config_->target_q.at(sdk_slot));
  }
  return targets;
}

std::vector<float> RuntimeControlLoop::locoUpperStandbyTargets() const {
  return fixstandUpperTargets();
}

std::vector<float> RuntimeControlLoop::currentLocoUpperTargets() const {
  if (!active_ || !loco_upper_) {
    return {};
  }
  if (loco_upper_->last_upper.size() >= kPolicyJointCount) {
    return loco_upper_->last_upper;
  }
  std::vector<float> current;
  if (active_->frame < loco_upper_->upper_frames.size()) {
    current = upperTargetsFromCompiledFrame(
        loco_upper_->upper_frames.at(active_->frame));
  }
  if (current.size() < kPolicyJointCount) {
    current = loco_upper_->final_upper;
  }
  return current;
}

void RuntimeControlLoop::beginLocoUpperStopping(StopReason reason) {
  if (!active_ || active_->executor != MotionExecutor::LocoUpper) {
    return;
  }
  if (!loco_upper_) {
    return;
  }
  active_->state = MotionState::Stopping;
  active_->stop_reason = reason;
  active_->err = ErrorCode::Ok;
  loco_upper_->phase_total_ticks = ticksForPeriod(
      transitionDurationForUse().value_or(1.0 / std::max(1.0, config_.hz)));
  loco_upper_->phase_ticks_remaining = loco_upper_->phase_total_ticks;
  loco_upper_->transition_start_upper = currentLocoUpperTargets();
  loco_upper_->transition_target_upper = locoUpperStandbyTargets();
  setLocoPhase(LocoPhase::Stopping);
  publishActive();
  enterInternalState(RuntimeInternalState::LocoUpperActive);
}

void RuntimeControlLoop::completeLocoUpperStopped() {
  if (!active_) {
    return;
  }
  const ControlMode post_stop_control = post_stop_control_;
  active_->loco.phase = LocoPhase::Stopped;
  finishActive(MotionState::Stopped, active_->stop_reason, ErrorCode::Ok);
  if (post_stop_control == ControlMode::FixStand) {
    enterFixStandState();
  } else {
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterVelocityState();
  }
}

void RuntimeControlLoop::finishLocoUpperDone() {
  if (!active_) {
    return;
  }
  active_->loco.phase = LocoPhase::Done;
  finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
  post_stop_control_ = ControlMode::StandbyVelocity;
  enterVelocityState();
}

void RuntimeControlLoop::setLocoPhase(LocoPhase phase) {
  if (!active_) {
    return;
  }
  active_->loco.phase = phase;
  active_->loco.radius_clamped =
      loco_upper_ ? loco_upper_->radius_clamped : active_->loco.radius_clamped;
  active_->loco.radius_limit_reached =
      loco_upper_ ? loco_upper_->radius_limit_reached
                  : active_->loco.radius_limit_reached;
  active_->loco.envelope_clamped =
      loco_upper_ ? loco_upper_->envelope_clamped : active_->loco.envelope_clamped;
  active_->loco.upper_clamped =
      loco_upper_ ? loco_upper_->upper_clamped : active_->loco.upper_clamped;
  active_->loco.upper_rate_limited =
      loco_upper_ ? loco_upper_->upper_rate_limited
                  : active_->loco.upper_rate_limited;
  active_->loco.raw_action_clamped =
      loco_upper_ ? loco_upper_->raw_action_clamped : active_->loco.raw_action_clamped;
  active_->loco.lower_q_limited =
      loco_upper_ ? loco_upper_->lower_q_limited : active_->loco.lower_q_limited;
  active_->loco.lower_action_clamped =
      loco_upper_ ? loco_upper_->lower_action_clamped
                  : active_->loco.lower_action_clamped;
}

void RuntimeControlLoop::advanceActiveWithPolicy() {
  if (!active_) {
    active_track_.reset();
    policy_runner_.reset();
    clearReference();
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    transition_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  if (active_kind_ == ActiveKind::User &&
      active_->state == MotionState::Holding) {
    advanceHoldingWithPolicy();
    return;
  }

  if (!policy_runner_) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed");
    return;
  }

  if (advanceUserPolicyStartupHold()) {
    return;
  }

  if (!consumeGeneralTrackerPolicyDue()) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return;
    }
    if (readinessRequiresFault(readiness)) {
      failActiveWithFault(readiness.err,
                          readiness.robot,
                          readiness.block,
                          readiness.low_ms);
    } else {
      finishActive(MotionState::Failed, StopReason::None, readiness.err);
      enterPassiveState(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_first_advance_ = false;
  }
  active_->frame = activePlaybackFrame();
  publishActive();
  publishReferenceActive();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 readiness.low_ms);
    } else {
      failActiveWithFault(ErrorCode::ModelInferenceFailed,
                          RobotState::Fault,
                          "policy_inference_failed",
                          readiness.low_ms);
    }
    return;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
      enterFault(ErrorCode::InternalError,
                 RobotState::Fault,
                 "lowcmd_write_failed",
                 readiness.low_ms);
    } else {
      failActiveWithFault(ErrorCode::InternalError,
                          RobotState::Fault,
                          "lowcmd_write_failed",
                          readiness.low_ms);
    }
    return;
  }

  advanceActivePlaybackTime();
  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    const std::size_t last_frame = active_->frames == 0 ? 0 : active_->frames - 1;
    active_->frame = last_frame;
    publishReferenceActive();
    if (active_kind_ == ActiveKind::User && active_->hold) {
      active_->state = MotionState::Holding;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      publishActive();
      return;
    }
    if (active_kind_ == ActiveKind::User &&
        startTransitionFromCompletedUserToNextUser()) {
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToIdle()) {
      return;
    }
    if (active_kind_ == ActiveKind::User && startTransitionFromCompletedUserToStandby()) {
      return;
    }
    if (active_kind_ == ActiveKind::Idle && startTransitionFromCompletedIdleToIdle()) {
      return;
    }
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  publishActive();
}

bool RuntimeControlLoop::advanceUserPolicyStartupHold() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      policy_startup_hold_steps_remaining_ == 0) {
    return false;
  }

  std::optional<LowStateSample> low_state;
  std::optional<RobotReadinessStatus> readiness;

  auto readReadyLowState = [&]() -> bool {
    low_state = readLowStateForStatus();
    readiness =
        mapRobotReadiness(low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness->err != ErrorCode::Ok) {
      failActiveReadiness(*readiness);
      return false;
    }
    applyReadiness(*readiness);
    return true;
  };

  if (!consumeGeneralTrackerPolicyDue()) {
    return true;
  }

  if (!readReadyLowState()) {
    return true;
  }

  active_->started_at = active_first_advance_
                            ? std::chrono::steady_clock::now()
                            : active_->started_at;
  active_first_advance_ = false;
  active_->frame = 0;
  publishActive();
  publishReferenceActive();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(0, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed",
                        readiness ? readiness->low_ms : std::nullopt);
    return true;
  }

  try {
    applyPolicyStartupUpperBodyInterpolation(step.low_cmd);
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "lowcmd_write_failed",
                        readiness ? readiness->low_ms : std::nullopt);
    return true;
  }

  --policy_startup_hold_steps_remaining_;
  if (policy_startup_hold_steps_remaining_ > 0) {
    return true;
  }
  advanceActivePlaybackTime();

  try {
    policy_runner_->recalibrateObservationAnchor(*low_state);
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed",
                        readiness ? readiness->low_ms : std::nullopt);
  }
  return true;
}

void RuntimeControlLoop::advanceHolding() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding) {
    return;
  }

  if (!waiting_.empty()) {
    if (isGeneralTrackerRequest(waiting_.front())) {
      if (startTransitionFromHoldingToNextUser()) {
        return;
      }
    } else {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
      return;
    }
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
  publishReferenceActive();
  publishActive();
}

void RuntimeControlLoop::advanceHoldingWithPolicy() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding) {
    return;
  }

  if (!waiting_.empty()) {
    if (isGeneralTrackerRequest(waiting_.front())) {
      if (startTransitionFromHoldingToNextUser()) {
        return;
      }
    } else {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
      return;
    }
  }

  if (!policy_runner_) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed");
    return;
  }

  if (!consumeGeneralTrackerPolicyDue()) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    failActiveReadiness(readiness);
    return;
  }
  applyReadiness(readiness);

  active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
  publishActive();
  publishReferenceActive();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed",
                        readiness.low_ms);
    return;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "lowcmd_write_failed",
                        readiness.low_ms);
    return;
  }

  publishActive();
}

bool RuntimeControlLoop::startTransitionFromHoldingToNextUser() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding || !active_track_ ||
      waiting_.empty()) {
    return false;
  }
  if (!isGeneralTrackerRequest(waiting_.front())) {
    return false;
  }

  auto releaseHoldingToQueuedFallback = [this]() {
    MotionRequest pending = std::move(waiting_.front());
    waiting_.pop_front();
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    waiting_.push_front(std::move(pending));
  };

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    // A holding GeneralTracker run reaches this state from a validated loaded
    // track and pins frame to frames-1, so this should not be reachable through
    // the public queue path. If the invariant is violated, release the held
    // source once and let the queued target continue through the no-active path
    // instead of retrying the same impossible handoff every tick.
    releaseHoldingToQueuedFallback();
    return true;
  }

  const MotionRequest target_request = waiting_.front();
  const UserHandoffResult handoff_result =
      tryStartUserHandoffFromFrame(*source_frame,
                                   target_request,
                                   MotionState::Done,
                                   StopReason::None,
                                   ErrorCode::Ok,
                                   true);
  if (handoff_result == UserHandoffResult::Started ||
      handoff_result == UserHandoffResult::TargetFailed ||
      handoff_result == UserHandoffResult::SafetyTerminal) {
    if (!waiting_.empty() && waiting_.front().id == target_request.id) {
      waiting_.pop_front();
    }
    return true;
  }

  releaseHoldingToQueuedFallback();
  return true;
}

bool RuntimeControlLoop::startTransitionFromHoldingToStandby() {
  if (!active_ || active_kind_ != ActiveKind::User ||
      active_->state != MotionState::Holding || !active_track_ ||
      !waiting_.empty() || !standby_track_) {
    return false;
  }

  const std::optional<TrkFrameView> reference_source_frame =
      active_track_->frame(active_->frame);
  const std::optional<TrkFrameView> target_frame = standby_track_->frame(0);
  if (!reference_source_frame || !target_frame) {
    return false;
  }

  std::optional<TrkTrack> source_track;
  std::optional<TrkFrameView> source_frame = reference_source_frame;
  if (hasPolicyRuntime()) {
    source_track = controllableStandbySourceTrack(*reference_source_frame,
                                                  active_track_->metadata.fps,
                                                  *target_frame);
    if (!source_track) {
      return isSafetyTerminalState();
    }
    source_frame = source_track->frame(0);
    if (!source_frame) {
      return false;
    }
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Standby;
  target.target_track = standby_track_;
  return startSyntheticTransitionFromFrame(std::move(target), *source_frame);
}

bool RuntimeControlLoop::startTransitionFromCurrentReferenceToUser(
    MotionRequest target_request,
    StopReason replaced_reason) {
  if (!isGeneralTrackerRequest(target_request)) {
    return false;
  }
  if (active_kind_ != ActiveKind::Transition || !active_ || !active_track_) {
    return false;
  }

  auto abortPreemptedTransition = [this, replaced_reason] {
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
    enterGeneralTrackerIdleState();
  };

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    target_request.state = MotionState::Failed;
    target_request.frame = 0;
    target_request.err = ErrorCode::InternalError;
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortPreemptedTransition();
    return true;
  }

  target_request.state = MotionState::Queued;
  target_request.frame = 0;
  target_request.err = ErrorCode::Ok;
  target_request.stop_reason = StopReason::None;

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    target_request.state = MotionState::Failed;
    target_request.frame = 0;
    target_request.err = toCoreErrorCode(loaded.code);
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortPreemptedTransition();
    return true;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<const TrkTrack>(std::move(*loaded.track));

  ErrorCode transition_error = ErrorCode::InternalError;
  bool transition_build_rejected = false;
  std::optional<UserTransitionTracks> tracks =
      makeUserTransitionTracks(*source_frame,
                               target_track,
                               transition_error,
                               transition_build_rejected);

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = latest_low_state_;
    if (!entry_low_state) {
      entry_low_state = readLowStateForStatus();
    }
  }

  if (!tracks) {
    tracks = makeUserTransitionTracksFromControllableSource(*source_frame,
                                                            target_track,
                                                            entry_low_state,
                                                            transition_error,
                                                            transition_build_rejected);
  }

  if (!tracks) {
    target_request.state = MotionState::Failed;
    target_request.err = transition_error;
    target_request.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(target_request));
    abortPreemptedTransition();
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(tracks->target_track);
  target.target_request = std::move(target_request);

  finishTransitionTarget(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
  const bool started = startInternalTransition(std::move(tracks->transition_track),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (!started) {
    abortTransition(MotionState::Canceled, replaced_reason, ErrorCode::Ok);
  }
  return true;
}

bool RuntimeControlLoop::startTransitionFromCompletedIdleToIdle() {
  if (!active_ || active_kind_ != ActiveKind::Idle || !active_track_ ||
      !waiting_.empty() || idle_config_.empty()) {
    return false;
  }

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    stopIdleActive();
    return true;
  }

  if (idle_next_index_ >= idle_config_.size()) {
    idle_next_index_ = 0;
  }
  const std::size_t index = idle_next_index_;
  idle_next_index_ = (idle_next_index_ + 1) % idle_config_.size();
  const IdleMotion& motion = idle_config_.at(index);

  auto failTransition = [this] {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    }
  };

  TrkLoadResult loaded = loader_.load(motion.path);
  if (!loaded.ok()) {
    failTransition();
    return true;
  }

  MotionRequest idle_request;
  idle_request.id.clear();
  idle_request.path = motion.path;
  idle_request.state = MotionState::Queued;
  idle_request.frame = 0;
  idle_request.frames = loaded.track->metadata.frames;
  idle_request.fps = loaded.track->metadata.fps;
  idle_request.duration_s = loaded.track->metadata.duration_s;
  idle_request.err = ErrorCode::Ok;
  idle_request.stop_reason = StopReason::None;

  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  std::optional<TrkTrack> aligned_target_track =
      alignTrackRootPlanarPose(*target_track, *source_frame);
  if (!aligned_target_track) {
    failTransition();
    return true;
  }
  target_track = std::make_shared<TrkTrack>(std::move(*aligned_target_track));
  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!target_frame) {
    failTransition();
    return true;
  }
  const std::optional<bool> yaw_residual_ok =
      rootYawResidualAllowsBridge(*source_frame, *target_frame);
  if (!yaw_residual_ok || !*yaw_residual_ok) {
    failTransition();
    return true;
  }

  const std::optional<double> transition_duration_s = transitionDurationForUse();
  if (!transition_duration_s) {
    failTransition();
    return true;
  }

  const double transition_fps = transitionSampleFpsForTarget(*target_track);
  std::optional<TrkTrack> transition_track =
      makeSyntheticTransitionTrk(*source_frame,
                                 *target_frame,
                                 transition_fps,
                                 transitionOptionsForTarget(*target_track));
  if (!transition_track) {
    failTransition();
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Idle;
  target.target_id.clear();
  target.target_state = MotionState::Running;
  target.target_track = std::move(target_track);
  target.target_request = std::move(idle_request);
  target.idle_index = index;

  auto transition_track_ptr =
      std::make_shared<TrkTrack>(std::move(*transition_track));
  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = latest_low_state_;
    if (!entry_low_state) {
      entry_low_state = readLowStateForStatus();
    }
  }

  const bool started = startInternalTransition(std::move(transition_track_ptr),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (!started) {
    if (active_kind_ == ActiveKind::Idle) {
      stopIdleActive();
    }
  }
  return true;
}

bool RuntimeControlLoop::startTransitionFromCompletedUserToNextUser() {
  if (!active_ || active_kind_ != ActiveKind::User || active_->hold ||
      active_->executor != MotionExecutor::GeneralTracker || !active_track_ ||
      waiting_.empty()) {
    return false;
  }
  if (!isGeneralTrackerRequest(waiting_.front())) {
    return false;
  }

  auto failQueuedTargetAndCompleteActive = [this](ErrorCode error) {
    MotionRequest failed = std::move(waiting_.front());
    waiting_.pop_front();
    failed.state = MotionState::Failed;
    failed.frame = 0;
    failed.err = error;
    failed.stop_reason = StopReason::None;
    failed.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(failed));
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
  };

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  const std::optional<TrkFrameView> source_frame =
      active_track_->frame(active_->frame);
  if (!source_frame) {
    return false;
  }

  const MotionRequest source_to_complete = *active_;
  auto publishSourceDone = [this, source_to_complete] {
    MotionRequest completed = source_to_complete;
    completed.state = MotionState::Done;
    completed.err = ErrorCode::Ok;
    completed.stop_reason = StopReason::None;
    completed.ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(completed));
  };

  MotionRequest target_request = waiting_.front();
  target_request.state = MotionState::Queued;
  target_request.frame = 0;
  target_request.err = ErrorCode::Ok;
  target_request.stop_reason = StopReason::None;

  TrkLoadResult loaded = loader_.load(target_request.path);
  if (!loaded.ok()) {
    failQueuedTargetAndCompleteActive(toCoreErrorCode(loaded.code));
    return true;
  }

  target_request.frames = loaded.track->metadata.frames;
  target_request.fps = loaded.track->metadata.fps;
  target_request.duration_s = loaded.track->metadata.duration_s;
  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  auto original_target_track =
      std::make_shared<const TrkTrack>(*target_track);
  std::optional<TrkTrack> aligned_target_track =
      alignTrackRootPlanarPose(*target_track, *source_frame);
  if (!aligned_target_track) {
    failQueuedTargetAndCompleteActive(ErrorCode::InternalError);
    return true;
  }
  target_track = std::make_shared<TrkTrack>(std::move(*aligned_target_track));

  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!target_frame) {
    failQueuedTargetAndCompleteActive(ErrorCode::InternalError);
    return true;
  }
  const std::optional<bool> yaw_residual_ok =
      rootYawResidualAllowsBridge(*source_frame, *target_frame);
  if (!yaw_residual_ok) {
    failQueuedTargetAndCompleteActive(ErrorCode::InternalError);
    return true;
  }
  if (!*yaw_residual_ok) {
    return false;
  }

  const std::optional<double> transition_duration_s = transitionDurationForUse();
  const double transition_fps = transitionSampleFpsForTarget(*target_track);
  std::optional<TrkTrack> transition_track =
      transition_duration_s
          ? makeSyntheticTransitionTrk(*source_frame,
                                       *target_frame,
                                       transition_fps,
                                       transitionOptionsForTarget(*target_track))
          : std::nullopt;
  if (!transition_track) {
    ErrorCode fallback_error = ErrorCode::InternalError;
    bool fallback_transition_build_rejected = false;
    std::optional<LowStateSample> fallback_entry_low_state = entry_low_state;
    std::optional<UserTransitionTracks> fallback_tracks =
        makeUserTransitionTracksFromControllableSource(
            *source_frame,
            original_target_track,
            fallback_entry_low_state,
            fallback_error,
            fallback_transition_build_rejected);
    if (fallback_tracks) {
      PendingTransition target;
      target.target_kind = TransitionTargetKind::User;
      target.target_id = target_request.id;
      target.target_state = MotionState::Queued;
      target.target_track = std::move(fallback_tracks->target_track);
      target.target_request = target_request;
      target.reduced_target_startup_hold = true;

      const bool started =
          startInternalTransition(std::move(fallback_tracks->transition_track),
                                  std::move(target),
                                  std::move(fallback_entry_low_state),
                                  false);
      if (started) {
        waiting_.pop_front();
        publishSourceDone();
        return true;
      }
    }
    if (transition_duration_s &&
        allowStandbyRawStartAfterTransitionBuildFailure(*source_frame,
                                                        *target_frame,
                                                        *transition_duration_s,
                                                        transition_fps,
                                                        config_.transition_root_yaw_residual_limit_rad,
                                                        transitionLimitsForUse())) {
      return false;
    }
    failQueuedTargetAndCompleteActive(ErrorCode::InternalError);
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::User;
  target.target_id = target_request.id;
  target.target_state = MotionState::Queued;
  target.target_track = std::move(target_track);
  target.target_request = std::move(target_request);
  target.reduced_target_startup_hold = true;

  auto transition_track_ptr =
      std::make_shared<TrkTrack>(std::move(*transition_track));
  const bool started = startInternalTransition(std::move(transition_track_ptr),
                                               std::move(target),
                                               std::move(entry_low_state),
                                               false);
  if (!started) {
    failQueuedTargetAndCompleteActive(ErrorCode::InternalError);
    return true;
  }

  waiting_.pop_front();
  publishSourceDone();
  return true;
}

bool RuntimeControlLoop::startTransitionFromCompletedUserToIdle() {
  if (!active_ || active_kind_ != ActiveKind::User || active_->hold ||
      !active_track_ || !waiting_.empty() || idle_config_.empty()) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      active_->state = MotionState::Done;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  if (idle_next_index_ >= idle_config_.size()) {
    idle_next_index_ = 0;
  }
  const std::size_t index = idle_next_index_;
  idle_next_index_ = (idle_next_index_ + 1) % idle_config_.size();
  const IdleMotion& motion = idle_config_.at(index);

  TrkLoadResult loaded = loader_.load(motion.path);
  if (!loaded.ok()) {
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return true;
  }

  MotionRequest idle_request;
  idle_request.id.clear();
  idle_request.path = motion.path;
  idle_request.state = MotionState::Queued;
  idle_request.frame = 0;
  idle_request.frames = loaded.track->metadata.frames;
  idle_request.fps = loaded.track->metadata.fps;
  idle_request.duration_s = loaded.track->metadata.duration_s;
  idle_request.err = ErrorCode::Ok;
  idle_request.stop_reason = StopReason::None;

  auto target_track = std::make_shared<TrkTrack>(std::move(*loaded.track));
  const std::optional<TrkFrameView> target_frame = target_track->frame(0);
  if (!target_frame) {
    finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return true;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Idle;
  target.target_id.clear();
  target.target_state = MotionState::Running;
  target.target_track = std::move(target_track);
  target.target_request = std::move(idle_request);
  target.idle_index = index;
  return startSyntheticTransitionFromActiveFrame(std::move(target));
}

bool RuntimeControlLoop::startTransitionFromCompletedUserToStandby() {
  if (!active_ || active_kind_ != ActiveKind::User || active_->hold ||
      !active_track_ || !waiting_.empty() || !idle_config_.empty() ||
      !standby_track_) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      active_->state = MotionState::Done;
      active_->err = ErrorCode::Ok;
      active_->stop_reason = StopReason::None;
      active_->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*active_));
      active_.reset();
      active_track_.reset();
      policy_runner_.reset();
      clearReference();
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  const std::optional<TrkFrameView> target_frame = standby_track_->frame(0);
  if (!target_frame) {
    return false;
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Standby;
  target.target_track = standby_track_;
  return startSyntheticTransitionFromActiveFrame(std::move(target));
}

RuntimeControlLoop::StandbyTransitionResult
RuntimeControlLoop::startTransitionFromActiveToStandbyCancellation() {
  if (!active_ || !active_track_ || !standby_track_) {
    return StandbyTransitionResult::Fallback;
  }

  if (active_kind_ == ActiveKind::Transition && transition_ &&
      transition_->target_kind == TransitionTargetKind::Standby) {
    return StandbyTransitionResult::Started;
  }

  if (active_kind_ != ActiveKind::User &&
      active_kind_ != ActiveKind::Transition) {
    return StandbyTransitionResult::Fallback;
  }
  if (active_kind_ == ActiveKind::User &&
      active_->executor != MotionExecutor::GeneralTracker) {
    return StandbyTransitionResult::Fallback;
  }
  if (active_kind_ == ActiveKind::Transition && !transition_) {
    return StandbyTransitionResult::Fallback;
  }

  auto failStandbyHandoff = [this]() {
    failActiveWithFault(ErrorCode::InternalError,
                        RobotState::Fault,
                        "standby_transition_failed",
                        runtime_state_.low_ms);
    return StandbyTransitionResult::SafetyTerminal;
  };

  const std::optional<TrkFrameView> reference_source_frame =
      active_track_->frame(active_->frame);
  const std::optional<TrkFrameView> standby_target_frame = standby_track_->frame(0);
  if (!reference_source_frame || !standby_target_frame) {
    return failStandbyHandoff();
  }

  std::optional<TrkTrack> source_track;
  std::optional<TrkFrameView> source_frame = reference_source_frame;
  if (hasPolicyRuntime()) {
    source_track = controllableStandbySourceTrack(*reference_source_frame,
                                                  active_track_->metadata.fps,
                                                  *standby_target_frame);
    if (!source_track) {
      if (isSafetyTerminalState()) {
        return StandbyTransitionResult::SafetyTerminal;
      }
      return failStandbyHandoff();
    }
    source_frame = source_track->frame(0);
    if (!source_frame) {
      return failStandbyHandoff();
    }
  }

  if (active_kind_ == ActiveKind::Transition &&
      transition_->target_kind == TransitionTargetKind::User) {
    finishTransitionTarget(MotionState::Canceled,
                           StopReason::Stop,
                           ErrorCode::Ok);
  }

  PendingTransition target;
  target.target_kind = TransitionTargetKind::Standby;
  target.target_track = standby_track_;
  if (active_kind_ == ActiveKind::User) {
    target.source_completion_state = MotionState::Stopped;
    target.source_completion_reason = StopReason::Stop;
    target.source_completion_error = ErrorCode::Ok;
  }

  if (startSyntheticTransitionFromFrame(std::move(target), *source_frame)) {
    return StandbyTransitionResult::Started;
  }
  if (isSafetyTerminalState()) {
    return StandbyTransitionResult::SafetyTerminal;
  }
  return failStandbyHandoff();
}

bool RuntimeControlLoop::startSyntheticTransitionFromActiveFrame(
    PendingTransition target) {
  if (!active_ || !active_track_) {
    return false;
  }

  const std::optional<TrkFrameView> source_frame = active_track_->frame(active_->frame);
  if (!source_frame) {
    return false;
  }
  return startSyntheticTransitionFromFrame(std::move(target), *source_frame);
}

bool RuntimeControlLoop::startSyntheticTransitionFromFrame(
    PendingTransition target,
    const TrkFrameView& source_frame) {
  if (!active_ || !active_track_) {
    return false;
  }
  if (!target.target_track) {
    return false;
  }
  std::optional<TrkTrack> aligned_target_track =
      alignTrackRootPlanarPose(*target.target_track, source_frame);
  if (!aligned_target_track) {
    if (target.target_request && !target.target_request->id.empty()) {
      target.target_request->state = MotionState::Failed;
      target.target_request->err = ErrorCode::InternalError;
      target.target_request->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*target.target_request));
    } else if (target.target_kind == TransitionTargetKind::Idle &&
               active_kind_ == ActiveKind::User) {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
    } else if (target.target_kind == TransitionTargetKind::Standby) {
      return false;
    }
    return true;
  }
  target.target_track = std::make_shared<TrkTrack>(std::move(*aligned_target_track));
  const std::optional<TrkFrameView> target_frame = target.target_track->frame(0);
  if (!target_frame) {
    return false;
  }
  const std::optional<bool> yaw_residual_ok =
      rootYawResidualAllowsBridge(source_frame, *target_frame);
  if (!yaw_residual_ok || !*yaw_residual_ok) {
    if (target.target_request && !target.target_request->id.empty()) {
      target.target_request->state = MotionState::Failed;
      target.target_request->err = ErrorCode::InternalError;
      target.target_request->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*target.target_request));
    } else if (target.target_kind == TransitionTargetKind::Idle &&
               active_kind_ == ActiveKind::User) {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
    } else if (target.target_kind == TransitionTargetKind::Standby) {
      return false;
    }
    return true;
  }

  const std::optional<double> transition_duration_s = transitionDurationForUse();
  const double transition_fps = transitionSampleFpsForTarget(*target.target_track);
  const SyntheticTransitionOptions transition_options =
      target.target_kind == TransitionTargetKind::Standby
          ? controlledTransitionOptionsForTarget(*target.target_track)
          : transitionOptionsForTarget(*target.target_track);
  std::optional<TrkTrack> transition_track =
      transition_duration_s
          ? makeSyntheticTransitionTrk(source_frame,
                                       *target_frame,
                                       transition_fps,
                                       transition_options)
          : std::nullopt;
  if (!transition_track) {
    if (target.target_request && !target.target_request->id.empty()) {
      target.target_request->state = MotionState::Failed;
      target.target_request->err = ErrorCode::InternalError;
      target.target_request->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*target.target_request));
    } else if (target.target_kind == TransitionTargetKind::Idle &&
               active_kind_ == ActiveKind::User) {
      finishActive(MotionState::Done, StopReason::None, ErrorCode::Ok);
      post_stop_control_ = ControlMode::StandbyVelocity;
      enterGeneralTrackerIdleState();
    } else if (target.target_kind == TransitionTargetKind::Standby) {
      return false;
    }
    return true;
  }

  std::optional<MotionRequest> source_to_complete;
  if (active_kind_ == ActiveKind::User) {
    source_to_complete = active_;
  }
  const MotionState source_completion_state = target.source_completion_state;
  const StopReason source_completion_reason = target.source_completion_reason;
  const ErrorCode source_completion_error = target.source_completion_error;

  auto transition_track_ptr =
      std::make_shared<TrkTrack>(std::move(*transition_track));
  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = latest_low_state_;
    if (!entry_low_state) {
      entry_low_state = readLowStateForStatus();
    }
  }
  const bool started = startInternalTransition(std::move(transition_track_ptr),
                                               std::move(target),
                                               std::move(entry_low_state));
  if (started && source_to_complete) {
    source_to_complete->state = source_completion_state;
    source_to_complete->err = source_completion_error;
    source_to_complete->stop_reason = source_completion_reason;
    source_to_complete->ended_at = std::chrono::steady_clock::now();
    status_.publishRunStatus(toStatus(*source_to_complete));
  }
  return started;
}

bool RuntimeControlLoop::startInternalTransition(
    std::shared_ptr<const TrkTrack> track,
    PendingTransition target,
    std::optional<LowStateSample> entry_low_state,
    bool publish_target_failure) {
  last_transition_start_fatal_.reset();
  if (fault_next_transition_start_for_test_) {
    fault_next_transition_start_for_test_ = false;
    failActiveWithFault(ErrorCode::ModelInferenceFailed,
                        RobotState::Fault,
                        "policy_inference_failed",
                        runtime_state_.low_ms);
    return false;
  }

  if (fail_next_transition_start_for_test_) {
    fail_next_transition_start_for_test_ = false;
    if (publish_target_failure && target.target_request &&
        !target.target_request->id.empty()) {
      target.target_request->state = MotionState::Failed;
      target.target_request->err = ErrorCode::InternalError;
      target.target_request->ended_at = std::chrono::steady_clock::now();
      status_.publishRunStatus(toStatus(*target.target_request));
    }
    return false;
  }

  std::optional<PolicyStepRunner> runner;
  if (hasPolicyRuntime()) {
    if (!entry_low_state) {
      if (publish_target_failure && target.target_request &&
          !target.target_request->id.empty()) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = ErrorCode::RobotNotReady;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      if (!publish_target_failure) {
        last_transition_start_fatal_ = TransitionStartFatal{
            ErrorCode::RobotNotReady,
            RobotState::NotReady,
            "low_state_missing",
            std::nullopt,
        };
        return false;
      }
      failActiveWithFault(ErrorCode::RobotNotReady,
                          RobotState::NotReady,
                          "low_state_missing");
      return false;
    }
    try {
      runner.emplace(*deploy_config_, track, *entry_low_state, expected_mode_machine_);
    } catch (const std::exception&) {
      if (publish_target_failure && target.target_request &&
          !target.target_request->id.empty()) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = ErrorCode::ModelInferenceFailed;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      if (!publish_target_failure) {
        last_transition_start_fatal_ = TransitionStartFatal{
            ErrorCode::ModelInferenceFailed,
            RobotState::Fault,
            "policy_inference_failed",
            runtime_state_.low_ms,
        };
        return false;
      }
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 runtime_state_.low_ms);
      return false;
    }
  }

  MotionRequest transition_request;
  transition_request.state = MotionState::Running;
  transition_request.frame = 0;
  transition_request.frames = track->metadata.frames;
  transition_request.fps = track->metadata.fps;
  transition_request.duration_s = track->metadata.duration_s;
  transition_request.err = ErrorCode::Ok;
  transition_request.stop_reason = StopReason::None;
  transition_request.started_at = std::chrono::steady_clock::now();

  active_ = std::move(transition_request);
  active_kind_ = ActiveKind::Transition;
  active_track_ = std::move(track);
  transition_ = std::move(target);
  idle_current_index_.reset();
  policy_runner_ = std::move(runner);
  enterInternalState(RuntimeInternalState::GeneralTrackerTransition);
  publishReferenceTransition();
  return true;
}

bool RuntimeControlLoop::startStandbyPlayback(PendingTransition target) {
  if (target.target_kind != TransitionTargetKind::Standby ||
      !target.target_track) {
    return false;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return true;
    }
    applyReadiness(readiness);
  }

  std::optional<PolicyStepRunner> runner;
  if (hasPolicyRuntime()) {
    try {
      runner.emplace(*deploy_config_,
                     target.target_track,
                     *entry_low_state,
                     expected_mode_machine_);
    } catch (const std::exception&) {
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 runtime_state_.low_ms);
      return true;
    }
  }

  MotionRequest playback_request;
  playback_request.state = MotionState::Running;
  playback_request.frame = 0;
  playback_request.frames = target.target_track->metadata.frames;
  playback_request.fps = target.target_track->metadata.fps;
  playback_request.duration_s = target.target_track->metadata.duration_s;
  playback_request.err = ErrorCode::Ok;
  playback_request.stop_reason = StopReason::None;
  playback_request.started_at = std::chrono::steady_clock::now();

  PendingTransition playback_target;
  playback_target.target_kind = TransitionTargetKind::Standby;

  active_ = std::move(playback_request);
  active_kind_ = ActiveKind::Transition;
  active_track_ = std::move(target.target_track);
  transition_ = std::move(playback_target);
  idle_current_index_.reset();
  policy_runner_ = std::move(runner);
  enterInternalState(RuntimeInternalState::GeneralTrackerTransition);
  publishReferenceTransition();
  return true;
}

void RuntimeControlLoop::advanceTransition() {
  if (!active_ || active_kind_ != ActiveKind::Transition || !active_track_) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::InternalError);
    enterGeneralTrackerIdleState();
    return;
  }

  if (!consumeStepDue(active_policy_ticks_until_next_, activePolicyIntervalTicks())) {
    return;
  }

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_->frame = 0;
    active_first_advance_ = false;
  } else if (active_->frames > 0 && active_->frame + 1 < active_->frames) {
    ++active_->frame;
  }

  publishReferenceTransition();
  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
    publishReferenceTransition();
    completeTransition();
  }
}

void RuntimeControlLoop::advanceTransitionWithPolicy() {
  if (!active_ || active_kind_ != ActiveKind::Transition || !active_track_) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::InternalError);
    enterGeneralTrackerIdleState();
    return;
  }

  if (!policy_runner_) {
    enterFault(ErrorCode::ModelInferenceFailed,
               RobotState::Fault,
               "policy_inference_failed");
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::ModelInferenceFailed);
    return;
  }

  if (!consumeGeneralTrackerPolicyDue()) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state,
                        robot_io_->lowCmdOccupancy(),
                        expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    abortTransition(MotionState::Failed, StopReason::None, readiness.err);
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  if (active_first_advance_) {
    active_->started_at = std::chrono::steady_clock::now();
    active_first_advance_ = false;
  }
  active_->frame = activePlaybackFrame();
  publishReferenceTransition();

  PolicyStepResult step;
  try {
    step = policy_runner_->step(active_->frame, *low_state, *policy_, lowCmdBaseFrame());
  } catch (const std::exception&) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::ModelInferenceFailed);
    enterFault(ErrorCode::ModelInferenceFailed,
               RobotState::Fault,
               "policy_inference_failed",
               readiness.low_ms);
    return;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    abortTransition(MotionState::Failed,
                    StopReason::None,
                    ErrorCode::InternalError);
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return;
  }

  advanceActivePlaybackTime();
  if (active_->frames == 0 || active_->frame + 1 >= active_->frames) {
    active_->frame = active_->frames == 0 ? 0 : active_->frames - 1;
    publishReferenceTransition();
    completeTransition();
  }
}

void RuntimeControlLoop::completeTransition() {
  if (!transition_) {
    abortTransition();
    enterGeneralTrackerIdleState();
    return;
  }

  PendingTransition target = std::move(*transition_);
  transition_.reset();
  active_.reset();
  active_track_.reset();
  policy_runner_.reset();
  clearReference();

  if (target.target_kind == TransitionTargetKind::Standby) {
    if (target.target_track && startStandbyPlayback(std::move(target))) {
      return;
    }
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    post_stop_control_ = ControlMode::StandbyVelocity;
    enterGeneralTrackerIdleState();
    return;
  }

  if (!target.target_request || !target.target_track) {
    active_kind_ = ActiveKind::None;
    idle_current_index_.reset();
    enterGeneralTrackerIdleState();
    return;
  }

  std::optional<LowStateSample> entry_low_state;
  if (hasPolicyRuntime()) {
    entry_low_state = readLowStateForStatus();
    const RobotReadinessStatus readiness =
        mapRobotReadiness(entry_low_state,
                          robot_io_->lowCmdOccupancy(),
                          expected_mode_machine_);
    if (readiness.err != ErrorCode::Ok) {
      if (target.target_kind == TransitionTargetKind::User) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = readiness.err;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      if (readinessRequiresFault(readiness)) {
        enterFault(readiness.err,
                   readiness.robot,
                   readiness.block,
                   readiness.low_ms);
      } else {
        enterPassiveState(readiness);
      }
      return;
    }
    applyReadiness(readiness);
  }

  std::optional<PolicyStepRunner> runner;
  if (hasPolicyRuntime()) {
    try {
      runner.emplace(*deploy_config_,
                     target.target_track,
                     *entry_low_state,
                     expected_mode_machine_);
    } catch (const std::exception&) {
      if (target.target_kind == TransitionTargetKind::User) {
        target.target_request->state = MotionState::Failed;
        target.target_request->err = ErrorCode::ModelInferenceFailed;
        target.target_request->ended_at = std::chrono::steady_clock::now();
        status_.publishRunStatus(toStatus(*target.target_request));
      }
      active_kind_ = ActiveKind::None;
      idle_current_index_.reset();
      enterFault(ErrorCode::ModelInferenceFailed,
                 RobotState::Fault,
                 "policy_inference_failed",
                 runtime_state_.low_ms);
      return;
    }
  }

  target.target_request->state = MotionState::Running;
  target.target_request->frame = 0;
  target.target_request->err = ErrorCode::Ok;
  target.target_request->stop_reason = StopReason::None;
  target.target_request->started_at = std::chrono::steady_clock::now();
  active_ = std::move(*target.target_request);
  active_track_ = std::move(target.target_track);
  policy_runner_ = std::move(runner);
  active_kind_ = target.target_kind == TransitionTargetKind::User ? ActiveKind::User
                                                                  : ActiveKind::Idle;
  idle_current_index_ = target.target_kind == TransitionTargetKind::Idle
                            ? target.idle_index
                            : std::optional<std::size_t>{};
  enterTrackActiveState(entry_low_state,
                        target.reduced_target_startup_hold ? StartupHoldMode::Reduced
                                                           : StartupHoldMode::Run);
  publishReferenceActive();
  if (active_kind_ == ActiveKind::User) {
    publishActive();
  }
}

void RuntimeControlLoop::finishTransitionTarget(MotionState state,
                                                StopReason reason,
                                                ErrorCode error) {
  if (!transition_ || transition_->target_kind != TransitionTargetKind::User ||
      !transition_->target_request || transition_->target_request->id.empty()) {
    return;
  }

  MotionRequest target = *transition_->target_request;
  target.state = state;
  target.stop_reason = reason;
  target.err = error;
  target.ended_at = std::chrono::steady_clock::now();
  status_.publishRunStatus(toStatus(target));
  transition_->target_request = std::move(target);
}

void RuntimeControlLoop::abortTransition(MotionState target_state,
                                         StopReason reason,
                                         ErrorCode error) {
  if (active_kind_ != ActiveKind::Transition) {
    return;
  }
  finishTransitionTarget(target_state, reason, error);
  active_.reset();
  active_kind_ = ActiveKind::None;
  active_track_.reset();
  transition_.reset();
  policy_runner_.reset();
  idle_current_index_.reset();
  clearReference();
}

bool RuntimeControlLoop::failActiveReadiness(
    const RobotReadinessStatus& readiness) {
  if (active_kind_ == ActiveKind::Idle) {
    stopIdleActive();
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return true;
  }
  if (readinessRequiresFault(readiness)) {
    failActiveWithFault(readiness.err,
                        readiness.robot,
                        readiness.block,
                        readiness.low_ms);
  } else {
    finishActive(MotionState::Failed, StopReason::None, readiness.err);
    enterPassiveState(readiness);
  }
  return true;
}

void RuntimeControlLoop::publishControlIfReady() {
  if (!hasControlRuntime() || active_ || !waiting_.empty()) {
    return;
  }

  if (ctrl_ == ControllerState::FixStand) {
    writeFixStand();
  } else if (ctrl_ == ControllerState::StandbyVelocity) {
    writeStandbyVelocity();
  }
}

bool RuntimeControlLoop::writeFixStand() {
  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const LowCmdOccupancy occupancy = robot_io_->lowCmdOccupancy();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, occupancy, expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
      return false;
    }
    if (shouldLatchPassiveReason(readiness)) {
      enterPassiveState(readiness);
      return false;
    }
    applyReadiness(readiness);
    return false;
  }
  applyReadiness(readiness);

  LowCmdFrame frame;
  try {
    frame = fixstand_runner_->step(*low_state, lowCmdBaseFrame());
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }

  try {
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }
  return true;
}

bool RuntimeControlLoop::writeStandbyVelocity() {
  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return false;
  }
  applyReadiness(readiness);

  if (!consumeStepDue(velocity_policy_ticks_until_next_, velocityPolicyIntervalTicks())) {
    return republishLowCmdBuffer();
  }

  VelocityStepResult step;
  try {
    step = velocity_runner_->step(*low_state, *velocity_policy_, lowCmdBaseFrame());
    if (fsm_state_ == RuntimeInternalState::GeneralTrackerIdle) {
      applyGeneralTrackerIdleHold(step.low_cmd);
    }
  } catch (const std::exception&) {
    enterFault(ErrorCode::ModelInferenceFailed,
               RobotState::Fault,
               "policy_inference_failed",
               readiness.low_ms);
    return false;
  }

  try {
    writeLowCmdFrame(step.low_cmd);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }
  return true;
}

void RuntimeControlLoop::publishIdleHoldIfReady() {
  if (!hasPolicyRuntime() || ctrl_ != ControllerState::Idle || active_ ||
      !waiting_.empty() || !runtime_state_.ready || runtime_state_.err != ErrorCode::Ok) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return;
  }
  applyReadiness(readiness);

  try {
    const LowCmdFrame frame =
        makeHoldLowCmdFrame(*deploy_config_, *low_state, expected_mode_machine_);
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
  }
}

bool RuntimeControlLoop::writeStoppingHold() {
  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readiness.err != ErrorCode::Ok) {
    stopping_hold_ticks_remaining_ = 0;
    stop_to_idle_pending_ = false;
    failStoppingActiveWithFault(readiness.err);
    if (readinessRequiresFault(readiness)) {
      enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    } else {
      enterPassiveState(readiness);
    }
    return false;
  }
  applyReadiness(readiness);

  try {
    const LowCmdFrame frame =
        makeHoldLowCmdFrame(*deploy_config_, *low_state, expected_mode_machine_);
    writeLowCmdFrame(frame);
  } catch (const std::exception&) {
    stopping_hold_ticks_remaining_ = 0;
    stop_to_idle_pending_ = false;
    failStoppingActiveWithFault(ErrorCode::InternalError);
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               readiness.low_ms);
    return false;
  }

  return true;
}

void RuntimeControlLoop::applyGeneralTrackerIdleHold(LowCmdFrame& frame) const {
  if (!fixstand_config_ || !velocity_deploy_config_) {
    return;
  }

  std::array<bool, kSdkMotorCount> velocity_controlled{};
  for (std::size_t i = 0; i < velocity_deploy_config_->joint_ids_map.size(); ++i) {
    const int logical = velocity_deploy_config_->joint_ids_map.at(i);
    if (logical < 0) {
      continue;
    }
    int slot = logical;
    if (!velocity_deploy_config_->sdk_joint_ids_map.empty()) {
      const auto logical_index = static_cast<std::size_t>(logical);
      if (logical_index >= velocity_deploy_config_->sdk_joint_ids_map.size()) {
        continue;
      }
      slot = velocity_deploy_config_->sdk_joint_ids_map.at(logical_index);
    }
    if (slot >= 0 && slot < static_cast<int>(kSdkMotorCount)) {
      velocity_controlled.at(static_cast<std::size_t>(slot)) = true;
    }
  }

  const std::size_t count =
      std::min<std::size_t>({kFixStandMotorCount,
                             kSdkMotorCount,
                             fixstand_config_->target_q.size(),
                             fixstand_config_->kp.size(),
                             fixstand_config_->kd.size()});
  for (std::size_t slot = 0; slot < count; ++slot) {
    if (velocity_controlled.at(slot)) {
      continue;
    }
    MotorCommand& motor = frame.motors.at(slot);
    motor.mode = 1;
    motor.q = static_cast<float>(fixstand_config_->target_q.at(slot));
    motor.dq = 0.0F;
    motor.kp = static_cast<float>(fixstand_config_->kp.at(slot));
    motor.kd = static_cast<float>(fixstand_config_->kd.at(slot));
    motor.tau = 0.0F;
  }
}

bool RuntimeControlLoop::republishLowCmdBuffer() {
  if (!lowcmd_buffer_) {
    return false;
  }

  try {
    writeLowCmdFrame(*lowcmd_buffer_);
  } catch (const std::exception&) {
    enterFault(ErrorCode::InternalError,
               RobotState::Fault,
               "lowcmd_write_failed",
               runtime_state_.low_ms);
    return false;
  }
  return true;
}

const LowCmdFrame* RuntimeControlLoop::lowCmdBaseFrame() const {
  return lowcmd_buffer_ ? &*lowcmd_buffer_ : nullptr;
}

void RuntimeControlLoop::writeLowCmdFrame(const LowCmdFrame& frame) {
  robot_io_->writeLowCmd(frame);
  lowcmd_buffer_ = frame;
}

void RuntimeControlLoop::markActiveStopping(StopReason reason) {
  if (!active_) {
    return;
  }
  if (active_kind_ == ActiveKind::Transition) {
    abortTransition(MotionState::Canceled, reason, ErrorCode::Ok);
    return;
  }

  active_->state = MotionState::Stopping;
  active_->stop_reason = reason;
  active_->err = ErrorCode::Ok;
  if (active_->executor == MotionExecutor::LocoUpper) {
    active_->loco.phase = LocoPhase::Stopping;
  }
  if (active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(*active_));
  }
  active_track_.reset();
  loco_upper_.reset();
  policy_runner_.reset();
  clearReference();
}

void RuntimeControlLoop::completeStoppingActive(MotionState state, ErrorCode error) {
  if (!active_ || active_->state != MotionState::Stopping) {
    return;
  }

  active_->state = state;
  active_->err = error;
  if (active_->executor == MotionExecutor::LocoUpper) {
    if (state == MotionState::Stopped) {
      active_->loco.phase = LocoPhase::Stopped;
    } else if (state == MotionState::Failed) {
      active_->loco.phase = LocoPhase::Failed;
    }
  }
  active_->ended_at = std::chrono::steady_clock::now();
  if (active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(*active_));
  }
  active_.reset();
  active_kind_ = ActiveKind::None;
  idle_current_index_.reset();
  active_track_.reset();
  loco_upper_.reset();
  transition_.reset();
  clearReference();
}

std::optional<MotionRequest> RuntimeControlLoop::finishActive(MotionState state,
                                                              StopReason reason,
                                                              ErrorCode error) {
  if (!active_) {
    return std::nullopt;
  }

  active_->state = state;
  active_->stop_reason = reason;
  active_->err = error;
  if (active_->executor == MotionExecutor::LocoUpper) {
    switch (state) {
      case MotionState::Done:
        active_->loco.phase = LocoPhase::Done;
        break;
      case MotionState::Stopped:
        active_->loco.phase = LocoPhase::Stopped;
        break;
      case MotionState::Canceled:
        active_->loco.phase = LocoPhase::Canceled;
        break;
      case MotionState::Failed:
        active_->loco.phase = LocoPhase::Failed;
        break;
      case MotionState::Holding:
        active_->loco.phase = LocoPhase::Holding;
        break;
      case MotionState::Stopping:
        active_->loco.phase = LocoPhase::Stopping;
        break;
      case MotionState::Queued:
        active_->loco.phase = LocoPhase::Queued;
        break;
      case MotionState::Running:
        break;
    }
  }
  active_->ended_at = std::chrono::steady_clock::now();
  MotionRequest completed = *active_;
  if (active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(completed));
  }
  active_.reset();
  active_kind_ = ActiveKind::None;
  idle_current_index_.reset();
  active_track_.reset();
  loco_upper_.reset();
  policy_runner_.reset();
  transition_.reset();
  clearReference();
  return completed;
}

void RuntimeControlLoop::stopIdleActive() {
  if (active_kind_ != ActiveKind::Idle) {
    return;
  }
  active_.reset();
  active_kind_ = ActiveKind::None;
  idle_current_index_.reset();
  active_track_.reset();
  loco_upper_.reset();
  policy_runner_.reset();
  transition_.reset();
  clearReference();
  if (fsm_state_ == RuntimeInternalState::GeneralTrackerActive ||
      fsm_state_ == RuntimeInternalState::GeneralTrackerTransition ||
      fsm_state_ == RuntimeInternalState::Stopping) {
    enterGeneralTrackerIdleState();
  }
}

void RuntimeControlLoop::clearIdleConfig() {
  idle_config_.clear();
  idle_next_index_ = 0;
  idle_current_index_.reset();
  status_.clearIdleConfig();
}

void RuntimeControlLoop::enterStopping(StopReason reason) {
  if (fsm_state_ == RuntimeInternalState::Stopping) {
    return;
  }

  enterInternalState(RuntimeInternalState::Stopping);
  stop_reason_ = reason;
  stop_to_idle_pending_ = true;
  stopping_hold_ticks_remaining_ = hasPolicyRuntime() ? stopHoldTicks(reason) : 0;
}

void RuntimeControlLoop::enterUrgentStopping() {
  enterInternalState(RuntimeInternalState::Stopping);
  ctrl_ = ControllerState::UrgentStopping;
  stop_reason_ = StopReason::UrgentStop;
  stop_to_idle_pending_ = true;
  stopping_hold_ticks_remaining_ =
      hasPolicyRuntime() ? stopHoldTicks(StopReason::UrgentStop) : 0;
}

std::size_t RuntimeControlLoop::ticksForPeriod(double seconds) const {
  if (!std::isfinite(seconds) || !std::isfinite(config_.hz) || seconds <= 0.0 ||
      config_.hz <= 0.0) {
    return 1;
  }

  const double max_ticks =
      static_cast<double>(std::numeric_limits<std::size_t>::max());
  const double ticks = std::ceil(seconds * config_.hz);
  if (!std::isfinite(ticks) || ticks <= 1.0) {
    return 1;
  }
  if (ticks >= max_ticks) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(ticks);
}

std::size_t RuntimeControlLoop::ticksForRate(double rate_hz) const {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    return 1;
  }
  return ticksForPeriod(1.0 / rate_hz);
}

bool RuntimeControlLoop::consumeStepDue(std::size_t& ticks_until_next,
                                        std::size_t interval_ticks) {
  if (interval_ticks <= 1) {
    ticks_until_next = 0;
    return true;
  }
  if (ticks_until_next > 0) {
    --ticks_until_next;
    return false;
  }
  ticks_until_next = interval_ticks - 1;
  return true;
}

double RuntimeControlLoop::generalTrackerPolicyStepDt() const {
  if (deploy_config_ && std::isfinite(deploy_config_->step_dt) &&
      deploy_config_->step_dt > 0.0) {
    return deploy_config_->step_dt;
  }
  if (std::isfinite(config_.hz) && config_.hz > 0.0) {
    return 1.0 / config_.hz;
  }
  return 0.02;
}

void RuntimeControlLoop::resetGeneralTrackerPolicyTiming(double playback_time_s) {
  general_tracker_policy_phase_s_ = 0.0;
  general_tracker_next_policy_time_s_ = 0.0;
  active_playback_time_s_ =
      std::isfinite(playback_time_s) && playback_time_s > 0.0
          ? playback_time_s
          : 0.0;
}

bool RuntimeControlLoop::consumeGeneralTrackerPolicyDue() {
  const double step_dt = generalTrackerPolicyStepDt();
  const double runtime_dt =
      std::isfinite(config_.hz) && config_.hz > 0.0 ? 1.0 / config_.hz : step_dt;
  constexpr double kPhaseEpsilon = 1.0e-12;
  const bool due =
      general_tracker_policy_phase_s_ + kPhaseEpsilon >=
      general_tracker_next_policy_time_s_;
  if (due) {
    general_tracker_next_policy_time_s_ += step_dt;
  }
  general_tracker_policy_phase_s_ += runtime_dt;
  return due;
}

double RuntimeControlLoop::locoLowerPolicyStepDt() const {
  if (loco_lower_deploy_config_ &&
      std::isfinite(loco_lower_deploy_config_->step_dt) &&
      loco_lower_deploy_config_->step_dt > 0.0) {
    return loco_lower_deploy_config_->step_dt;
  }
  if (std::isfinite(config_.hz) && config_.hz > 0.0) {
    return 1.0 / config_.hz;
  }
  return 0.02;
}

void RuntimeControlLoop::resetLocoLowerPolicyTiming() {
  loco_lower_policy_phase_s_ = 0.0;
  loco_lower_next_policy_time_s_ = 0.0;
}

bool RuntimeControlLoop::consumeLocoLowerPolicyDue() {
  const double step_dt = locoLowerPolicyStepDt();
  const double runtime_dt =
      std::isfinite(config_.hz) && config_.hz > 0.0 ? 1.0 / config_.hz : step_dt;
  constexpr double kPhaseEpsilon = 1.0e-12;
  const bool due =
      loco_lower_policy_phase_s_ + kPhaseEpsilon >=
      loco_lower_next_policy_time_s_;
  if (due) {
    loco_lower_next_policy_time_s_ += step_dt;
  }
  loco_lower_policy_phase_s_ += runtime_dt;
  return due;
}

std::size_t RuntimeControlLoop::activePlaybackFrame() const {
  if (!active_ || active_->frames == 0) {
    return 0;
  }
  return referenceFrameIndex(active_playback_time_s_, active_->fps, active_->frames);
}

double RuntimeControlLoop::activePlaybackEndTime() const {
  if (!active_) {
    return 0.0;
  }
  if (std::isfinite(active_->duration_s) && active_->duration_s >= 0.0) {
    return active_->duration_s;
  }
  if (active_->frames > 0 && std::isfinite(active_->fps) && active_->fps > 0.0) {
    return static_cast<double>(active_->frames - 1) / active_->fps;
  }
  return 0.0;
}

void RuntimeControlLoop::advanceActivePlaybackTime() {
  if (!active_) {
    active_playback_time_s_ = 0.0;
    return;
  }
  const double end_time = activePlaybackEndTime();
  const double next_time = active_playback_time_s_ + generalTrackerPolicyStepDt();
  active_playback_time_s_ =
      std::isfinite(end_time) && end_time >= 0.0 ? std::min(next_time, end_time)
                                                 : next_time;
}

std::size_t RuntimeControlLoop::velocityPolicyIntervalTicks() const {
  if (!velocity_deploy_config_) {
    return 1;
  }
  return ticksForPeriod(velocity_deploy_config_->step_dt);
}

std::size_t RuntimeControlLoop::activePolicyIntervalTicks() const {
  if (!active_) {
    return 1;
  }
  return ticksForRate(active_->fps);
}

std::size_t RuntimeControlLoop::policyStartupHoldPolicySteps(double duration_s) const {
  const double step_dt = generalTrackerPolicyStepDt();
  if (!std::isfinite(step_dt) || step_dt <= 0.0 ||
      !std::isfinite(duration_s) || duration_s <= 0.0) {
    return 1;
  }

  const double max_steps =
      static_cast<double>(std::numeric_limits<std::size_t>::max());
  const double steps = std::ceil(duration_s / step_dt);
  if (!std::isfinite(steps) || steps <= 1.0) {
    return 1;
  }
  if (steps >= max_steps) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(steps);
}

std::size_t RuntimeControlLoop::stopHoldTicks(StopReason reason) const {
  if (reason == StopReason::UrgentStop) {
    return 0;
  }
  if (std::isfinite(config_.stop_hold_s) && config_.stop_hold_s > 0.0) {
    return ticksForPeriod(config_.stop_hold_s);
  }
  return 0;
}

std::optional<double> RuntimeControlLoop::transitionDurationForUse() const {
  if (!std::isfinite(config_.transition_duration_s) ||
      config_.transition_duration_s <= 0.0 ||
      config_.transition_duration_s > kMaxTransitionDurationS) {
    return std::nullopt;
  }
  return config_.transition_duration_s;
}

SyntheticTransitionLimits RuntimeControlLoop::transitionLimitsForUse() const {
  return SyntheticTransitionLimits{
      config_.transition_max_velocity,
      config_.transition_max_acceleration,
      config_.transition_max_jerk,
  };
}

double RuntimeControlLoop::transitionSampleFpsForTarget(const TrkTrack& target) const {
  if (hasPolicyRuntime()) {
    const double step_dt = generalTrackerPolicyStepDt();
    if (std::isfinite(step_dt) && step_dt > 0.0) {
      return 1.0 / step_dt;
    }
  }
  return target.metadata.fps;
}

std::optional<std::size_t> RuntimeControlLoop::transitionFrameCountForTarget(
    const TrkTrack& target) const {
  const std::optional<double> duration_s = transitionDurationForUse();
  const double fps = transitionSampleFpsForTarget(target);
  if (!duration_s || !std::isfinite(fps) || fps <= 0.0) {
    return std::nullopt;
  }

  const double intervals_d =
      std::ceil(*duration_s * fps - kTransitionFrameCountEpsilon);
  const double max_size =
      static_cast<double>(std::numeric_limits<std::size_t>::max() - 1);
  if (!std::isfinite(intervals_d) || intervals_d < 1.0 ||
      intervals_d > max_size) {
    return std::nullopt;
  }

  const std::size_t duration_intervals =
      static_cast<std::size_t>(intervals_d);
  const std::size_t min_intervals =
      config_.transition_min_frames > 0
          ? std::max<std::size_t>(1, config_.transition_min_frames - 1)
          : 1;
  return std::max(duration_intervals, min_intervals) + 1;
}

SyntheticTransitionOptions RuntimeControlLoop::transitionOptionsForTarget(
    const TrkTrack& target) const {
  SyntheticTransitionOptions options;
  options.max_duration_s = config_.transition_duration_s;
  options.min_frames = config_.transition_min_frames;
  if (const std::optional<std::size_t> frames =
          transitionFrameCountForTarget(target)) {
    const double fps = transitionSampleFpsForTarget(target);
    options.min_frames = *frames;
    options.max_duration_s = static_cast<double>(*frames - 1) / fps;
  }
  options.duration_dt_tolerance_s = config_.transition_duration_dt_tolerance_s;
  options.limits = transitionLimitsForUse();
  return options;
}

SyntheticTransitionOptions RuntimeControlLoop::controlledTransitionOptionsForTarget(
    const TrkTrack& target) const {
  SyntheticTransitionOptions options = transitionOptionsForTarget(target);
  options.contact_mode = SyntheticTransitionContactMode::kSameValid;
  return options;
}

void RuntimeControlLoop::publishActive() {
  if (active_ && active_kind_ == ActiveKind::User) {
    status_.publishRunStatus(toStatus(*active_));
  }
}

void RuntimeControlLoop::publishReferenceActive() {
  if (reference_sink_ == nullptr || !active_ || !active_track_ ||
      (active_kind_ != ActiveKind::User && active_kind_ != ActiveKind::Idle)) {
    return;
  }
  try {
    const std::string id = active_kind_ == ActiveKind::User ? active_->id : "";
    const auto snapshot = makeReferenceFrameSnapshot(id, *active_track_, active_->frame);
    if (snapshot) {
      reference_sink_->publish(*snapshot);
    } else {
      reference_sink_->clear();
    }
  } catch (...) {
  }
}

void RuntimeControlLoop::publishReferenceTransition() {
  if (reference_sink_ == nullptr || !active_ ||
      active_kind_ != ActiveKind::Transition || !active_track_) {
    return;
  }
  try {
    const auto snapshot = makeReferenceFrameSnapshot("", *active_track_, active_->frame);
    if (snapshot) {
      reference_sink_->publish(*snapshot);
    } else {
      reference_sink_->clear();
    }
  } catch (...) {
  }
}

void RuntimeControlLoop::clearReference() {
  if (reference_sink_ == nullptr) {
    return;
  }
  try {
    reference_sink_->clear();
  } catch (...) {
  }
}

void RuntimeControlLoop::publishSnapshot() {
  StatusSnapshot snapshot;
  snapshot.ready = hasPolicyRuntime() ? runtime_state_.ready : true;
  snapshot.mode = mode_;
  snapshot.robot =
      hasPolicyRuntime() && !runtime_state_.ready ? runtime_state_.robot : robotState();
  snapshot.ctrl = ctrl_;
  snapshot.stop_reason =
      (ctrl_ == ControllerState::Stopping ||
       ctrl_ == ControllerState::UrgentStopping)
          ? stop_reason_
          : StopReason::None;
  snapshot.hz = config_.hz;
  const bool public_loco_standby_handoff =
      active_kind_ == ActiveKind::User && active_ &&
      active_->executor == MotionExecutor::LocoUpper &&
      active_->state == MotionState::Stopping &&
      active_->stop_reason == StopReason::Stop &&
      post_stop_control_ == ControlMode::StandbyVelocity;
  if (public_loco_standby_handoff) {
    snapshot.active = {ActiveKind::Transition, ""};
  } else if (active_kind_ == ActiveKind::User && active_) {
    snapshot.active = {ActiveKind::User, active_->id};
  } else if (active_kind_ == ActiveKind::Idle && active_) {
    snapshot.active = {ActiveKind::Idle, ""};
  } else if (active_kind_ == ActiveKind::Transition && active_) {
    snapshot.active = {ActiveKind::Transition, ""};
  } else {
    snapshot.active = {ActiveKind::None, ""};
  }
  snapshot.queue.limit = config_.queue_limit;
  snapshot.queue.ids = waitingIds();
  snapshot.queue.n = snapshot.queue.ids.size();
  snapshot.idle = idleStatus();
  snapshot.loco_upper.ready = hasLocoUpperRuntime();
  if (hasPolicyRuntime()) {
    snapshot.low_ms = runtime_state_.low_ms;
    snapshot.block = runtime_state_.block;
    snapshot.err = runtime_state_.err;
  } else {
    snapshot.err = ErrorCode::Ok;
  }
  snapshot.passive_reason = passive_reason_;
  if (active_ && active_kind_ == ActiveKind::User &&
      !public_loco_standby_handoff) {
    snapshot.exec = toStatus(*active_);
  }
  if (public_loco_standby_handoff && active_) {
    snapshot.transition.active = true;
    snapshot.transition.target = "standby";
    snapshot.transition.frame = active_->frame;
    snapshot.transition.frames = active_->frames;
    snapshot.transition.progress =
        computeProgress(active_->frame, active_->frames, active_->state);
  }
  if (active_ && active_kind_ == ActiveKind::Transition && transition_) {
    snapshot.transition.active = true;
    switch (transition_->target_kind) {
      case TransitionTargetKind::User:
        snapshot.transition.target = "user";
        break;
      case TransitionTargetKind::Idle:
        snapshot.transition.target = "idle";
        break;
      case TransitionTargetKind::Standby:
        snapshot.transition.target = "standby";
        break;
    }
    snapshot.transition.target_id = transition_->target_id;
    snapshot.transition.target_state = transition_->target_state;
    snapshot.transition.frame = active_->frame;
    snapshot.transition.frames = active_->frames;
    snapshot.transition.progress =
        computeProgress(active_->frame, active_->frames, active_->state);
  }
  fillSnapshotPose(snapshot);
  const HealthSnapshot health = healthFromSnapshot(snapshot);
  status_.publishSnapshot(std::move(snapshot));
  for (const auto& ack : consumed_control_acks_) {
    status_.clearPendingControl(ack.mode, ack.sequence);
  }
  consumed_control_acks_.clear();
  status_.publishHealthSnapshot(health);
}

std::optional<LowStateSample> RuntimeControlLoop::readLowStateForStatus() {
  if (robot_io_ == nullptr) {
    latest_low_state_.reset();
    return std::nullopt;
  }
  latest_low_state_ = robot_io_->readLowState();
  return latest_low_state_;
}

std::optional<HighStateSample> RuntimeControlLoop::readHighStateForStatus() {
  if (robot_io_ == nullptr) {
    latest_high_state_.reset();
    return std::nullopt;
  }
  latest_high_state_ = robot_io_->readHighState();
  return latest_high_state_;
}

void RuntimeControlLoop::fillSnapshotPose(StatusSnapshot& snapshot) {
  if (!hasPolicyRuntime()) {
    return;
  }

  readHighStateForStatus();
  if (latest_low_state_) {
    snapshot.pose.q_wxyz = latest_low_state_->quat_wxyz;
    snapshot.pose.gyro_xyz = latest_low_state_->gyro;
  }
  if (latest_high_state_ && latest_high_state_->fresh) {
    snapshot.pose.position_xyz = latest_high_state_->position;
    snapshot.pose.velocity_xyz = latest_high_state_->linear_velocity;
  }
}

MotionStatus RuntimeControlLoop::toStatus(const MotionRequest& request) const {
  return makeMotionStatus(request);
}

std::vector<std::string> RuntimeControlLoop::waitingIds() const {
  std::vector<std::string> ids;
  ids.reserve(waiting_.size());
  for (const auto& request : waiting_) {
    ids.push_back(request.id);
  }
  return ids;
}

IdleStatus RuntimeControlLoop::idleStatus() const {
  IdleStatus status;
  status.enabled = !idle_config_.empty();
  status.n = idle_config_.size();
  status.active = active_kind_ == ActiveKind::Idle && active_.has_value();
  if (!status.active || !active_) {
    return status;
  }

  status.current = idle_current_index_;
  status.frame = active_->frame;
  status.frames = active_->frames;
  status.time_s = active_->fps > 0.0 ? static_cast<double>(active_->frame) / active_->fps
                                     : 0.0;
  status.duration_s = active_->duration_s;
  status.progress = computeProgress(active_->frame, active_->frames, active_->state);
  return status;
}

RobotState RuntimeControlLoop::robotState() const {
  if (ctrl_ == ControllerState::Fault) {
    return RobotState::Fault;
  }
  if (ctrl_ == ControllerState::Running || ctrl_ == ControllerState::Preparing) {
    return RobotState::Running;
  }
  if (ctrl_ == ControllerState::Stopping ||
      ctrl_ == ControllerState::UrgentStopping) {
    return RobotState::Holding;
  }
  return RobotState::Idle;
}

bool RuntimeControlLoop::hasPolicyRuntime() const {
  return robot_io_ != nullptr && policy_ != nullptr && deploy_config_.has_value();
}

bool RuntimeControlLoop::hasControlRuntime() const {
  return hasPolicyRuntime() && velocity_policy_ != nullptr &&
         velocity_deploy_config_.has_value() && fixstand_config_.has_value() &&
         passive_config_.has_value() &&
         fixstand_runner_.has_value() && velocity_runner_.has_value();
}

bool RuntimeControlLoop::hasLocoUpperRuntime() const {
  return hasPolicyRuntime() && loco_lower_policy_ != nullptr &&
         loco_lower_deploy_config_.has_value() &&
         loco_upper_composer_config_.has_value() &&
         fixstand_config_.has_value();
}

void RuntimeControlLoop::refreshReadinessForPolicyRuntime() {
  if (!hasPolicyRuntime()) {
    return;
  }

  const std::optional<LowStateSample> low_state = readLowStateForStatus();
  const RobotReadinessStatus readiness =
      mapRobotReadiness(low_state, robot_io_->lowCmdOccupancy(), expected_mode_machine_);
  if (readinessRequiresFault(readiness)) {
    enterFault(readiness.err, readiness.robot, readiness.block, readiness.low_ms);
    return;
  }
  if (readiness.err != ErrorCode::Ok) {
    enterPassiveState(readiness);
    return;
  }
  applyReadiness(readiness);
}

void RuntimeControlLoop::applyReadiness(const RobotReadinessStatus& readiness) {
  runtime_state_.ready = readiness.err == ErrorCode::Ok;
  runtime_state_.robot = readiness.robot;
  runtime_state_.low_ms = readiness.low_ms.value_or(0);
  runtime_state_.block = readiness.block;
  runtime_state_.err = readiness.err;
}

bool RuntimeControlLoop::readinessRequiresFault(
    const RobotReadinessStatus& readiness) const {
  return readiness.block == "lowcmd_occupied";
}

bool RuntimeControlLoop::isSafetyTerminalState() const {
  return fsm_state_ == RuntimeInternalState::Fault ||
         fsm_state_ == RuntimeInternalState::Passive;
}

void RuntimeControlLoop::enterFault(ErrorCode error,
                                    RobotState robot,
                                    std::string block,
                                    std::optional<std::size_t> low_ms) {
  failWaiting(error);
  enterInternalState(RuntimeInternalState::Fault);
  runtime_state_.ready = false;
  runtime_state_.robot = robot;
  runtime_state_.low_ms = low_ms.value_or(runtime_state_.low_ms);
  runtime_state_.block = std::move(block);
  runtime_state_.err = error;
}

void RuntimeControlLoop::failActiveWithFault(ErrorCode error,
                                             RobotState robot,
                                             std::string block,
                                             std::optional<std::size_t> low_ms) {
  finishActive(MotionState::Failed, StopReason::None, error);
  enterFault(error, robot, std::move(block), low_ms);
}

void RuntimeControlLoop::failStoppingActiveWithFault(ErrorCode error) {
  if (!active_ || active_->state != MotionState::Stopping) {
    return;
  }

  completeStoppingActive(MotionState::Failed, error);
}

}  // namespace agentic_et1_tracker
