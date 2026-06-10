#include "agentic_et1_tracker/policy/observation_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>

#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointDim = TrkSchema::kJointDim;
constexpr std::size_t kSdkDim = kSdkMotorCount;
constexpr std::size_t kAnchorBodyIndex = 14;
constexpr std::size_t kClnFutureCommandWidth = 35;
constexpr std::size_t kClnFootstateFutureCommandWidth = 41;
constexpr double kQuatNormEpsilon = 1.0e-12;
const Vec kIdentityRootOriB{1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F};
const Vec kZeroXyYawVel{0.0F, 0.0F, 0.0F};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct FrameKinematics {
  Quat ref_q;
  Quat robot_q;
  Quat relative_q;
};

ObservationBuilderError error(const std::string& message) {
  return ObservationBuilderError("observation builder error: " + message);
}

std::string sizeMessage(const std::string& field,
                        std::size_t expected,
                        std::size_t actual) {
  std::ostringstream out;
  out << field << " must contain at least " << expected << " entries, got " << actual;
  return out.str();
}

void requireSizeAtLeast(const std::string& field,
                        std::size_t actual,
                        std::size_t expected) {
  if (actual < expected) {
    throw error(sizeMessage(field, expected, actual));
  }
}

template <typename T>
void requireView(const std::string& field, const TrkArrayView<T>& view, std::size_t size) {
  requireSizeAtLeast(field, view.size, size);
  if (view.ptr == nullptr) {
    throw error(field + " must not be null");
  }
}

void requireExactSize(const std::string& field,
                      std::size_t actual,
                      std::size_t expected) {
  if (actual != expected) {
    std::ostringstream out;
    out << field << " must contain " << expected << " entries, got " << actual;
    throw error(out.str());
  }
}

void requireFinite(const std::string& field, double value) {
  if (!std::isfinite(value)) {
    throw error(field + " must be finite");
  }
}

Quat normalizeQuat(Quat q, const std::string& field) {
  requireFinite(field + ".w", q.w);
  requireFinite(field + ".x", q.x);
  requireFinite(field + ".y", q.y);
  requireFinite(field + ".z", q.z);

  const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!std::isfinite(norm_sq) || norm_sq <= kQuatNormEpsilon) {
    throw error(field + " must be a non-zero finite quaternion");
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  return {q.w * inv_norm, q.x * inv_norm, q.y * inv_norm, q.z * inv_norm};
}

Quat quatFromArray(const std::array<float, 4>& values, const std::string& field) {
  return normalizeQuat({values[0], values[1], values[2], values[3]}, field);
}

Quat quatFromView(const TrkArrayView<float>& view, const std::string& field) {
  requireView(field, view, 4);
  return normalizeQuat({view.ptr[0], view.ptr[1], view.ptr[2], view.ptr[3]}, field);
}

Quat conjugate(Quat q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Quat multiply(Quat a, Quat b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Quat yawQuat(double yaw) {
  return {std::cos(yaw * 0.5), 0.0, 0.0, std::sin(yaw * 0.5)};
}

Quat angleAxisX(double radians) {
  return {std::cos(radians * 0.5), std::sin(radians * 0.5), 0.0, 0.0};
}

Quat angleAxisZ(double radians) {
  return yawQuat(radians);
}

double yawFromQuat(Quat q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Vec3 rotateByQuat(Quat q, Vec3 v) {
  const Quat vec{0.0, v.x, v.y, v.z};
  const Quat rotated = multiply(multiply(q, vec), conjugate(q));
  return {rotated.x, rotated.y, rotated.z};
}

Vec3 rotateByYaw(double yaw, Vec3 v) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

Quat alignReferenceQuat(Quat q,
                        const ObservationBuilderState& state,
                        bool no_global_mode) {
  if (!no_global_mode) {
    return q;
  }
  return normalizeQuat(multiply(yawQuat(state.ref_world_align_yaw), q),
                       "anchor-aligned reference quaternion");
}

void rotationFirstTwoColumns(Quat q, Vec& out) {
  out = {
      static_cast<float>(1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
      static_cast<float>(2.0 * (q.x * q.y - q.z * q.w)),
      static_cast<float>(2.0 * (q.x * q.y + q.z * q.w)),
      static_cast<float>(1.0 - 2.0 * (q.x * q.x + q.z * q.z)),
      static_cast<float>(2.0 * (q.x * q.z - q.y * q.w)),
      static_cast<float>(2.0 * (q.y * q.z + q.x * q.w)),
  };
}

Vec copyView(const std::string& field, const TrkArrayView<float>& view, std::size_t size) {
  requireView(field, view, size);
  Vec out;
  out.reserve(size);
  for (std::size_t i = 0; i < size; ++i) {
    out.push_back(view.ptr[i]);
  }
  return out;
}

Vec3 rootVector(const std::string& field, const TrkArrayView<float>& view) {
  requireView(field, view, 3);
  return {view.ptr[0], view.ptr[1], view.ptr[2]};
}

Vec supportState(const TrkFrameView& frame) {
  requireView("left_foot_contact_state", frame.left_foot_contact_state, 1);
  requireView("right_foot_contact_state", frame.right_foot_contact_state, 1);

  const std::int64_t left = frame.left_foot_contact_state.ptr[0];
  const std::int64_t right = frame.right_foot_contact_state.ptr[0];
  if (left < 0 || left > 2) {
    throw error("left_foot_contact_state must be 0, 1, or 2");
  }
  if (right < 0 || right > 2) {
    throw error("right_foot_contact_state must be 0, 1, or 2");
  }

  Vec out(6, 0.0F);
  out[static_cast<std::size_t>(left)] = 1.0F;
  out[3 + static_cast<std::size_t>(right)] = 1.0F;
  return out;
}

void validateDeployConfigForBuilder(const DeployConfig& config) {
  requireExactSize("DeployConfig.joint_dim", config.joint_dim, kJointDim);
  requireExactSize("DeployConfig.default_joint_pos", config.default_joint_pos.size(),
                   kJointDim);
  requireExactSize("DeployConfig.sdk_joint_ids_map", config.sdk_joint_ids_map.size(),
                   kJointDim);

  for (std::size_t i = 0; i < config.default_joint_pos.size(); ++i) {
    requireFinite("DeployConfig.default_joint_pos[" + std::to_string(i) + "]",
                  config.default_joint_pos[i]);
  }
  for (std::size_t i = 0; i < config.sdk_joint_ids_map.size(); ++i) {
    const int sdk_index = config.sdk_joint_ids_map[i];
    if (sdk_index < 0 || sdk_index >= static_cast<int>(kSdkDim)) {
      throw error("DeployConfig.sdk_joint_ids_map[" + std::to_string(i) +
                  "] is outside the SDK motor slots");
    }
  }
}

void validateDeployConfigForState(const DeployConfig& config) {
  requireExactSize("DeployConfig.joint_dim", config.joint_dim, kJointDim);
  requireExactSize("DeployConfig.sdk_joint_ids_map", config.sdk_joint_ids_map.size(),
                   kJointDim);

  for (std::size_t i = 0; i < config.sdk_joint_ids_map.size(); ++i) {
    const int sdk_index = config.sdk_joint_ids_map[i];
    if (sdk_index < 0 || sdk_index >= static_cast<int>(kSdkDim)) {
      throw error("DeployConfig.sdk_joint_ids_map[" + std::to_string(i) +
                  "] is outside the SDK motor slots");
    }
  }
}

void validateLastAction(const Vec& last_action) {
  requireExactSize("last_action", last_action.size(), kJointDim);
}

Vec projectedGravity(Quat robot_q) {
  const Vec3 gravity = rotateByQuat(conjugate(robot_q), {0.0, 0.0, -1.0});
  return {static_cast<float>(gravity.x), static_cast<float>(gravity.y),
          static_cast<float>(gravity.z)};
}

Vec baseAngularVelocity(const LowStateSample& low_state) {
  return {low_state.gyro[0], low_state.gyro[1], low_state.gyro[2]};
}

Quat bodyQuat(const TrkFrameView& frame, std::size_t body_index, const std::string& field) {
  requireView(field, frame.body_quat_w, (body_index + 1) * 4);
  const std::size_t offset = body_index * 4;
  return normalizeQuat({frame.body_quat_w.ptr[offset + 0],
                        frame.body_quat_w.ptr[offset + 1],
                        frame.body_quat_w.ptr[offset + 2],
                        frame.body_quat_w.ptr[offset + 3]},
                       field);
}

std::array<double, kJointDim> zeroPolicyJointPositions() {
  std::array<double, kJointDim> joint_pos{};
  return joint_pos;
}

std::array<double, kJointDim> policyJointPositions(const DeployConfig& config,
                                                   const LowStateSample& low_state) {
  validateDeployConfigForState(config);
  std::array<double, kJointDim> joint_pos{};
  for (std::size_t policy_index = 0; policy_index < kJointDim; ++policy_index) {
    const auto sdk_index = static_cast<std::size_t>(config.sdk_joint_ids_map[policy_index]);
    joint_pos[policy_index] = low_state.motors[sdk_index].q;
  }
  return joint_pos;
}

Quat robotAnchorQuat(Quat robot_root_q,
                     const std::array<double, kJointDim>& policy_joint_pos) {
  Quat q = normalizeQuat(robot_root_q, "robot_anchor.root_quat");
  q = multiply(q, angleAxisX(policy_joint_pos[12]));
  q = multiply(q, angleAxisZ(policy_joint_pos[13]));
  return normalizeQuat(q, "robot_anchor quaternion");
}

double anchorFrameOffsetYaw(Quat ref_anchor_q, Quat robot_anchor_q) {
  const Quat offset_q =
      normalizeQuat(multiply(robot_anchor_q, conjugate(ref_anchor_q)),
                    "anchor frame offset quaternion");
  return yawFromQuat(offset_q);
}

std::array<bool, kJointDim> overrideJointMask(const DeployConfig& config) {
  std::array<bool, kJointDim> mask{};
  for (const int joint_id : config.override_joint_ids) {
    if (joint_id < 0 || joint_id >= static_cast<int>(kJointDim)) {
      throw error("DeployConfig.override_joint_ids contains an id outside policy order");
    }
    const auto index = static_cast<std::size_t>(joint_id);
    if (mask[index]) {
      throw error("DeployConfig.override_joint_ids contains duplicate ids");
    }
    mask[index] = true;
  }
  return mask;
}

void fillJointObservations(const DeployConfig& config,
                           const LowStateSample& low_state,
                           const Vec& reference_joint_pos,
                           Vec& joint_pos_rel,
                           Vec& joint_vel_rel) {
  joint_pos_rel.clear();
  joint_vel_rel.clear();
  joint_pos_rel.reserve(kJointDim);
  joint_vel_rel.reserve(kJointDim);
  requireExactSize("reference_joint_pos", reference_joint_pos.size(), kJointDim);
  const auto override_mask = overrideJointMask(config);

  for (std::size_t policy_index = 0; policy_index < kJointDim; ++policy_index) {
    const auto sdk_index = static_cast<std::size_t>(config.sdk_joint_ids_map[policy_index]);
    const MotorStateSample& motor = low_state.motors[sdk_index];
    const float q = override_mask[policy_index] ? reference_joint_pos[policy_index]
                                                : motor.q;
    joint_pos_rel.push_back(q -
                            static_cast<float>(config.default_joint_pos[policy_index]));
    joint_vel_rel.push_back(motor.dq);
  }
}

Vec commandVelocity(const TrkFrameView& frame,
                    Quat ref_q,
                    const ObservationBuilderState& state,
                    const ObservationBuilderConfig& builder_config) {
  Vec3 lin = rootVector("body_lin_vel_w", frame.body_lin_vel_w);
  Vec3 ang = rootVector("body_ang_vel_w", frame.body_ang_vel_w);

  if (builder_config.no_global_mode) {
    lin = rotateByYaw(state.ref_world_align_yaw, lin);
    ang = rotateByYaw(state.ref_world_align_yaw, ang);
  }

  const double current_ref_yaw = yawFromQuat(ref_q);
  lin = rotateByYaw(-current_ref_yaw, lin);
  ang = rotateByYaw(-current_ref_yaw, ang);

  return {static_cast<float>(lin.x), static_cast<float>(lin.y), static_cast<float>(ang.z)};
}

FrameKinematics frameKinematics(const TrkFrameView& frame,
                                const LowStateSample& low_state,
                                const ObservationBuilderState& state,
                                const ObservationBuilderConfig& builder_config) {
  const Quat ref_q_raw = quatFromView(frame.body_quat_w, "body_quat_w");
  const Quat robot_q_raw = quatFromArray(low_state.quat_wxyz, "low_state.quat_wxyz");
  const Quat ref_q =
      alignReferenceQuat(ref_q_raw, state, builder_config.no_global_mode);
  const Quat robot_q = robot_q_raw;
  const Quat relative_q =
      normalizeQuat(multiply(conjugate(robot_q), ref_q), "command_root_ori_b quaternion");
  return {ref_q, robot_q, relative_q};
}

Vec commandYaw(Quat ref_q, Quat robot_q) {
  const double yaw_error = yawFromQuat(ref_q) - yawFromQuat(robot_q);
  return {static_cast<float>(std::cos(yaw_error)),
          static_cast<float>(std::sin(yaw_error))};
}

TrkFrameView requireTrackFrame(const TrkTrack& track, std::size_t frame_index) {
  const auto frame = track.frame(frame_index);
  if (!frame.has_value()) {
    throw error("frame index is outside track frames");
  }
  return *frame;
}

void appendFutureCommand(const TrkFrameView& future_frame,
                         Quat robot_q,
                         Vec& out,
                         const ObservationBuilderState& state,
                         const ObservationBuilderConfig& builder_config,
                         bool force_motion_root_velocity) {
  const Quat future_ref_q_raw = quatFromView(future_frame.body_quat_w,
                                             "future_commands.body_quat_w");
  const Quat future_ref_q =
      alignReferenceQuat(future_ref_q_raw, state, builder_config.no_global_mode);
  const Quat future_relative_q =
      normalizeQuat(multiply(conjugate(robot_q), future_ref_q),
                    "future_commands.root_ori_b quaternion");

  Vec future_root_ori_b;
  if (force_motion_root_velocity || builder_config.use_motion_root_command) {
    rotationFirstTwoColumns(future_relative_q, future_root_ori_b);
  } else {
    future_root_ori_b = kIdentityRootOriB;
  }
  out.insert(out.end(), future_root_ori_b.begin(), future_root_ori_b.end());

  const Vec future_xy_yaw_vel =
      (force_motion_root_velocity || builder_config.use_motion_velocity_command)
          ? commandVelocity(future_frame, future_ref_q, state, builder_config)
          : kZeroXyYawVel;
  out.insert(out.end(), future_xy_yaw_vel.begin(), future_xy_yaw_vel.end());

  const Vec future_joint_pos = copyView("future_commands.joint_pos",
                                        future_frame.joint_pos, kJointDim);
  out.insert(out.end(), future_joint_pos.begin(), future_joint_pos.end());
}

Vec futureCommands(const DeployConfig& config,
                   const TrkTrack& track,
                   std::size_t frame_index,
                   Quat robot_q,
                   const ObservationBuilderState& state,
                   const ObservationBuilderConfig& builder_config) {
  if (track.metadata.frames == 0) {
    throw error("track must contain at least one frame");
  }
  const std::size_t expected_size = config.obs_history_length * config.obs_history_width;
  const bool include_foot_support =
      config.observation_contract == ObservationContract::GeneralTrackerCLNFootstate;
  const std::size_t expected_width =
      include_foot_support ? kClnFootstateFutureCommandWidth : kClnFutureCommandWidth;
  if (config.obs_history_width != expected_width) {
    std::ostringstream out;
    out << "CLN future command width must be " << expected_width;
    throw error(out.str());
  }

  Vec out;
  out.reserve(expected_size);
  for (std::size_t horizon_idx = 0; horizon_idx < config.obs_history_length; ++horizon_idx) {
    const std::size_t future_frame_index =
        std::min(frame_index + horizon_idx + 1, track.metadata.frames - 1);
    const TrkFrameView future_frame = requireTrackFrame(track, future_frame_index);
    appendFutureCommand(future_frame, robot_q, out, state, builder_config,
                        include_foot_support);
    if (include_foot_support) {
      const Vec future_support = supportState(future_frame);
      out.insert(out.end(), future_support.begin(), future_support.end());
    }
  }
  requireExactSize("future_commands", out.size(), expected_size);
  return out;
}

ObservationBuilderState makeObservationBuilderStateWithJoints(
    const TrkFrameView& first_frame,
    const LowStateSample& entry_low_state,
    const std::array<double, kJointDim>& entry_policy_joint_pos,
    const ObservationBuilderConfig& config) {
  ObservationBuilderState state;
  const Quat first_ref_q = quatFromView(first_frame.body_quat_w, "body_quat_w");
  const Quat entry_robot_q = quatFromArray(entry_low_state.quat_wxyz, "low_state.quat_wxyz");
  if (!config.no_global_mode) {
    return state;
  }

  state.first_ref_root_yaw = yawFromQuat(first_ref_q);
  state.entry_robot_yaw = yawFromQuat(entry_robot_q);
  const Quat ref_anchor_q =
      bodyQuat(first_frame, kAnchorBodyIndex, "body_quat_w anchor body");
  const Quat robot_anchor_q = robotAnchorQuat(entry_robot_q, entry_policy_joint_pos);
  state.ref_world_align_yaw = anchorFrameOffsetYaw(ref_anchor_q, robot_anchor_q);
  return state;
}

}  // namespace

ObservationBuilderConfig defaultObservationBuilderConfig(const DeployConfig& config) {
  ObservationBuilderConfig builder_config;
  if (config.observation_contract == ObservationContract::GeneralTracker) {
    builder_config.use_motion_root_command = true;
    builder_config.use_motion_velocity_command = true;
  }
  return builder_config;
}

ObservationBuilderState makeObservationBuilderState(
    const TrkFrameView& first_frame,
    const LowStateSample& entry_low_state,
    const ObservationBuilderConfig& config) {
  const std::array<double, kJointDim> joint_pos = zeroPolicyJointPositions();
  return makeObservationBuilderStateWithJoints(first_frame, entry_low_state, joint_pos,
                                               config);
}

ObservationBuilderState makeObservationBuilderState(
    const DeployConfig& deploy_config,
    const TrkFrameView& first_frame,
    const LowStateSample& entry_low_state,
    const ObservationBuilderConfig& config) {
  const std::array<double, kJointDim> joint_pos =
      policyJointPositions(deploy_config, entry_low_state);
  return makeObservationBuilderStateWithJoints(first_frame, entry_low_state, joint_pos,
                                               config);
}

PolicyObservationParts buildObservationParts(
    const DeployConfig& config,
    const TrkFrameView& frame,
    const LowStateSample& low_state,
    const Vec& last_action,
    const ObservationBuilderState& state,
    const ObservationBuilderConfig& builder_config) {
  validateDeployConfigForBuilder(config);
  validateLastAction(last_action);

  const FrameKinematics kinematics =
      frameKinematics(frame, low_state, state, builder_config);

  PolicyObservationParts parts;
  parts.command_yaw = commandYaw(kinematics.ref_q, kinematics.robot_q);
  if (builder_config.use_motion_root_command) {
    rotationFirstTwoColumns(kinematics.relative_q, parts.command_root_ori_b);
  } else {
    parts.command_root_ori_b = kIdentityRootOriB;
  }
  parts.command_xy_yaw_vel =
      builder_config.use_motion_velocity_command
          ? commandVelocity(frame, kinematics.ref_q, state, builder_config)
          : kZeroXyYawVel;
  parts.command_jnt_pos = copyView("joint_pos", frame.joint_pos, kJointDim);
  parts.projected_gravity = projectedGravity(kinematics.robot_q);
  parts.base_ang_vel = baseAngularVelocity(low_state);
  fillJointObservations(config, low_state, parts.command_jnt_pos, parts.joint_pos_rel,
                        parts.joint_vel_rel);
  parts.last_action = last_action;
  if (config.observation_contract == ObservationContract::GeneralTracker ||
      config.observation_contract == ObservationContract::GeneralTrackerCLNFootstate) {
    parts.command_foot_support_state = supportState(frame);
  }
  if (config.observation_contract == ObservationContract::GeneralTracker) {
    parts.ref_com_rel_navi = copyView("ref_com_rel_navi", frame.ref_com_rel_navi, 3);
    parts.ref_com_vel_navi = copyView("ref_com_vel_navi", frame.ref_com_vel_navi, 3);
  }
  return parts;
}

PolicyObservationParts buildObservationParts(
    const DeployConfig& config,
    const TrkTrack& track,
    std::size_t frame_index,
    const LowStateSample& low_state,
    const Vec& last_action,
    const ObservationBuilderState& state,
    const ObservationBuilderConfig& builder_config) {
  const TrkFrameView frame = requireTrackFrame(track, frame_index);
  PolicyObservationParts parts =
      buildObservationParts(config, frame, low_state, last_action, state, builder_config);
  if (config.observation_contract == ObservationContract::GeneralTrackerCLN ||
      config.observation_contract == ObservationContract::GeneralTrackerCLNFootstate) {
    const FrameKinematics kinematics =
        frameKinematics(frame, low_state, state, builder_config);
    parts.future_commands =
        futureCommands(config, track, frame_index, kinematics.robot_q, state, builder_config);
  }
  return parts;
}

std::size_t referenceFrameIndex(double elapsed_s, double fps, std::size_t frame_count) {
  if (frame_count == 0) {
    throw error("frame_count must be positive");
  }
  if (!std::isfinite(elapsed_s)) {
    throw error("elapsed_s must be finite");
  }
  if (!std::isfinite(fps) || fps <= 0.0) {
    throw error("fps must be positive and finite");
  }

  const double frame_position = elapsed_s * fps;
  if (!std::isfinite(frame_position)) {
    return frame_position > 0.0 ? frame_count - 1 : 0;
  }

  const double rounded = std::round(frame_position);
  if (rounded <= 0.0) {
    return 0;
  }

  const double max_index = static_cast<double>(frame_count - 1);
  if (rounded >= max_index) {
    return frame_count - 1;
  }
  return static_cast<std::size_t>(rounded);
}

}  // namespace agentic_et1_tracker
