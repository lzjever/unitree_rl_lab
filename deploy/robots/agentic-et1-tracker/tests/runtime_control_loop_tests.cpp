#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"
#include "agentic_et1_tracker/control/fixstand.hpp"
#include "agentic_et1_tracker/control/passive.hpp"
#include "agentic_et1_tracker/policy/policy_math.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kPolicyJointDim = TrkSchema::kJointDim;
constexpr std::uint8_t kExpectedModeMachine = 7;
constexpr std::size_t kObsCurrentRootOffset = 0;
constexpr std::size_t kObsCurrentCommandJointOffset = 9;
constexpr std::size_t kStartupHoldPolicyStepsAt50Fps = 25;
constexpr std::size_t kStartupUpperBodyFirstPolicyJoint = 14;
constexpr std::size_t kStartupUpperBodyLastPolicyJointExclusive = 26;

struct TempTree {
  TempTree() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_runtime_control_loop_tests_" + std::to_string(now));
    allowed = root / "allowed";
    std::filesystem::create_directories(allowed);
  }

  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  TrkValidationConfig trkConfig(double fps = 50.0) const {
    TrkValidationConfig config;
    config.allowlist_dirs = {allowed};
    config.fps = fps;
    config.max_duration_s = 120.0;
    return config;
  }

  std::filesystem::path root;
  std::filesystem::path allowed;
};

struct ArrayFixture {
  std::string name;
  TrkDtype dtype{TrkDtype::Float32};
  std::vector<std::uint64_t> shape;
  std::optional<std::array<float, 3>> root_body_position;
  std::optional<std::array<float, 4>> root_body_quat;
  bool identity_quaternions{false};
  bool invalid_contact{false};
  std::size_t invalid_contact_index{0};
  std::int64_t invalid_contact_value{3};
  bool zero_first_quaternion{false};
};

template <typename T>
void writeScalar(std::ofstream& out, T value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

std::uint64_t elementCount(const std::vector<std::uint64_t>& shape) {
  std::uint64_t elements = 1;
  for (const auto dim : shape) {
    elements *= dim;
  }
  return elements;
}

std::vector<std::uint64_t> shapeFor(const TrkRequiredArraySpec& spec,
                                    std::uint64_t frames) {
  std::vector<std::uint64_t> shape{frames};
  for (std::uint32_t i = 0; i < spec.trailing_rank; ++i) {
    shape.push_back(spec.trailing_shape[i]);
  }
  return shape;
}

void writePayload(std::ofstream& out, const ArrayFixture& array) {
  const auto elements = elementCount(array.shape);
  if (array.dtype == TrkDtype::Int64) {
    for (std::size_t i = 0; i < elements; ++i) {
      const auto value = array.invalid_contact && i == array.invalid_contact_index
                             ? array.invalid_contact_value
                             : static_cast<std::int64_t>(i % 3);
      writeScalar(out, value);
    }
    return;
  }

  for (std::size_t i = 0; i < elements; ++i) {
    float value = array.zero_first_quaternion && i < 4
                      ? 0.0F
                      : static_cast<float>(i) * 0.25F;
    if (array.identity_quaternions && array.name == "body_quat_w") {
      value = i % 4 == 0 ? 1.0F : 0.0F;
    }
    if (array.root_body_position && array.name == "body_pos_w" && i < 3) {
      value = array.root_body_position->at(i);
    }
    if (array.root_body_quat && array.name == "body_quat_w" && i < 4) {
      value = array.root_body_quat->at(i);
    }
    writeScalar(out, value);
  }
}

std::vector<ArrayFixture> requiredArrays(std::uint64_t frames) {
  std::vector<ArrayFixture> arrays;
  for (const auto& spec : TrkSchema::kRequiredArrays) {
    arrays.push_back({std::string(spec.name),
                      spec.dtype_family == TrkDtypeFamily::Contact ? TrkDtype::Int64
                                                                    : TrkDtype::Float32,
                      shapeFor(spec, frames)});
  }
  return arrays;
}

ArrayFixture& arrayNamed(std::vector<ArrayFixture>& arrays, const std::string& name) {
  for (auto& array : arrays) {
    if (array.name == name) {
      return array;
    }
  }
  FAIL("missing fixture array " << name);
  return arrays.front();
}

void writeTrk(const std::filesystem::path& path, const std::vector<ArrayFixture>& arrays) {
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);

  out.write(TrkSchema::kMagic.data(), static_cast<std::streamsize>(TrkSchema::kMagic.size()));
  writeScalar(out, TrkSchema::kVersion);
  writeScalar(out, static_cast<std::uint32_t>(arrays.size()));

  for (const auto& array : arrays) {
    writeScalar(out, static_cast<std::uint32_t>(array.name.size()));
    out.write(array.name.data(), static_cast<std::streamsize>(array.name.size()));
    writeScalar(out, static_cast<std::uint32_t>(array.dtype));
    writeScalar(out, static_cast<std::uint32_t>(array.shape.size()));
    for (const auto dim : array.shape) {
      REQUIRE(dim <= std::numeric_limits<std::uint32_t>::max());
      writeScalar(out, static_cast<std::uint32_t>(dim));
    }
    writeScalar(out, elementCount(array.shape) * trkDtypeSize(array.dtype));
    writePayload(out, array);
  }
}

std::filesystem::path validTrk(TempTree& tmp, const std::string& name, std::uint64_t frames) {
  const auto path = tmp.allowed / name;
  writeTrk(path, requiredArrays(frames));
  return path;
}

std::array<float, 4> yawQuat(float yaw_rad) {
  return {std::cos(0.5F * yaw_rad), 0.0F, 0.0F, std::sin(0.5F * yaw_rad)};
}

float yawFromQuat(std::array<float, 4> quat_wxyz) {
  const float w = quat_wxyz.at(0);
  const float x = quat_wxyz.at(1);
  const float y = quat_wxyz.at(2);
  const float z = quat_wxyz.at(3);
  return std::atan2(2.0F * (w * z + x * y),
                    1.0F - 2.0F * (y * y + z * z));
}

Vec rootOriFromYaw(float yaw_rad) {
  const float c = std::cos(yaw_rad);
  const float s = std::sin(yaw_rad);
  return {c, -s, s, c, 0.0F, 0.0F};
}

void requireObsSliceApprox(const Vec& actual,
                           std::size_t offset,
                           const Vec& expected,
                           float margin = 1.0e-5F) {
  REQUIRE(offset + expected.size() <= actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    REQUIRE(actual.at(offset + i) == Catch::Approx(expected.at(i)).margin(margin));
  }
}

std::filesystem::path rootPoseTrk(TempTree& tmp,
                                  const std::string& name,
                                  std::uint64_t frames,
                                  std::array<float, 3> root_body_position,
                                  std::array<float, 4> root_body_quat) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_pos_w").root_body_position = root_body_position;
  arrayNamed(arrays, "body_quat_w").root_body_quat = root_body_quat;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::shared_ptr<const TrkTrack> loadTrack(const TrkValidationConfig& config,
                                          const std::filesystem::path& path) {
  TrkLoadResult loaded = TrkLoader(config).load(path);
  REQUIRE(loaded.ok());
  return std::make_shared<TrkTrack>(std::move(*loaded.track));
}

std::shared_ptr<const TrkTrack> validStandbyTrack(TempTree& tmp,
                                                  const std::string& name) {
  return loadTrack(tmp.trkConfig(), validTrk(tmp, name, 2));
}

std::filesystem::path invalidContactTrk(TempTree& tmp, const std::string& name) {
  auto arrays = requiredArrays(3);
  auto& left = arrayNamed(arrays, "left_foot_contact_state");
  left.invalid_contact = true;
  left.invalid_contact_index = 1;
  left.invalid_contact_value = 3;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path zeroQuaternionTrk(TempTree& tmp,
                                        const std::string& name,
                                        std::uint64_t frames = 2) {
  auto arrays = requiredArrays(frames);
  auto& quat = arrayNamed(arrays, "body_quat_w");
  quat.zero_first_quaternion = true;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path identityQuaternionTrk(TempTree& tmp,
                                           const std::string& name,
                                           std::uint64_t frames) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

RuntimeConfig runtimeConfig() {
  RuntimeConfig config;
  config.queue_limit = 8;
  config.recent_limit = 16;
  config.hz = 50.0;
  return config;
}

std::vector<float> floatSeq(float start, std::size_t count) {
  std::vector<float> values;
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

std::vector<int> frozenSdkMap() {
  return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
          13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30};
}

std::vector<ObservationTerm> observationTerms(
    std::initializer_list<std::pair<const char*, std::size_t>> specs) {
  std::vector<ObservationTerm> out;
  std::size_t offset = 0;
  for (const auto& spec : specs) {
    out.push_back({spec.first, spec.second, offset});
    offset += spec.second;
  }
  return out;
}

DeployConfig deployConfig() {
  DeployConfig config;
  config.joint_dim = kPolicyJointDim;
  config.sdk_joint_ids_map = frozenSdkMap();
  config.default_joint_pos = doubleSeq(0.25, 0.5, kPolicyJointDim);
  config.policy_kp = doubleSeq(10.0, 1.0, kPolicyJointDim);
  config.policy_kd = doubleSeq(0.5, 0.05, kPolicyJointDim);
  config.action_scale = doubleSeq(0.25, 0.125, kPolicyJointDim);
  config.action_offset = doubleSeq(-1.0, 0.2, kPolicyJointDim);
  config.obs_current_terms = observationTerms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kPolicyJointDim},
      {"joint_vel_rel", kPolicyJointDim},
      {"last_action", kPolicyJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_history_terms = observationTerms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kPolicyJointDim},
      {"joint_vel_rel", kPolicyJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_current_dim = 131;
  config.obs_history_width = 105;
  config.obs_history_length = 25;
  return config;
}

LowStateSample readyLowState(const DeployConfig& config,
                             std::uint8_t mode_machine = kExpectedModeMachine,
                             bool fresh = true,
                             std::size_t age_ms = 4) {
  LowStateSample low;
  low.fresh = fresh;
  low.age_ms = age_ms;
  low.mode_machine = mode_machine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low.gyro = {0.3F, -0.4F, 0.5F};

  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map[policy_index]);
    low.motors.at(sdk_slot).q = 10.0F + static_cast<float>(policy_index);
    low.motors.at(sdk_slot).dq = 20.0F + static_cast<float>(policy_index);
  }
  return low;
}

LowStateSample badOrientationLowState(const DeployConfig& config,
                                      std::size_t age_ms = 44) {
  LowStateSample low = readyLowState(config, kExpectedModeMachine, true, age_ms);
  low.quat_wxyz = {0.70710677F, 0.70710677F, 0.0F, 0.0F};
  return low;
}

class FakeRobotIO final : public RobotIO {
 public:
  explicit FakeRobotIO(LowStateSample state) : low_state(std::move(state)) {}

  std::optional<LowStateSample> readLowState() const override {
    ++read_low_calls;
    return low_state;
  }

  std::optional<HighStateSample> readHighState() const override {
    ++read_high_calls;
    return high_state;
  }

  LowCmdOccupancy lowCmdOccupancy() const override {
    ++occupancy_calls;
    return occupancy;
  }

  void writeLowCmd(const LowCmdFrame& frame) override {
    ++write_attempts;
    if (throw_on_write) {
      throw RobotIOError("fake write failed");
    }
    writes.push_back(frame);
    if (on_write) {
      on_write(frame);
    }
  }

  std::optional<LowStateSample> low_state;
  std::optional<HighStateSample> high_state;
  LowCmdOccupancy occupancy;
  mutable int read_low_calls{0};
  mutable int read_high_calls{0};
  mutable int occupancy_calls{0};
  int write_attempts{0};
  bool throw_on_write{false};
  std::vector<LowCmdFrame> writes;
  std::function<void(const LowCmdFrame&)> on_write;
};

class FakePolicy final : public PolicyInference {
 public:
  explicit FakePolicy(Vec raw = floatSeq(0.0F, kPolicyJointDim))
      : next_raw(std::move(raw)) {}

  Vec infer(const PolicyInputs& inputs) override {
    ++calls;
    inputs_seen.push_back(inputs);
    if (throw_on_infer) {
      throw std::runtime_error("fake policy failed");
    }
    return next_raw;
  }

  Vec next_raw;
  bool throw_on_infer{false};
  int calls{0};
  std::vector<PolicyInputs> inputs_seen;
};

class FakeVelocityPolicy final : public VelocityPolicyInference {
 public:
  explicit FakeVelocityPolicy(Vec raw = Vec(kVelocityPolicyJointDim, 0.0F))
      : next_raw(std::move(raw)) {}

  Vec infer(const VelocityPolicyInputs& inputs) override {
    ++calls;
    inputs_seen.push_back(inputs);
    if (throw_on_infer) {
      throw std::runtime_error("fake velocity policy failed");
    }
    return next_raw;
  }

  Vec next_raw;
  bool throw_on_infer{false};
  int calls{0};
  std::vector<VelocityPolicyInputs> inputs_seen;
};

class FakeReferenceSink final : public ReferenceFrameSink {
 public:
  void publish(ReferenceFrameSnapshot snapshot) override {
    ++publish_calls;
    if (throw_on_publish) {
      throw std::runtime_error("fake reference publish failed");
    }
    latest = snapshot;
    published.push_back(std::move(snapshot));
  }

  void clear() override {
    ++clear_calls;
    if (throw_on_clear) {
      throw std::runtime_error("fake reference clear failed");
    }
    latest = ReferenceFrameSnapshot{};
  }

  int publish_calls{0};
  int clear_calls{0};
  bool throw_on_publish{false};
  bool throw_on_clear{false};
  ReferenceFrameSnapshot latest;
  std::vector<ReferenceFrameSnapshot> published;
};

VelocityDeployConfig velocityDeployConfig() {
  VelocityDeployConfig config;
  config.joint_dim = kVelocityPolicyJointDim;
  config.joint_ids_map = {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11};
  config.stiffness = doubleSeq(30.0, 1.0, kVelocityPolicyJointDim);
  config.damping = doubleSeq(1.0, 0.1, kVelocityPolicyJointDim);
  config.default_joint_pos = doubleSeq(0.0, 0.05, kVelocityPolicyJointDim);
  config.action_scale = std::vector<double>(kVelocityPolicyJointDim, 0.25);
  config.action_offset = doubleSeq(-0.2, 0.02, kVelocityPolicyJointDim);
  config.observation_terms = {
      {"base_ang_vel", 3, 0, {0.2, 0.2, 0.2}},
      {"projected_gravity", 3, 3, {1.0, 1.0, 1.0}},
      {"keyboard_velocity_commands", 3, 6, {1.0, 1.0, 1.0}},
      {"joint_pos_rel", kVelocityPolicyJointDim, 9,
       std::vector<double>(kVelocityPolicyJointDim, 1.0)},
      {"joint_vel_rel", kVelocityPolicyJointDim, 21,
       std::vector<double>(kVelocityPolicyJointDim, 0.05)},
      {"last_action", kVelocityPolicyJointDim, 33,
       std::vector<double>(kVelocityPolicyJointDim, 1.0)},
  };
  config.obs_row_width = kVelocityPolicyObsRowWidth;
  config.obs_history_length = kVelocityPolicyHistoryLength;
  config.obs_dim = kVelocityPolicyObsDim;
  config.step_dt = 0.02;
  return config;
}

FixStandConfig fixStandConfig() {
  FixStandConfig config;
  config.kp = doubleSeq(20.0, 1.0, kFixStandMotorCount);
  config.kd = doubleSeq(1.0, 0.1, kFixStandMotorCount);
  config.target_q = doubleSeq(0.5, 0.05, kFixStandMotorCount);
  config.duration_s = 3.0;
  return config;
}

PassiveConfig passiveConfig() {
  PassiveConfig config;
  config.mode = std::vector<int>(kFixStandMotorCount, 1);
  config.kd = doubleSeq(3.25, 0.25, kFixStandMotorCount);
  config.mode.at(14) = 0;
  config.mode.at(31) = 0;
  config.mode.at(32) = 0;
  config.kd.at(14) = 0.0;
  config.kd.at(31) = 0.0;
  config.kd.at(32) = 0.0;
  return config;
}

std::filesystem::path appRoot() {
  return std::filesystem::path(__FILE__).parent_path().parent_path();
}

ExecuteCommand executeCommand(std::string id,
                              const std::filesystem::path& path,
                              MotionMode mode = MotionMode::Queue,
                              std::size_t frames = 3,
                              bool hold = false) {
  ExecuteCommand command;
  command.id = std::move(id);
  command.path = path.string();
  command.mode = mode;
  command.hold = hold;
  command.track.frames = frames;
  command.track.duration_s = static_cast<double>(frames) / 50.0;
  command.track.fps = 50.0;
  command.track.canonical_path = path.string();
  return command;
}

IdleMotion idleMotion(const std::filesystem::path& path, std::size_t frames = 3) {
  IdleMotion motion;
  motion.path = path.string();
  motion.track.frames = frames;
  motion.track.duration_s = static_cast<double>(frames) / 50.0;
  motion.track.fps = 50.0;
  motion.track.canonical_path = path.string();
  return motion;
}

RuntimeControlLoop makeLoop(RuntimeConfig config,
                            RuntimeBridge& bridge,
                            RuntimeStatusStore& store,
                            const TrkValidationConfig& trk_config) {
  return RuntimeControlLoop(config, bridge, store, TrkLoader(trk_config));
}

RuntimeControlLoop makePolicyLoop(RuntimeConfig config,
                                  RuntimeBridge& bridge,
                                  RuntimeStatusStore& store,
                                  const TrkValidationConfig& trk_config,
                                  FakeRobotIO& robot,
                                  FakePolicy& policy,
                                  DeployConfig deploy_config = deployConfig(),
                                  PassiveConfig passive_config = passiveConfig()) {
  return RuntimeControlLoop(config,
                            bridge,
                            store,
                            TrkLoader(trk_config),
                            robot,
                            policy,
                            std::move(deploy_config),
                            std::move(passive_config),
                            kExpectedModeMachine,
                            RuntimeMode::Real);
}

RuntimeControlLoop makeControlLoop(RuntimeConfig config,
                                   RuntimeBridge& bridge,
                                   RuntimeStatusStore& store,
                                   const TrkValidationConfig& trk_config,
                                   FakeRobotIO& robot,
                                   FakePolicy& tracker_policy,
                                   FakeVelocityPolicy& velocity_policy,
                                   DeployConfig deploy_config = deployConfig(),
                                   VelocityDeployConfig velocity_deploy_config =
                                       velocityDeployConfig(),
                                   FixStandConfig fixstand_config = fixStandConfig(),
                                   ControlMode startup_control = ControlMode::FixStand,
                                   PassiveConfig passive_config = passiveConfig(),
                                   ReferenceFrameSink* reference_sink = nullptr,
                                   std::shared_ptr<const TrkTrack> standby_track = nullptr) {
  return RuntimeControlLoop(config,
                            bridge,
                            store,
                            TrkLoader(trk_config),
                            robot,
                            tracker_policy,
                            std::move(deploy_config),
                            velocity_policy,
                            std::move(velocity_deploy_config),
                            std::move(fixstand_config),
                            std::move(passive_config),
                            startup_control,
                            kExpectedModeMachine,
                            RuntimeMode::Real,
                            reference_sink,
                            std::move(standby_track));
}

void requireIdle(const RuntimeStatusStore& store) {
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE(snapshot.active.id.empty());
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
}

void requireHoldFrame(const LowCmdFrame& frame,
                      const DeployConfig& config,
                      const LowStateSample& low_state) {
  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t policy_joint = 0; policy_joint < kPolicyJointDim; ++policy_joint) {
    const auto sdk_slot =
        static_cast<std::size_t>(config.sdk_joint_ids_map.at(policy_joint));
    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q == low_state.motors.at(sdk_slot).q);
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp ==
            static_cast<float>(config.policy_kp.at(policy_joint)));
    REQUIRE(frame.motors.at(sdk_slot).kd ==
            static_cast<float>(config.policy_kd.at(policy_joint)));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

void requireFixStandFrameFromCurrentQ(const LowCmdFrame& frame,
                                      const FixStandConfig& config,
                                      const LowStateSample& low_state) {
  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    REQUIRE(frame.motors.at(i).mode == 1);
    REQUIRE(frame.motors.at(i).q == low_state.motors.at(i).q);
    REQUIRE(frame.motors.at(i).kp == static_cast<float>(config.kp.at(i)));
    REQUIRE(frame.motors.at(i).kd == static_cast<float>(config.kd.at(i)));
    REQUIRE(frame.motors.at(i).tau == 0.0F);
  }
}

void requireVelocityFrame(const LowCmdFrame& frame,
                          const VelocityDeployConfig& config,
                          const Vec& raw_actions) {
  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t i = 0; i < kVelocityPolicyJointDim; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.joint_ids_map.at(i));
    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q ==
            raw_actions.at(i) * static_cast<float>(config.action_scale.at(i)) +
                static_cast<float>(config.action_offset.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp == static_cast<float>(config.stiffness.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).kd == static_cast<float>(config.damping.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

Vec fixtureFirstFrameJointPos() {
  Vec out;
  out.reserve(kPolicyJointDim);
  for (std::size_t policy_joint = 0; policy_joint < kPolicyJointDim; ++policy_joint) {
    out.push_back(static_cast<float>(policy_joint) * 0.25F);
  }
  return out;
}

void requireStartupHoldInterpolatedFrame(const LowCmdFrame& frame,
                                         const DeployConfig& config,
                                         const LowStateSample& startup_low_state,
                                         const Vec& raw_action,
                                         const Vec& first_frame_joint_pos,
                                         std::size_t write_index) {
  REQUIRE(first_frame_joint_pos.size() == kPolicyJointDim);
  const PolicyOutput policy_output = scaleAction(config, raw_action);
  const float alpha =
      kStartupHoldPolicyStepsAt50Fps <= 1
          ? 1.0F
          : static_cast<float>(write_index) /
                static_cast<float>(kStartupHoldPolicyStepsAt50Fps - 1);

  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t policy_joint = 0; policy_joint < kPolicyJointDim; ++policy_joint) {
    const auto sdk_slot =
        static_cast<std::size_t>(config.sdk_joint_ids_map.at(policy_joint));
    const bool startup_upper_body =
        policy_joint >= kStartupUpperBodyFirstPolicyJoint &&
        policy_joint < kStartupUpperBodyLastPolicyJointExclusive;
    const float expected_q =
        startup_upper_body
            ? startup_low_state.motors.at(sdk_slot).q +
                  alpha * (first_frame_joint_pos.at(policy_joint) -
                           startup_low_state.motors.at(sdk_slot).q)
            : policy_output.target_q.at(policy_joint);

    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q ==
            Catch::Approx(expected_q).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp ==
            Catch::Approx(policy_output.kp.at(policy_joint)).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).kd ==
            Catch::Approx(policy_output.kd.at(policy_joint)).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

void requireMotorPreserved(const LowCmdFrame& frame,
                           const LowCmdFrame& base,
                           std::size_t sdk_slot) {
  REQUIRE(frame.motors.at(sdk_slot).mode == base.motors.at(sdk_slot).mode);
  REQUIRE(frame.motors.at(sdk_slot).q == base.motors.at(sdk_slot).q);
  REQUIRE(frame.motors.at(sdk_slot).dq == base.motors.at(sdk_slot).dq);
  REQUIRE(frame.motors.at(sdk_slot).kp == base.motors.at(sdk_slot).kp);
  REQUIRE(frame.motors.at(sdk_slot).kd == base.motors.at(sdk_slot).kd);
  REQUIRE(frame.motors.at(sdk_slot).tau == base.motors.at(sdk_slot).tau);
}

void requirePassiveDampingFrame(const LowCmdFrame& frame,
                                const LowStateSample& low_state,
                                const PassiveConfig& passive_config) {
  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t i : {std::size_t{0}, std::size_t{14}, std::size_t{29},
                        std::size_t{30}, std::size_t{31}}) {
    REQUIRE(frame.motors.at(i).mode ==
            static_cast<std::uint8_t>(passive_config.mode.at(i)));
    REQUIRE(frame.motors.at(i).q == low_state.motors.at(i).q);
    REQUIRE(frame.motors.at(i).dq == 0.0F);
    REQUIRE(frame.motors.at(i).kp == 0.0F);
    REQUIRE(frame.motors.at(i).kd == static_cast<float>(passive_config.kd.at(i)));
    REQUIRE(frame.motors.at(i).tau == 0.0F);
  }
}

void requireFixStandHoldMotor(const LowCmdFrame& frame,
                              const FixStandConfig& config,
                              std::size_t sdk_slot) {
  REQUIRE(frame.motors.at(sdk_slot).mode == 1);
  REQUIRE(frame.motors.at(sdk_slot).q ==
          static_cast<float>(config.target_q.at(sdk_slot)));
  REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
  REQUIRE(frame.motors.at(sdk_slot).kp ==
          static_cast<float>(config.kp.at(sdk_slot)));
  REQUIRE(frame.motors.at(sdk_slot).kd ==
          static_cast<float>(config.kd.at(sdk_slot)));
  REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
}

enum class StartQueuedRunMode {
  RequireDirectStart,
  AllowStandbyTransition,
};

void startQueuedRun(RuntimeControlLoop& loop,
                    RuntimeStatusStore& store,
                    const std::string& id,
                    StartQueuedRunMode mode =
                        StartQueuedRunMode::RequireDirectStart) {
  loop.tick();
  auto snapshot = store.snapshot();
  if (snapshot.active.kind == ActiveKind::Transition &&
      snapshot.transition.active && snapshot.transition.target == "user") {
    REQUIRE(mode == StartQueuedRunMode::AllowStandbyTransition);
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.transition.target_id == id);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(store.findRun(id).run->state == MotionState::Queued);
    const std::size_t transition_frames = snapshot.transition.frames;
    bool saw_user = false;
    for (std::size_t i = 0; i < transition_frames * 32 + 32; ++i) {
      loop.tick();
      snapshot = store.snapshot();
      if (snapshot.active.kind == ActiveKind::User) {
        saw_user = true;
        break;
      }
    }
    REQUIRE(saw_user);
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == id);
    return;
  }

  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == id);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == id);
}

void startHoldingRun(RuntimeControlLoop& loop,
                     RuntimeStatusStore& store,
                     RuntimeBridge& bridge,
                     TempTree& tmp,
                     const std::string& id,
                     FakeReferenceSink* reference = nullptr,
                     StartQueuedRunMode mode =
                         StartQueuedRunMode::RequireDirectStart) {
  const auto path = validTrk(tmp, id + ".trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand(id, path, MotionMode::Queue, 1, true))
              .ok());
  startQueuedRun(loop, store, id, mode);

  StatusSnapshot snapshot;
  bool saw_holding = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec.has_value() && snapshot.exec->id == id &&
        snapshot.exec->state == MotionState::Holding) {
      saw_holding = true;
      break;
    }
  }
  REQUIRE(saw_holding);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.active.id == id);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == id);
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->frame == 0);
  REQUIRE(snapshot.exec->frames == 1);
  REQUIRE(snapshot.exec->progress == 1.0);
  if (reference != nullptr) {
    REQUIRE(reference->latest.active);
    REQUIRE(reference->latest.id == id);
    REQUIRE(reference->latest.frame == 0);
  }
}

void startHoldingToUserTransition(RuntimeControlLoop& loop,
                                  RuntimeStatusStore& store,
                                  RuntimeBridge& bridge,
                                  TempTree& tmp,
                                  const std::string& held_id,
                                  const std::string& target_id,
                                  FakeReferenceSink* reference = nullptr,
                                  StartQueuedRunMode held_start_mode =
                                      StartQueuedRunMode::RequireDirectStart) {
  startHoldingRun(loop, store, bridge, tmp, held_id, reference, held_start_mode);
  const auto target_path = validTrk(tmp, target_id + ".trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand(target_id, target_path, MotionMode::Queue, 2))
              .ok());

  loop.tick();
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == target_id);
  REQUIRE(snapshot.queue.ids.empty());
}

void requireUserTransitionStatus(const RuntimeStatusStore& store,
                                 const std::string& target_id) {
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == target_id);
  REQUIRE(snapshot.transition.target_state == MotionState::Queued);
  REQUIRE(snapshot.transition.frame == 0);
  REQUIRE(snapshot.queue.ids.empty());
}

void requireNoActiveWorkOrBackground(const RuntimeStatusStore& store) {
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
}

void requireReferenceStartsFrom(const FakeReferenceSink& reference,
                                const ReferenceFrameSnapshot& source) {
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.p == source.p);
  REQUIRE(reference.latest.q.size() == source.q.size());
  for (std::size_t body = 0; body < reference.latest.q.size(); ++body) {
    for (std::size_t axis = 0; axis < reference.latest.q.at(body).size(); ++axis) {
      REQUIRE(reference.latest.q.at(body).at(axis) ==
              Catch::Approx(source.q.at(body).at(axis)).margin(1.0e-5F));
    }
  }
  REQUIRE(reference.latest.c == source.c);
  REQUIRE(reference.latest.com == source.com);
}

std::filesystem::path startCompletedUserToIdleTransition(
    RuntimeControlLoop& loop,
    RuntimeStatusStore& store,
    RuntimeBridge& bridge,
    TempTree& tmp,
    const std::string& id_prefix,
    StartQueuedRunMode start_mode = StartQueuedRunMode::RequireDirectStart) {
  const auto idle_path = validTrk(tmp, id_prefix + "_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();

  const auto user_path = validTrk(tmp, id_prefix + "_user.trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand(id_prefix + "-source-user",
                             user_path,
                             MotionMode::Queue,
                             1))
              .ok());
  startQueuedRun(loop, store, id_prefix + "-source-user", start_mode);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "idle");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun(id_prefix + "-source-user").run->state == MotionState::Done);
  return idle_path;
}

void startCompletedUserToStandbyTransition(RuntimeControlLoop& loop,
                                           RuntimeStatusStore& store,
                                           RuntimeBridge& bridge,
                                           TempTree& tmp,
                                           const std::string& id_prefix) {
  const auto user_path = validTrk(tmp, id_prefix + "_user.trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand(id_prefix + "-source-user",
                             user_path,
                             MotionMode::Queue,
                             1))
              .ok());
  startQueuedRun(loop, store, id_prefix + "-source-user");
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun(id_prefix + "-source-user").run->state == MotionState::Done);
  REQUIRE(store.findRun(id_prefix + "-source-user").run->stop_reason ==
          StopReason::None);
}

void advanceBackgroundTransition(RuntimeControlLoop& loop,
                                 RuntimeStatusStore& store,
                                 const std::string& target) {
  for (int i = 0; i < 16; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::Transition &&
        snapshot.transition.active && snapshot.transition.target == target &&
        snapshot.transition.target_id.empty() && snapshot.transition.frame > 0) {
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE(snapshot.queue.ids.empty());
      return;
    }
  }
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == target);
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE(snapshot.transition.frame > 0);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
}

void advanceToStandbyPlayback(RuntimeControlLoop& loop,
                              RuntimeStatusStore& store,
                              std::size_t standby_frames) {
  for (int i = 0; i < 96; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::Transition &&
        snapshot.transition.active && snapshot.transition.target == "standby" &&
        snapshot.transition.frames == standby_frames) {
      REQUIRE(snapshot.transition.target_id.empty());
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE(snapshot.queue.ids.empty());
      return;
    }
  }
  FAIL("standby playback did not start");
}

void requireStandbyGateFailure(const RuntimeControlLoop& loop,
                               const RuntimeStatusStore& store,
                               const FakeReferenceSink& reference,
                               const FakeRobotIO& robot,
                               const FakePolicy& tracker_policy,
                               const FakeVelocityPolicy& velocity_policy,
                               const std::string& id,
                               ErrorCode expected_error) {
  const auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Velocity);
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(reference.latest.active);
  REQUIRE(reference.publish_calls == 0);
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(velocity_policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(robot.writes.empty());

  const auto failed = store.findRun(id);
  REQUIRE(failed.ok());
  REQUIRE(failed.run.has_value());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == expected_error);
}

TEST_CASE("PassiveConfig loads app-owned ET1 passive damping asset") {
  const PassiveConfig config =
      loadPassiveConfig(appRoot() / "config/posture/passive/v0/passive.yaml");

  REQUIRE(config.mode.size() == kFixStandMotorCount);
  REQUIRE(config.kd.size() == kFixStandMotorCount);
  REQUIRE(config.mode.at(0) == 1);
  REQUIRE(config.kd.at(0) == 8.0);
  REQUIRE(config.mode.at(14) == 0);
  REQUIRE(config.kd.at(14) == 0.0);
  REQUIRE(config.mode.at(30) == 1);
  REQUIRE(config.kd.at(30) == 0.5);
  REQUIRE(config.mode.at(31) == 0);
  REQUIRE(config.kd.at(31) == 0.0);

  const LowStateSample low_state = readyLowState(deployConfig());
  const LowCmdFrame frame =
      makePassiveLowCmdFrame(config, low_state, kExpectedModeMachine);

  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  REQUIRE(frame.motors.at(0).mode == 1);
  REQUIRE(frame.motors.at(0).q == low_state.motors.at(0).q);
  REQUIRE(frame.motors.at(0).dq == 0.0F);
  REQUIRE(frame.motors.at(0).kp == 0.0F);
  REQUIRE(frame.motors.at(0).kd == 8.0F);
  REQUIRE(frame.motors.at(0).tau == 0.0F);
  REQUIRE(frame.motors.at(14).mode == 0);
  REQUIRE(frame.motors.at(14).q == low_state.motors.at(14).q);
  REQUIRE(frame.motors.at(14).kd == 0.0F);
  REQUIRE(frame.motors.at(30).mode == 1);
  REQUIRE(frame.motors.at(30).kd == 0.5F);
}

}  // namespace

TEST_CASE("RuntimeControlLoop loads queued trk and publishes deterministic progress to done") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 100.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig(25.0));

  const auto path = validTrk(tmp, "valid.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("run-a", path)).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "run-a");
  REQUIRE(snapshot.exec->state == MotionState::Queued);
  REQUIRE(snapshot.queue.ids.empty());

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "run-a");
  REQUIRE(snapshot.exec->state == MotionState::Running);
  REQUIRE(snapshot.exec->frame == 0);
  REQUIRE(snapshot.exec->frames == 3);
  REQUIRE(snapshot.exec->time_s == 0.0);
  REQUIRE(snapshot.exec->progress == 1.0 / 3.0);
  REQUIRE(snapshot.queue.ids.empty());

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec->frame == 0);
  REQUIRE(snapshot.exec->time_s == 0.0);
  REQUIRE(snapshot.exec->progress == 1.0 / 3.0);

  for (int i = 0; i < 3; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    REQUIRE(snapshot.exec->frame == 0);
  }
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(snapshot.exec->time_s == 0.04);
  REQUIRE(snapshot.exec->progress == 2.0 / 3.0);

  for (int i = 0; i < 4; ++i) {
    loop.tick();
  }
  requireIdle(store);

  const auto found = store.findRun("run-a");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Done);
  REQUIRE(found.run->frame == 2);
  REQUIRE(found.run->time_s == 0.08);
  REQUIRE(found.run->progress == 1.0);
}

TEST_CASE("RuntimeControlLoop publishes active raw reference frames then clears") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto path = validTrk(tmp, "reference_frames.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("ref-run", path)).ok());

  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Preparing);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Running);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.id == "ref-run");
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.frames == 3);
  REQUIRE(reference.latest.p.at(0) == std::array<float, 3>{{0.0F, 0.25F, 0.5F}});
  REQUIRE(reference.latest.q.at(0) == std::array<float, 4>{{0.0F, 0.25F, 0.5F, 0.75F}});
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{{0, 0}});
  REQUIRE(reference.latest.com == std::array<float, 3>{{0.0F, 0.25F, 0.5F}});
  REQUIRE(reference.latest.comv == std::array<float, 3>{{0.0F, 0.25F, 0.5F}});

  loop.tick();
  loop.tick();
  REQUIRE(reference.latest.frame == 1);
  REQUIRE(reference.latest.p.at(0) == std::array<float, 3>{{20.25F, 20.5F, 20.75F}});
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{{1, 1}});

  loop.tick();
  REQUIRE_FALSE(reference.latest.active);
  const auto found = store.findRun("ref-run");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Done);
  REQUIRE(reference.published.size() >= 3);
  REQUIRE(reference.published.at(reference.published.size() - 1).frame == 2);
  REQUIRE(reference.published.at(reference.published.size() - 1).p.at(0) ==
          std::array<float, 3>{{40.5F, 40.75F, 41.0F}});
}

TEST_CASE("RuntimeControlLoop hold true keeps user run on final reference frame") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto path = validTrk(tmp, "hold_last.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("hold-last", path, MotionMode::Queue, 2, true))
              .ok());
  startQueuedRun(loop, store, "hold-last");

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.exec->state == MotionState::Running);
  REQUIRE(snapshot.exec->frame == 0);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.active.id == "hold-last");
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "hold-last");
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->hold);
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(snapshot.exec->frames == 2);
  REQUIRE(snapshot.exec->progress == 1.0);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.id == "hold-last");
  REQUIRE(reference.latest.frame == 1);
  const int publishes_at_hold = reference.publish_calls;

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 1);
  REQUIRE(reference.publish_calls > publishes_at_hold);

  const auto found = store.findRun("hold-last");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Holding);
  REQUIRE(found.run->progress == 1.0);
}

TEST_CASE("RuntimeControlLoop policy holding keeps executing final frame") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  auto loop = makePolicyLoop(config,
                             bridge,
                             store,
                             tmp.trkConfig(),
                             robot,
                             policy,
                             deploy_config);

  const auto path = validTrk(tmp, "policy_hold_final.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("policy-hold", path, MotionMode::Queue, 2, true))
              .ok());
  startQueuedRun(loop, store, "policy-hold");

  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
    REQUIRE(store.snapshot().exec->state == MotionState::Running);
    REQUIRE(store.snapshot().exec->frame == 0);
    REQUIRE(policy.calls == static_cast<int>(tick + 1));
  }

  loop.tick();
  REQUIRE(store.snapshot().exec->state == MotionState::Holding);
  REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  const float final_frame_joint0 = 6.5F;
  REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
          final_frame_joint0);

  loop.tick();
  REQUIRE(store.snapshot().exec->state == MotionState::Holding);
  REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 2));
  REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
          final_frame_joint0);
}

TEST_CASE("RuntimeControlLoop hold false preserves completed run behavior") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto path = validTrk(tmp, "hold_false.trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand("hold-false", path, MotionMode::Queue, 1, false))
              .ok());
  startQueuedRun(loop, store, "hold-false");

  loop.tick();

  requireIdle(store);
  const auto found = store.findRun("hold-false");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Done);
  REQUIRE(found.run->progress == 1.0);
}

TEST_CASE("RuntimeControlLoop control commands leave holding with required terminal state") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* id;
    ControlMode mode;
    MotionState expected_state;
    ControllerState expected_ctrl;
    StopReason expected_reason;
    int ticks_after_command;
  };

  for (const auto& item : std::vector<Case>{
           {"holding-stop", ControlMode::StandbyVelocity, MotionState::Stopped,
            ControllerState::StandbyVelocity, StopReason::Stop, 2},
           {"holding-passive", ControlMode::Passive, MotionState::Stopped,
            ControllerState::Passive, StopReason::Stop, 1},
           {"holding-fixstand", ControlMode::FixStand, MotionState::Stopped,
            ControllerState::FixStand, StopReason::Stop, 2},
           {"holding-standby", ControlMode::StandbyVelocity, MotionState::Done,
            ControllerState::StandbyVelocity, StopReason::None, 40},
       }) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    FakeReferenceSink reference;
    const auto startup_standby_track =
        validStandbyTrack(tmp, std::string(item.id) + "_startup_standby.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference,
                                startup_standby_track);
    startHoldingRun(loop,
                    store,
                    bridge,
                    tmp,
                    item.id,
                    &reference,
                    StartQueuedRunMode::AllowStandbyTransition);

    if (std::string(item.id) == "holding-stop") {
      REQUIRE(bridge.stop().state == ControllerState::Stopping);
    } else if (item.mode == ControlMode::Passive) {
      REQUIRE(bridge.passive().ok());
    } else if (item.mode == ControlMode::FixStand) {
      REQUIRE(bridge.fixStand().ok());
    } else {
      REQUIRE(bridge.standbyVelocity().ok());
    }

    for (int i = 0; i < item.ticks_after_command; ++i) {
      loop.tick();
    }

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(reference.latest.active);
    const auto found = store.findRun(item.id);
    REQUIRE(found.ok());
    REQUIRE(found.run->state == item.expected_state);
    REQUIRE(found.run->stop_reason == item.expected_reason);
  }
}

TEST_CASE("RuntimeControlLoop standby velocity from holding gates through standby reference") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "internal_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeReferenceSink reference;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference,
                              standby_track);
  startHoldingRun(loop,
                  store,
                  bridge,
                  tmp,
                  "held-to-standby-ref",
                  &reference,
                  StartQueuedRunMode::AllowStandbyTransition);

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.active.id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.transition.target_state.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.id.empty());
  REQUIRE(store.findRun("held-to-standby-ref").run->state == MotionState::Done);
  REQUIRE(store.findRun("standby_ref").code == ErrorCode::RunNotFound);

  bool saw_standby_playback = false;
  const std::size_t synthetic_frames = snapshot.transition.frames;
  for (std::size_t i = 0; i < synthetic_frames + 4; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.queue.ids.empty());
    if (snapshot.active.kind == ActiveKind::Transition &&
        snapshot.transition.target == "standby" &&
        snapshot.transition.frames == standby_track->metadata.frames) {
      saw_standby_playback = true;
      break;
    }
  }
  REQUIRE(saw_standby_playback);

  for (std::size_t i = 0; i < standby_track->metadata.frames + 4; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::None) {
      break;
    }
  }

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(reference.latest.active);
  REQUIRE(store.findRun("held-to-standby-ref").run->state == MotionState::Done);
}

TEST_CASE("RuntimeControlLoop completed user gates through standby reference when idle has no work") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "complete_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          &reference,
                          standby_track);

  const auto path = validTrk(tmp, "complete_before_standby.trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand("complete-before-standby", path, MotionMode::Queue, 1))
              .ok());
  startQueuedRun(loop, store, "complete-before-standby");

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.transition.target_state.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("complete-before-standby").run->state == MotionState::Done);
  REQUIRE(store.findRun("complete-before-standby").run->stop_reason ==
          StopReason::None);
  REQUIRE(store.findRun("standby_ref").code == ErrorCode::RunNotFound);
}

TEST_CASE("RuntimeControlLoop standby velocity cancels running user through standby reference") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "cancel_running_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeReferenceSink reference;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference,
                              standby_track);

  const auto path = validTrk(tmp, "cancel_running_user.trk", 5);
  REQUIRE(bridge.submitQueue(
              executeCommand("cancel-running-user", path, MotionMode::Queue, 5))
              .ok());
  startQueuedRun(loop,
                 store,
                 "cancel-running-user",
                 StartQueuedRunMode::AllowStandbyTransition);
  const auto waiting_path = validTrk(tmp, "cancel_running_waiting.trk", 3);
  REQUIRE(bridge.submitQueue(
              executeCommand("cancel-running-waiting",
                             waiting_path,
                             MotionMode::Queue,
                             3))
              .ok());

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.active.id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.transition.target_state.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(store.findRun("cancel-running-waiting").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("cancel-running-waiting").run->stop_reason == StopReason::Stop);
  const auto source = store.findRun("cancel-running-user");
  REQUIRE(source.ok());
  REQUIRE(source.run->state == MotionState::Stopped);
  REQUIRE(source.run->stop_reason == StopReason::Stop);
  REQUIRE(source.run->err == ErrorCode::Ok);

  advanceToStandbyPlayback(loop, store, standby_track->metadata.frames);
  for (std::size_t i = 0; i < standby_track->metadata.frames + 4; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::None) {
      break;
    }
  }

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("cancel-running-user").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("cancel-running-user").run->stop_reason == StopReason::Stop);
}

TEST_CASE("RuntimeControlLoop standby velocity rebuilds active idle transition to standby") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "transition_idle_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          &reference,
                          standby_track);

  startCompletedUserToIdleTransition(loop,
                                     store,
                                     bridge,
                                     tmp,
                                     "standby-cancel-idle-transition");
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
  REQUIRE(store.snapshot().transition.target == "idle");
  const ReferenceFrameSnapshot source_frame = reference.latest;
  REQUIRE(source_frame.active);

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.queue.ids.empty());
  requireReferenceStartsFrom(reference, source_frame);

  advanceToStandbyPlayback(loop, store, standby_track->metadata.frames);
  for (std::size_t i = 0; i < standby_track->metadata.frames + 4; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::None) {
      break;
    }
  }

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
}

TEST_CASE("RuntimeControlLoop standby velocity cancels active user transition to standby") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "transition_user_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          &reference,
                          standby_track);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "standby-cancel-held-source",
                               "standby-cancel-old-target",
                               &reference);
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
  REQUIRE(store.snapshot().transition.target == "user");
  REQUIRE(store.snapshot().transition.target_id == "standby-cancel-old-target");
  const ReferenceFrameSnapshot source_frame = reference.latest;
  REQUIRE(source_frame.active);

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  requireReferenceStartsFrom(reference, source_frame);

  const auto old_target = store.findRun("standby-cancel-old-target");
  REQUIRE(old_target.ok());
  REQUIRE(old_target.run->state == MotionState::Canceled);
  REQUIRE(old_target.run->stop_reason == StopReason::Stop);

  advanceToStandbyPlayback(loop, store, standby_track->metadata.frames);
  for (std::size_t i = 0; i < standby_track->metadata.frames + 4; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::None) {
      break;
    }
  }

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("standby-cancel-old-target").run->state ==
          MotionState::Canceled);
  REQUIRE(store.findRun("standby-cancel-old-target").run->stop_reason ==
          StopReason::Stop);
}

TEST_CASE("RuntimeControlLoop standby cancellation transition failure falls back to stopping") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "cancel_failure_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              nullptr,
                              standby_track);

  const auto path = validTrk(tmp, "cancel_failure_user.trk", 5);
  REQUIRE(bridge.submitQueue(
              executeCommand("cancel-failure-user", path, MotionMode::Queue, 5))
              .ok());
  startQueuedRun(loop,
                 store,
                 "cancel-failure-user",
                 StartQueuedRunMode::AllowStandbyTransition);

  loop.failNextTransitionStartForTest();
  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "cancel-failure-user");
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("cancel-failure-user").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("cancel-failure-user").run->stop_reason == StopReason::Stop);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("cancel-failure-user").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("cancel-failure-user").run->stop_reason == StopReason::Stop);
}

TEST_CASE("RuntimeControlLoop standby cancellation transition fault remains fault") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "cancel_fault_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              nullptr,
                              standby_track);

  const auto path = validTrk(tmp, "cancel_fault_user.trk", 5);
  REQUIRE(bridge.submitQueue(
              executeCommand("cancel-fault-user", path, MotionMode::Queue, 5))
              .ok());
  startQueuedRun(loop,
                 store,
                 "cancel-fault-user",
                 StartQueuedRunMode::AllowStandbyTransition);

  loop.faultNextTransitionStartForTest();
  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("cancel-fault-user").run->state == MotionState::Failed);
  REQUIRE(store.findRun("cancel-fault-user").run->err ==
          ErrorCode::ModelInferenceFailed);
}

TEST_CASE("RuntimeControlLoop standby velocity while preparing falls back without transition") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "preparing_cancel_standby_ref.trk", 2));
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          robot,
                          tracker_policy,
                          deploy_config,
                          passiveConfig(),
                          kExpectedModeMachine,
                          RuntimeMode::Real,
                          nullptr,
                          standby_track);

  const auto path = validTrk(tmp, "preparing_cancel_user.trk", 5);
  REQUIRE(bridge.submitQueue(
              executeCommand("preparing-cancel-user", path, MotionMode::Queue, 5))
              .ok());
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Preparing);
  REQUIRE(store.snapshot().active.kind == ActiveKind::User);
  REQUIRE_FALSE(store.snapshot().transition.active);

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "preparing-cancel-user");
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("preparing-cancel-user").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("preparing-cancel-user").run->stop_reason == StopReason::Stop);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("preparing-cancel-user").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("preparing-cancel-user").run->stop_reason == StopReason::Stop);
}

TEST_CASE("RuntimeControlLoop stop passive and fixstand preempt standby cancellation transition") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* id;
    ControlMode mode;
    ControllerState expected_ctrl;
    ActiveKind expected_active;
  };

  for (const auto& item : std::vector<Case>{
           {"cancel-transition-stop", ControlMode::StandbyVelocity,
            ControllerState::Stopping, ActiveKind::None},
           {"cancel-transition-passive", ControlMode::Passive,
            ControllerState::Passive, ActiveKind::None},
           {"cancel-transition-fixstand", ControlMode::FixStand,
            ControllerState::FixStand, ActiveKind::None},
       }) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    const auto standby_track =
        validStandbyTrack(tmp, std::string(item.id) + "_standby_ref.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                nullptr,
                                standby_track);

    const auto path = validTrk(tmp, std::string(item.id) + "_user.trk", 5);
    REQUIRE(bridge.submitQueue(
                executeCommand(item.id, path, MotionMode::Queue, 5))
                .ok());
    startQueuedRun(loop,
                   store,
                   item.id,
                   StartQueuedRunMode::AllowStandbyTransition);

    REQUIRE(bridge.standbyVelocity().ok());
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
    REQUIRE(store.snapshot().transition.target == "standby");
    REQUIRE(store.findRun(item.id).run->state == MotionState::Stopped);
    REQUIRE(store.findRun(item.id).run->stop_reason == StopReason::Stop);

    if (std::string(item.id) == "cancel-transition-stop") {
      REQUIRE(bridge.stop().state == ControllerState::Stopping);
    } else if (item.mode == ControlMode::Passive) {
      REQUIRE(bridge.passive().ok());
    } else {
      REQUIRE(bridge.fixStand().ok());
    }

    loop.tick();
    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE(snapshot.active.kind == item.expected_active);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(store.findRun(item.id).run->state == MotionState::Stopped);
    REQUIRE(store.findRun(item.id).run->stop_reason == StopReason::Stop);
  }
}

TEST_CASE("RuntimeControlLoop holding to user transition is internal status only") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingRun(loop, store, bridge, tmp, "held-user", &reference);
  const auto next_path = validTrk(tmp, "held_to_next.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("next-user", next_path, MotionMode::Queue, 2))
              .ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.active.id.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "next-user");
  REQUIRE(snapshot.transition.frame == 0);
  REQUIRE(snapshot.transition.frames > 1);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.id.empty());
  REQUIRE(store.findRun("held-user").run->state == MotionState::Done);
  const auto target = store.findRun("next-user");
  REQUIRE(target.ok());
  REQUIRE(target.run->state == MotionState::Queued);
  REQUIRE(target.run->frame == 0);
  REQUIRE(target.run->frames == 2);
  REQUIRE(target.run->progress == 0.0);
  REQUIRE(store.findRun("transition").code == ErrorCode::RunNotFound);
}

TEST_CASE("RuntimeControlLoop transition completion starts target user at frame zero") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "held-complete-transition",
                               "target-after-transition",
                               &reference);
  const std::size_t transition_frames = store.snapshot().transition.frames;
  REQUIRE(transition_frames > 1);

  for (std::size_t i = 0; i < transition_frames + 2; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::User) {
      REQUIRE(snapshot.exec.has_value());
      REQUIRE(snapshot.exec->id == "target-after-transition");
      REQUIRE(snapshot.exec->state == MotionState::Running);
      REQUIRE(snapshot.exec->frame == 0);
      REQUIRE(snapshot.exec->frames == 2);
      REQUIRE(snapshot.exec->progress == 0.5);
      REQUIRE_FALSE(snapshot.transition.active);
      REQUIRE(snapshot.queue.ids.empty());
      REQUIRE(store.findRun("held-complete-transition").run->state ==
              MotionState::Done);
      return;
    }
  }

  FAIL("target user did not start after synthetic transition");
}

TEST_CASE("RuntimeControlLoop aligns synthetic transition target root pose in memory") {
  constexpr float kHalfPi = 1.57079632679F;
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          robot,
                          tracker_policy,
                          deploy_config,
                          passiveConfig(),
                          kExpectedModeMachine,
                          RuntimeMode::Real,
                          &reference);

  const auto held_path =
      rootPoseTrk(tmp,
                  "held_five_meters_yaw90.trk",
                  1,
                  {5.0F, 0.0F, 0.0F},
                  yawQuat(kHalfPi));
  const auto target_path =
      rootPoseTrk(tmp,
                  "target_origin_yaw0.trk",
                  2,
                  {0.0F, 0.0F, 0.0F},
                  yawQuat(0.0F));
  REQUIRE(bridge.submitQueue(
              executeCommand("held-five-meters",
                             held_path,
                             MotionMode::Queue,
                             1,
                             true))
              .ok());
  startQueuedRun(loop, store, "held-five-meters");
  bool saw_holding = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    if (store.snapshot().exec.has_value() &&
        store.snapshot().exec->state == MotionState::Holding) {
      saw_holding = true;
      break;
    }
  }
  REQUIRE(saw_holding);

  REQUIRE(bridge.submitQueue(
              executeCommand("target-origin", target_path, MotionMode::Queue, 2))
              .ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.target_id == "target-origin");
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.p.at(0) == std::array<float, 3>{{5.0F, 0.0F, 0.0F}});
  REQUIRE(yawFromQuat(reference.latest.q.at(0)) ==
          Catch::Approx(kHalfPi).margin(1.0e-5F));

  loop.tick();
  REQUIRE_FALSE(tracker_policy.inputs_seen.empty());
  const Vec& transition_obs = tracker_policy.inputs_seen.back().obs_current;
  REQUIRE(transition_obs.size() > 8);
  REQUIRE(transition_obs.at(6) == Catch::Approx(0.0F).margin(1.0F));

  const std::size_t transition_frames = snapshot.transition.frames;
  for (std::size_t i = 0; i < transition_frames + 3; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::User) {
      REQUIRE(snapshot.exec.has_value());
      REQUIRE(snapshot.exec->id == "target-origin");
      REQUIRE(snapshot.exec->frame == 0);
      REQUIRE(reference.latest.active);
      REQUIRE(reference.latest.id == "target-origin");
      REQUIRE(reference.latest.p.at(0) ==
              std::array<float, 3>{{5.0F, 0.0F, 0.0F}});
      REQUIRE(yawFromQuat(reference.latest.q.at(0)) ==
              Catch::Approx(kHalfPi).margin(1.0e-5F));
      return;
    }
  }

  FAIL("aligned target user did not start after synthetic transition");
}

TEST_CASE("RuntimeControlLoop transition abort controls terminate target user") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* target_id;
    ControlMode mode;
    ControllerState expected_ctrl;
    int ticks_after_command;
  };

  for (const auto& item : std::vector<Case>{
           {"transition-passive-target", ControlMode::Passive,
            ControllerState::Passive, 1},
           {"transition-fixstand-target", ControlMode::FixStand,
            ControllerState::FixStand, 1},
       }) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    FakeReferenceSink reference;
    const auto startup_standby_track =
        validStandbyTrack(tmp, std::string(item.target_id) + "_startup_standby.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference,
                                startup_standby_track);
    startHoldingToUserTransition(loop,
                                 store,
                                 bridge,
                                 tmp,
                                 std::string(item.target_id) + "-held",
                                 item.target_id,
                                 &reference,
                                 StartQueuedRunMode::AllowStandbyTransition);

    if (item.mode == ControlMode::Passive) {
      REQUIRE(bridge.passive().ok());
    } else if (item.mode == ControlMode::FixStand) {
      REQUIRE(bridge.fixStand().ok());
    } else {
      REQUIRE(bridge.standbyVelocity().ok());
    }

    for (int i = 0; i < item.ticks_after_command; ++i) {
      loop.tick();
    }

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE_FALSE(reference.latest.active);

    const auto target = store.findRun(item.target_id);
    REQUIRE(target.ok());
    REQUIRE(target.run->state == MotionState::Canceled);
    REQUIRE(target.run->stop_reason == StopReason::Stop);
  }
}

TEST_CASE("RuntimeControlLoop interrupt during transition rebuilds to urgent target") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "held-before-interrupt-transition",
                               "old-transition-target",
                               &reference);
  loop.tick();
  const auto before = store.snapshot();
  REQUIRE(before.active.kind == ActiveKind::Transition);
  REQUIRE(before.transition.target_id == "old-transition-target");
  REQUIRE(before.transition.frame >= 0);

  const auto urgent_path = validTrk(tmp, "urgent_transition_target.trk", 3);
  REQUIRE(bridge.submitInterrupt(
              executeCommand("urgent-transition-target",
                             urgent_path,
                             MotionMode::Interrupt,
                             3))
              .ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "urgent-transition-target");
  REQUIRE(snapshot.transition.frame == 0);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(reference.latest.active);

  const auto old_target = store.findRun("old-transition-target");
  REQUIRE(old_target.ok());
  REQUIRE(old_target.run->state == MotionState::Canceled);
  REQUIRE(old_target.run->stop_reason == StopReason::Interrupt);
  const auto urgent = store.findRun("urgent-transition-target");
  REQUIRE(urgent.ok());
  REQUIRE(urgent.run->state == MotionState::Queued);
}

TEST_CASE("RuntimeControlLoop failed transition build leaves held source holding") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingRun(loop, store, bridge, tmp, "held-transition-build-fails", &reference);
  const auto invalid_path = invalidContactTrk(tmp, "invalid_transition_target.trk");
  REQUIRE(bridge.submitQueue(
              executeCommand("invalid-transition-target",
                             invalid_path,
                             MotionMode::Queue,
                             3))
              .ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.active.id == "held-transition-build-fails");
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("held-transition-build-fails").run->state ==
          MotionState::Holding);
  REQUIRE(store.findRun("invalid-transition-target").run->state ==
          MotionState::Failed);
}

TEST_CASE("RuntimeControlLoop transition start failure does not complete held source") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          robot,
                          policy,
                          deploy_config,
                          passiveConfig(),
                          kExpectedModeMachine,
                          RuntimeMode::Real,
                          &reference);

  startHoldingRun(loop, store, bridge, tmp, "held-transition-start-fails", &reference);
  const auto target_path = validTrk(tmp, "transition_start_failure_target.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("transition-start-failure-target",
                             target_path,
                             MotionMode::Queue,
                             2))
              .ok());

  loop.failNextTransitionStartForTest();
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.active.id == "held-transition-start-fails");
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("held-transition-start-fails").run->state ==
          MotionState::Holding);
  const auto target = store.findRun("transition-start-failure-target");
  REQUIRE(target.ok());
  REQUIRE(target.run->state == MotionState::Failed);
  REQUIRE(target.run->err == ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop interrupt transition restarts from current transition frame") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "held-current-frame-interrupt",
                               "old-current-frame-target",
                               &reference);
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
  REQUIRE(store.snapshot().transition.frame > 0);
  const ReferenceFrameSnapshot interrupted_frame = reference.latest;
  REQUIRE(interrupted_frame.active);

  const auto urgent_path = validTrk(tmp, "urgent_current_frame_target.trk", 3);
  REQUIRE(bridge.submitInterrupt(
              executeCommand("urgent-current-frame-target",
                             urgent_path,
                             MotionMode::Interrupt,
                             3))
              .ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.target_id == "urgent-current-frame-target");
  REQUIRE(snapshot.transition.frame == 0);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.p == interrupted_frame.p);
  REQUIRE(reference.latest.q == interrupted_frame.q);
  REQUIRE(reference.latest.c == interrupted_frame.c);
  REQUIRE(reference.latest.com == interrupted_frame.com);
  REQUIRE(reference.latest.comv == interrupted_frame.comv);
}

TEST_CASE("RuntimeControlLoop transition duration config controls synthetic frame count") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "held-duration-config",
                               "target-duration-config",
                               &reference);

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.frames == 51);
}

TEST_CASE("RuntimeControlLoop completed user enters idle through internal transition") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "idle_priority_standby_ref.trk", 2));
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          &reference,
                          standby_track);

  const auto idle_path = validTrk(tmp, "user_to_idle_idle.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 2)}).ok());
  loop.tick();
  REQUIRE(store.snapshot().idle.enabled);
  REQUIRE_FALSE(store.snapshot().idle.active);

  const auto user_path = validTrk(tmp, "user_to_idle_user.trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand("user-before-idle", user_path, MotionMode::Queue, 1))
              .ok());
  startQueuedRun(loop, store, "user-before-idle");

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "idle");
  REQUIRE(snapshot.transition.target_id.empty());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(store.findRun("user-before-idle").run->state == MotionState::Done);
}

TEST_CASE("RuntimeControlLoop invalid idle transition build completes user and falls back") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto idle_path = zeroQuaternionTrk(tmp, "user_to_invalid_idle.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 2)}).ok());
  loop.tick();
  REQUIRE(store.snapshot().idle.enabled);
  REQUIRE_FALSE(store.snapshot().idle.active);

  const auto user_path = validTrk(tmp, "user_before_invalid_idle.trk", 1);
  REQUIRE(bridge.submitQueue(
              executeCommand("user-before-invalid-idle",
                             user_path,
                             MotionMode::Queue,
                             1))
              .ok());
  startQueuedRun(loop, store, "user-before-invalid-idle");

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(store.findRun("user-before-invalid-idle").run->state ==
          MotionState::Done);

  loop.tick();
  const auto next = store.snapshot();
  REQUIRE(next.active.kind != ActiveKind::User);
  REQUIRE_FALSE(next.transition.active);
  REQUIRE(store.findRun("user-before-invalid-idle").run->state ==
          MotionState::Done);
}

TEST_CASE("RuntimeControlLoop stop aborts active transition without playing standby reference") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingRun(loop, store, bridge, tmp, "held-before-stop", &reference);
  const auto next_path = validTrk(tmp, "transition_stop_next.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("next-after-stop", next_path)).ok());
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  const auto post_stop_path = validTrk(tmp, "post_stop_after_transition.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("post-stop-after-transition",
                                            post_stop_path,
                                            MotionMode::Queue,
                                            2))
              .ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids ==
          std::vector<std::string>{"post-stop-after-transition"});
  REQUIRE_FALSE(reference.latest.active);
  REQUIRE(store.findRun("next-after-stop").run->state == MotionState::Canceled);

  loop.tick();
  requireIdle(store);
  REQUIRE_FALSE(store.snapshot().transition.active);
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"post-stop-after-transition"});
}

TEST_CASE("RuntimeControlLoop controls abort standby reference gate without resuming playback") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "abort_standby_ref.trk", 2));

  struct Case {
    const char* id;
    ControlMode mode;
    ControllerState expected_ctrl;
  };

  for (const auto& item : std::vector<Case>{
           {"abort-standby-stop", ControlMode::StandbyVelocity,
            ControllerState::Stopping},
           {"abort-standby-passive", ControlMode::Passive, ControllerState::Passive},
           {"abort-standby-fixstand", ControlMode::FixStand, ControllerState::FixStand},
       }) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    FakeReferenceSink reference;
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixStandConfig(),
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference,
                                standby_track);
    startHoldingRun(loop,
                    store,
                    bridge,
                    tmp,
                    item.id,
                    &reference,
                    StartQueuedRunMode::AllowStandbyTransition);

    REQUIRE(bridge.standbyVelocity().ok());
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
    REQUIRE(store.snapshot().transition.target == "standby");

    if (std::string(item.id) == "abort-standby-stop") {
      REQUIRE(bridge.stop().state == ControllerState::Stopping);
    } else if (item.mode == ControlMode::Passive) {
      REQUIRE(bridge.passive().ok());
    } else {
      REQUIRE(bridge.fixStand().ok());
    }
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE_FALSE(reference.latest.active);
    REQUIRE(store.findRun(item.id).run->state == MotionState::Done);
    REQUIRE(store.findRun("standby_ref").code == ErrorCode::RunNotFound);
  }
}

TEST_CASE("RuntimeControlLoop clears reference on stop interrupt and loader failure") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();

  auto started_loop = [&](RuntimeStatusStore& store,
                          RuntimeBridge& bridge,
                          FakeReferenceSink& reference,
                          const std::string& id) {
    auto loop =
        RuntimeControlLoop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);
    const auto path = validTrk(tmp, id + ".trk", 4);
    REQUIRE(bridge.submitQueue(executeCommand(id, path, MotionMode::Queue, 4)).ok());
    loop.tick();
    loop.tick();
    REQUIRE(reference.latest.active);
    return loop;
  };

  SECTION("stop clears active reference") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    auto loop = started_loop(store, bridge, reference, "ref-stop");

    REQUIRE(bridge.stop().ok());
    loop.tick();

    REQUIRE_FALSE(reference.latest.active);
    REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  }

  SECTION("interrupt clears active reference") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    auto loop = started_loop(store, bridge, reference, "ref-interrupt");
    const auto next_path = validTrk(tmp, "ref-next.trk", 2);

    REQUIRE(bridge.submitInterrupt(
                executeCommand("ref-next", next_path, MotionMode::Interrupt, 2))
                .ok());
    loop.tick();

    REQUIRE_FALSE(reference.latest.active);
    REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  }

  SECTION("loader failure clears reference") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);
    const auto path = invalidContactTrk(tmp, "reference_invalid.trk");

    REQUIRE(bridge.submitQueue(executeCommand("ref-bad", path)).ok());
    loop.tick();
    loop.tick();

    REQUIRE_FALSE(reference.latest.active);
    REQUIRE(store.findRun("ref-bad").run->state == MotionState::Failed);
  }
}

TEST_CASE("RuntimeControlLoop reference sink failures do not change motion state") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  reference.throw_on_publish = true;
  reference.throw_on_clear = true;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto path = validTrk(tmp, "reference_throw.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("ref-throw", path, MotionMode::Queue, 2))
              .ok());

  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Running);
  loop.tick();
  loop.tick();

  const auto found = store.findRun("ref-throw");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Done);
  REQUIRE(found.run->err == ErrorCode::Ok);
}

TEST_CASE("RuntimeControlLoop records loader validation failure without entering fault") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto path = invalidContactTrk(tmp, "invalid_contact.trk");
  REQUIRE(bridge.submitQueue(executeCommand("bad-contact", path)).ok());

  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Preparing);

  loop.tick();

  requireIdle(store);
  REQUIRE(store.snapshot().queue.ids.empty());
  const auto found = store.findRun("bad-contact");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::TrkValidationFailed);
}

TEST_CASE("RuntimeControlLoop policy mode refreshes idle readiness without queued work") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  auto run_case = [&](std::optional<LowStateSample> low_state,
                      ErrorCode expected_err,
                      RobotState expected_robot,
                      std::string expected_block,
                      bool expected_ready,
                      std::size_t expected_low_ms) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    robot.low_state = low_state;
    FakePolicy policy;
    auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                               deploy_config);

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl ==
            (expected_ready ? ControllerState::Idle : ControllerState::Passive));
    REQUIRE(snapshot.ready == expected_ready);
    REQUIRE(snapshot.robot == expected_robot);
    REQUIRE(snapshot.err == expected_err);
    REQUIRE(snapshot.block == expected_block);
    REQUIRE(snapshot.low_ms == expected_low_ms);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE_FALSE(snapshot.exec.has_value());
    const auto health = store.health();
    const ServiceHealth expected_health =
        expected_ready && expected_err == ErrorCode::Ok && expected_block.empty()
            ? ServiceHealth::Ready
            : ServiceHealth::Starting;
    REQUIRE(health.state == expected_health);
    REQUIRE(health.mode == RuntimeMode::Real);
    REQUIRE(health.err == expected_err);
    REQUIRE(health.block == expected_block);
    REQUIRE(robot.read_low_calls == (expected_ready ? 2 : 1));
    REQUIRE(robot.occupancy_calls == (expected_ready ? 2 : 1));
    REQUIRE(policy.calls == 0);
    REQUIRE(robot.write_attempts == (expected_ready ? 1 : 0));
  };

  SECTION("ready lowstate clears constructor not-ready state") {
    run_case(readyLowState(deploy_config, kExpectedModeMachine, true, 17),
             ErrorCode::Ok,
             RobotState::Idle,
             "",
             true,
             17);
  }

  SECTION("stale lowstate remains not ready with block") {
    run_case(readyLowState(deploy_config, kExpectedModeMachine, false, 123),
             ErrorCode::RobotNotReady,
             RobotState::NotReady,
             "lowstate_timeout",
             false,
             123);
  }

  SECTION("missing lowstate remains disconnected with block") {
    run_case(std::nullopt,
             ErrorCode::RobotDisconnected,
             RobotState::Disconnected,
             "lowstate_missing",
             false,
             0);
  }
}

TEST_CASE("RuntimeControlLoop policy idle readiness maps safety sink and LowCmd occupancy") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  struct Case {
    const char* id;
    LowStateSample low_state;
    LowCmdOccupancy occupancy;
    RuntimeInternalState expected_fsm;
    ControllerState expected_ctrl;
    ErrorCode err;
    RobotState robot;
    std::string block;
  };

  const std::vector<Case> cases{
      {"idle-bad-orientation",
       badOrientationLowState(deploy_config, 61),
       {},
       RuntimeInternalState::Passive,
       ControllerState::Passive,
       ErrorCode::RobotBadOrientation,
       RobotState::Fault,
       "bad_orientation"},
      {"idle-lowcmd-occupied",
       readyLowState(deploy_config, kExpectedModeMachine, true, 62),
       {true, 3},
       RuntimeInternalState::Fault,
       ControllerState::Fault,
       ErrorCode::RobotNotReady,
       RobotState::NotReady,
       "lowcmd_occupied"},
  };

  for (const auto& item : cases) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(item.low_state);
    robot.occupancy = item.occupancy;
    FakePolicy policy;
    auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                               deploy_config);

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == item.expected_fsm);
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.robot == item.robot);
    REQUIRE(snapshot.err == item.err);
    REQUIRE(snapshot.block == item.block);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(store.health().state == ServiceHealth::Error);
    REQUIRE(policy.calls == 0);
    REQUIRE(robot.write_attempts == 0);
  }
}

TEST_CASE("RuntimeControlLoop publishes compact pose from latest low and high state") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  LowStateSample low = readyLowState(deploy_config, kExpectedModeMachine, true, 12);
  low.quat_wxyz = {0.9F, 0.1F, 0.2F, 0.3F};
  low.gyro = {1.0F, 2.0F, 3.0F};
  FakeRobotIO robot(low);
  HighStateSample high;
  high.fresh = true;
  high.age_ms = 8;
  high.position = {4.0F, 5.0F, 6.0F};
  high.linear_velocity = {7.0F, 8.0F, 9.0F};
  robot.high_state = high;
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE(snapshot.pose.q_wxyz == std::array<float, 4>{{0.9F, 0.1F, 0.2F, 0.3F}});
  REQUIRE(snapshot.pose.gyro_xyz == std::array<float, 3>{{1.0F, 2.0F, 3.0F}});
  REQUIRE(snapshot.pose.position_xyz == std::array<float, 3>{{4.0F, 5.0F, 6.0F}});
  REQUIRE(snapshot.pose.velocity_xyz == std::array<float, 3>{{7.0F, 8.0F, 9.0F}});

  robot.high_state->fresh = false;
  loop.tick();

  const StatusSnapshot stale_high = store.snapshot();
  REQUIRE(stale_high.pose.q_wxyz.has_value());
  REQUIRE(stale_high.pose.gyro_xyz.has_value());
  REQUIRE_FALSE(stale_high.pose.position_xyz.has_value());
  REQUIRE_FALSE(stale_high.pose.velocity_xyz.has_value());
}

TEST_CASE("RuntimeControlLoop policy integration starts then steps fake runtime") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config, kExpectedModeMachine, true, 11));
  FakePolicy policy(floatSeq(0.25F, kPolicyJointDim));
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "policy_happy.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("policy-run", path)).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "policy-run");
  REQUIRE(snapshot.exec->state == MotionState::Queued);
  REQUIRE(robot.read_low_calls == 1);
  REQUIRE(robot.occupancy_calls == 1);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.mode == RuntimeMode::Real);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "policy-run");
  REQUIRE(snapshot.exec->frame == 0);
  REQUIRE(robot.read_low_calls == 2);
  REQUIRE(robot.occupancy_calls == 2);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->frame == 0);
  REQUIRE(robot.read_low_calls == 3);
  REQUIRE(robot.occupancy_calls == 3);
  REQUIRE(policy.calls == 1);
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(robot.writes.size() == 1);
  REQUIRE(robot.writes.back().mode_machine == kExpectedModeMachine);
}

TEST_CASE("RuntimeControlLoop user policy startup holds frame zero then reanchors") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LowStateSample low = readyLowState(deploy_config);
  const Vec raw_action = floatSeq(0.0F, kPolicyJointDim);
  const Vec first_frame_joint_pos = fixtureFirstFrameJointPos();
  low.motors.at(static_cast<std::size_t>(deploy_config.sdk_joint_ids_map.at(12))).q =
      0.0F;
  low.motors.at(static_cast<std::size_t>(deploy_config.sdk_joint_ids_map.at(13))).q =
      0.0F;
  const LowStateSample startup_low = low;
  FakeRobotIO robot(low);
  FakePolicy policy(raw_action);
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = identityQuaternionTrk(tmp, "startup_hold_user.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("startup-hold-user", path)).ok());
  startQueuedRun(loop, store, "startup-hold-user");
  REQUIRE(kStartupHoldPolicyStepsAt50Fps == 25);

  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    if (tick == kStartupHoldPolicyStepsAt50Fps / 2) {
      low.quat_wxyz = yawQuat(0.5F);
      robot.low_state = low;
    }
    loop.tick();
    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == "startup-hold-user");
    REQUIRE(snapshot.exec->state == MotionState::Running);
    REQUIRE(snapshot.exec->frame == 0);
    REQUIRE(policy.calls == static_cast<int>(tick + 1));
    REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
            0.0F);
    REQUIRE(robot.writes.size() == tick + 1);
    requireStartupHoldInterpolatedFrame(robot.writes.back(),
                                        deploy_config,
                                        startup_low,
                                        raw_action,
                                        first_frame_joint_pos,
                                        tick);
  }

  loop.tick();
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
          6.5F);
  requireObsSliceApprox(policy.inputs_seen.back().obs_current,
                        kObsCurrentRootOffset,
                        rootOriFromYaw(0.0F));
}

TEST_CASE("RuntimeControlLoop user startup hold counts policy writes at high runtime hz") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 1000.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = identityQuaternionTrk(tmp, "startup_hold_high_rate.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("startup-hold-high-rate", path)).ok());
  startQueuedRun(loop, store, "startup-hold-high-rate");

  loop.tick();
  REQUIRE(policy.calls == 1);
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);
  REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
          0.0F);

  for (std::size_t write = 1; write < kStartupHoldPolicyStepsAt50Fps; ++write) {
    for (int non_due_tick = 0; non_due_tick < 19; ++non_due_tick) {
      loop.tick();
      REQUIRE(policy.calls == static_cast<int>(write));
      REQUIRE(store.snapshot().exec.has_value());
      REQUIRE(store.snapshot().exec->frame == 0);
    }

    loop.tick();
    REQUIRE(policy.calls == static_cast<int>(write + 1));
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->frame == 0);
    REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
            0.0F);
  }

  REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps));
  REQUIRE(store.snapshot().exec->frame == 0);

  for (int non_due_tick = 0; non_due_tick < 19; ++non_due_tick) {
    loop.tick();
    REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps));
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->frame == 0);
  }

  loop.tick();
  REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 1);
  REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
          6.5F);
}

TEST_CASE("RuntimeControlLoop stop during user startup hold still preempts") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = identityQuaternionTrk(tmp, "startup_hold_stop.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("startup-hold-stop", path)).ok());
  startQueuedRun(loop, store, "startup-hold-stop");

  const std::size_t preempt_ticks = kStartupHoldPolicyStepsAt50Fps - 1;
  for (std::size_t i = 0; i < preempt_ticks; ++i) {
    loop.tick();
    REQUIRE(policy.calls == static_cast<int>(i + 1));
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->frame == 0);
  }

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(policy.calls == static_cast<int>(preempt_ticks));

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(policy.calls == static_cast<int>(preempt_ticks));
  const auto stopped = store.findRun("startup-hold-stop");
  REQUIRE(stopped.ok());
  REQUIRE(stopped.run->state == MotionState::Stopped);
  REQUIRE(stopped.run->stop_reason == StopReason::Stop);
}

TEST_CASE("RuntimeControlLoop synthetic transition is not held but target user startup is") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const LowStateSample startup_low = readyLowState(deploy_config);
  const Vec raw_action = floatSeq(0.0F, kPolicyJointDim);
  const Vec first_frame_joint_pos = fixtureFirstFrameJointPos();
  FakeRobotIO robot(startup_low);
  FakePolicy policy(raw_action);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          robot,
                          policy,
                          deploy_config,
                          passiveConfig(),
                          kExpectedModeMachine,
                          RuntimeMode::Real,
                          &reference);

  startHoldingRun(loop, store, bridge, tmp, "startup-held-source", &reference);
  const auto target_path = identityQuaternionTrk(tmp, "startup_hold_target.trk", 3);
  REQUIRE(bridge.submitQueue(
              executeCommand("startup-target-user", target_path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
  const int calls_before_transition_step = policy.calls;

  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
  REQUIRE(policy.calls == calls_before_transition_step + 1);

  bool saw_user = false;
  for (int i = 0; i < 96; ++i) {
    loop.tick();
    if (store.snapshot().active.kind == ActiveKind::User) {
      saw_user = true;
      break;
    }
  }
  REQUIRE(saw_user);
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->id == "startup-target-user");
  REQUIRE(store.snapshot().exec->frame == 0);
  const int calls_at_target_start = policy.calls;
  const std::size_t writes_at_target_start = robot.writes.size();

  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::User);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->frame == 0);
    REQUIRE(policy.calls == calls_at_target_start + static_cast<int>(tick + 1));
    REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
            0.0F);
    REQUIRE(robot.writes.size() == writes_at_target_start + tick + 1);
    requireStartupHoldInterpolatedFrame(robot.writes.back(),
                                        deploy_config,
                                        startup_low,
                                        raw_action,
                                        first_frame_joint_pos,
                                        tick);
  }

  loop.tick();
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(policy.calls ==
          calls_at_target_start + static_cast<int>(kStartupHoldPolicyStepsAt50Fps) + 1);
  REQUIRE(policy.inputs_seen.back().obs_current.at(kObsCurrentCommandJointOffset) ==
          6.5F);
}

TEST_CASE("RuntimeControlLoop policy integration writes final frame before done") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  std::optional<MotionState> state_at_write;
  std::optional<std::size_t> frame_at_write;
  robot.on_write = [&](const LowCmdFrame&) {
    const auto found = store.findRun("final-frame");
    REQUIRE(found.ok());
    state_at_write = found.run->state;
    frame_at_write = found.run->frame;
  };
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "policy_final.trk", 1);
  REQUIRE(bridge.submitQueue(executeCommand("final-frame", path, MotionMode::Queue, 1))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Preparing);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Running);

  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps + 1; ++tick) {
    loop.tick();
  }
  requireIdle(store);
  REQUIRE(robot.write_attempts == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  REQUIRE(policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  REQUIRE(state_at_write == MotionState::Running);
  REQUIRE(frame_at_write == 0);

  const auto found = store.findRun("final-frame");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Done);
  REQUIRE(found.run->frame == 0);
}

TEST_CASE("RuntimeControlLoop start gate keeps queued work when robot is not ready") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config, kExpectedModeMachine, false, 123));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto disallowed_path = tmp.root / "not_loaded.trk";
  REQUIRE(bridge.submitQueue(executeCommand("wait-robot", disallowed_path)).ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::NotReady);
  REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
  REQUIRE(snapshot.block == "lowstate_timeout");
  REQUIRE(snapshot.low_ms == 123);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"wait-robot"});
  REQUIRE(store.findRun("wait-robot").run->state == MotionState::Queued);
  REQUIRE(robot.read_low_calls == 1);
  REQUIRE(robot.occupancy_calls == 1);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
}

TEST_CASE("RuntimeControlLoop preparing gate keeps queued work when robot becomes not ready") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config, kExpectedModeMachine, true, 21));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "preparing_lowstate_stale.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("prep-stale", path)).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "prep-stale");
  REQUIRE(snapshot.queue.ids.empty());

  robot.low_state = readyLowState(deploy_config, kExpectedModeMachine, false, 222);
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::NotReady);
  REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
  REQUIRE(snapshot.block == "lowstate_timeout");
  REQUIRE(snapshot.low_ms == 222);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"prep-stale"});
  REQUIRE(store.findRun("prep-stale").ok());
  REQUIRE(store.findRun("prep-stale").run->state == MotionState::Queued);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);

  robot.low_state = readyLowState(deploy_config, kExpectedModeMachine, true, 23);
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"prep-stale"});
  REQUIRE(store.findRun("prep-stale").run->state == MotionState::Queued);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  requirePassiveDampingFrame(robot.writes.back(), *robot.low_state, passiveConfig());
}

TEST_CASE("RuntimeControlLoop policy start gate enters Passive except LowCmd occupancy") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  struct Case {
    const char* id;
    LowStateSample low_state;
    LowCmdOccupancy occupancy;
    RuntimeInternalState expected_fsm;
    ControllerState expected_ctrl;
    ErrorCode err;
    RobotState robot;
    std::string block;
  };

  const std::vector<Case> cases{
      {"start-bad-orientation",
       badOrientationLowState(deploy_config, 71),
       {},
       RuntimeInternalState::Passive,
       ControllerState::Passive,
       ErrorCode::RobotBadOrientation,
       RobotState::Fault,
       "bad_orientation"},
      {"start-lowcmd-occupied",
       readyLowState(deploy_config, kExpectedModeMachine, true, 72),
       {true, 4},
       RuntimeInternalState::Fault,
       ControllerState::Fault,
       ErrorCode::RobotNotReady,
       RobotState::NotReady,
       "lowcmd_occupied"},
  };

  for (const auto& item : cases) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(item.low_state);
    robot.occupancy = item.occupancy;
    FakePolicy policy;
    auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                               deploy_config);
    const auto path = validTrk(tmp, std::string(item.id) + ".trk", 3);
    REQUIRE(bridge.submitQueue(executeCommand(item.id, path)).ok());

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == item.expected_fsm);
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.robot == item.robot);
    REQUIRE(snapshot.err == item.err);
    REQUIRE(snapshot.block == item.block);
    REQUIRE(snapshot.low_ms == item.low_state.age_ms);
    REQUIRE(snapshot.queue.ids == std::vector<std::string>{item.id});
    REQUIRE(store.health().state == ServiceHealth::Error);
    REQUIRE(store.findRun(item.id).run->state == MotionState::Queued);
    REQUIRE(policy.calls == 0);
    REQUIRE(robot.write_attempts == 0);
  }
}

TEST_CASE("RuntimeControlLoop Passive does not auto resume or start queued motion") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(badOrientationLowState(deploy_config, 71));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  loop.tick();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(store.snapshot().ctrl == ControllerState::Passive);
  REQUIRE(robot.write_attempts == 0);

  const auto path = validTrk(tmp, "passive_queued.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("passive-queued", path)).ok());
  robot.low_state = readyLowState(deploy_config);

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"passive-queued"});
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(velocity_policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  requirePassiveDampingFrame(robot.writes.back(), *robot.low_state, passiveConfig());

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"passive-queued"});
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(robot.write_attempts == 2);
}

TEST_CASE("RuntimeControlLoop stop in Passive cancels queued motion without leaving Passive") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(badOrientationLowState(deploy_config, 72));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  loop.tick();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);

  const auto path = validTrk(tmp, "passive_stop_queued.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("passive-stop-queued", path)).ok());
  robot.low_state = readyLowState(deploy_config);
  loop.tick();
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"passive-stop-queued"});
  REQUIRE(robot.write_attempts == 1);

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(velocity_policy.calls == 0);
  REQUIRE(store.findRun("passive-stop-queued").run->state == MotionState::Canceled);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(robot.write_attempts == 2);
  REQUIRE(velocity_policy.calls == 0);
  requirePassiveDampingFrame(robot.writes.back(), *robot.low_state, passiveConfig());
}

TEST_CASE("RuntimeControlLoop stop in Passive without queued motion is no-op") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(badOrientationLowState(deploy_config, 73));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  loop.tick();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  robot.low_state = readyLowState(deploy_config);
  loop.tick();
  REQUIRE(robot.write_attempts == 1);

  REQUIRE(bridge.stop().state == ControllerState::Passive);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(robot.write_attempts == 2);
  REQUIRE(velocity_policy.calls == 0);
  requirePassiveDampingFrame(robot.writes.back(), *robot.low_state, passiveConfig());
}

TEST_CASE("RuntimeControlLoop loader failure in policy mode stays non-fault") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = invalidContactTrk(tmp, "policy_invalid_contact.trk");
  REQUIRE(bridge.submitQueue(executeCommand("bad-policy-track", path)).ok());

  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Preparing);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(policy.calls == 0);
  const auto found = store.findRun("bad-policy-track");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::TrkValidationFailed);
}

TEST_CASE("RuntimeControlLoop policy failure maps to fault status") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  auto run_case = [&](FakePolicy& policy, const std::string& id) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                               deploy_config);
    const auto path = validTrk(tmp, id + ".trk", 2);
    REQUIRE(bridge.submitQueue(executeCommand(id, path)).ok());

    startQueuedRun(loop, store, id);

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
    REQUIRE(snapshot.block == "policy_inference_failed");
    REQUIRE(policy.calls == 1);
    REQUIRE(robot.write_attempts == 0);
    const auto health = store.health();
    REQUIRE(health.state == ServiceHealth::Error);
    REQUIRE(health.mode == RuntimeMode::Real);
    REQUIRE(health.err == ErrorCode::ModelInferenceFailed);
    REQUIRE(health.block == "policy_inference_failed");
    const auto found = store.findRun(id);
    REQUIRE(found.ok());
    REQUIRE(found.run->state == MotionState::Failed);
    REQUIRE(found.run->err == ErrorCode::ModelInferenceFailed);
  };

  SECTION("bad raw action size") {
    FakePolicy policy(floatSeq(0.0F, kPolicyJointDim - 1));
    run_case(policy, "bad-action");
  }

  SECTION("policy throws") {
    FakePolicy policy;
    policy.throw_on_infer = true;
    run_case(policy, "policy-throws");
  }
}

TEST_CASE("RuntimeControlLoop policy fault latches across stop and readiness refresh") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim - 1));
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto fault_path = validTrk(tmp, "fault_latch.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("fault-latch", fault_path)).ok());
  startQueuedRun(loop, store, "fault-latch");
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");

  const auto queued_path = validTrk(tmp, "queued_after_fault.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("queued-after-fault", queued_path)).ok());
  REQUIRE(bridge.stop().state == ControllerState::Stopping);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");
  REQUIRE(snapshot.queue.ids.empty());
  const auto canceled = store.findRun("queued-after-fault");
  REQUIRE(canceled.ok());
  REQUIRE(canceled.run->state == MotionState::Canceled);
  REQUIRE(canceled.run->stop_reason == StopReason::Stop);

  loop.tick();
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");
  REQUIRE(policy.calls == 1);
  REQUIRE(robot.write_attempts == 0);
}

TEST_CASE("RuntimeControlLoop running lowstate readiness failure enters Passive") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  struct Case {
    const char* id;
    std::optional<LowStateSample> low_state;
    ErrorCode err;
    std::string block;
  };

  const std::vector<Case> cases{
      {"missing-lowstate", std::nullopt, ErrorCode::RobotDisconnected, "lowstate_missing"},
      {"stale-lowstate",
       readyLowState(deploy_config, kExpectedModeMachine, false, 51),
       ErrorCode::RobotNotReady,
       "lowstate_timeout"},
      {"mode-mismatch",
       readyLowState(deploy_config, kExpectedModeMachine + 1, true, 52),
       ErrorCode::RobotNotReady,
       "mode_machine_mismatch"},
  };

  for (const auto& item : cases) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy policy;
    auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                               deploy_config);
    const auto path = validTrk(tmp, std::string(item.id) + ".trk", 2);
    REQUIRE(bridge.submitQueue(executeCommand(item.id, path)).ok());

    startQueuedRun(loop, store, item.id);
    robot.low_state = item.low_state;

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
    REQUIRE(snapshot.ctrl == ControllerState::Passive);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == item.err);
    REQUIRE(snapshot.block == item.block);
    REQUIRE(policy.calls == 0);
    REQUIRE(robot.write_attempts == 0);
    const auto found = store.findRun(item.id);
    REQUIRE(found.ok());
    REQUIRE(found.run->state == MotionState::Failed);
    REQUIRE(found.run->err == item.err);
  }
}

TEST_CASE("RuntimeControlLoop running bad orientation enters Passive before policy write") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);
  const auto path = validTrk(tmp, "running_bad_orientation.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("running-bad-orientation", path)).ok());

  startQueuedRun(loop, store, "running-bad-orientation");
  robot.low_state = badOrientationLowState(deploy_config, 81);

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::Fault);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.low_ms == 81);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  const auto found = store.findRun("running-bad-orientation");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::RobotBadOrientation);
}

TEST_CASE("RuntimeControlLoop running LowCmd occupancy fails before policy write") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);
  const auto path = validTrk(tmp, "running_lowcmd_occupied.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("running-lowcmd-occupied", path)).ok());

  startQueuedRun(loop, store, "running-lowcmd-occupied");
  robot.occupancy = {true, 4};

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::NotReady);
  REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
  REQUIRE(snapshot.block == "lowcmd_occupied");
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(robot.writes.empty());
  const auto found = store.findRun("running-lowcmd-occupied");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::RobotNotReady);
}

TEST_CASE("RuntimeControlLoop pure sim ignores bad orientation safety gate") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());
  const auto path = validTrk(tmp, "pure_sim_no_orientation_gate.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("pure-sim", path)).ok());

  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Preparing);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "pure-sim");
}

TEST_CASE("RuntimeControlLoop LowCmd write failure maps to fault block") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  robot.throw_on_write = true;
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "write_fail.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("write-fail", path)).ok());

  startQueuedRun(loop, store, "write-fail");

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::InternalError);
  REQUIRE(snapshot.block == "lowcmd_write_failed");
  REQUIRE(policy.calls == 1);
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(robot.writes.empty());
  const auto found = store.findRun("write-fail");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop policy stop ignores configured hold time and writes no hold-current frame") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const LowStateSample hold_state = readyLowState(deploy_config);
  FakeRobotIO robot(hold_state);
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "policy_stop.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("policy-stop", path)).ok());
  startQueuedRun(loop, store, "policy-stop");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  const auto found = store.findRun("policy-stop");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Stopping);
  REQUIRE(found.run->stop_reason == StopReason::Stop);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(robot.writes.empty());
  const auto stopped = store.findRun("policy-stop");
  REQUIRE(stopped.ok());
  REQUIRE(stopped.run->state == MotionState::Stopped);
  REQUIRE(stopped.run->stop_reason == StopReason::Stop);
}

TEST_CASE("RuntimeControlLoop policy idle hold_current writes hold command when ready") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const LowStateSample hold_state = readyLowState(deploy_config);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(hold_state);
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.ready);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(robot.writes.size() == 1);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);
}

TEST_CASE("RuntimeControlLoop control startup FixStand waits for readiness then publishes frame") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  const LowStateSample ready_state = readyLowState(deploy_config);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(ready_state);
  robot.low_state = readyLowState(deploy_config, kExpectedModeMachine, false, 123);
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::FixStand);

  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
  REQUIRE(snapshot.block == "lowstate_timeout");
  REQUIRE(robot.write_attempts == 0);

  robot.low_state = ready_state;
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.ready);
  REQUIRE(robot.write_attempts == 1);
  requirePassiveDampingFrame(robot.writes.back(), ready_state, passiveConfig());

  REQUIRE(bridge.fixStand().ok());
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.ready);
  REQUIRE(robot.write_attempts == 2);
  requireFixStandFrameFromCurrentQ(robot.writes.back(), fixstand_config, ready_state);
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(velocity_policy.calls == 0);
}

TEST_CASE("RuntimeControlLoop FixStand can recover from Passive bad orientation") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(badOrientationLowState(deploy_config, 77));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Velocity);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(robot.write_attempts == 0);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(robot.write_attempts == 1);
  requirePassiveDampingFrame(robot.writes.back(), *robot.low_state, passiveConfig());

  REQUIRE(bridge.fixStand().ok());
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::Fault);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.low_ms == 77);
  REQUIRE(robot.write_attempts == 2);
  requireFixStandFrameFromCurrentQ(robot.writes.back(), fixstand_config,
                                   *robot.low_state);

  const LowStateSample ready_state = readyLowState(deploy_config, kExpectedModeMachine,
                                                   true, 8);
  robot.low_state = ready_state;
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(robot.write_attempts == 3);
}

TEST_CASE("RuntimeControlLoop lowcmd occupancy overrides bad orientation before control writes") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  auto require_fault_without_write = [&](ControlMode startup_control) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(badOrientationLowState(deploy_config, 88));
    robot.occupancy = {true, 4};
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                startup_control);

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
    REQUIRE(snapshot.block == "lowcmd_occupied");
    REQUIRE(robot.write_attempts == 0);
  };

  SECTION("standby velocity preserves queued work and starts it from FixStand") {
    require_fault_without_write(ControlMode::StandbyVelocity);
  }

  SECTION("fixstand") {
    require_fault_without_write(ControlMode::FixStand);
  }

  SECTION("passive damping") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(badOrientationLowState(deploy_config, 89));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity);

    loop.tick();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
    REQUIRE(robot.write_attempts == 0);

    robot.occupancy = {true, 5};
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
    REQUIRE(snapshot.block == "lowcmd_occupied");
    REQUIRE(robot.write_attempts == 0);
  }
}

TEST_CASE("RuntimeControlLoop Passive treats lowcmd occupancy as fault before stale or mode blocks") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* id;
    LowStateSample low_state;
  };

  const std::vector<Case> cases{
      {"stale", readyLowState(deploy_config, kExpectedModeMachine, false, 91)},
      {"mode-mismatch", readyLowState(deploy_config, kExpectedModeMachine + 1, true, 92)},
  };

  for (const auto& item : cases) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(badOrientationLowState(deploy_config, 90));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity);

    loop.tick();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
    REQUIRE(robot.write_attempts == 0);

    robot.low_state = item.low_state;
    robot.occupancy = {true, 6};
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
    REQUIRE(snapshot.block == "lowcmd_occupied");
    REQUIRE(robot.write_attempts == 0);
  }
}

TEST_CASE("RuntimeControlLoop FixStand blocks non-orientation readiness failures") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* id;
    std::optional<LowStateSample> low_state;
    LowCmdOccupancy occupancy;
    ControllerState expected_ctrl;
    RuntimeInternalState expected_fsm;
    ErrorCode err;
    std::string block;
  };

  const std::vector<Case> cases{
      {"missing",
       std::nullopt,
       {},
       ControllerState::Passive,
       RuntimeInternalState::Passive,
       ErrorCode::RobotDisconnected,
       "lowstate_missing"},
      {"stale",
       readyLowState(deploy_config, kExpectedModeMachine, false, 51),
       {},
       ControllerState::Passive,
       RuntimeInternalState::Passive,
       ErrorCode::RobotNotReady,
       "lowstate_timeout"},
      {"mode-mismatch",
       readyLowState(deploy_config, kExpectedModeMachine + 1, true, 52),
       {},
       ControllerState::Passive,
       RuntimeInternalState::Passive,
       ErrorCode::RobotNotReady,
       "mode_machine_mismatch"},
      {"lowcmd-occupied",
       readyLowState(deploy_config, kExpectedModeMachine, true, 53),
       {true, 3},
       ControllerState::Fault,
       RuntimeInternalState::Fault,
       ErrorCode::RobotNotReady,
       "lowcmd_occupied"},
  };

  for (const auto& item : cases) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    robot.low_state = item.low_state;
    robot.occupancy = item.occupancy;
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::FixStand);

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == item.expected_fsm);
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == item.err);
    REQUIRE(snapshot.block == item.block);
    REQUIRE(robot.write_attempts == 0);
  }
}

TEST_CASE("RuntimeControlLoop completed track returns to StandbyVelocity") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  const auto standby_track = validStandbyTrack(tmp, "done_to_standby_ref.trk");
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              nullptr,
                              standby_track);

  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Velocity);
  const auto path = validTrk(tmp, "done_to_standby.trk", 1);
  REQUIRE(bridge.submitQueue(executeCommand("done-standby", path, MotionMode::Queue, 1))
              .ok());
  startQueuedRun(loop,
                 store,
                 "done-standby",
                 StartQueuedRunMode::AllowStandbyTransition);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerActive);

  StatusSnapshot snapshot;
  bool saw_standby_transition = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::Transition &&
        snapshot.transition.active &&
        snapshot.transition.target == "standby") {
      saw_standby_transition = true;
      break;
    }
  }
  REQUIRE(saw_standby_transition);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerTransition);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("done-standby").run->state == MotionState::Done);
  const LowCmdFrame tracker_frame = robot.writes.back();

  bool reached_standby = false;
  for (int i = 0; i < 200; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::None) {
      reached_standby = true;
      break;
    }
  }
  REQUIRE(reached_standby);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerIdle);
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerIdle);
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(robot.write_attempts >= 2);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);
  REQUIRE(robot.writes.back().motors.at(12).q != tracker_frame.motors.at(12).q);
  requireFixStandHoldMotor(robot.writes.back(), fixstand_config, 12);
}

TEST_CASE("RuntimeControlLoop standby no-active user gates through standby reference") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  for (const MotionMode mode : {MotionMode::Queue, MotionMode::Interrupt}) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    const auto standby_track =
        loadTrack(tmp.trkConfig(),
                  identityQuaternionTrk(
                      tmp,
                      std::string("standby_source_") + toString(mode) + ".trk",
                      3));
    const auto expected_source =
        makeReferenceFrameSnapshot("",
                                   *standby_track,
                                   standby_track->metadata.frames - 1);
    REQUIRE(expected_source.has_value());
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference,
                                standby_track);

    const std::string id = std::string("standby-first-user-") + toString(mode);
    const auto user_path =
        validTrk(tmp, std::string("standby_first_user_") + toString(mode) + ".trk", 2);
    if (mode == MotionMode::Interrupt) {
      REQUIRE(bridge.submitInterrupt(executeCommand(id, user_path, mode, 2)).ok());
    } else {
      REQUIRE(bridge.submitQueue(executeCommand(id, user_path, mode, 2)).ok());
    }

    loop.tick();

    auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.active.kind == ActiveKind::Transition);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.transition.active);
    REQUIRE(snapshot.transition.target == "user");
    REQUIRE(snapshot.transition.target_id == id);
    REQUIRE(snapshot.transition.target_state == MotionState::Queued);
    REQUIRE(snapshot.transition.frame == 0);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE_FALSE(snapshot.idle.active);
    REQUIRE(reference.latest.active);
    REQUIRE(reference.latest.id.empty());
    REQUIRE(reference.latest.frame == 0);
    REQUIRE(reference.latest.p == expected_source->p);
    REQUIRE(reference.latest.q == expected_source->q);
    REQUIRE(reference.latest.c == expected_source->c);
    REQUIRE(reference.latest.com == expected_source->com);
    REQUIRE(store.findRun(id).run->state == MotionState::Queued);

    bool saw_user = false;
    const std::size_t transition_frames = snapshot.transition.frames;
    for (std::size_t i = 0; i < transition_frames + 3; ++i) {
      loop.tick();
      snapshot = store.snapshot();
      if (snapshot.active.kind == ActiveKind::User) {
        saw_user = true;
        REQUIRE(snapshot.ctrl == ControllerState::Running);
        REQUIRE(snapshot.exec.has_value());
        REQUIRE(snapshot.exec->id == id);
        REQUIRE(snapshot.exec->state == MotionState::Running);
        REQUIRE(snapshot.exec->frame == 0);
        REQUIRE(snapshot.exec->frames == 2);
        REQUIRE_FALSE(snapshot.transition.active);
        REQUIRE(reference.latest.active);
        REQUIRE(reference.latest.id == id);
        REQUIRE(reference.latest.frame == 0);
        break;
      }
    }
    REQUIRE(saw_user);
  }
}

TEST_CASE("RuntimeControlLoop standby no-active missing standby track fails user gate") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto user_path = validTrk(tmp, "missing_standby_first_user.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("missing-standby-first-user", user_path, MotionMode::Queue, 2))
              .ok());

  loop.tick();

  requireStandbyGateFailure(loop,
                            store,
                            reference,
                            robot,
                            tracker_policy,
                            velocity_policy,
                            "missing-standby-first-user",
                            ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop standby no-active user gate failures do not raw start") {
  TempTree tmp;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* id;
    bool missing_target;
    bool invalid_transition_duration;
    ErrorCode expected_error;
  };

  for (const auto& item : std::vector<Case>{
           {"standby-missing-target", true, false, ErrorCode::TrkFileNotFound},
           {"standby-invalid-duration", false, true, ErrorCode::InternalError},
       }) {
    RuntimeConfig config = runtimeConfig();
    if (item.invalid_transition_duration) {
      config.transition_duration_s = 6.0;
    }
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    const auto standby_track =
        validStandbyTrack(tmp, std::string(item.id) + "_standby_ref.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference,
                                standby_track);

    const std::filesystem::path user_path =
        item.missing_target
            ? tmp.allowed / (std::string(item.id) + ".trk")
            : validTrk(tmp, std::string(item.id) + ".trk", 2);
    REQUIRE(bridge.submitQueue(
                executeCommand(item.id, user_path, MotionMode::Queue, 2))
                .ok());

    loop.tick();

    requireStandbyGateFailure(loop,
                              store,
                              reference,
                              robot,
                              tracker_policy,
                              velocity_policy,
                              item.id,
                              item.expected_error);
  }
}

TEST_CASE("RuntimeControlLoop idle config stays out of user run and queue state") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  const auto idle_path = validTrk(tmp, "idle_pool.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 2)}).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.active.kind == ActiveKind::Idle);
  REQUIRE(snapshot.active.id.empty());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.active);
  REQUIRE(snapshot.idle.current == 0);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Idle);
  REQUIRE(snapshot.idle.active);
  REQUIRE(snapshot.idle.frames == 2);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
}

TEST_CASE("RuntimeControlLoop active idle publishes reference and controls clear it") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* suffix;
    ControlMode mode;
    ControllerState expected_ctrl;
    bool expect_no_active;
  };

  for (const auto& item : std::vector<Case>{
           {"stop", ControlMode::StandbyVelocity, ControllerState::StandbyVelocity,
            true},
           {"passive", ControlMode::Passive, ControllerState::Passive, true},
           {"fixstand", ControlMode::FixStand, ControllerState::FixStand, true},
           {"standby", ControlMode::StandbyVelocity, ControllerState::Preparing,
            false},
       }) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference);

    const auto idle_path =
        validTrk(tmp, std::string("idle_reference_") + item.suffix + ".trk", 3);
    REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
    loop.tick();
    loop.tick();
    loop.tick();

    auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.active.kind == ActiveKind::Idle);
    REQUIRE(snapshot.idle.active);
    REQUIRE(reference.latest.active);
    REQUIRE(reference.latest.id.empty());
    REQUIRE(reference.latest.frame == 0);
    REQUIRE(reference.latest.frames == 3);

    loop.tick();
    loop.tick();
    snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::Idle);
    REQUIRE(reference.latest.active);
    REQUIRE(reference.latest.id.empty());
    REQUIRE(reference.latest.frame == 1);

    if (std::string(item.suffix) == "stop") {
      REQUIRE(bridge.stop().ok());
    } else if (item.mode == ControlMode::Passive) {
      REQUIRE(bridge.passive().ok());
    } else if (item.mode == ControlMode::FixStand) {
      REQUIRE(bridge.fixStand().ok());
    } else {
      REQUIRE(bridge.standbyVelocity().ok());
    }
    loop.tick();

    snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == item.expected_ctrl);
    if (item.expect_no_active) {
      REQUIRE(snapshot.active.kind == ActiveKind::None);
      REQUIRE_FALSE(snapshot.idle.active);
    } else {
      REQUIRE(snapshot.active.kind == ActiveKind::Idle);
      REQUIRE(snapshot.idle.active);
    }
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(reference.latest.active);
    REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);
  }
}

TEST_CASE("RuntimeControlLoop completed idle enters next idle through internal transition") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_a = validTrk(tmp, "idle_to_idle_a.trk", 2);
  const auto idle_b = validTrk(tmp, "idle_to_idle_b.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_a, 2), idleMotion(idle_b, 2)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.current == 0);

  bool saw_transition = false;
  for (int i = 0; i < 6; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::Transition) {
      saw_transition = true;
      REQUIRE(snapshot.transition.active);
      REQUIRE(snapshot.transition.target == "idle");
      REQUIRE(snapshot.transition.target_id.empty());
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE_FALSE(snapshot.idle.active);
      REQUIRE(snapshot.queue.ids.empty());
      REQUIRE(store.findRun(idle_a.string()).code == ErrorCode::RunNotFound);
      REQUIRE(store.findRun(idle_b.string()).code == ErrorCode::RunNotFound);
      break;
    }
  }
  REQUIRE(saw_transition);
}

TEST_CASE("RuntimeControlLoop idle to idle transition build failure does not stick or record history") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_a = validTrk(tmp, "idle_to_bad_idle_a.trk", 2);
  const auto idle_bad = zeroQuaternionTrk(tmp, "idle_to_bad_idle_b.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_a, 2), idleMotion(idle_bad, 2)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  bool stopped_idle = false;
  for (int i = 0; i < 6; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind != ActiveKind::Transition);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE(store.findRun(idle_a.string()).code == ErrorCode::RunNotFound);
    REQUIRE(store.findRun(idle_bad.string()).code == ErrorCode::RunNotFound);
    if (snapshot.active.kind == ActiveKind::None) {
      REQUIRE_FALSE(snapshot.idle.enabled);
      REQUIRE_FALSE(snapshot.idle.active);
      stopped_idle = true;
      break;
    }
  }
  REQUIRE(stopped_idle);

  loop.tick();
  const auto next = store.snapshot();
  REQUIRE(next.active.kind == ActiveKind::None);
  REQUIRE_FALSE(next.idle.enabled);
  REQUIRE_FALSE(next.idle.active);
  REQUIRE_FALSE(next.transition.active);
  REQUIRE_FALSE(next.exec.has_value());
}

TEST_CASE("RuntimeControlLoop stop clears consumed inactive idle config") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              nullptr);

  const auto idle_path = validTrk(tmp, "stop_consumed_inactive_idle.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 2)}).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());

  REQUIRE(bridge.stop().ok());
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
}

TEST_CASE("RuntimeControlLoop rereads safety before auto-starting idle") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  const auto idle_path = validTrk(tmp, "idle_safety_reread.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 2)}).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.active.kind == ActiveKind::None);

  robot.low_state = badOrientationLowState(deploy_config, 91);
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
}

TEST_CASE("RuntimeControlLoop user execute preempts active idle without idle run history") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  for (const MotionMode mode : {MotionMode::Queue, MotionMode::Interrupt}) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference);
    const auto idle_path = validTrk(tmp, std::string("preempt_idle_") +
                                             toString(mode) + ".trk", 3);
    const auto user_path = validTrk(tmp, std::string("preempt_user_") +
                                             toString(mode) + ".trk", 2);
    REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
    loop.tick();
    loop.tick();
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
    REQUIRE(store.snapshot().idle.active);
    REQUIRE_FALSE(store.snapshot().exec.has_value());
    const auto idle_track = loadTrack(tmp.trkConfig(), idle_path);
    const auto expected_source =
        makeReferenceFrameSnapshot("", *idle_track, store.snapshot().idle.frame);
    REQUIRE(expected_source.has_value());

    if (mode == MotionMode::Interrupt) {
      REQUIRE(bridge.submitInterrupt(executeCommand("user", user_path, mode, 2)).ok());
    } else {
      REQUIRE(bridge.submitQueue(executeCommand("user", user_path, mode, 2)).ok());
    }
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.active.kind == ActiveKind::Transition);
    REQUIRE(snapshot.transition.active);
    REQUIRE(snapshot.transition.target == "user");
    REQUIRE(snapshot.transition.target_id == "user");
    REQUIRE(snapshot.transition.target_state == MotionState::Queued);
    REQUIRE(reference.latest.active);
    REQUIRE(reference.latest.frame == 0);
    REQUIRE(reference.latest.p == expected_source->p);
    REQUIRE(reference.latest.c == expected_source->c);
    REQUIRE(reference.latest.com == expected_source->com);
    REQUIRE_FALSE(snapshot.idle.active);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE(store.findRun("user").ok());
    REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);
  }
}

TEST_CASE("RuntimeControlLoop queue preempts background transition and playback to user",
          "[runtime-p1]") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();

  SECTION("transition target idle") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

    startCompletedUserToIdleTransition(loop, store, bridge, tmp, "queue-idle");
    advanceBackgroundTransition(loop, store, "idle");
    const ReferenceFrameSnapshot source = reference.latest;
    REQUIRE(source.active);

    const auto user_path = validTrk(tmp, "queue_idle_preempt_user.trk", 2);
    REQUIRE(bridge.submitQueue(
                executeCommand("queue-idle-preempt-user",
                               user_path,
                               MotionMode::Queue,
                               2))
                .ok());
    loop.tick();

    requireUserTransitionStatus(store, "queue-idle-preempt-user");
    requireReferenceStartsFrom(reference, source);
    REQUIRE(store.findRun("queue-idle-preempt-user").run->state ==
            MotionState::Queued);
  }

  SECTION("transition target standby") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    const auto standby_track = validStandbyTrack(tmp, "queue_standby_ref.trk");
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference,
                            standby_track);

    startCompletedUserToStandbyTransition(
        loop, store, bridge, tmp, "queue-standby");
    advanceBackgroundTransition(loop, store, "standby");
    const ReferenceFrameSnapshot source = reference.latest;
    REQUIRE(source.active);

    const auto user_path = validTrk(tmp, "queue_standby_preempt_user.trk", 2);
    REQUIRE(bridge.submitQueue(
                executeCommand("queue-standby-preempt-user",
                               user_path,
                               MotionMode::Queue,
                               2))
                .ok());
    loop.tick();

    requireUserTransitionStatus(store, "queue-standby-preempt-user");
    requireReferenceStartsFrom(reference, source);
  }

  SECTION("standby playback") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    const auto standby_track = validStandbyTrack(tmp, "queue_playback_ref.trk");
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference,
                            standby_track);

    startCompletedUserToStandbyTransition(
        loop, store, bridge, tmp, "queue-playback");
    advanceToStandbyPlayback(loop, store, standby_track->metadata.frames);
    const ReferenceFrameSnapshot source = reference.latest;
    REQUIRE(source.active);

    const auto user_path = validTrk(tmp, "queue_playback_preempt_user.trk", 2);
    REQUIRE(bridge.submitQueue(
                executeCommand("queue-playback-preempt-user",
                               user_path,
                               MotionMode::Queue,
                               2))
                .ok());
    loop.tick();

    requireUserTransitionStatus(store, "queue-playback-preempt-user");
    requireReferenceStartsFrom(reference, source);
  }
}

TEST_CASE("RuntimeControlLoop queue during user transition waits behind existing target",
          "[runtime-p1]") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "queue-user-owned-held",
                               "queue-user-owned-old",
                               &reference);

  const auto user_path = validTrk(tmp, "queue_user_owned_new.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("queue-user-owned-new",
                             user_path,
                             MotionMode::Queue,
                             2))
              .ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "queue-user-owned-old");
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"queue-user-owned-new"});
  REQUIRE(store.findRun("queue-user-owned-old").run->state == MotionState::Queued);
  REQUIRE(store.findRun("queue-user-owned-new").run->state == MotionState::Queued);
}

TEST_CASE("RuntimeControlLoop failed queue preempt of standby playback drops background",
          "[runtime-p1]") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  const auto standby_track = validStandbyTrack(tmp, "queue_fail_playback_ref.trk");
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig()),
                          &reference,
                          standby_track);

  startCompletedUserToStandbyTransition(
      loop, store, bridge, tmp, "queue-fail-playback");
  advanceToStandbyPlayback(loop, store, standby_track->metadata.frames);

  const auto invalid_path = invalidContactTrk(tmp, "queue_fail_bad_user.trk");
  REQUIRE(bridge.submitQueue(
              executeCommand("queue-fail-bad-user",
                             invalid_path,
                             MotionMode::Queue,
                             3))
              .ok());
  loop.tick();

  requireNoActiveWorkOrBackground(store);

  const auto failed = store.findRun("queue-fail-bad-user");
  REQUIRE(failed.ok());
  REQUIRE(failed.run.has_value());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::TrkValidationFailed);
  REQUIRE(failed.run->stop_reason == StopReason::None);

  loop.tick();
  requireNoActiveWorkOrBackground(store);
}

TEST_CASE("RuntimeControlLoop failed queue preempt of idle transition drops background",
          "[runtime-p1]") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startCompletedUserToIdleTransition(loop, store, bridge, tmp, "queue-fail-idle");
  advanceBackgroundTransition(loop, store, "idle");

  const auto invalid_path = invalidContactTrk(tmp, "queue_fail_idle_bad_user.trk");
  REQUIRE(bridge.submitQueue(
              executeCommand("queue-fail-idle-bad-user",
                             invalid_path,
                             MotionMode::Queue,
                             3))
              .ok());
  loop.tick();

  requireNoActiveWorkOrBackground(store);

  const auto failed = store.findRun("queue-fail-idle-bad-user");
  REQUIRE(failed.ok());
  REQUIRE(failed.run.has_value());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::TrkValidationFailed);
  REQUIRE(failed.run->stop_reason == StopReason::None);

  loop.tick();
  requireNoActiveWorkOrBackground(store);
}

TEST_CASE("RuntimeControlLoop interrupt preempts background transition and playback from current reference",
          "[runtime-p1]") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();

  SECTION("transition target idle") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

    startCompletedUserToIdleTransition(loop, store, bridge, tmp, "interrupt-idle");
    advanceBackgroundTransition(loop, store, "idle");
    const ReferenceFrameSnapshot source = reference.latest;
    REQUIRE(source.active);

    const auto user_path = validTrk(tmp, "interrupt_idle_user.trk", 2);
    REQUIRE(bridge.submitInterrupt(
                executeCommand("interrupt-idle-user",
                               user_path,
                               MotionMode::Interrupt,
                               2))
                .ok());
    loop.tick();

    requireUserTransitionStatus(store, "interrupt-idle-user");
    requireReferenceStartsFrom(reference, source);
  }

  SECTION("transition target standby") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    const auto standby_track = validStandbyTrack(tmp, "interrupt_standby_ref.trk");
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference,
                            standby_track);

    startCompletedUserToStandbyTransition(
        loop, store, bridge, tmp, "interrupt-standby");
    advanceBackgroundTransition(loop, store, "standby");
    const ReferenceFrameSnapshot source = reference.latest;
    REQUIRE(source.active);

    const auto user_path = validTrk(tmp, "interrupt_standby_user.trk", 2);
    REQUIRE(bridge.submitInterrupt(
                executeCommand("interrupt-standby-user",
                               user_path,
                               MotionMode::Interrupt,
                               2))
                .ok());
    loop.tick();

    requireUserTransitionStatus(store, "interrupt-standby-user");
    requireReferenceStartsFrom(reference, source);
  }

  SECTION("standby playback") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    const auto standby_track = validStandbyTrack(tmp, "interrupt_playback_ref.trk");
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference,
                            standby_track);

    startCompletedUserToStandbyTransition(
        loop, store, bridge, tmp, "interrupt-playback");
    advanceToStandbyPlayback(loop, store, standby_track->metadata.frames);
    const ReferenceFrameSnapshot source = reference.latest;
    REQUIRE(source.active);

    const auto user_path = validTrk(tmp, "interrupt_playback_user.trk", 2);
    REQUIRE(bridge.submitInterrupt(
                executeCommand("interrupt-playback-user",
                               user_path,
                               MotionMode::Interrupt,
                               2))
                .ok());
    loop.tick();

    requireUserTransitionStatus(store, "interrupt-playback-user");
    requireReferenceStartsFrom(reference, source);
  }
}

TEST_CASE("RuntimeControlLoop idle to user transition target load failure publishes user failed") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_path = validTrk(tmp, "idle_to_bad_user_idle.trk", 3);
  const auto bad_user_path = invalidContactTrk(tmp, "idle_to_bad_user.trk");
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  REQUIRE(bridge.submitQueue(executeCommand("bad-user", bad_user_path)).ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  const auto failed = store.findRun("bad-user");
  REQUIRE(failed.ok());
  REQUIRE(failed.run.has_value());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::TrkValidationFailed);
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

  loop.tick();
  const auto next = store.snapshot();
  REQUIRE(next.active.kind == ActiveKind::None);
  REQUIRE_FALSE(next.idle.enabled);
  REQUIRE_FALSE(next.idle.active);
  REQUIRE_FALSE(next.transition.active);
  REQUIRE_FALSE(next.exec.has_value());
}

TEST_CASE("RuntimeControlLoop idle to user transition interrupt failure does not resume idle") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_path = validTrk(tmp, "idle_interrupt_bad_user_idle.trk", 3);
  const auto bad_user_path = invalidContactTrk(tmp, "idle_interrupt_bad_user.trk");
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  REQUIRE(bridge.submitInterrupt(
              executeCommand("bad-interrupt-user",
                             bad_user_path,
                             MotionMode::Interrupt,
                             3))
              .ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  const auto failed = store.findRun("bad-interrupt-user");
  REQUIRE(failed.ok());
  REQUIRE(failed.run.has_value());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::TrkValidationFailed);
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
}

TEST_CASE("RuntimeControlLoop invalid transition duration safely fails idle to user transition") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 6.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_path = validTrk(tmp, "idle_invalid_transition_duration.trk", 3);
  const auto user_path = validTrk(tmp, "user_invalid_transition_duration.trk", 2);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  REQUIRE(bridge.submitQueue(executeCommand("duration-invalid-user", user_path)).ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  const auto failed = store.findRun("duration-invalid-user");
  REQUIRE(failed.ok());
  REQUIRE(failed.run.has_value());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::InternalError);
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

  loop.tick();
  const auto next = store.snapshot();
  REQUIRE(next.active.kind == ActiveKind::None);
  REQUIRE_FALSE(next.idle.enabled);
  REQUIRE_FALSE(next.idle.active);
  REQUIRE_FALSE(next.transition.active);
  REQUIRE_FALSE(next.exec.has_value());
}

TEST_CASE("RuntimeControlLoop stop clears idle config and active idle without stop history") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  const auto idle_path = validTrk(tmp, "stop_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.active);

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 0);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);
}

TEST_CASE("RuntimeControlLoop passive clears idle config before returning to standby") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  const auto idle_path = validTrk(tmp, "passive_clears_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.enabled);
  REQUIRE(store.snapshot().idle.active);

  REQUIRE(bridge.passive().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 0);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());

  REQUIRE(bridge.fixStand().ok());
  loop.tick();
  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();
  for (int i = 0; i < 5; ++i) {
    loop.tick();
  }

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 0);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);
}

TEST_CASE("RuntimeControlLoop active idle bad orientation enters Passive without run history") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  const auto idle_path = validTrk(tmp, "idle_bad_orientation.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  robot.low_state = badOrientationLowState(deploy_config, 83);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);
}

TEST_CASE("RuntimeControlLoop standby velocity overlays latest FixStand LowCmd frame") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.5F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::FixStand);

  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::FixStand);
  REQUIRE(robot.write_attempts == 1);
  const LowCmdFrame fixstand_frame = robot.writes.back();

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Velocity);
  REQUIRE(store.snapshot().ctrl == ControllerState::StandbyVelocity);
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(robot.write_attempts == 2);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);
  requireMotorPreserved(robot.writes.back(), fixstand_frame, 12);
}

TEST_CASE("RuntimeControlLoop standby velocity republishes LowCmd at high rate but throttles policy") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 1000.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.5F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::StandbyVelocity);
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(robot.write_attempts == 1);
  const LowCmdFrame first_velocity = robot.writes.back();

  for (int i = 0; i < 19; ++i) {
    loop.tick();
  }
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(robot.write_attempts == 20);
  requireMotorPreserved(robot.writes.back(), first_velocity, 12);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);

  loop.tick();
  REQUIRE(velocity_policy.calls == 2);
  REQUIRE(robot.write_attempts == 21);
}

TEST_CASE("RuntimeControlLoop active tracker advances one frame per policy period without jitter skips") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 1000.0;
  const DeployConfig deploy_config = deployConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  std::vector<std::size_t> frames_at_write;
  robot.on_write = [&](const LowCmdFrame&) {
    const auto found = store.findRun("high-rate-track");
    if (found.ok() && found.run->state == MotionState::Running) {
      frames_at_write.push_back(found.run->frame);
    }
  };
  auto loop = makePolicyLoop(config,
                             bridge,
                             store,
                             tmp.trkConfig(),
                             robot,
                             tracker_policy,
                             deploy_config);

  const auto path = validTrk(tmp, "high_rate_track.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("high-rate-track", path, MotionMode::Queue, 3))
              .ok());
  startQueuedRun(loop, store, "high-rate-track");
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerActive);

  loop.tick();
  REQUIRE(tracker_policy.calls == 1);
  REQUIRE(frames_at_write == std::vector<std::size_t>{0});

  for (int i = 0; i < 19; ++i) {
    loop.tick();
  }
  REQUIRE(tracker_policy.calls == 1);
  REQUIRE(frames_at_write == std::vector<std::size_t>{0});
  REQUIRE(store.snapshot().exec->frame == 0);

  loop.tick();
  REQUIRE(tracker_policy.calls == 2);
  REQUIRE(frames_at_write == std::vector<std::size_t>{0, 0});
  REQUIRE(store.snapshot().exec->frame == 0);

  constexpr int kHighRateTicksPerPolicyStep = 20;
  const int ticks_to_last_startup_hold_write =
      static_cast<int>(kStartupHoldPolicyStepsAt50Fps - 2) *
          kHighRateTicksPerPolicyStep +
      (kHighRateTicksPerPolicyStep - 1);
  for (int i = 0; i < ticks_to_last_startup_hold_write; ++i) {
    loop.tick();
  }
  REQUIRE(tracker_policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps));
  REQUIRE(frames_at_write ==
          std::vector<std::size_t>(kStartupHoldPolicyStepsAt50Fps, 0));
  REQUIRE(store.snapshot().exec->frame == 0);

  loop.tick();
  REQUIRE(tracker_policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  std::vector<std::size_t> expected_frames(kStartupHoldPolicyStepsAt50Fps, 0);
  expected_frames.push_back(1);
  REQUIRE(frames_at_write == expected_frames);
  REQUIRE(store.snapshot().exec->frame == 1);

  for (int i = 0; i < 19; ++i) {
    loop.tick();
  }
  REQUIRE(tracker_policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  REQUIRE(frames_at_write == expected_frames);
  REQUIRE(store.snapshot().exec->frame == 1);

  loop.tick();
  expected_frames.push_back(2);
  REQUIRE(tracker_policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 2));
  REQUIRE(frames_at_write == expected_frames);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerIdle);
  REQUIRE_FALSE(store.snapshot().exec.has_value());
  REQUIRE(store.findRun("high-rate-track").run->state == MotionState::Done);
}

TEST_CASE("RuntimeControlLoop stop transitions active run to StandbyVelocity") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.5F));
  const auto standby_track = validStandbyTrack(tmp, "stop_to_standby_ref.trk");
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              nullptr,
                              standby_track);

  const auto path = validTrk(tmp, "stop_to_standby.trk", 5);
  REQUIRE(bridge.submitQueue(executeCommand("active", path)).ok());
  startQueuedRun(loop,
                 store,
                 "active",
                 StartQueuedRunMode::AllowStandbyTransition);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerActive);

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Stopping);
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerIdle);
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);

  loop.tick();
  REQUIRE(velocity_policy.calls == 1);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);
}

TEST_CASE("RuntimeControlLoop idle FixStand stop stays in FixStand") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.5F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::FixStand);

  REQUIRE(store.snapshot().ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().queue.ids.empty());

  REQUIRE(bridge.stop().state == ControllerState::FixStand);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(velocity_policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  requireFixStandFrameFromCurrentQ(robot.writes.back(), fixstand_config,
                                   *robot.low_state);
}

TEST_CASE("RuntimeControlLoop stop in Passive remains Passive") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.5F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity);

  REQUIRE(bridge.passive().ok());
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Passive);

  REQUIRE(bridge.stop().state == ControllerState::Passive);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(velocity_policy.calls == 0);
}

TEST_CASE("RuntimeControlLoop standby in FixStand cancels queued work") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  const auto standby_track = validStandbyTrack(tmp, "fixstand_queued_standby_ref.trk");
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::FixStand,
                              passiveConfig(),
                              nullptr,
                              standby_track);

  const auto path = validTrk(tmp, "fixstand_queued.trk", 2);
  REQUIRE(bridge.submitQueue(executeCommand("fixstand-queued", path)).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"fixstand-queued"});
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(tracker_policy.calls == 0);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"fixstand-queued"});
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(tracker_policy.calls == 0);

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Velocity);
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("fixstand-queued").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("fixstand-queued").run->stop_reason == StopReason::Stop);
  REQUIRE(velocity_policy.calls == 1);
}

TEST_CASE("RuntimeControlLoop fixstand and standby commands cancel queued work and target control") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  SECTION("fixstand") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity);
    const auto queued_path = validTrk(tmp, "queued_fixstand.trk", 3);
    REQUIRE(bridge.submitQueue(executeCommand("queued", queued_path)).ok());
    REQUIRE(bridge.fixStand().ok());

    loop.tick();

    REQUIRE(store.snapshot().ctrl == ControllerState::FixStand);
    REQUIRE(store.snapshot().queue.ids.empty());
    REQUIRE(store.findRun("queued").run->state == MotionState::Canceled);
    REQUIRE(robot.write_attempts == 1);
    requireFixStandFrameFromCurrentQ(robot.writes.back(), fixstand_config,
                                     *robot.low_state);
  }

  SECTION("standby velocity") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.75F));
    const auto standby_track = validStandbyTrack(tmp, "queued_standby_ref.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                nullptr,
                                standby_track);
    const auto queued_path = validTrk(tmp, "queued_standby.trk", 3);
    REQUIRE(bridge.submitQueue(executeCommand("queued", queued_path)).ok());
    REQUIRE(bridge.standbyVelocity().ok());

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(store.findRun("queued").run->state == MotionState::Canceled);
    REQUIRE(store.findRun("queued").run->stop_reason == StopReason::Stop);
    REQUIRE(velocity_policy.calls == 1);
  }
}

TEST_CASE("RuntimeControlLoop fixstand command while running stops active and clears waiting only") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  SECTION("fixstand") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.5F));
    const auto standby_track =
        validStandbyTrack(tmp, "fixstand_control_standby_ref.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocity_config,
                                fixstand_config,
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                nullptr,
                                standby_track);
    const auto active_path = validTrk(tmp, "fixstand_active.trk", 5);
    const auto waiting_path = validTrk(tmp, "fixstand_waiting.trk", 3);
    REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
    startQueuedRun(loop,
                   store,
                   "active",
                   StartQueuedRunMode::AllowStandbyTransition);
    REQUIRE(bridge.submitQueue(executeCommand("waiting", waiting_path)).ok());

    REQUIRE(bridge.fixStand().ok());

    loop.tick();
    auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Stopping);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->state == MotionState::Stopping);
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE(store.findRun("waiting").run->state == MotionState::Canceled);
    REQUIRE(store.findRun("waiting").run->stop_reason == StopReason::Stop);

    loop.tick();
    snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::FixStand);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
    REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  }
}

TEST_CASE("RuntimeControlLoop policy stop_hold_s zero writes no hold command") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "policy_zero_hold.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("zero-hold", path)).ok());
  startQueuedRun(loop, store, "zero-hold");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  REQUIRE(robot.write_attempts == 0);

  loop.tick();
  requireIdle(store);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  const auto found = store.findRun("zero-hold");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Stopped);
}

TEST_CASE("RuntimeControlLoop stop before consumption cancels queued work on stop tick") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto path = validTrk(tmp, "queued_stop.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("queued", path)).ok());
  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(store.findRun("queued").run->state == MotionState::Queued);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"queued"});

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("queued").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("queued").run->stop_reason == StopReason::Stop);

  loop.tick();
  requireIdle(store);
}

TEST_CASE("RuntimeControlLoop stop while running publishes stopped then idles") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto path = validTrk(tmp, "running_stop.trk", 5);
  REQUIRE(bridge.submitQueue(executeCommand("active", path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(store.findRun("active").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);

  loop.tick();
  requireIdle(store);
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
}

TEST_CASE("RuntimeControlLoop stop watermark preserves post-stop queue until idle") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto active_path = validTrk(tmp, "active_stop_then_queue.trk", 5);
  const auto queued_path = validTrk(tmp, "post_stop_queue.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitQueue(executeCommand("post-stop", queued_path)).ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"post-stop"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("post-stop").run->state == MotionState::Queued);

  loop.tick();
  requireIdle(store);
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"post-stop"});

  startQueuedRun(loop, store, "post-stop");
  REQUIRE(store.snapshot().queue.ids.empty());
}

TEST_CASE("RuntimeControlLoop fixstand after pending stop keeps stop watermark") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  const auto standby_track =
      validStandbyTrack(tmp, "active_stop_then_fixstand_standby_ref.trk");
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              nullptr,
                              standby_track);

  const auto active_path = validTrk(tmp, "active_stop_then_fixstand.trk", 5);
  const auto old_path = validTrk(tmp, "old_before_fixstand.trk", 3);
  const auto post_path = validTrk(tmp, "post_stop_after_fixstand.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop,
                 store,
                 "active",
                 StartQueuedRunMode::AllowStandbyTransition);

  REQUIRE(bridge.submitQueue(executeCommand("old-before-stop", old_path)).ok());
  loop.tick();
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"old-before-stop"});

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitQueue(executeCommand("post-stop", post_path)).ok());
  REQUIRE(bridge.fixStand().ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"post-stop"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("old-before-stop").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-before-stop").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("post-stop").run->state == MotionState::Queued);
  REQUIRE(store.findRun("post-stop").run->stop_reason == StopReason::None);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"post-stop"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
}

TEST_CASE("RuntimeControlLoop stop watermark preserves post-stop interrupt reason and work") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto active_path = validTrk(tmp, "active_stop_then_interrupt.trk", 5);
  const auto urgent_path = validTrk(tmp, "post_stop_urgent.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitInterrupt(
              executeCommand("urgent", urgent_path, MotionMode::Interrupt))
              .ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"urgent"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("urgent").run->state == MotionState::Queued);
  REQUIRE(store.findRun("urgent").run->stop_reason == StopReason::None);

  loop.tick();
  requireIdle(store);
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"urgent"});

  startQueuedRun(loop, store, "urgent");
}

TEST_CASE("RuntimeControlLoop interrupt during preparing uses controlled stop",
          "[runtime-p1]") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto active_path = validTrk(tmp, "preparing_interrupt_active.trk", 5);
  const auto urgent_path = validTrk(tmp, "preparing_interrupt_urgent.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("preparing-active", active_path)).ok());
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "preparing-active");
  REQUIRE_FALSE(snapshot.transition.active);

  REQUIRE(bridge.submitInterrupt(
              executeCommand("preparing-urgent",
                             urgent_path,
                             MotionMode::Interrupt,
                             3))
              .ok());
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Interrupt);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "preparing-active");
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"preparing-urgent"});
  REQUIRE(store.findRun("preparing-active").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("preparing-active").run->stop_reason ==
          StopReason::Interrupt);
  REQUIRE(store.findRun("preparing-urgent").run->state == MotionState::Queued);
  REQUIRE(store.findRun("preparing-urgent").run->stop_reason == StopReason::None);
}

TEST_CASE("RuntimeControlLoop interrupt stops active, cancels local waiting, then starts urgent") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  auto loop = makeLoop(config, bridge, store, tmp.trkConfig());

  const auto active_path = validTrk(tmp, "active.trk", 5);
  const auto old_path = validTrk(tmp, "old.trk", 4);
  const auto urgent_path = validTrk(tmp, "urgent.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.submitQueue(executeCommand("old-waiting", old_path)).ok());
  loop.tick();
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"old-waiting"});

  REQUIRE(bridge.submitInterrupt(
              executeCommand("urgent", urgent_path, MotionMode::Interrupt))
              .ok());
  REQUIRE(store.findRun("old-waiting").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-waiting").run->stop_reason == StopReason::Interrupt);

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Interrupt);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"urgent"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopping);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Interrupt);

  loop.tick();
  requireIdle(store);
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"urgent"});

  startQueuedRun(loop, store, "urgent");
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec->frame == 0);
}

}  // namespace agentic_et1_tracker
