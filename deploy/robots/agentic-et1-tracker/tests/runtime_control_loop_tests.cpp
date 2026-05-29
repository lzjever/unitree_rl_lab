#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kPolicyJointDim = TrkSchema::kJointDim;
constexpr std::uint8_t kExpectedModeMachine = 7;

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
  bool invalid_contact{false};
  std::size_t invalid_contact_index{0};
  std::int64_t invalid_contact_value{3};
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
    writeScalar(out, static_cast<float>(i) * 0.25F);
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

ExecuteCommand executeCommand(std::string id,
                              const std::filesystem::path& path,
                              MotionMode mode = MotionMode::Queue,
                              std::size_t frames = 3) {
  ExecuteCommand command;
  command.id = std::move(id);
  command.path = path.string();
  command.mode = mode;
  command.track.frames = frames;
  command.track.duration_s = static_cast<double>(frames) / 50.0;
  command.track.fps = 50.0;
  command.track.canonical_path = path.string();
  return command;
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
                                  DeployConfig deploy_config = deployConfig()) {
  return RuntimeControlLoop(config,
                            bridge,
                            store,
                            TrkLoader(trk_config),
                            robot,
                            policy,
                            std::move(deploy_config),
                            kExpectedModeMachine,
                            RuntimeMode::Real);
}

void requireIdle(const RuntimeStatusStore& store) {
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
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

void startQueuedRun(RuntimeControlLoop& loop,
                    RuntimeStatusStore& store,
                    const std::string& id) {
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == id);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == id);
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

  std::this_thread::sleep_for(std::chrono::milliseconds(45));
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(snapshot.exec->time_s == 0.04);
  REQUIRE(snapshot.exec->progress == 2.0 / 3.0);

  for (int i = 0; i < 20; ++i) {
    if (!store.snapshot().exec.has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
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
    REQUIRE(snapshot.ctrl == ControllerState::Idle);
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

TEST_CASE("RuntimeControlLoop policy idle readiness faults on bad orientation and lowcmd occupancy") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  struct Case {
    const char* id;
    LowStateSample low_state;
    LowCmdOccupancy occupancy;
    ErrorCode err;
    RobotState robot;
    std::string block;
  };

  const std::vector<Case> cases{
      {"idle-bad-orientation",
       badOrientationLowState(deploy_config, 61),
       {},
       ErrorCode::RobotBadOrientation,
       RobotState::Fault,
       "bad_orientation"},
      {"idle-lowcmd-occupied",
       readyLowState(deploy_config, kExpectedModeMachine, true, 62),
       {true, 3},
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
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
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

  loop.tick();
  requireIdle(store);
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(policy.calls == 1);
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
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
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
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
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
  REQUIRE(snapshot.ctrl == ControllerState::Preparing);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "prep-stale");
  REQUIRE(snapshot.queue.ids.empty());

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "prep-stale");
  REQUIRE(snapshot.exec->state == MotionState::Running);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
}

TEST_CASE("RuntimeControlLoop policy start gate faults on bad orientation and lowcmd occupancy") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  struct Case {
    const char* id;
    LowStateSample low_state;
    LowCmdOccupancy occupancy;
    ErrorCode err;
    RobotState robot;
    std::string block;
  };

  const std::vector<Case> cases{
      {"start-bad-orientation",
       badOrientationLowState(deploy_config, 71),
       {},
       ErrorCode::RobotBadOrientation,
       RobotState::Fault,
       "bad_orientation"},
      {"start-lowcmd-occupied",
       readyLowState(deploy_config, kExpectedModeMachine, true, 72),
       {true, 4},
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
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
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

TEST_CASE("RuntimeControlLoop running lowstate readiness failure maps to fault") {
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
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
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

TEST_CASE("RuntimeControlLoop running bad orientation fails active motion and faults") {
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
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
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

TEST_CASE("RuntimeControlLoop policy stop writes hold-current frames for configured ticks") {
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
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(robot.writes.size() == 1);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);
  REQUIRE(policy.calls == 0);

  loop.tick();
  REQUIRE(robot.write_attempts == 2);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);
  REQUIRE(policy.calls == 0);

  loop.tick();
  REQUIRE(robot.write_attempts == 3);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);
  REQUIRE(policy.calls == 0);
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE_FALSE(snapshot.exec.has_value());
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

TEST_CASE("RuntimeControlLoop policy stop hold readiness failures map to fault without writing") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  const DeployConfig deploy_config = deployConfig();

  struct Case {
    const char* id;
    std::optional<LowStateSample> low_state;
    ErrorCode err;
    std::string block;
  };

  const std::vector<Case> cases{
      {"hold-missing", std::nullopt, ErrorCode::RobotDisconnected, "lowstate_missing"},
      {"hold-stale",
       readyLowState(deploy_config, kExpectedModeMachine, false, 111),
       ErrorCode::RobotNotReady,
       "lowstate_timeout"},
      {"hold-mode-mismatch",
       readyLowState(deploy_config, kExpectedModeMachine + 1, true, 112),
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
    const auto path = validTrk(tmp, std::string(item.id) + ".trk", 3);
    REQUIRE(bridge.submitQueue(executeCommand(item.id, path)).ok());
    startQueuedRun(loop, store, item.id);

    REQUIRE(bridge.stop().state == ControllerState::Stopping);
    loop.tick();
    REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
    robot.low_state = item.low_state;

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Fault);
    REQUIRE_FALSE(snapshot.ready);
    REQUIRE(snapshot.err == item.err);
    REQUIRE(snapshot.block == item.block);
    REQUIRE(policy.calls == 0);
    REQUIRE(robot.write_attempts == 0);
    const auto found = store.findRun(item.id);
    REQUIRE(found.ok());
    REQUIRE(found.run->state == MotionState::Failed);
    REQUIRE(found.run->err == item.err);

    loop.tick();
    REQUIRE(robot.write_attempts == 0);
  }
}

TEST_CASE("RuntimeControlLoop policy stop hold bad orientation maps to fault without writing") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "hold_bad_orientation.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("hold-bad-orientation", path)).ok());
  startQueuedRun(loop, store, "hold-bad-orientation");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  robot.low_state = badOrientationLowState(deploy_config, 91);

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::Fault);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.low_ms == 91);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
  const auto found = store.findRun("hold-bad-orientation");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::RobotBadOrientation);
}

TEST_CASE("RuntimeControlLoop policy stop hold lowcmd occupancy maps to fault without writing") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "hold_lowcmd_occupied.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("hold-occupied", path)).ok());
  startQueuedRun(loop, store, "hold-occupied");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  REQUIRE(robot.write_attempts == 0);

  robot.occupancy = {true, 3};
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.robot == RobotState::NotReady);
  REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
  REQUIRE(snapshot.block == "lowcmd_occupied");
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.occupancy_calls == 3);
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(robot.writes.empty());
  const auto found = store.findRun("hold-occupied");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::RobotNotReady);

  loop.tick();
  REQUIRE(robot.occupancy_calls == 3);
  REQUIRE(robot.write_attempts == 0);
}

TEST_CASE("RuntimeControlLoop policy stop hold write failure maps to fault block") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto path = validTrk(tmp, "hold_write_fail.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("hold-write-fail", path)).ok());
  startQueuedRun(loop, store, "hold-write-fail");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  robot.throw_on_write = true;

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::InternalError);
  REQUIRE(snapshot.block == "lowcmd_write_failed");
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(robot.writes.empty());
  const auto found = store.findRun("hold-write-fail");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::InternalError);

  loop.tick();
  REQUIRE(robot.write_attempts == 1);
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

TEST_CASE("RuntimeControlLoop policy stop hold preserves post-stop queue until hold completes") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto active_path = validTrk(tmp, "policy_hold_active.trk", 5);
  const auto queued_path = validTrk(tmp, "policy_hold_post_stop_queue.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);

  loop.tick();
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);

  REQUIRE(bridge.submitQueue(executeCommand("post-stop", queued_path)).ok());
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"post-stop"});

  loop.tick();
  REQUIRE(robot.write_attempts == 2);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"post-stop"});

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(robot.write_attempts == 3);
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"post-stop"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("post-stop").run->state == MotionState::Queued);

  startQueuedRun(loop, store, "post-stop");
  snapshot = store.snapshot();
  REQUIRE(snapshot.queue.ids.empty());
}

TEST_CASE("RuntimeControlLoop policy stop hold repeated stop is idempotent") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto active_path = validTrk(tmp, "policy_hold_repeated_stop.trk", 5);
  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  REQUIRE(store.snapshot().stop_reason == StopReason::Stop);

  loop.tick();
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(robot.write_attempts == 2);
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  REQUIRE(store.snapshot().stop_reason == StopReason::Stop);

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(robot.write_attempts == 3);
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.stop_reason == StopReason::None);
  REQUIRE_FALSE(snapshot.exec.has_value());

  loop.tick();
  REQUIRE(robot.write_attempts == 4);
  requireIdle(store);
}

TEST_CASE("RuntimeControlLoop policy stop hold preserves post-stop interrupt until hold completes") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);

  const auto active_path = validTrk(tmp, "policy_hold_interrupt_active.trk", 5);
  const auto urgent_path = validTrk(tmp, "policy_hold_urgent.trk", 3);

  REQUIRE(bridge.submitQueue(executeCommand("active", active_path)).ok());
  startQueuedRun(loop, store, "active");

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);

  loop.tick();
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);

  REQUIRE(bridge.submitInterrupt(
              executeCommand("urgent", urgent_path, MotionMode::Interrupt))
              .ok());
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"urgent"});

  loop.tick();
  REQUIRE(robot.write_attempts == 2);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"urgent"});

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(robot.write_attempts == 3);
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"urgent"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("urgent").run->state == MotionState::Queued);
  REQUIRE(store.findRun("urgent").run->stop_reason == StopReason::None);

  startQueuedRun(loop, store, "urgent");
  snapshot = store.snapshot();
  REQUIRE(snapshot.queue.ids.empty());
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
