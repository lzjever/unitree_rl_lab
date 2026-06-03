#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/policy/policy_step_runner.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr std::size_t kJointDim = TrkSchema::kJointDim;
constexpr std::size_t kBodyCount = TrkSchema::kBodyCount;
constexpr std::size_t kHistoryLength = 25;
constexpr std::size_t kHistoryWidth = 105;
constexpr std::size_t kClnHistoryWidth = 35;
constexpr std::size_t kObsCurrentLastActionOffset = 93;
constexpr std::size_t kClnObsCurrentLastActionOffset = 95;
constexpr std::size_t kObsCurrentCommandJointOffset = 9;
constexpr std::size_t kObsHistoryCommandJointOffset = 9;
constexpr std::size_t kClnFutureCommandJointOffset = 9;
constexpr float kPi = 3.14159265358979323846F;

std::array<float, 4> yawQuat(float radians) {
  return {std::cos(radians * 0.5F), 0.0F, 0.0F, std::sin(radians * 0.5F)};
}

Vec seq(float start, std::size_t count) {
  Vec values;
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

std::vector<ObservationTerm> terms(
    std::initializer_list<std::pair<const char*, std::size_t>> specs) {
  std::vector<ObservationTerm> out;
  std::size_t offset = 0;
  for (const auto& spec : specs) {
    out.push_back({spec.first, spec.second, offset});
    offset += spec.second;
  }
  return out;
}

DeployConfig validConfig() {
  DeployConfig config;
  config.joint_dim = kJointDim;
  config.sdk_joint_ids_map = frozenSdkMap();
  config.default_joint_pos = doubleSeq(0.25, 0.5, kJointDim);
  config.policy_kp = doubleSeq(10.0, 1.0, kJointDim);
  config.policy_kd = doubleSeq(0.5, 0.05, kJointDim);
  config.action_scale = doubleSeq(0.25, 0.125, kJointDim);
  config.action_offset = doubleSeq(-1.0, 0.2, kJointDim);
  config.obs_current_terms = terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_history_terms = terms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_current_dim = 131;
  config.obs_history_width = kHistoryWidth;
  config.obs_history_length = kHistoryLength;
  return config;
}

DeployConfig validClnConfig() {
  DeployConfig config = validConfig();
  config.observation_contract = ObservationContract::GeneralTrackerCLN;
  config.obs_current_terms = terms({
      {"command_yaw", 2},
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kJointDim},
      {"joint_vel_rel", kJointDim},
      {"last_action", kJointDim},
  });
  config.obs_history_terms = terms({
      {"future_commands", kClnHistoryWidth},
  });
  config.obs_current_dim = 121;
  config.obs_history_width = kClnHistoryWidth;
  config.obs_history_length = kHistoryLength;
  return config;
}

void resizeFloatArray(TrkFloatArray& array,
                      std::size_t frames,
                      std::size_t frame_size) {
  array.frame_size = frame_size;
  array.values.assign(frames * frame_size, 0.0F);
}

void resizeContactArray(TrkContactArray& array,
                        std::size_t frames,
                        std::size_t frame_size) {
  array.frame_size = frame_size;
  array.values.assign(frames * frame_size, 0);
}

float frameJointBase(std::size_t frame_index) {
  return 100.0F + 100.0F * static_cast<float>(frame_index);
}

void fillFrame(TrkTrack& track, std::size_t frame_index) {
  const std::size_t joint_offset = frame_index * track.joint_pos.frame_size;
  const std::size_t body_quat_offset = frame_index * track.body_quat_w.frame_size;
  const std::size_t body_lin_offset = frame_index * track.body_lin_vel_w.frame_size;
  const std::size_t body_ang_offset = frame_index * track.body_ang_vel_w.frame_size;
  const std::size_t ref_com_offset = frame_index * track.ref_com_rel_navi.frame_size;
  const float base = frameJointBase(frame_index);

  for (std::size_t i = 0; i < kJointDim; ++i) {
    track.joint_pos.values.at(joint_offset + i) = base + static_cast<float>(i);
    track.joint_vel.values.at(joint_offset + i) = -base - static_cast<float>(i);
  }

  for (std::size_t body = 0; body < kBodyCount; ++body) {
    track.body_quat_w.values.at(body_quat_offset + body * 4) = 1.0F;
  }

  track.body_lin_vel_w.values.at(body_lin_offset) = 1.0F + static_cast<float>(frame_index);
  track.body_lin_vel_w.values.at(body_lin_offset + 1) =
      -2.0F - static_cast<float>(frame_index);
  track.body_ang_vel_w.values.at(body_ang_offset + 2) =
      0.25F + static_cast<float>(frame_index);

  track.left_foot_contact_state.values.at(frame_index) = 1;
  track.right_foot_contact_state.values.at(frame_index) = 2;

  track.ref_com_rel_navi.values.at(ref_com_offset) = 0.1F + static_cast<float>(frame_index);
  track.ref_com_rel_navi.values.at(ref_com_offset + 1) =
      0.2F + static_cast<float>(frame_index);
  track.ref_com_rel_navi.values.at(ref_com_offset + 2) =
      0.3F + static_cast<float>(frame_index);
  track.ref_com_vel_navi.values.at(ref_com_offset) = -0.1F - static_cast<float>(frame_index);
  track.ref_com_vel_navi.values.at(ref_com_offset + 1) =
      -0.2F - static_cast<float>(frame_index);
  track.ref_com_vel_navi.values.at(ref_com_offset + 2) =
      -0.3F - static_cast<float>(frame_index);
}

TrkTrack makeTrack(std::size_t frames) {
  TrkTrack track;
  track.metadata.frames = frames;
  track.metadata.fps = TrkSchema::kDefaultFps;
  resizeFloatArray(track.joint_pos, frames, kJointDim);
  resizeFloatArray(track.joint_vel, frames, kJointDim);
  resizeFloatArray(track.body_pos_w, frames, kBodyCount * 3);
  resizeFloatArray(track.body_quat_w, frames, kBodyCount * 4);
  resizeFloatArray(track.body_lin_vel_w, frames, kBodyCount * 3);
  resizeFloatArray(track.body_ang_vel_w, frames, kBodyCount * 3);
  resizeContactArray(track.left_foot_contact_state, frames, 1);
  resizeContactArray(track.right_foot_contact_state, frames, 1);
  resizeFloatArray(track.ref_com_rel_navi, frames, 3);
  resizeFloatArray(track.ref_com_vel_navi, frames, 3);

  for (std::size_t frame = 0; frame < frames; ++frame) {
    fillFrame(track, frame);
  }
  return track;
}

void setRootQuat(TrkTrack& track,
                 std::size_t frame_index,
                 const std::array<float, 4>& quat_wxyz) {
  const std::size_t offset = frame_index * track.body_quat_w.frame_size;
  for (std::size_t i = 0; i < quat_wxyz.size(); ++i) {
    track.body_quat_w.values.at(offset + i) = quat_wxyz[i];
  }
}

void setRootLinVel(TrkTrack& track, std::size_t frame_index, float x, float y, float z) {
  const std::size_t offset = frame_index * track.body_lin_vel_w.frame_size;
  track.body_lin_vel_w.values.at(offset) = x;
  track.body_lin_vel_w.values.at(offset + 1) = y;
  track.body_lin_vel_w.values.at(offset + 2) = z;
}

void setRootAngVel(TrkTrack& track, std::size_t frame_index, float x, float y, float z) {
  const std::size_t offset = frame_index * track.body_ang_vel_w.frame_size;
  track.body_ang_vel_w.values.at(offset) = x;
  track.body_ang_vel_w.values.at(offset + 1) = y;
  track.body_ang_vel_w.values.at(offset + 2) = z;
}

LowStateSample liveState(const DeployConfig& config) {
  LowStateSample low;
  low.fresh = true;
  low.mode_machine = 7;
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

class RecordingPolicy final : public PolicyInference {
 public:
  explicit RecordingPolicy(Vec raw) : next_raw(std::move(raw)) {}

  Vec infer(const PolicyInputs& inputs) override {
    calls.push_back(inputs);
    return next_raw;
  }

  Vec next_raw;
  std::vector<PolicyInputs> calls;
};

void requireSliceApprox(const Vec& actual,
                        std::size_t offset,
                        const Vec& expected,
                        float margin = 1.0e-5F) {
  REQUIRE(offset + expected.size() <= actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    REQUIRE(actual[offset + i] == Catch::Approx(expected[i]).margin(margin));
  }
}

void requireZeroSlice(const Vec& actual, std::size_t offset, std::size_t count) {
  requireSliceApprox(actual, offset, Vec(count, 0.0F));
}

void requireHistoryJointRow(const Vec& history,
                            std::size_t row,
                            float joint_base) {
  requireSliceApprox(history,
                     row * kHistoryWidth + kObsHistoryCommandJointOffset,
                     seq(joint_base, kJointDim));
}

void requireClnFutureJointRow(const Vec& history,
                              std::size_t row,
                              float joint_base) {
  requireSliceApprox(history,
                     row * kClnHistoryWidth + kClnFutureCommandJointOffset,
                     seq(joint_base, kJointDim));
}

template <typename Fn>
void requirePolicyStepThrowsWith(Fn&& fn, const std::string& text) {
  try {
    fn();
    FAIL("expected PolicyStepError");
  } catch (const PolicyStepError& err) {
    REQUIRE_THAT(err.what(), ContainsSubstring(text));
  }
}

}  // namespace

TEST_CASE("PolicyStepRunner first step uses zero last action and emits scaled LowCmd") {
  const DeployConfig config = validConfig();
  const TrkTrack track = makeTrack(2);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  const Vec raw = seq(0.25F, kJointDim);
  RecordingPolicy policy(raw);

  const PolicyStepResult result = runner.step(0, low, policy);

  REQUIRE(policy.calls.size() == 1);
  REQUIRE(result.frame == 0);
  requireZeroSlice(result.inputs.obs_current, kObsCurrentLastActionOffset, kJointDim);
  requireZeroSlice(policy.calls.at(0).obs_current, kObsCurrentLastActionOffset,
                   kJointDim);

  for (const std::size_t policy_joint : {std::size_t{0}, std::size_t{14},
                                         std::size_t{25}}) {
    const float expected_q =
        raw[policy_joint] * static_cast<float>(config.action_scale[policy_joint]) +
        static_cast<float>(config.action_offset[policy_joint]);
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map[policy_joint]);
    REQUIRE(result.output.target_q.at(policy_joint) ==
            Catch::Approx(expected_q).margin(1.0e-5F));
    REQUIRE(result.low_cmd.motors.at(sdk_slot).q ==
            Catch::Approx(expected_q).margin(1.0e-5F));
    REQUIRE(result.low_cmd.motors.at(sdk_slot).kp ==
            Catch::Approx(static_cast<float>(config.policy_kp[policy_joint])));
    REQUIRE(result.low_cmd.motors.at(sdk_slot).kd ==
            Catch::Approx(static_cast<float>(config.policy_kd[policy_joint])));
  }
  REQUIRE(result.low_cmd.mode_machine == 7);
}

TEST_CASE("PolicyStepRunner overlays active track LowCmd on a base frame") {
  const DeployConfig config = validConfig();
  const TrkTrack track = makeTrack(1);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  RecordingPolicy policy(seq(0.1F, kJointDim));

  LowCmdFrame base;
  base.mode_machine = 3;
  base.motors.at(14).mode = 6;
  base.motors.at(14).q = 42.0F;
  base.motors.at(14).dq = -2.0F;
  base.motors.at(14).kp = 12.0F;
  base.motors.at(14).kd = 1.25F;
  base.motors.at(14).tau = 0.75F;

  const PolicyStepResult result = runner.step(0, low, policy, &base);

  REQUIRE(result.low_cmd.mode_machine == 7);
  REQUIRE(result.low_cmd.motors.at(14).mode == base.motors.at(14).mode);
  REQUIRE(result.low_cmd.motors.at(14).q == base.motors.at(14).q);
  REQUIRE(result.low_cmd.motors.at(14).dq == base.motors.at(14).dq);
  REQUIRE(result.low_cmd.motors.at(14).kp == base.motors.at(14).kp);
  REQUIRE(result.low_cmd.motors.at(14).kd == base.motors.at(14).kd);
  REQUIRE(result.low_cmd.motors.at(14).tau == base.motors.at(14).tau);

  const auto mapped_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(0));
  REQUIRE(result.low_cmd.motors.at(mapped_slot).q != base.motors.at(mapped_slot).q);
  REQUIRE(result.low_cmd.motors.at(mapped_slot).kp ==
          Catch::Approx(static_cast<float>(config.policy_kp.at(0))));
}

TEST_CASE("PolicyStepRunner second step feeds prior action and oldest-to-newest history") {
  const DeployConfig config = validConfig();
  const TrkTrack track = makeTrack(2);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  const Vec raw0 = seq(-0.5F, kJointDim);
  const Vec raw1 = seq(0.75F, kJointDim);
  RecordingPolicy policy(raw0);

  runner.step(0, low, policy);
  policy.next_raw = raw1;
  const PolicyStepResult second = runner.step(1, low, policy);

  REQUIRE(policy.calls.size() == 2);
  requireSliceApprox(second.inputs.obs_current, kObsCurrentLastActionOffset, raw0);
  requireSliceApprox(policy.calls.at(1).obs_current, kObsCurrentLastActionOffset, raw0);
  requireSliceApprox(second.inputs.obs_current, kObsCurrentCommandJointOffset,
                     seq(frameJointBase(1), kJointDim));
  requireHistoryJointRow(second.inputs.obs_history, 0, frameJointBase(0));
  requireHistoryJointRow(second.inputs.obs_history, kHistoryLength - 1,
                         frameJointBase(1));
}

TEST_CASE("PolicyStepRunner builds GeneralTrackerCLN future_commands from .trk frames") {
  const DeployConfig config = validClnConfig();
  const TrkTrack track = makeTrack(3);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  RecordingPolicy policy(Vec(kJointDim, 0.0F));

  const PolicyStepResult result = runner.step(1, low, policy);

  REQUIRE(policy.calls.size() == 1);
  REQUIRE(result.inputs.obs_current.size() == 121);
  REQUIRE(result.inputs.obs_history.size() == kHistoryLength * kClnHistoryWidth);
  requireZeroSlice(result.inputs.obs_current, kClnObsCurrentLastActionOffset, kJointDim);
  requireClnFutureJointRow(result.inputs.obs_history, 0, frameJointBase(2));
  requireClnFutureJointRow(result.inputs.obs_history, 1, frameJointBase(2));
  requireClnFutureJointRow(result.inputs.obs_history, kHistoryLength - 1,
                           frameJointBase(2));
}

TEST_CASE("PolicyStepRunner keeps CLN future_commands root and velocity golden order") {
  const DeployConfig config = validClnConfig();
  TrkTrack track = makeTrack(3);
  setRootQuat(track, 2, yawQuat(kPi * 0.5F));
  setRootLinVel(track, 2, 2.0F, -3.0F, 9.0F);
  setRootAngVel(track, 2, 0.0F, 0.0F, 0.75F);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  RecordingPolicy policy(Vec(kJointDim, 0.0F));

  const PolicyStepResult result = runner.step(1, low, policy);

  REQUIRE(result.inputs.obs_history.size() == kHistoryLength * kClnHistoryWidth);
  requireSliceApprox(result.inputs.obs_history,
                     0,
                     {0.0F, -1.0F, 1.0F, 0.0F, 0.0F, 0.0F, -3.0F, -2.0F, 0.75F},
                     1.0e-5F);
}

TEST_CASE("PolicyStepRunner owns a moved track across ticks") {
  const DeployConfig config = validConfig();
  TrkTrack track = makeTrack(2);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, std::move(track), low, 7);
  track.metadata.frames = 0;
  track.joint_pos.values.clear();
  track.body_quat_w.values.clear();

  RecordingPolicy policy(seq(0.2F, kJointDim));
  const PolicyStepResult first = runner.step(0, low, policy);
  policy.next_raw = seq(0.4F, kJointDim);
  const PolicyStepResult second = runner.step(1, low, policy);

  REQUIRE(first.frame == 0);
  REQUIRE(second.frame == 1);
  requireSliceApprox(second.inputs.obs_current, kObsCurrentCommandJointOffset,
                     seq(frameJointBase(1), kJointDim));
  requireHistoryJointRow(second.inputs.obs_history, 0, frameJointBase(0));
  requireHistoryJointRow(second.inputs.obs_history, kHistoryLength - 1,
                         frameJointBase(1));
}

TEST_CASE("PolicyStepRunner reports invalid frame and missing first frame as step errors") {
  const DeployConfig config = validConfig();
  const TrkTrack track = makeTrack(1);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  RecordingPolicy policy(seq(0.0F, kJointDim));

  requirePolicyStepThrowsWith([&] { runner.step(1, low, policy); }, "frame index");
  REQUIRE(policy.calls.empty());

  const TrkTrack empty_track = makeTrack(0);
  requirePolicyStepThrowsWith([&] { PolicyStepRunner bad(config, empty_track, low, 7); },
                              "first frame");
}

TEST_CASE("PolicyStepRunner wraps bad policy action and builder failures") {
  const DeployConfig config = validConfig();
  LowStateSample low = liveState(config);

  SECTION("raw action length") {
    const TrkTrack track = makeTrack(1);
    PolicyStepRunner runner(config, track, low, 7);
    RecordingPolicy policy(seq(0.0F, kJointDim - 1));

    requirePolicyStepThrowsWith([&] { runner.step(0, low, policy); }, "raw_action");
  }

  SECTION("invalid contact") {
    TrkTrack track = makeTrack(1);
    track.left_foot_contact_state.values.at(0) = 3;
    PolicyStepRunner runner(config, track, low, 7);
    RecordingPolicy policy(seq(0.0F, kJointDim));

    requirePolicyStepThrowsWith([&] { runner.step(0, low, policy); },
                                "left_foot_contact_state");
  }
}

TEST_CASE("PolicyStepRunner reset clears last action and starts a fresh history window") {
  const DeployConfig config = validConfig();
  const TrkTrack track = makeTrack(2);
  const LowStateSample low = liveState(config);
  PolicyStepRunner runner(config, track, low, 7);
  RecordingPolicy policy(seq(0.1F, kJointDim));

  runner.step(0, low, policy);
  policy.next_raw = seq(1.1F, kJointDim);
  runner.step(1, low, policy);

  runner.reset(low);
  policy.calls.clear();
  policy.next_raw = seq(2.1F, kJointDim);
  const PolicyStepResult after_reset = runner.step(1, low, policy);

  REQUIRE(policy.calls.size() == 1);
  requireZeroSlice(after_reset.inputs.obs_current, kObsCurrentLastActionOffset,
                   kJointDim);
  requireHistoryJointRow(after_reset.inputs.obs_history, 0, frameJointBase(1));
  requireHistoryJointRow(after_reset.inputs.obs_history, kHistoryLength - 1,
                         frameJointBase(1));
}

}  // namespace agentic_et1_tracker
