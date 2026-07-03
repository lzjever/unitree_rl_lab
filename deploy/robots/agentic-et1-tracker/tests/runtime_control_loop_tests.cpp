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
#include "agentic_et1_tracker/loco_upper/loco_lower_policy.hpp"
#include "agentic_et1_tracker/loco_upper/lowcmd_composer.hpp"
#include "agentic_et1_tracker/policy/policy_math.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"
#include "agentic_et1_tracker/policy/velocity_policy_runner.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/synthetic_transition.hpp"
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
  std::vector<std::array<float, 3>> root_body_positions;
  std::optional<std::array<float, 4>> root_body_quat;
  std::vector<std::array<float, 4>> root_body_quats;
  std::vector<Vec> joint_pos_frames;
  std::optional<float> fill_value;
  std::optional<std::int64_t> contact_fill_value;
  std::optional<float> manual_gate_variant;
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
      auto value = array.contact_fill_value.value_or(static_cast<std::int64_t>(i % 3));
      if (array.manual_gate_variant) {
        value =
            (i + static_cast<int>(*array.manual_gate_variant * 10.0F)) % 2 == 0
                ? 1
                : 2;
      }
      if (array.invalid_contact && i == array.invalid_contact_index) {
        value = array.invalid_contact_value;
      }
      writeScalar(out, value);
    }
    return;
  }

  for (std::size_t i = 0; i < elements; ++i) {
    float value = array.zero_first_quaternion && i < 4
                      ? 0.0F
                      : static_cast<float>(i) * 0.25F;
    if (array.manual_gate_variant) {
      value = *array.manual_gate_variant +
              static_cast<float>(i % 97) * 0.001F;
    }
    if (array.fill_value) {
      value = *array.fill_value;
    }
    if (array.identity_quaternions && array.name == "body_quat_w") {
      value = i % 4 == 0 ? 1.0F : 0.0F;
    }
    if (array.name == "body_pos_w") {
      if (!array.root_body_positions.empty() && array.shape.size() >= 2) {
        std::uint64_t frame_size = 1;
        for (std::size_t dim = 1; dim < array.shape.size(); ++dim) {
          frame_size *= array.shape.at(dim);
        }
        const std::uint64_t frame = frame_size == 0 ? 0 : i / frame_size;
        const std::uint64_t frame_offset = frame_size == 0 ? i : i % frame_size;
        if (frame < array.root_body_positions.size() && frame_offset < 3) {
          value = array.root_body_positions.at(static_cast<std::size_t>(frame))
                      .at(static_cast<std::size_t>(frame_offset));
        }
      } else if (array.root_body_position && i < 3) {
        value = array.root_body_position->at(i);
      }
    }
    if (array.name == "body_quat_w") {
      if (!array.root_body_quats.empty() && array.shape.size() >= 2) {
        std::uint64_t frame_size = 1;
        for (std::size_t dim = 1; dim < array.shape.size(); ++dim) {
          frame_size *= array.shape.at(dim);
        }
        const std::uint64_t frame = frame_size == 0 ? 0 : i / frame_size;
        const std::uint64_t frame_offset = frame_size == 0 ? i : i % frame_size;
        if (frame < array.root_body_quats.size() && frame_offset < 4) {
          value = array.root_body_quats.at(static_cast<std::size_t>(frame))
                      .at(static_cast<std::size_t>(frame_offset));
        }
      } else if (array.root_body_quat && i < 4) {
        value = array.root_body_quat->at(i);
      }
    }
    if (array.name == "joint_pos" && !array.joint_pos_frames.empty() &&
        array.shape.size() >= 2) {
      std::uint64_t frame_size = 1;
      for (std::size_t dim = 1; dim < array.shape.size(); ++dim) {
        frame_size *= array.shape.at(dim);
      }
      const std::uint64_t frame = frame_size == 0 ? 0 : i / frame_size;
      const std::uint64_t frame_offset = frame_size == 0 ? i : i % frame_size;
      if (frame < array.joint_pos_frames.size() &&
          frame_offset < array.joint_pos_frames.at(static_cast<std::size_t>(frame)).size()) {
        value = array.joint_pos_frames.at(static_cast<std::size_t>(frame))
                    .at(static_cast<std::size_t>(frame_offset));
      }
    }
    writeScalar(out, value);
  }
}

std::vector<ArrayFixture> requiredArrays(std::uint64_t frames) {
  std::vector<ArrayFixture> arrays;
  for (const auto& spec : TrkSchema::kRequiredArrays) {
    ArrayFixture fixture{std::string(spec.name),
                         spec.dtype_family == TrkDtypeFamily::Contact ? TrkDtype::Int64
                                                                       : TrkDtype::Float32,
                         shapeFor(spec, frames)};
    if (fixture.name == "left_foot_contact_state") {
      fixture.contact_fill_value = 1;
    } else if (fixture.name == "right_foot_contact_state") {
      fixture.contact_fill_value = 2;
    }
    arrays.push_back(std::move(fixture));
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

std::filesystem::path constantIdentityTrk(TempTree& tmp,
                                          const std::string& name,
                                          std::uint64_t frames,
                                          float joint_velocity = 0.0F) {
  auto arrays = requiredArrays(frames);
  for (auto& array : arrays) {
    if (array.dtype == TrkDtype::Float32) {
      array.fill_value = 0.0F;
    }
  }
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "joint_vel").fill_value = joint_velocity;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path transitionReadyTrk(TempTree& tmp,
                                         const std::string& name,
                                         std::uint64_t frames,
                                         float start_offset = 0.1F) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "body_pos_w").root_body_positions =
      std::vector<std::array<float, 3>>(frames, {0.0F, 0.0F, 0.0F});
  auto& joint_pos = arrayNamed(arrays, "joint_pos");
  joint_pos.joint_pos_frames.reserve(frames);
  for (std::uint64_t frame = 0; frame < frames; ++frame) {
    joint_pos.joint_pos_frames.push_back(
        Vec(kPolicyJointDim, start_offset + 0.1F * static_cast<float>(frame)));
  }
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path transitionReadyContactTrk(TempTree& tmp,
                                                const std::string& name,
                                                std::uint64_t frames,
                                                std::int64_t left_contact,
                                                std::int64_t right_contact,
                                                float start_offset = 0.1F) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "body_pos_w").root_body_positions =
      std::vector<std::array<float, 3>>(frames, {0.0F, 0.0F, 0.0F});
  arrayNamed(arrays, "left_foot_contact_state").contact_fill_value = left_contact;
  arrayNamed(arrays, "right_foot_contact_state").contact_fill_value = right_contact;
  auto& joint_pos = arrayNamed(arrays, "joint_pos");
  joint_pos.joint_pos_frames.reserve(frames);
  for (std::uint64_t frame = 0; frame < frames; ++frame) {
    joint_pos.joint_pos_frames.push_back(
        Vec(kPolicyJointDim, start_offset + 0.1F * static_cast<float>(frame)));
  }
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path transitionReadyContactZeroVelTrk(
    TempTree& tmp,
    const std::string& name,
    std::uint64_t frames,
    std::int64_t left_contact,
    std::int64_t right_contact,
    float start_offset = 0.1F) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "body_pos_w").root_body_positions =
      std::vector<std::array<float, 3>>(frames, {0.0F, 0.0F, 0.0F});
  arrayNamed(arrays, "joint_vel").fill_value = 0.0F;
  arrayNamed(arrays, "body_lin_vel_w").fill_value = 0.0F;
  arrayNamed(arrays, "body_ang_vel_w").fill_value = 0.0F;
  arrayNamed(arrays, "ref_com_vel_navi").fill_value = 0.0F;
  arrayNamed(arrays, "left_foot_contact_state").contact_fill_value = left_contact;
  arrayNamed(arrays, "right_foot_contact_state").contact_fill_value = right_contact;
  auto& joint_pos = arrayNamed(arrays, "joint_pos");
  joint_pos.joint_pos_frames.reserve(frames);
  for (std::uint64_t frame = 0; frame < frames; ++frame) {
    joint_pos.joint_pos_frames.push_back(
        Vec(kPolicyJointDim, start_offset + 0.01F * static_cast<float>(frame)));
  }
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path manualGateTrk(TempTree& tmp,
                                    const std::string& name,
                                    std::uint64_t frames,
                                    float variant) {
  auto arrays = requiredArrays(frames);
  for (auto& array : arrays) {
    if (array.name == "body_quat_w") {
      array.identity_quaternions = true;
    } else {
      array.manual_gate_variant = variant;
    }
  }
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path manualGateStandbyReferenceTrk(TempTree& tmp,
                                                    const std::string& name) {
  auto arrays = requiredArrays(25);
  for (auto& array : arrays) {
    if (array.dtype == TrkDtype::Float32) {
      array.fill_value = 0.0F;
    }
  }
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "left_foot_contact_state").contact_fill_value = 0;
  arrayNamed(arrays, "right_foot_contact_state").contact_fill_value = 0;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

Vec existingTurnAroundFirstJointPos() {
  return {-0.532F, -0.418F, -0.306F, -0.194F, -0.082F, 0.030F, 0.142F,
          0.254F,  0.366F,  0.478F,  0.590F,  0.702F, 0.814F, 0.926F,
          1.038F,  1.150F,  1.197F,  1.086F,  0.974F, 0.862F, 0.750F,
          0.638F,  0.526F,  0.414F,  0.302F,  0.190F};
}

std::filesystem::path existingTurnAroundZeroContactLikeTrk(TempTree& tmp,
                                                           const std::string& name) {
  constexpr std::uint64_t kFrames = 149;
  auto arrays = requiredArrays(kFrames);
  arrayNamed(arrays, "joint_vel").fill_value = 0.0F;
  arrayNamed(arrays, "body_lin_vel_w").fill_value = 0.0F;
  arrayNamed(arrays, "body_ang_vel_w").fill_value = 0.0F;
  arrayNamed(arrays, "ref_com_vel_navi").fill_value = 0.0F;
  arrayNamed(arrays, "left_foot_contact_state").contact_fill_value = 0;
  arrayNamed(arrays, "right_foot_contact_state").contact_fill_value = 0;

  auto& joint_pos = arrayNamed(arrays, "joint_pos");
  const Vec first_joints = existingTurnAroundFirstJointPos();
  joint_pos.joint_pos_frames.reserve(kFrames);
  for (std::uint64_t frame = 0; frame < kFrames; ++frame) {
    Vec values = first_joints;
    const float phase = static_cast<float>(frame) /
                        static_cast<float>(kFrames - 1);
    for (std::size_t i = 0; i < values.size(); ++i) {
      const int centered = static_cast<int>(i % 5) - 2;
      values.at(i) += 0.02F * phase * static_cast<float>(centered);
    }
    joint_pos.joint_pos_frames.push_back(std::move(values));
  }

  auto& body_pos = arrayNamed(arrays, "body_pos_w");
  body_pos.root_body_positions.reserve(kFrames);
  for (std::uint64_t frame = 0; frame < kFrames; ++frame) {
    const float t = static_cast<float>(frame) / static_cast<float>(kFrames - 1);
    body_pos.root_body_positions.push_back(
        {0.0025F + (-0.0713F - 0.0025F) * t,
         -0.0312F + (-0.0780F + 0.0312F) * t,
         0.7450F + (0.7444F - 0.7450F) * t});
  }

  auto& body_quat = arrayNamed(arrays, "body_quat_w");
  body_quat.identity_quaternions = true;
  body_quat.root_body_quats.reserve(kFrames);
  for (std::uint64_t frame = 0; frame < kFrames; ++frame) {
    const float t = static_cast<float>(frame) / static_cast<float>(kFrames - 1);
    const float yaw = -0.0064F + (3.0183F + 0.0064F) * t;
    body_quat.root_body_quats.push_back(
        {static_cast<float>(std::cos(0.5F * yaw)),
         0.0F,
         0.0F,
         static_cast<float>(std::sin(0.5F * yaw))});
  }

  auto& com = arrayNamed(arrays, "ref_com_rel_navi");
  com.fill_value = 0.0F;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

void setLowStateToManualGateFirstFrame(LowStateSample& low,
                                       const DeployConfig& config,
                                       float variant) {
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low.gyro = {0.0F, 0.0F, 0.0F};
  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const int sdk_slot_raw = config.sdk_joint_ids_map.at(policy_index);
    REQUIRE(sdk_slot_raw >= 0);
    auto& motor = low.motors.at(static_cast<std::size_t>(sdk_slot_raw));
    const float value =
        variant + static_cast<float>(policy_index % 97) * 0.001F;
    motor.q = value;
    motor.dq = value;
  }
}

void setLowStateNearExistingTurnAroundFirstFrame(LowStateSample& low,
                                                 const DeployConfig& config) {
  low.quat_wxyz = {0.9982F, -0.0040F, -0.0595F, -0.0034F};
  low.gyro = {-0.030F, 0.022F, -0.079F};
  const Vec first_joints = existingTurnAroundFirstJointPos();
  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const int sdk_slot_raw = config.sdk_joint_ids_map.at(policy_index);
    REQUIRE(sdk_slot_raw >= 0);
    auto& motor = low.motors.at(static_cast<std::size_t>(sdk_slot_raw));
    motor.q = first_joints.at(policy_index) + 0.05F;
    motor.dq = 0.0F;
  }
}

std::filesystem::path rootPositionsTrk(
    TempTree& tmp,
    const std::string& name,
    const std::vector<std::array<float, 3>>& root_positions) {
  auto arrays = requiredArrays(root_positions.size());
  arrayNamed(arrays, "body_pos_w").root_body_positions = root_positions;
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path yawOnlyTrk(TempTree& tmp,
                                 const std::string& name,
                                 const std::vector<float>& yaws_rad) {
  auto arrays = requiredArrays(yaws_rad.size());
  arrayNamed(arrays, "body_pos_w").root_body_positions =
      std::vector<std::array<float, 3>>(yaws_rad.size(), {0.0F, 0.0F, 0.0F});
  auto& quat = arrayNamed(arrays, "body_quat_w");
  quat.root_body_quats.reserve(yaws_rad.size());
  for (const float yaw_rad : yaws_rad) {
    quat.root_body_quats.push_back(yawQuat(yaw_rad));
  }
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path jointPosTrk(TempTree& tmp,
                                  const std::string& name,
                                  const std::vector<Vec>& joint_pos_frames) {
  auto arrays = requiredArrays(joint_pos_frames.size());
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "joint_pos").joint_pos_frames = joint_pos_frames;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path jointVelocityTrk(TempTree& tmp,
                                       const std::string& name,
                                       std::uint64_t frames,
                                       float joint_velocity) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "joint_vel").fill_value = joint_velocity;
  const auto path = tmp.allowed / name;
  writeTrk(path, arrays);
  return path;
}

std::filesystem::path bodyAngularVelocityTrk(TempTree& tmp,
                                             const std::string& name,
                                             std::uint64_t frames,
                                             float body_angular_velocity) {
  auto arrays = requiredArrays(frames);
  arrayNamed(arrays, "body_quat_w").identity_quaternions = true;
  arrayNamed(arrays, "body_ang_vel_w").fill_value = body_angular_velocity;
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
  config.step_dt = 0.02;
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
      throw std::runtime_error(throw_message);
    }
    return next_raw;
  }

  Vec next_raw;
  bool throw_on_infer{false};
  std::string throw_message{"fake velocity policy failed"};
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

LocoLowerDeployConfig locoLowerDeployConfig() {
  LocoLowerDeployConfig config;
  config.joint_dim = kLocoLowerPolicyJointDim;
  config.joint_ids_map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  config.sdk_joint_ids_map = config.joint_ids_map;
  config.stiffness = doubleSeq(40.0, 1.0, kLocoLowerPolicyJointDim);
  config.damping = doubleSeq(2.0, 0.1, kLocoLowerPolicyJointDim);
  config.default_joint_pos = doubleSeq(0.0, 0.05, kLocoLowerPolicyJointDim);
  config.joint_min_q = std::vector<double>(kLocoLowerPolicyJointDim, -1000.0);
  config.joint_max_q = std::vector<double>(kLocoLowerPolicyJointDim, 1000.0);
  config.action_scale = std::vector<double>(kLocoLowerPolicyJointDim, 0.1);
  config.action_offset = doubleSeq(-0.3, 0.03, kLocoLowerPolicyJointDim);
  config.observation_terms = {
      {"base_ang_vel", 3, 0, {1.0, 1.0, 1.0}},
      {"projected_gravity", 3, 3, {1.0, 1.0, 1.0}},
      {"keyboard_velocity_commands", 3, 6, {1.0, 1.0, 1.0}},
      {"joint_pos_rel", kLocoLowerPolicyJointDim, 9,
       std::vector<double>(kLocoLowerPolicyJointDim, 1.0)},
      {"joint_vel_rel", kLocoLowerPolicyJointDim, 21,
       std::vector<double>(kLocoLowerPolicyJointDim, 0.05)},
      {"last_action", kLocoLowerPolicyJointDim, 33,
       std::vector<double>(kLocoLowerPolicyJointDim, 1.0)},
  };
  config.command_ranges.lin_vel_x = {-2.0, 2.0};
  config.command_ranges.lin_vel_y = {-1.0, 1.0};
  config.command_ranges.ang_vel_z = {-3.0, 3.0};
  config.obs_row_width = kLocoLowerPolicyObsRowWidth;
  config.obs_history_length = kLocoLowerPolicyHistoryLength;
  config.obs_dim = kLocoLowerPolicyObsDim;
  config.policy_decimation = 1;
  config.step_dt = 0.02;
  return config;
}

LocoUpperLowCmdComposerConfig locoUpperComposerConfig() {
  LocoUpperLowCmdComposerConfig config;
  config.logical_to_sdk = frozenSdkMap();
  config.upper_kp.assign(kPolicyJointDim, 0.0F);
  config.upper_kd.assign(kPolicyJointDim, 0.0F);
  config.lower_min_q.assign(kLocoLowerPolicyJointDim, -1000.0F);
  config.lower_max_q.assign(kLocoLowerPolicyJointDim, 1000.0F);
  config.upper_min_q.assign(kPolicyJointDim, -10.0F);
  config.upper_max_q.assign(kPolicyJointDim, 10.0F);
  config.upper_max_vel_radps.assign(kPolicyJointDim, 0.0F);
  config.upper_max_accel_radps2.assign(kPolicyJointDim, 0.0F);
  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive;
       ++logical) {
    config.upper_kp.at(logical) = 55.0F;
    config.upper_kd.at(logical) = 5.0F;
    config.upper_min_q.at(logical) = -1000.0F;
    config.upper_max_q.at(logical) = 1000.0F;
    config.upper_max_vel_radps.at(logical) = 1000.0F;
    config.upper_max_accel_radps2.at(logical) = 10000.0F;
  }
  config.unowned_safe.mode = 0;
  config.unowned_safe.q = 0.0F;
  config.unowned_safe.kp = 0.0F;
  config.unowned_safe.kd = 0.0F;
  config.expected_mode_machine = kExpectedModeMachine;
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

ExecuteCommand locoUpperCommand(std::string id,
                                const std::filesystem::path& path,
                                MotionMode mode = MotionMode::Queue,
                                std::size_t frames = 3,
                                bool hold = false,
                                double max_radius_m = 0.0) {
  ExecuteCommand command = executeCommand(std::move(id), path, mode, frames, hold);
  command.executor = MotionExecutor::LocoUpper;
  command.loco_options.max_radius_m = max_radius_m;
  command.loco_options.hold = hold;
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

RuntimeControlLoop makeLocoControlLoop(
    RuntimeConfig config,
    RuntimeBridge& bridge,
    RuntimeStatusStore& store,
    const TrkValidationConfig& trk_config,
    FakeRobotIO& robot,
    FakePolicy& tracker_policy,
    FakeVelocityPolicy& velocity_policy,
    FakeVelocityPolicy& loco_lower_policy,
    DeployConfig deploy_config = deployConfig(),
    VelocityDeployConfig velocity_deploy_config = velocityDeployConfig(),
    FixStandConfig fixstand_config = fixStandConfig(),
    ControlMode startup_control = ControlMode::StandbyVelocity,
    PassiveConfig passive_config = passiveConfig(),
    LocoLowerDeployConfig loco_lower_deploy_config = locoLowerDeployConfig(),
    LocoUpperLowCmdComposerConfig loco_upper_composer_config =
        locoUpperComposerConfig(),
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
                            loco_lower_policy,
                            std::move(loco_lower_deploy_config),
                            std::move(loco_upper_composer_config),
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

void requireLocoLowerFrame(const LowCmdFrame& frame,
                           const LocoLowerDeployConfig& config,
                           const Vec& raw_actions) {
  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
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

void requireLocoLowerEntryHandoffFrame(const LowCmdFrame& frame,
                                       const LocoLowerDeployConfig& config,
                                       const LowStateSample& entry_low_state,
                                       const Vec& raw_actions,
                                       float alpha) {
  REQUIRE(frame.mode_machine == kExpectedModeMachine);
  REQUIRE(frame.mode_pr == 0);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(i));
    const float start_q = entry_low_state.motors.at(sdk_slot).q;
    const float policy_q =
        raw_actions.at(i) * static_cast<float>(config.action_scale.at(i)) +
        static_cast<float>(config.action_offset.at(i));
    const float expected_q = start_q + alpha * (policy_q - start_q);
    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q == Catch::Approx(expected_q).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp == static_cast<float>(config.stiffness.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).kd == static_cast<float>(config.damping.at(i)));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

void requireLocoUpperFrame(const LowCmdFrame& frame,
                           const LocoUpperLowCmdComposerConfig& config,
                           const TrkTrack& track,
                           std::size_t trk_frame) {
  const auto source = track.frame(trk_frame);
  REQUIRE(source.has_value());
  REQUIRE(source->joint_pos.ptr != nullptr);
  for (std::size_t logical = config.upper_start_joint;
       logical < config.upper_end_joint_exclusive;
       ++logical) {
    const auto sdk_slot = static_cast<std::size_t>(config.logical_to_sdk.at(logical));
    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q ==
            Catch::Approx(source->joint_pos.ptr[logical]).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp ==
            Catch::Approx(config.upper_kp.at(logical)).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).kd ==
            Catch::Approx(config.upper_kd.at(logical)).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

float locoUpperWrittenQ(const LowCmdFrame& frame,
                        const LocoUpperLowCmdComposerConfig& config,
                        std::size_t logical) {
  const auto sdk_slot = static_cast<std::size_t>(config.logical_to_sdk.at(logical));
  return frame.motors.at(sdk_slot).q;
}

void requireFixStandUpperFrame(const LowCmdFrame& frame,
                               const LocoUpperLowCmdComposerConfig& composer_config,
                               const FixStandConfig& fixstand_config) {
  for (std::size_t logical = composer_config.upper_start_joint;
       logical < composer_config.upper_end_joint_exclusive;
       ++logical) {
    const auto sdk_slot =
        static_cast<std::size_t>(composer_config.logical_to_sdk.at(logical));
    REQUIRE(frame.motors.at(sdk_slot).mode == 1);
    REQUIRE(frame.motors.at(sdk_slot).q ==
            Catch::Approx(static_cast<float>(fixstand_config.target_q.at(sdk_slot)))
                .margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).dq == 0.0F);
    REQUIRE(frame.motors.at(sdk_slot).kp ==
            Catch::Approx(composer_config.upper_kp.at(logical)).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).kd ==
            Catch::Approx(composer_config.upper_kd.at(logical)).margin(1.0e-5F));
    REQUIRE(frame.motors.at(sdk_slot).tau == 0.0F);
  }
}

void applyLowCmdPositionToLowState(const LowCmdFrame& frame,
                                   LowStateSample& low_state) {
  low_state.mode_machine = frame.mode_machine;
  low_state.mode_pr = frame.mode_pr;
  low_state.fresh = true;
  low_state.age_ms = 0;
  for (std::size_t i = 0; i < frame.motors.size(); ++i) {
    low_state.motors.at(i).q = frame.motors.at(i).q;
    low_state.motors.at(i).dq = 0.0F;
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

void requireCurrentUserFullStartupHold(RuntimeControlLoop& loop,
                                       RuntimeStatusStore& store,
                                       FakePolicy& policy,
                                       const std::string& id,
                                       float first_frame_joint,
                                       float second_frame_joint) {
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->id == id);
  REQUIRE(store.snapshot().exec->frame == 0);
  const int calls_at_target_start = policy.calls;

  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::User);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == id);
    REQUIRE(snapshot.exec->state == MotionState::Running);
    REQUIRE(snapshot.exec->frame == 0);
    REQUIRE(policy.calls == calls_at_target_start + static_cast<int>(tick + 1));
    REQUIRE(policy.inputs_seen.back()
                .obs_current.at(kObsCurrentCommandJointOffset) ==
            Catch::Approx(first_frame_joint).margin(1.0e-5F));
  }

  loop.tick();
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == id);
  REQUIRE(snapshot.exec->frame == 1);
  REQUIRE(policy.calls ==
          calls_at_target_start + static_cast<int>(kStartupHoldPolicyStepsAt50Fps) + 1);
  REQUIRE(policy.inputs_seen.back()
              .obs_current.at(kObsCurrentCommandJointOffset) ==
          Catch::Approx(second_frame_joint).margin(1.0e-5F));
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

StatusSnapshot advanceThroughFirstLocoMotionWrite(RuntimeControlLoop& loop,
                                                  RuntimeStatusStore& store) {
  StatusSnapshot snapshot;
  bool saw_motion_phase = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->executor == MotionExecutor::LocoUpper &&
        snapshot.exec->loco.phase == LocoPhase::Motion) {
      if (saw_motion_phase) {
        return snapshot;
      }
      saw_motion_phase = true;
    }
  }
  FAIL("expected loco_upper motion write");
  return snapshot;
}

void startHoldingRun(RuntimeControlLoop& loop,
                     RuntimeStatusStore& store,
                     RuntimeBridge& bridge,
                     TempTree& tmp,
                     const std::string& id,
                     FakeReferenceSink* reference = nullptr,
                     StartQueuedRunMode mode =
                         StartQueuedRunMode::RequireDirectStart) {
  const auto path = transitionReadyTrk(tmp, id + ".trk", 1, 20.0F);
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
  const auto target_path = transitionReadyTrk(tmp, target_id + ".trk", 2);
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

StatusSnapshot requireActiveUserAfterTransition(RuntimeControlLoop& loop,
                                                RuntimeStatusStore& store,
                                                const std::string& id) {
  for (int i = 0; i < 160; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.active.kind == ActiveKind::User) {
      REQUIRE(snapshot.exec.has_value());
      REQUIRE(snapshot.exec->id == id);
      REQUIRE(snapshot.exec->state == MotionState::Running);
      REQUIRE_FALSE(snapshot.transition.active);
      return snapshot;
    }
  }
  FAIL("target user did not start after synthetic transition");
  return store.snapshot();
}

void requireRunDoneEventually(RuntimeControlLoop& loop,
                              RuntimeStatusStore& store,
                              const std::string& id,
                              int max_ticks = 256) {
  for (int i = 0; i < max_ticks; ++i) {
    const auto found = store.findRun(id);
    REQUIRE(found.ok());
    REQUIRE(found.run.has_value());
    REQUIRE(found.run->state != MotionState::Failed);
    if (found.run->state == MotionState::Done) {
      return;
    }
    loop.tick();
  }
  FAIL("run did not reach done: " << id);
}

std::filesystem::path startCompletedUserToIdleTransition(
    RuntimeControlLoop& loop,
    RuntimeStatusStore& store,
    RuntimeBridge& bridge,
    TempTree& tmp,
    const std::string& id_prefix,
    StartQueuedRunMode start_mode = StartQueuedRunMode::RequireDirectStart) {
  const auto idle_path = transitionReadyTrk(tmp, id_prefix + "_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();

  const auto user_path = transitionReadyTrk(tmp, id_prefix + "_user.trk", 1, 20.0F);
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
  const auto user_path = transitionReadyTrk(tmp, id_prefix + "_user.trk", 1, 20.0F);
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

TEST_CASE("RuntimeStatusStore suppresses stale active idle after idle overlay clear") {
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  REQUIRE(bridge.configureIdle({idleMotion("stale_idle.trk", 3)}).ok());

  StatusSnapshot active_idle;
  active_idle.ready = true;
  active_idle.ctrl = ControllerState::Running;
  active_idle.robot = RobotState::Running;
  active_idle.active = {ActiveKind::Idle, ""};
  active_idle.idle.enabled = true;
  active_idle.idle.n = 1;
  active_idle.idle.active = true;
  store.publishSnapshot(active_idle);

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Idle);
  REQUIRE(snapshot.idle.active);

  REQUIRE(bridge.configureIdle({}).ok());
  store.publishSnapshot(active_idle);

  snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 0);
  REQUIRE_FALSE(snapshot.idle.active);
}

TEST_CASE("RuntimeControlLoop publishes observed loco-upper readiness from runtime deps") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();

  {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    FakeVelocityPolicy loco_lower_policy;
    auto loop = makeLocoControlLoop(config,
                                    bridge,
                                    store,
                                    tmp.trkConfig(),
                                    robot,
                                    tracker_policy,
                                    velocity_policy,
                                    loco_lower_policy,
                                    deploy_config);

    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Velocity);
    REQUIRE(store.snapshot().loco_upper.ready);
  }

  {
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
                                deploy_config);

    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
    REQUIRE_FALSE(store.snapshot().loco_upper.ready);
  }
}

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
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{{1, 2}});
  REQUIRE(reference.latest.com == std::array<float, 3>{{0.0F, 0.25F, 0.5F}});
  REQUIRE(reference.latest.comv == std::array<float, 3>{{0.0F, 0.25F, 0.5F}});

  loop.tick();
  loop.tick();
  REQUIRE(reference.latest.frame == 1);
  REQUIRE(reference.latest.p.at(0) == std::array<float, 3>{{20.25F, 20.5F, 20.75F}});
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{{1, 2}});

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
	            ControllerState::StandbyVelocity, StopReason::Stop, 40},
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

TEST_CASE("RuntimeControlLoop standby from holding faults on handoff failure without clearing idle") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const auto standby_track =
      loadTrack(tmp.trkConfig(), validTrk(tmp, "holding_failure_standby_ref.trk", 2));
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

  const auto idle_path = validTrk(tmp, "holding_failure_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  REQUIRE(store.snapshot().idle.enabled);

  startHoldingRun(loop,
                  store,
                  bridge,
                  tmp,
                  "held-standby-failure",
                  &reference,
                  StartQueuedRunMode::AllowStandbyTransition);

  loop.failNextTransitionStartForTest();
  const int writes_before_standby = robot.write_attempts;
  REQUIRE(bridge.standby().ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
  REQUIRE(snapshot.err == ErrorCode::InternalError);
  REQUIRE(snapshot.block == "standby_transition_failed");
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(robot.write_attempts == writes_before_standby);

  const auto found = store.findRun("held-standby-failure");
  REQUIRE(found.ok());
  REQUIRE(found.run->state == MotionState::Failed);
  REQUIRE(found.run->err == ErrorCode::InternalError);
  REQUIRE(found.run->stop_reason == StopReason::None);
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

TEST_CASE("RuntimeControlLoop standby cancellation starts from controllable state") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LowStateSample low_state = readyLowState(deploy_config);
  low_state.quat_wxyz = yawQuat(0.35F);
  FakeRobotIO robot(low_state);
  HighStateSample high_state;
  high_state.fresh = true;
  high_state.age_ms = 3;
  high_state.position = {2.0F, -3.0F, 0.82F};
  high_state.linear_velocity = {0.1F, -0.2F, 0.0F};
  high_state.angular_velocity = {0.0F, 0.0F, 0.3F};
  robot.high_state = high_state;
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy;
  FakeReferenceSink reference;
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                transitionReadyTrk(tmp, "actual_state_standby_ref.trk", 2, 10.0F));
  auto loop = makeControlLoop(config,
                              bridge,
                              store,
                              tmp.trkConfig(),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocityDeployConfig(),
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference,
                              standby_track);

  const auto active_path =
      transitionReadyTrk(tmp, "actual_state_active.trk", 5, 40.0F);
  REQUIRE(bridge.submitQueue(
              executeCommand("actual-state-active", active_path, MotionMode::Queue, 5))
              .ok());
  startQueuedRun(loop,
                 store,
                 "actual-state-active",
                 StartQueuedRunMode::AllowStandbyTransition);
  const int calls_before_cancel_transition = tracker_policy.calls;

  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.p.at(0) == high_state.position);
  REQUIRE(reference.latest.q.at(0).at(0) ==
          Catch::Approx(low_state.quat_wxyz.at(0)).margin(1.0e-5F));
  REQUIRE(reference.latest.q.at(0).at(3) ==
          Catch::Approx(low_state.quat_wxyz.at(3)).margin(1.0e-5F));

  for (int i = 0; i < 8 &&
                  tracker_policy.calls == calls_before_cancel_transition;
       ++i) {
    loop.tick();
  }
  REQUIRE(tracker_policy.calls == calls_before_cancel_transition + 1);
  Vec expected_joint_pos;
  expected_joint_pos.reserve(kPolicyJointDim);
  for (std::size_t policy_index = 0; policy_index < kPolicyJointDim; ++policy_index) {
    const auto sdk_slot =
        static_cast<std::size_t>(deploy_config.sdk_joint_ids_map.at(policy_index));
    expected_joint_pos.push_back(low_state.motors.at(sdk_slot).q);
  }
  requireObsSliceApprox(tracker_policy.inputs_seen.back().obs_current,
                        kObsCurrentCommandJointOffset,
                        expected_joint_pos);
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

TEST_CASE("RuntimeControlLoop standby cancellation transition start failure faults without clearing idle") {
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

  const auto idle_path = validTrk(tmp, "cancel_failure_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  REQUIRE(store.snapshot().idle.enabled);

  const auto path = validTrk(tmp, "cancel_failure_user.trk", 5);
  REQUIRE(bridge.submitQueue(
              executeCommand("cancel-failure-user", path, MotionMode::Queue, 5))
              .ok());
  startQueuedRun(loop,
                 store,
                 "cancel-failure-user",
                 StartQueuedRunMode::AllowStandbyTransition);

  loop.failNextTransitionStartForTest();
  const int writes_before_stop = robot.write_attempts;
  REQUIRE(bridge.standbyVelocity().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
  REQUIRE(snapshot.err == ErrorCode::InternalError);
  REQUIRE(snapshot.block == "standby_transition_failed");
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(robot.write_attempts == writes_before_stop);
  REQUIRE(store.findRun("cancel-failure-user").run->state == MotionState::Failed);
  REQUIRE(store.findRun("cancel-failure-user").run->err == ErrorCode::InternalError);

  loop.tick();
  REQUIRE(robot.write_attempts == writes_before_stop);
  REQUIRE(velocity_policy.calls == 0);
}

TEST_CASE("RuntimeControlLoop standby fallback bridges zero-contact standby reference and preserves idle") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.04;
  config.user_bridge_reduced_startup_hold_s = 0.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                transitionReadyContactZeroVelTrk(tmp,
                                                 "zero_contact_standby_ref.trk",
                                                 3,
                                                 0,
                                                 0,
                                                 0.02F));

  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  LowStateSample low_state = readyLowState(deploy_config);
  low_state.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low_state.gyro = {0.0F, 0.0F, 0.0F};
  for (std::size_t policy_index = 0;
       policy_index < deploy_config.sdk_joint_ids_map.size();
       ++policy_index) {
    const auto sdk_slot =
        static_cast<std::size_t>(deploy_config.sdk_joint_ids_map.at(policy_index));
    low_state.motors.at(sdk_slot).q = 0.0F;
    low_state.motors.at(sdk_slot).dq = 0.0F;
  }
  FakeRobotIO robot(low_state);
  HighStateSample high_state;
  high_state.fresh = true;
  high_state.age_ms = 3;
  high_state.position = {0.0F, 0.0F, 0.738F};
  high_state.linear_velocity = {0.0F, 0.0F, 0.0F};
  high_state.angular_velocity = {0.0F, 0.0F, 0.0F};
  robot.high_state = high_state;
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

  const auto idle_path =
      transitionReadyContactZeroVelTrk(tmp, "zero_contact_idle.trk", 3, 1, 2, 0.02F);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.enabled);

  const auto user_path =
      transitionReadyContactZeroVelTrk(tmp, "zero_contact_user.trk", 20, 1, 2, 0.02F);
  REQUIRE(bridge.submitQueue(
              executeCommand("zero-contact-user", user_path, MotionMode::Queue, 20))
              .ok());
  startQueuedRun(loop,
                 store,
                 "zero-contact-user",
                 StartQueuedRunMode::AllowStandbyTransition);
  for (int i = 0; i < 12; ++i) {
    loop.tick();
  }
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.idle.enabled);

  REQUIRE(bridge.standby().ok());
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(store.findRun("zero-contact-user").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("zero-contact-user").run->stop_reason == StopReason::Stop);
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

TEST_CASE("RuntimeControlLoop stop preserves standby handoff while passive and fixstand preempt it") {
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
    if (std::string(item.id) == "cancel-transition-stop") {
      REQUIRE(snapshot.ctrl == ControllerState::Running);
      REQUIRE(snapshot.active.kind == ActiveKind::Transition);
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE(snapshot.transition.active);
      REQUIRE(snapshot.transition.target == "standby");
    } else {
      REQUIRE(snapshot.ctrl == item.expected_ctrl);
      REQUIRE(snapshot.active.kind == item.expected_active);
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE_FALSE(snapshot.transition.active);
    }
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

TEST_CASE("RuntimeControlLoop completed user bridges queued user through synthetic transition") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto source_path =
      transitionReadyTrk(tmp, "completed_bridge_source.trk", 2, 0.2F);
  const auto target_path =
      transitionReadyTrk(tmp, "completed_bridge_target.trk", 2, 0.1F);
  REQUIRE(bridge.submitQueue(
              executeCommand("completed-bridge-a",
                             source_path,
                             MotionMode::Queue,
                             2))
              .ok());
  startQueuedRun(loop, store, "completed-bridge-a");
  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);

  const auto source_track = loadTrack(tmp.trkConfig(), source_path);
  const auto source_reference =
      makeReferenceFrameSnapshot("completed-bridge-a", *source_track, 1);
  REQUIRE(source_reference.has_value());
  REQUIRE(bridge.submitQueue(
              executeCommand("completed-bridge-b",
                             target_path,
                             MotionMode::Queue,
                             2))
              .ok());
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"completed-bridge-b"});

  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "completed-bridge-b");
  REQUIRE(snapshot.transition.target_state == MotionState::Queued);
  REQUIRE(snapshot.transition.frame == 0);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("completed-bridge-a").run->state == MotionState::Done);
  REQUIRE(store.findRun("completed-bridge-b").run->state == MotionState::Queued);
  REQUIRE(store.findRun("transition").code == ErrorCode::RunNotFound);
  requireReferenceStartsFrom(reference, *source_reference);

  snapshot = requireActiveUserAfterTransition(loop, store, "completed-bridge-b");
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(snapshot.exec->frame == 0);
  REQUIRE(snapshot.exec->frames == 2);
}

TEST_CASE("RuntimeControlLoop completed user bridge uses controllable same-valid contact fallback") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
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

  const auto source_path =
      transitionReadyContactZeroVelTrk(tmp,
                                       "completed_same_valid_contact_a.trk",
                                       2,
                                       0,
                                       0,
                                       0.2F);
  const auto target_path =
      transitionReadyContactZeroVelTrk(tmp,
                                       "completed_same_valid_contact_b.trk",
                                       3,
                                       1,
                                       2,
                                       0.02F);
  REQUIRE(bridge.submitQueue(
              executeCommand("completed-same-valid-contact-a",
                             source_path,
                             MotionMode::Queue,
                             2))
              .ok());
  startQueuedRun(loop, store, "completed-same-valid-contact-a");
  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
  }
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);

  REQUIRE(bridge.submitQueue(
              executeCommand("completed-same-valid-contact-b",
                             target_path,
                             MotionMode::Queue,
                             3))
              .ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "completed-same-valid-contact-b");
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{1, 2});
  REQUIRE(store.findRun("completed-same-valid-contact-a").run->state ==
          MotionState::Done);
  REQUIRE(store.findRun("completed-same-valid-contact-b").run->state ==
          MotionState::Queued);

  requireActiveUserAfterTransition(loop, store, "completed-same-valid-contact-b");
  REQUIRE(store.findRun("completed-same-valid-contact-b").run->state ==
          MotionState::Running);
}

TEST_CASE("RuntimeControlLoop completed user bridge rejects nonzero raw fallback gap") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
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

  const auto source_path = constantIdentityTrk(tmp, "bridge_benign_gate_a.trk", 2);
  const auto target_path =
      constantIdentityTrk(tmp,
                          "bridge_benign_gate_b.trk",
                          3,
                          static_cast<float>(
                              defaultSyntheticTransitionLimits().max_velocity + 1.0));
  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-benign-gate-a",
                             source_path,
                             MotionMode::Queue,
                             2))
              .ok());
  startQueuedRun(loop, store, "bridge-benign-gate-a");
  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
  }
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);

  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-benign-gate-b",
                             target_path,
                             MotionMode::Queue,
                             3))
              .ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("bridge-benign-gate-a").run->state == MotionState::Done);
  const auto failed = store.findRun("bridge-benign-gate-b");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop completed user yaw residual gate reject keeps queued target") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto source_path =
      transitionReadyTrk(tmp, "bridge_yaw_residual_a.trk", 2, 0.2F);
  const auto target_path =
      transitionReadyTrk(tmp, "bridge_yaw_residual_b.trk", 2, 0.1F);
  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-yaw-residual-a",
                             source_path,
                             MotionMode::Queue,
                             2))
              .ok());
  startQueuedRun(loop, store, "bridge-yaw-residual-a");
  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);

  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-yaw-residual-b",
                             target_path,
                             MotionMode::Queue,
                             2))
              .ok());
  loop.forceNextRootYawResidualForTest(0.06);
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"bridge-yaw-residual-b"});
  REQUIRE(store.findRun("bridge-yaw-residual-a").run->state == MotionState::Done);
  REQUIRE(store.findRun("bridge-yaw-residual-b").run->state == MotionState::Queued);

  startQueuedRun(loop, store, "bridge-yaw-residual-b");
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->id == "bridge-yaw-residual-b");
  REQUIRE(store.snapshot().exec->state == MotionState::Running);
}

TEST_CASE("RuntimeControlLoop completed user unsafe bridge failure does not normal-start queued target") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
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

  const auto source_path = identityQuaternionTrk(tmp, "bridge_unsafe_a.trk", 2);
  const Vec large_gap(kPolicyJointDim, 1000.0F);
  const auto target_path =
      jointPosTrk(tmp, "bridge_unsafe_b.trk", {large_gap, large_gap});
  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-unsafe-a",
                             source_path,
                             MotionMode::Queue,
                             2))
              .ok());
  startQueuedRun(loop, store, "bridge-unsafe-a");
  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
  }
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);

  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-unsafe-b",
                             target_path,
                             MotionMode::Queue,
                             2))
              .ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("bridge-unsafe-a").run->state == MotionState::Done);
  const auto failed = store.findRun("bridge-unsafe-b");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::InternalError);

  loop.tick();
  REQUIRE_FALSE(store.snapshot().exec.has_value());
}

TEST_CASE("RuntimeControlLoop completed user unsafe bridge failures record target failed") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;

  struct Case {
    const char* id;
    std::filesystem::path (*make_target)(TempTree&, const std::string&);
    ErrorCode expected_error;
  };

  const auto high_endpoint_velocity = [](TempTree& tmp, const std::string& name) {
    return jointVelocityTrk(
        tmp,
        name,
        2,
        static_cast<float>(defaultSyntheticTransitionLimits().max_velocity + 1.0));
  };
  const auto invalid_root_quaternion = [](TempTree& tmp, const std::string& name) {
    return zeroQuaternionTrk(tmp, name, 2);
  };
  const auto invalid_contact = [](TempTree& tmp, const std::string& name) {
    return invalidContactTrk(tmp, name);
  };

  for (const auto& item : std::vector<Case>{
           {"bridge-high-endpoint-velocity",
            high_endpoint_velocity,
            ErrorCode::InternalError},
           {"bridge-invalid-root-quaternion",
            invalid_root_quaternion,
            ErrorCode::InternalError},
           {"bridge-target-validation-failure",
            invalid_contact,
            ErrorCode::TrkValidationFailed},
       }) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

    const auto source_path =
        transitionReadyTrk(tmp, std::string(item.id) + "_source.trk", 2, 0.2F);
    const auto target_path =
        item.make_target(tmp, std::string(item.id) + "_target.trk");
    REQUIRE(bridge.submitQueue(
                executeCommand(std::string(item.id) + "-source",
                               source_path,
                               MotionMode::Queue,
                               2))
                .ok());
    startQueuedRun(loop, store, std::string(item.id) + "-source");
    loop.tick();
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->frame == 0);

    REQUIRE(bridge.submitQueue(
                executeCommand(item.id, target_path, MotionMode::Queue, 2))
                .ok());
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.queue.ids.empty());
    REQUIRE(store.findRun(std::string(item.id) + "-source").run->state ==
            MotionState::Done);
    const auto failed = store.findRun(item.id);
    REQUIRE(failed.ok());
    REQUIRE(failed.run->state == MotionState::Failed);
    REQUIRE(failed.run->err == item.expected_error);

    loop.tick();
    REQUIRE_FALSE(store.snapshot().exec.has_value());
  }
}

TEST_CASE("RuntimeControlLoop robot unsafe before completed user bridge does not normal-start queued target") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
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

  const auto source_path =
      transitionReadyTrk(tmp, "bridge_robot_unsafe_a.trk", 2, 0.2F);
  const auto target_path =
      transitionReadyTrk(tmp, "bridge_robot_unsafe_b.trk", 2, 0.1F);
  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-robot-unsafe-a",
                             source_path,
                             MotionMode::Queue,
                             2))
              .ok());
  startQueuedRun(loop, store, "bridge-robot-unsafe-a");
  for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
    loop.tick();
  }
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->frame == 0);

  REQUIRE(bridge.submitQueue(
              executeCommand("bridge-robot-unsafe-b",
                             target_path,
                             MotionMode::Queue,
                             2))
              .ok());
  robot.low_state = badOrientationLowState(deploy_config, 91);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  const auto source = store.findRun("bridge-robot-unsafe-a");
  REQUIRE(source.ok());
  REQUIRE(source.run->state == MotionState::Failed);
  REQUIRE(source.run->err == ErrorCode::RobotBadOrientation);
  REQUIRE(store.findRun("bridge-robot-unsafe-b").run->state ==
          MotionState::Failed);
  REQUIRE(store.findRun("bridge-robot-unsafe-b").run->err ==
          ErrorCode::RobotBadOrientation);
}

TEST_CASE("RuntimeControlLoop bridge only applies to GeneralTracker non-hold user to GeneralTracker") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();

  SECTION("held source does not complete into the user-to-user bridge") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference);
    const auto source_path = validTrk(tmp, "bridge_scope_hold_a.trk", 1);
    const auto target_path = validTrk(tmp, "bridge_scope_hold_b.trk", 2);
    REQUIRE(bridge.submitQueue(
                executeCommand("bridge-scope-hold-a",
                               source_path,
                               MotionMode::Queue,
                               1,
                               true))
                .ok());
    startQueuedRun(loop, store, "bridge-scope-hold-a");
    REQUIRE(bridge.submitQueue(
                executeCommand("bridge-scope-hold-b",
                               target_path,
                               MotionMode::Queue,
                               2))
                .ok());

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::User);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == "bridge-scope-hold-a");
    REQUIRE(snapshot.exec->state == MotionState::Holding);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.queue.ids ==
            std::vector<std::string>{"bridge-scope-hold-b"});
  }

  SECTION("LocoUpper target remains queued for ordinary FIFO handling") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference);
    const auto source_path = validTrk(tmp, "bridge_scope_loco_a.trk", 1);
    const auto target_path = validTrk(tmp, "bridge_scope_loco_b.trk", 2);
    REQUIRE(bridge.submitQueue(
                executeCommand("bridge-scope-loco-a",
                               source_path,
                               MotionMode::Queue,
                               1))
                .ok());
    startQueuedRun(loop, store, "bridge-scope-loco-a");
    REQUIRE(bridge.submitQueue(
                locoUpperCommand("bridge-scope-loco-b",
                                 target_path,
                                 MotionMode::Queue,
                                 2))
                .ok());

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.queue.ids ==
            std::vector<std::string>{"bridge-scope-loco-b"});
    REQUIRE(store.findRun("bridge-scope-loco-a").run->state == MotionState::Done);
    REQUIRE(store.findRun("bridge-scope-loco-b").run->state == MotionState::Queued);
  }

  SECTION("completed user still prefers background idle transition when no user is queued") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config,
                            bridge,
                            store,
                            TrkLoader(tmp.trkConfig()),
                            &reference);
    startCompletedUserToIdleTransition(loop,
                                       store,
                                       bridge,
                                       tmp,
                                       "bridge-scope-idle");

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::Transition);
    REQUIRE(snapshot.transition.active);
    REQUIRE(snapshot.transition.target == "idle");
    REQUIRE(snapshot.transition.target_id.empty());
  }
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
  for (std::size_t body = 0; body < reference.latest.q.size(); ++body) {
    for (std::size_t axis = 0; axis < reference.latest.q.at(body).size(); ++axis) {
      REQUIRE(reference.latest.q.at(body).at(axis) ==
              Catch::Approx(interrupted_frame.q.at(body).at(axis)).margin(1.0e-6F));
    }
  }
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

TEST_CASE("RuntimeControlLoop policy synthetic transition uses deploy step dt when target fps differs") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy(floatSeq(0.0F, kPolicyJointDim));
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config,
                          bridge,
                          store,
                          TrkLoader(tmp.trkConfig(25.0)),
                          robot,
                          policy,
                          deploy_config,
                          passiveConfig(),
                          kExpectedModeMachine,
                          RuntimeMode::Real,
                          &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "held-policy-fps",
                               "target-policy-fps",
                               &reference);

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.frames == 51);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.fps == 50.0);
}

TEST_CASE("RuntimeControlLoop non-policy synthetic transition keeps target track fps") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 1.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig(25.0)), &reference);

  startHoldingToUserTransition(loop,
                               store,
                               bridge,
                               tmp,
                               "held-non-policy-fps",
                               "target-non-policy-fps",
                               &reference);

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.frames == 26);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.fps == 25.0);
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

TEST_CASE("RuntimeControlLoop stop preserves standby reference gate while controls abort it") {
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
    if (std::string(item.id) == "abort-standby-stop") {
      REQUIRE(snapshot.ctrl == ControllerState::Running);
      REQUIRE(snapshot.active.kind == ActiveKind::Transition);
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE(snapshot.transition.active);
      REQUIRE(snapshot.transition.target == "standby");
      REQUIRE(reference.latest.active);
    } else {
      REQUIRE(snapshot.ctrl == item.expected_ctrl);
      REQUIRE(snapshot.active.kind == ActiveKind::None);
      REQUIRE_FALSE(snapshot.exec.has_value());
      REQUIRE_FALSE(snapshot.transition.active);
      REQUIRE(snapshot.queue.ids.empty());
      REQUIRE_FALSE(reference.latest.active);
    }
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

TEST_CASE("RuntimeControlLoop arrived via user bridge uses reduced startup hold only for target") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.user_bridge_reduced_startup_hold_s = 0.10;

  SECTION("completed user bridge target uses reduced startup hold") {
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

    const auto source_path =
        transitionReadyTrk(tmp, "bridge_reduced_hold_a.trk", 2, 0.2F);
    const auto target_path =
        transitionReadyTrk(tmp, "bridge_reduced_hold_b.trk", 3, 0.1F);
    REQUIRE(bridge.submitQueue(
                executeCommand("bridge-reduced-hold-a",
                               source_path,
                               MotionMode::Queue,
                               2))
                .ok());
    startQueuedRun(loop, store, "bridge-reduced-hold-a");
    for (std::size_t tick = 0; tick < kStartupHoldPolicyStepsAt50Fps; ++tick) {
      loop.tick();
    }
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->frame == 0);

    REQUIRE(bridge.submitQueue(
                executeCommand("bridge-reduced-hold-b",
                               target_path,
                               MotionMode::Queue,
                               3))
                .ok());
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);

    const auto started =
        requireActiveUserAfterTransition(loop, store, "bridge-reduced-hold-b");
    REQUIRE(started.exec.has_value());
    REQUIRE(started.exec->frame == 0);
    const int calls_at_target_start = policy.calls;

    const std::size_t reduced_steps = 5;
    for (std::size_t tick = 0; tick < reduced_steps; ++tick) {
      loop.tick();
      REQUIRE(store.snapshot().exec.has_value());
      REQUIRE(store.snapshot().exec->frame == 0);
      REQUIRE(policy.calls == calls_at_target_start + static_cast<int>(tick + 1));
    }

    loop.tick();
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->frame == 1);
    REQUIRE(policy.calls == calls_at_target_start + static_cast<int>(reduced_steps) + 1);
  }

  SECTION("standby to user still runs startup hold") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    const DeployConfig deploy_config = deployConfig();
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    FakeReferenceSink reference;
    const auto standby_track = validStandbyTrack(tmp, "standby_startup_hold_ref.trk");
    auto loop = makeControlLoop(config,
                                bridge,
                                store,
                                tmp.trkConfig(),
                                robot,
                                tracker_policy,
                                velocity_policy,
                                deploy_config,
                                velocityDeployConfig(),
                                fixStandConfig(),
                                ControlMode::StandbyVelocity,
                                passiveConfig(),
                                &reference,
                                standby_track);

    const auto target_path = identityQuaternionTrk(tmp, "standby_startup_hold_user.trk", 3);
    REQUIRE(bridge.submitQueue(
                executeCommand("standby-startup-hold-user",
                               target_path,
                               MotionMode::Queue,
                               3))
                .ok());
    startQueuedRun(loop,
                   store,
                   "standby-startup-hold-user",
                   StartQueuedRunMode::AllowStandbyTransition);
    requireCurrentUserFullStartupHold(loop,
                                      store,
                                      tracker_policy,
                                      "standby-startup-hold-user",
                                      0.0F,
                                      6.5F);
  }
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
    const bool terminal_safety =
        item.expected_fsm == RuntimeInternalState::Fault ||
        item.err == ErrorCode::RobotBadOrientation;
    if (terminal_safety) {
      REQUIRE(snapshot.queue.ids.empty());
      REQUIRE(store.findRun(item.id).run->state == MotionState::Failed);
      REQUIRE(store.findRun(item.id).run->err == item.err);
    } else {
      REQUIRE(snapshot.queue.ids == std::vector<std::string>{item.id});
      REQUIRE(store.findRun(item.id).run->state == MotionState::Queued);
    }
    REQUIRE(store.health().state == ServiceHealth::Error);
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

TEST_CASE("RuntimeControlLoop safety fault terminates waiting queue") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy policy;
  auto loop = makePolicyLoop(config, bridge, store, tmp.trkConfig(), robot, policy,
                             deploy_config);
  const auto active_path = validTrk(tmp, "fault_clears_active.trk", 5);
  const auto waiting_path = validTrk(tmp, "fault_clears_waiting.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("fault-active", active_path)).ok());
  startQueuedRun(loop, store, "fault-active");
  REQUIRE(bridge.submitQueue(executeCommand("fault-waiting", waiting_path)).ok());
  loop.tick();
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"fault-waiting"});

  robot.occupancy = {true, 4};
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(snapshot.queue.ids.empty());
  const auto active = store.findRun("fault-active");
  REQUIRE(active.ok());
  REQUIRE(active.run->state == MotionState::Failed);
  REQUIRE(active.run->err == ErrorCode::RobotNotReady);
  const auto waiting = store.findRun("fault-waiting");
  REQUIRE(waiting.ok());
  REQUIRE(waiting.run->state == MotionState::Failed);
  REQUIRE(waiting.run->err == ErrorCode::RobotNotReady);
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

TEST_CASE("RuntimeControlLoop policy stop honors configured hold time") {
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
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Stopping);
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 2);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(policy.calls == 0);
  REQUIRE(robot.write_attempts == 3);
  requireHoldFrame(robot.writes.back(), deploy_config, hold_state);

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
  robot.low_state = std::nullopt;
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
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotDisconnected);
  REQUIRE(snapshot.block == "lowstate_missing");
  REQUIRE_FALSE(snapshot.passive_reason.has_value());
  REQUIRE(robot.write_attempts == 0);

  robot.low_state = readyLowState(deploy_config, kExpectedModeMachine, false, 123);
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotNotReady);
  REQUIRE(snapshot.block == "lowstate_timeout");
  REQUIRE(snapshot.low_ms == 123);
  REQUIRE_FALSE(snapshot.passive_reason.has_value());
  REQUIRE(robot.write_attempts == 0);

  robot.low_state = ready_state;
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE_FALSE(snapshot.passive_reason.has_value());
  REQUIRE(robot.write_attempts == 1);
  requireFixStandFrameFromCurrentQ(robot.writes.back(), fixstand_config, ready_state);
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(velocity_policy.calls == 0);
}

TEST_CASE("RuntimeControlLoop startup FixStand bad orientation enters Passive and latches reason") {
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
                              ControlMode::FixStand);

  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.low_ms == 77);
  REQUIRE(snapshot.passive_reason.has_value());
  REQUIRE(snapshot.passive_reason->code == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.passive_reason->block == "bad_orientation");
  REQUIRE(robot.write_attempts == 0);
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
  REQUIRE(snapshot.passive_reason.has_value());
  REQUIRE(snapshot.passive_reason->code == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.passive_reason->block == "bad_orientation");
  REQUIRE(robot.write_attempts == 0);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(robot.write_attempts == 1);
  requirePassiveDampingFrame(robot.writes.back(), *robot.low_state, passiveConfig());

  const LowStateSample ready_state = readyLowState(deploy_config, kExpectedModeMachine,
                                                   true, 8);
  robot.low_state = ready_state;
  REQUIRE(bridge.fixStand().ok());
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE_FALSE(snapshot.passive_reason.has_value());
  REQUIRE(robot.write_attempts >= 1);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(robot.write_attempts >= 2);
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

TEST_CASE("RuntimeControlLoop FixStand holds transient readiness failures and faults lowcmd occupancy") {
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
    bool recoverable;
  };

  const std::vector<Case> cases{
      {"missing",
       std::nullopt,
       {},
       ControllerState::FixStand,
       RuntimeInternalState::FixStand,
       ErrorCode::RobotDisconnected,
       "lowstate_missing",
       true},
      {"stale",
       readyLowState(deploy_config, kExpectedModeMachine, false, 51),
       {},
       ControllerState::FixStand,
       RuntimeInternalState::FixStand,
       ErrorCode::RobotNotReady,
       "lowstate_timeout",
       true},
      {"mode-mismatch",
       readyLowState(deploy_config, kExpectedModeMachine + 1, true, 52),
       {},
       ControllerState::FixStand,
       RuntimeInternalState::FixStand,
       ErrorCode::RobotNotReady,
       "mode_machine_mismatch",
       true},
      {"lowcmd-occupied",
       readyLowState(deploy_config, kExpectedModeMachine, true, 53),
       {true, 3},
       ControllerState::Fault,
       RuntimeInternalState::Fault,
       ErrorCode::RobotNotReady,
       "lowcmd_occupied",
       false},
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

    robot.low_state = readyLowState(deploy_config, kExpectedModeMachine, true, 54);
    robot.occupancy = {};
    loop.tick();
    const auto recovered = store.snapshot();
    if (item.recoverable) {
      REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
      REQUIRE(recovered.ctrl == ControllerState::FixStand);
      REQUIRE(recovered.ready);
      REQUIRE(recovered.err == ErrorCode::Ok);
      REQUIRE(recovered.block.empty());
      REQUIRE(robot.write_attempts == 1);
    } else {
      REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Fault);
      REQUIRE(recovered.ctrl == ControllerState::Fault);
      REQUIRE_FALSE(recovered.ready);
      REQUIRE(recovered.err == ErrorCode::RobotNotReady);
      REQUIRE(recovered.block == "lowcmd_occupied");
      REQUIRE(robot.write_attempts == 0);
    }
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

TEST_CASE("RuntimeControlLoop loco_upper starts active run with zero distance") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_start_distance.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {3.0F, 4.0F, 0.0F},
                                       {6.0F, 8.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-start-distance",
                                              path,
                                              MotionMode::Queue,
                                              3))
              .ok());

  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->executor == MotionExecutor::LocoUpper);
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Entry);
  REQUIRE(snapshot.exec->loco.distance_m == Catch::Approx(0.0));
  REQUIRE(snapshot.exec->loco.distance_m != Catch::Approx(10.0));
  REQUIRE(snapshot.exec->loco.radius_source.empty());
}

TEST_CASE("RuntimeControlLoop loco_upper records highstate radius limit without Passive") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.radius_tolerance_m = 0.05;
  config.loco_upper_max_lin_accel_mps2 = 10000.0;
  config.loco_upper_max_yaw_accel_radps2 = 10000.0;
  config.loco_upper_smoothing_window_frames = 1;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.command_ranges.lin_vel_x = {-100.0, 100.0};
  loco_lower_config.command_ranges.lin_vel_y = {-100.0, 100.0};
  loco_lower_config.command_ranges.ang_vel_z = {-100.0, 100.0};
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample high;
  high.fresh = true;
  high.position = {10.0F, 20.0F, 0.0F};
  robot.high_state = high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_highstate_radius.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {1.0F, 0.0F, 0.0F},
                                       {2.0F, 0.0F, 0.0F},
                                       {3.0F, 0.0F, 0.0F},
                                       {4.0F, 0.0F, 0.0F},
                                       {5.0F, 0.0F, 0.0F},
                                       {6.0F, 0.0F, 0.0F},
                                       {7.0F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-highstate-radius",
                                              path,
                                              MotionMode::Queue,
                                              8,
                                              false,
                                              5.0))
              .ok());

  loop.tick();
  StatusSnapshot snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.distance_m == Catch::Approx(0.0));
  REQUIRE(snapshot.exec->loco.radius_source.empty());

  robot.high_state->position = {13.0F, 24.0F, 0.0F};
  snapshot = advanceThroughFirstLocoMotionWrite(loop, store);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.radius_source == "highstate");
  REQUIRE(snapshot.exec->loco.distance_m == Catch::Approx(5.0).margin(1.0e-5));

  const int writes_before_breach = robot.write_attempts;
  robot.high_state->position = {13.1F, 24.0F, 0.0F};
  bool saw_bounded_write = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec.has_value() && robot.write_attempts > writes_before_breach) {
      saw_bounded_write = true;
      break;
    }
  }

  REQUIRE(saw_bounded_write);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::LocoUpperActive);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.exec->state == MotionState::Running);
  REQUIRE(snapshot.exec->loco.phase != LocoPhase::Failed);
  REQUIRE(snapshot.exec->loco.radius_limit_reached);
  REQUIRE(snapshot.exec->loco.reason == LocoReason::None);
  REQUIRE(snapshot.exec->loco.radius_source == "highstate");
  REQUIRE(snapshot.exec->loco.distance_m ==
          Catch::Approx(std::hypot(3.1, 4.0)).margin(1.0e-5));
  REQUIRE(robot.write_attempts > writes_before_breach);
  REQUIRE_FALSE(loco_lower_policy.inputs_seen.empty());
  const Vec& obs = loco_lower_policy.inputs_seen.back().obs;
  REQUIRE(obs.size() == kLocoLowerPolicyObsDim);
  const double radius = std::hypot(3.1, 4.0);
  const double outward_velocity =
      static_cast<double>(obs.at(6)) * (3.1 / radius) +
      static_cast<double>(obs.at(7)) * (4.0 / radius);
  REQUIRE(outward_velocity <= 1.0e-5);
}

TEST_CASE("RuntimeControlLoop loco_upper writes entry command when radius already exceeds limit") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.radius_tolerance_m = 0.05;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample high;
  high.fresh = true;
  high.position = {10.0F, 20.0F, 0.0F};
  robot.high_state = high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_entry_radius_guard.trk", 3);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-entry-radius-guard",
                                              path,
                                              MotionMode::Queue,
                                              3,
                                              false,
                                              0.5))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  robot.high_state->position = {10.6F, 20.0F, 0.0F};
  const int writes_before_breach = robot.write_attempts;

  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::LocoUpperActive);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.exec->loco.phase != LocoPhase::Failed);
  REQUIRE(snapshot.exec->loco.radius_limit_reached);
  REQUIRE(snapshot.exec->loco.reason == LocoReason::None);
  REQUIRE(snapshot.exec->loco.distance_m == Catch::Approx(0.6).margin(1.0e-5));
  REQUIRE(robot.write_attempts > writes_before_breach);
  REQUIRE(loco_lower_policy.calls > 0);
}

TEST_CASE("RuntimeControlLoop loco_upper integrates command when highstate position is unavailable") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample invalid_high;
  invalid_high.fresh = true;
  invalid_high.position = {
      std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
  robot.high_state = invalid_high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_integrated_distance.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.02F, 0.0F, 0.0F},
                                       {0.04F, 0.0F, 0.0F},
                                       {0.06F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-integrated-distance",
                                              path,
                                              MotionMode::Queue,
                                              4))
              .ok());

  loop.tick();
  const StatusSnapshot snapshot = advanceThroughFirstLocoMotionWrite(loop, store);

  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.radius_source == "integrated");
  REQUIRE(snapshot.exec->loco.distance_m == Catch::Approx(0.02).margin(1.0e-5));
}

TEST_CASE("RuntimeControlLoop loco_upper radius source reflects per-frame estimator") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample high;
  high.fresh = true;
  high.position = {10.0F, 20.0F, 0.0F};
  robot.high_state = high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_radius_source_switch.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.02F, 0.0F, 0.0F},
                                       {0.04F, 0.0F, 0.0F},
                                       {0.06F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-radius-source-switch",
                                              path,
                                              MotionMode::Queue,
                                              4))
              .ok());

  loop.tick();
  robot.high_state->position = {10.02F, 20.0F, 0.0F};
  StatusSnapshot snapshot = advanceThroughFirstLocoMotionWrite(loop, store);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.radius_source == "highstate");

  robot.high_state->position = {
      std::numeric_limits<float>::quiet_NaN(), 20.0F, 0.0F};
  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Motion);
  REQUIRE(snapshot.exec->loco.radius_source == "integrated");
}

TEST_CASE("RuntimeControlLoop strict_pose rejects loco_upper start without fresh highstate") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.loco_upper_strict_pose = true;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_strict_missing_start.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-strict-missing-start", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(loco_lower_policy.calls == 0);
  const auto failed = store.findRun("loco-strict-missing-start");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::SafetyLimitTriggered);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(failed.run->loco.reason == LocoReason::PoseMissing);
}

TEST_CASE("RuntimeControlLoop strict_pose aborts loco_upper when highstate becomes stale") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.loco_upper_strict_pose = true;
  config.loco_upper_pose_fresh_timeout_ms = 10;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample high;
  high.fresh = true;
  high.age_ms = 0;
  high.position = {10.0F, 20.0F, 0.0F};
  robot.high_state = high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_strict_stale_runtime.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-strict-stale-runtime", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  robot.high_state->age_ms = 11;

  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.err == ErrorCode::SafetyLimitTriggered);
  REQUIRE(snapshot.block == "pose_missing");
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(loco_lower_policy.calls == 0);
  const auto failed = store.findRun("loco-strict-stale-runtime");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::SafetyLimitTriggered);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(failed.run->loco.reason == LocoReason::PoseMissing);
}

TEST_CASE("RuntimeControlLoop strict_pose aborts loco_upper on highstate jump") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.loco_upper_strict_pose = true;
  config.loco_upper_pose_jump_reject_m = 0.05;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample high;
  high.fresh = true;
  high.age_ms = 0;
  high.position = {10.0F, 20.0F, 0.0F};
  robot.high_state = high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_strict_jump_runtime.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-strict-jump-runtime", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  robot.high_state->position = {10.2F, 20.0F, 0.0F};

  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.err == ErrorCode::SafetyLimitTriggered);
  REQUIRE(snapshot.block == "pose_jump");
  REQUIRE(robot.write_attempts == 0);
  REQUIRE(loco_lower_policy.calls == 0);
  const auto failed = store.findRun("loco-strict-jump-runtime");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::SafetyLimitTriggered);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(failed.run->loco.reason == LocoReason::PoseJump);
}

TEST_CASE("RuntimeControlLoop loco_upper command clamp marks envelope sticky") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.command_ranges.lin_vel_x = {-0.05, 0.05};
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_command_clamp.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.02F, 0.0F, 0.0F},
                                       {0.04F, 0.0F, 0.0F},
                                       {0.06F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-command-clamp",
                                              path,
                                              MotionMode::Queue,
                                              4))
              .ok());

  loop.tick();
  StatusSnapshot snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.exec->loco.envelope_clamped);

  snapshot = advanceThroughFirstLocoMotionWrite(loop, store);

  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(6) ==
          Catch::Approx(0.05F));
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.envelope_clamped);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.envelope_clamped);
}

TEST_CASE("RuntimeControlLoop loco_upper clamps processed lower q to joint limits and reports telemetry") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.joint_min_q = std::vector<double>(kLocoLowerPolicyJointDim, -0.2);
  loco_lower_config.joint_max_q = std::vector<double>(kLocoLowerPolicyJointDim, 0.2);
  loco_lower_config.action_scale = std::vector<double>(kLocoLowerPolicyJointDim, 0.5);
  loco_lower_config.action_offset = std::vector<double>(kLocoLowerPolicyJointDim, 0.0);
  LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  composer_config.lower_min_q.assign(kLocoLowerPolicyJointDim, -0.2F);
  composer_config.lower_max_q.assign(kLocoLowerPolicyJointDim, 0.2F);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 1.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config);

  const auto path = identityQuaternionTrk(tmp, "loco_lower_action_clamp.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-lower-action-clamp", path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  const StatusSnapshot snapshot = advanceThroughFirstLocoMotionWrite(loop, store);

  REQUIRE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.exec->loco.raw_action_clamped);
  REQUIRE(snapshot.exec->loco.lower_q_limited);
  REQUIRE(snapshot.exec->loco.lower_action_clamped);
  REQUIRE(loco_lower_policy.calls > 0);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    const auto slot = static_cast<std::size_t>(loco_lower_config.sdk_joint_ids_map.at(i));
    REQUIRE(robot.writes.back().motors.at(slot).q == Catch::Approx(0.2F));
  }
}

TEST_CASE("RuntimeControlLoop loco_upper planner limits are applied before lower policy input") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.001;
  config.loco_upper_smoothing_window_frames = 2;
  config.loco_upper_max_lin_accel_mps2 = 10000.0;
  config.loco_upper_max_yaw_accel_radps2 = 10000.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.command_ranges.lin_vel_x = {-100.0, 100.0};
  loco_lower_config.command_ranges.lin_vel_y = {-100.0, 100.0};
  loco_lower_config.command_ranges.ang_vel_z = {-100.0, 100.0};
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_runtime_planner_limits.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.0F, 0.0F, 0.0F},
                                       {1.0F, 0.0F, 0.0F},
                                       {2.0F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-runtime-planner-limits", path, MotionMode::Queue, 4))
              .ok());

  bool saw_smoothed_command = false;
  for (int i = 0; i < 32; ++i) {
    loop.tick();
    if (loco_lower_policy.inputs_seen.empty()) {
      continue;
    }
    const Vec& obs = loco_lower_policy.inputs_seen.back().obs;
    if (obs.at(6) == Catch::Approx(25.0F).margin(1.0e-5F)) {
      saw_smoothed_command = true;
      break;
    }
  }

  REQUIRE(saw_smoothed_command);
}

TEST_CASE("RuntimeControlLoop loco_upper keeps forward root motion in body frame") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.001;
  config.loco_upper_smoothing_window_frames = 1;
  config.loco_upper_max_lin_accel_mps2 = 10000.0;
  config.loco_upper_max_yaw_accel_radps2 = 10000.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.command_ranges.lin_vel_x = {-100.0, 100.0};
  loco_lower_config.command_ranges.lin_vel_y = {-100.0, 100.0};
  loco_lower_config.command_ranges.ang_vel_z = {-100.0, 100.0};
  LowStateSample low_state = readyLowState(deploy_config);
  low_state.quat_wxyz = yawQuat(3.14159265359F);
  FakeRobotIO robot(low_state);
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_runtime_body_forward.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.0F, 0.0F, 0.0F},
                                       {1.0F, 0.0F, 0.0F},
                                       {2.0F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-runtime-body-forward", path, MotionMode::Queue, 4))
              .ok());

  bool saw_forward_body_command = false;
  for (int i = 0; i < 64; ++i) {
    loop.tick();
    if (loco_lower_policy.inputs_seen.empty()) {
      continue;
    }
    const Vec& obs = loco_lower_policy.inputs_seen.back().obs;
    if (std::abs(obs.at(6)) <= 1.0e-5F) {
      continue;
    }
    REQUIRE(obs.at(6) > 0.0F);
    saw_forward_body_command = true;
    break;
  }

  REQUIRE(saw_forward_body_command);
}

TEST_CASE("RuntimeControlLoop loco_upper planner linear acceleration clamp sets telemetry") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.001;
  config.loco_upper_smoothing_window_frames = 1;
  config.loco_upper_max_lin_accel_mps2 = 1.0;
  config.loco_upper_max_yaw_accel_radps2 = 1000.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.command_ranges.lin_vel_x = {-100.0, 100.0};
  loco_lower_config.command_ranges.lin_vel_y = {-100.0, 100.0};
  loco_lower_config.command_ranges.ang_vel_z = {-100.0, 100.0};
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_runtime_planner_lin_accel_telemetry.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.0F, 0.0F, 0.0F},
                                       {1.0F, 0.0F, 0.0F},
                                       {2.0F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-runtime-planner-lin-accel", path, MotionMode::Queue, 4))
              .ok());

  bool saw_clamped_command = false;
  StatusSnapshot snapshot;
  for (int i = 0; i < 64; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (!snapshot.exec.has_value() || loco_lower_policy.inputs_seen.empty()) {
      continue;
    }
    if (snapshot.exec->loco.envelope_clamped &&
        loco_lower_policy.inputs_seen.back().obs.at(6) ==
            Catch::Approx(0.02F).margin(1.0e-5F)) {
      saw_clamped_command = true;
      break;
    }
  }

  REQUIRE(saw_clamped_command);
}

TEST_CASE("RuntimeControlLoop loco_upper planner yaw acceleration clamp sets telemetry") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.001;
  config.loco_upper_smoothing_window_frames = 1;
  config.loco_upper_max_lin_accel_mps2 = 1000.0;
  config.loco_upper_max_yaw_accel_radps2 = 1.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.command_ranges.lin_vel_x = {-100.0, 100.0};
  loco_lower_config.command_ranges.lin_vel_y = {-100.0, 100.0};
  loco_lower_config.command_ranges.ang_vel_z = {-100.0, 100.0};
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = yawOnlyTrk(tmp,
                               "loco_runtime_planner_yaw_accel_telemetry.trk",
                               {0.0F, 0.0F, 1.0F, 2.0F});
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-runtime-planner-yaw-accel", path, MotionMode::Queue, 4))
              .ok());

  bool saw_clamped_command = false;
  StatusSnapshot snapshot;
  for (int i = 0; i < 64; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (!snapshot.exec.has_value() || loco_lower_policy.inputs_seen.empty()) {
      continue;
    }
    if (snapshot.exec->loco.envelope_clamped &&
        loco_lower_policy.inputs_seen.back().obs.at(8) ==
            Catch::Approx(0.02F).margin(1.0e-5F)) {
      saw_clamped_command = true;
      break;
    }
  }

  REQUIRE(saw_clamped_command);
}

TEST_CASE("RuntimeControlLoop loco_upper starts from StandbyVelocity without GeneralTracker transition and composes LowCmd") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  const auto standby_track = validStandbyTrack(tmp, "loco_standby_start_ref.trk");
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocity_config,
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config,
                                  nullptr,
                                  standby_track);

  const auto path = identityQuaternionTrk(tmp, "loco_standby_motion.trk", 4);
  const auto loaded = loadTrack(tmp.trkConfig(), path);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-standby", path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.active.id == "loco-standby");
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->executor == MotionExecutor::LocoUpper);
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Entry);

  bool saw_motion = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Motion &&
        !loco_lower_policy.inputs_seen.empty()) {
      const Vec& motion_obs = loco_lower_policy.inputs_seen.back().obs;
      if (motion_obs.at(6) == 0.0F && motion_obs.at(7) == 0.0F &&
          motion_obs.at(8) == 0.0F) {
        continue;
      }
      saw_motion = true;
      break;
    }
  }
  REQUIRE(saw_motion);
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(velocity_policy.calls == 0);
  REQUIRE(loco_lower_policy.calls > 0);
  REQUIRE_FALSE(robot.writes.empty());
  requireLocoLowerFrame(robot.writes.back(), loco_lower_config, loco_lower_policy.next_raw);
  requireLocoUpperFrame(robot.writes.back(), composer_config, *loaded, snapshot.exec->frame);
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.size() == kLocoLowerPolicyObsDim);
  const Vec& obs = loco_lower_policy.inputs_seen.back().obs;
  REQUIRE((obs.at(6) != 0.0F || obs.at(7) != 0.0F || obs.at(8) != 0.0F));
}

TEST_CASE("RuntimeControlLoop loco_upper writes compiled position-clamped upper targets") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  composer_config.upper_max_q.at(12) = 1.0F;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  locoLowerDeployConfig(),
                                  composer_config);

  std::vector<Vec> frames(3, Vec(kPolicyJointDim, 0.0F));
  for (Vec& frame : frames) {
    frame.at(12) = 5.0F;
  }
  const auto path = jointPosTrk(tmp, "loco_upper_position_clamped.trk", frames);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-upper-position-clamped", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  const StatusSnapshot snapshot = advanceThroughFirstLocoMotionWrite(loop, store);

  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->loco.upper_clamped);
  REQUIRE_FALSE(snapshot.exec->loco.upper_rate_limited);
  REQUIRE_FALSE(robot.writes.empty());
  REQUIRE(locoUpperWrittenQ(robot.writes.back(), composer_config, 12) ==
          Catch::Approx(1.0F).margin(1.0e-5F));
}

TEST_CASE("RuntimeControlLoop loco_upper writes compiled rate-limited upper targets") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  composer_config.upper_max_vel_radps.at(12) = 1.0F;
  composer_config.upper_max_accel_radps2.at(12) = 10000.0F;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  locoLowerDeployConfig(),
                                  composer_config);

  std::vector<Vec> frames(3, Vec(kPolicyJointDim, 0.0F));
  frames.at(1).at(12) = 10.0F;
  frames.at(2).at(12) = 10.0F;
  const auto path = jointPosTrk(tmp, "loco_upper_rate_limited.trk", frames);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-upper-rate-limited", path, MotionMode::Queue, 3))
              .ok());

  bool saw_frame_one = false;
  StatusSnapshot snapshot;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Motion &&
        snapshot.exec->frame == 1 && !robot.writes.empty()) {
      saw_frame_one = true;
      break;
    }
  }

  REQUIRE(saw_frame_one);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.exec->loco.upper_clamped);
  REQUIRE(snapshot.exec->loco.upper_rate_limited);
  REQUIRE(locoUpperWrittenQ(robot.writes.back(), composer_config, 12) ==
          Catch::Approx(0.02F).margin(1.0e-5F));
}

TEST_CASE("RuntimeControlLoop loco_upper hold and stopping use compiled upper fallback") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  config.loco_upper_max_hold_s = 0.10;
  const DeployConfig deploy_config = deployConfig();
  LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  composer_config.upper_max_q.at(12) = 1.0F;

  SECTION("final hold uses compiled final frame") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
    auto loop = makeLocoControlLoop(config,
                                    bridge,
                                    store,
                                    tmp.trkConfig(),
                                    robot,
                                    tracker_policy,
                                    velocity_policy,
                                    loco_lower_policy,
                                    deploy_config,
                                    velocityDeployConfig(),
                                    fixStandConfig(),
                                    ControlMode::StandbyVelocity,
                                    passiveConfig(),
                                    locoLowerDeployConfig(),
                                    composer_config);

    std::vector<Vec> frames(2, Vec(kPolicyJointDim, 0.0F));
    frames.at(1).at(12) = 5.0F;
    const auto path = jointPosTrk(tmp, "loco_upper_hold_compiled.trk", frames);
    REQUIRE(bridge.submitQueue(
                locoUpperCommand("loco-upper-hold-compiled",
                                 path,
                                 MotionMode::Queue,
                                 2,
                                 true))
                .ok());

    bool saw_holding_write = false;
    StatusSnapshot snapshot;
    for (int i = 0; i < 128; ++i) {
      loop.tick();
      snapshot = store.snapshot();
      if (snapshot.exec && snapshot.exec->state == MotionState::Holding &&
          snapshot.exec->loco.phase == LocoPhase::Holding && !robot.writes.empty()) {
        saw_holding_write = true;
        break;
      }
    }

    REQUIRE(saw_holding_write);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->loco.upper_clamped);
    REQUIRE(locoUpperWrittenQ(robot.writes.back(), composer_config, 12) ==
            Catch::Approx(1.0F).margin(1.0e-5F));
  }

  SECTION("stopping after motion uses compiled last frame") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
    auto loop = makeLocoControlLoop(config,
                                    bridge,
                                    store,
                                    tmp.trkConfig(),
                                    robot,
                                    tracker_policy,
                                    velocity_policy,
                                    loco_lower_policy,
                                    deploy_config,
                                    velocityDeployConfig(),
                                    fixStandConfig(),
                                    ControlMode::StandbyVelocity,
                                    passiveConfig(),
                                    locoLowerDeployConfig(),
                                    composer_config);

    std::vector<Vec> frames(3, Vec(kPolicyJointDim, 0.0F));
    frames.at(0).at(12) = 5.0F;
    frames.at(1).at(12) = 5.0F;
    const auto path = jointPosTrk(tmp, "loco_upper_stop_compiled.trk", frames);
    REQUIRE(bridge.submitQueue(
                locoUpperCommand("loco-upper-stop-compiled", path, MotionMode::Queue, 3))
                .ok());

    StatusSnapshot snapshot = advanceThroughFirstLocoMotionWrite(loop, store);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->loco.upper_clamped);
    REQUIRE(locoUpperWrittenQ(robot.writes.back(), composer_config, 12) ==
            Catch::Approx(1.0F).margin(1.0e-5F));
    REQUIRE(bridge.standbyVelocity().ok());

    bool saw_stopping_write = false;
    for (int i = 0; i < 128; ++i) {
      loop.tick();
      snapshot = store.snapshot();
      const auto run = store.findRun("loco-upper-stop-compiled");
      if (run.ok() && run.run && run.run->state == MotionState::Stopping &&
          run.run->loco.phase == LocoPhase::Stopping && !robot.writes.empty()) {
        saw_stopping_write = true;
        break;
      }
    }

    REQUIRE(saw_stopping_write);
    REQUIRE(snapshot.active.kind == ActiveKind::Transition);
    REQUIRE(snapshot.transition.active);
    REQUIRE(snapshot.transition.target == "standby");
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(store.findRun("loco-upper-stop-compiled").run->loco.upper_clamped);
    REQUIRE(locoUpperWrittenQ(robot.writes.back(), composer_config, 12) <= 1.0F);
  }
}

TEST_CASE("RuntimeControlLoop loco_upper keeps root commands aligned to frame count") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = rootPositionsTrk(tmp,
                                     "loco_upper_aligned_terminal_zero.trk",
                                     {{{0.0F, 0.0F, 0.0F},
                                       {0.02F, 0.0F, 0.0F},
                                       {0.04F, 0.0F, 0.0F}}});
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-upper-aligned-terminal-zero",
                               path,
                               MotionMode::Queue,
                               3))
              .ok());

  bool saw_terminal_frame = false;
  StatusSnapshot snapshot;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->frame == 2) {
      saw_terminal_frame = true;
      break;
    }
  }

  REQUIRE(saw_terminal_frame);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->frame == 2);
  REQUIRE(snapshot.exec->frames == 3);
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(6) ==
          Catch::Approx(0.0F).margin(1.0e-5F));
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(7) ==
          Catch::Approx(0.0F).margin(1.0e-5F));
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(8) ==
          Catch::Approx(0.0F).margin(1.0e-5F));
}

TEST_CASE("RuntimeControlLoop loco_upper entry handoff starts lower target from current q") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  const LowStateSample entry_low_state = readyLowState(deploy_config);
  FakeRobotIO robot(entry_low_state);
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = identityQuaternionTrk(tmp, "loco_entry_lower_handoff.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-entry-lower-handoff", path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.writes.empty());

  loop.tick();
  REQUIRE(loco_lower_policy.calls == 1);
  REQUIRE(robot.writes.size() == 1);
  requireLocoLowerEntryHandoffFrame(robot.writes.back(),
                                    loco_lower_config,
                                    entry_low_state,
                                    loco_lower_policy.next_raw,
                                    0.0F);
  const Vec first_obs = loco_lower_policy.inputs_seen.back().obs;

  loop.tick();
  REQUIRE(loco_lower_policy.calls == 2);
  REQUIRE(robot.writes.size() == 2);
  requireLocoLowerEntryHandoffFrame(robot.writes.back(),
                                    loco_lower_config,
                                    entry_low_state,
                                    loco_lower_policy.next_raw,
                                    0.5F);
  REQUIRE(first_obs.at(6) == 0.0F);
  REQUIRE(first_obs.at(7) == 0.0F);
  REQUIRE(first_obs.at(8) == 0.0F);
  for (std::size_t i = 0; i < kLocoLowerPolicyJointDim; ++i) {
    REQUIRE(first_obs.at(kLocoLowerPolicyObsRowWidth - kLocoLowerPolicyJointDim + i) ==
            0.0F);
  }
}

TEST_CASE("RuntimeControlLoop loco_upper lower controller uses deploy step_dt cadence") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 1000.0;
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.policy_decimation = 10;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = identityQuaternionTrk(tmp, "loco_runtime_tick_lower.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-runtime-tick-lower", path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.write_attempts == 0);

  for (int i = 0; i < 10; ++i) {
    loop.tick();
  }

  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.write_attempts == 10);
  REQUIRE(loco_lower_policy.calls == 1);
  requireLocoLowerEntryHandoffFrame(robot.writes.back(),
                                    loco_lower_config,
                                    *robot.low_state,
                                    loco_lower_policy.next_raw,
                                    9.0F / 39.0F);

  for (int i = 0; i < 10; ++i) {
    loop.tick();
  }

  REQUIRE(robot.write_attempts == 20);
  REQUIRE(loco_lower_policy.calls == 1);

  loop.tick();
  REQUIRE(robot.write_attempts == 21);
  REQUIRE(loco_lower_policy.calls == 2);
}

TEST_CASE("RuntimeControlLoop loco_upper lower controller preserves fractional step_dt cadence") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 60.0;
  config.transition_duration_s = 2.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.step_dt = 0.02;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = identityQuaternionTrk(tmp, "loco_fractional_lower_step_dt.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-fractional-lower-step-dt",
                               path,
                               MotionMode::Queue,
                               4))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.write_attempts == 0);

  loop.tick();
  REQUIRE(robot.write_attempts == 1);
  REQUIRE(loco_lower_policy.calls == 1);

  loop.tick();
  REQUIRE(robot.write_attempts == 2);
  REQUIRE(loco_lower_policy.calls == 1);

  for (int i = 0; i < 4; ++i) {
    loop.tick();
  }
  REQUIRE(robot.write_attempts == 6);
  REQUIRE(loco_lower_policy.calls == 5);

  for (int i = 0; i < 54; ++i) {
    loop.tick();
  }
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.write_attempts == 60);
  REQUIRE(loco_lower_policy.calls == 50);
}

TEST_CASE("RuntimeControlLoop loco_upper motion writes every runtime tick while frame advances at track fps") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 1000.0;
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.policy_decimation = 10;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = identityQuaternionTrk(tmp, "loco_runtime_tick_motion.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-runtime-tick-motion", path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Motion);
  REQUIRE(robot.write_attempts == 1);

  loop.tick();
  REQUIRE(store.snapshot().exec->frame == 0);
  REQUIRE(robot.write_attempts == 2);

  for (int i = 0; i < 19; ++i) {
    loop.tick();
  }

  REQUIRE(store.snapshot().exec->frame == 0);
  REQUIRE(robot.write_attempts == 21);
  REQUIRE(loco_lower_policy.calls == 2);

  loop.tick();
  REQUIRE(store.snapshot().exec->frame == 1);
  REQUIRE(robot.write_attempts == 22);
}

TEST_CASE("RuntimeControlLoop loco_upper naturally exits and publishes done") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_done.trk", 2);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-done", path, MotionMode::Queue, 2))
              .ok());

  bool saw_exit = false;
  bool saw_done = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Exit) {
      saw_exit = true;
    }
    const auto found = store.findRun("loco-done");
    if (found.ok() && found.run && found.run->state == MotionState::Done) {
      saw_done = true;
      break;
    }
  }

  REQUIRE(saw_exit);
  REQUIRE(saw_done);
  REQUIRE(store.snapshot().active.kind == ActiveKind::None);
  const auto done = store.findRun("loco-done");
  REQUIRE(done.run->executor == MotionExecutor::LocoUpper);
  REQUIRE(done.run->loco.phase == LocoPhase::Done);
  REQUIRE(done.run->frame == 1);
  REQUIRE(done.run->progress == 1.0);
}

TEST_CASE("RuntimeControlLoop loco_upper hold times out through bounded exit") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  config.loco_upper_max_hold_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  locoLowerDeployConfig(),
                                  composer_config);

  const auto path = identityQuaternionTrk(tmp, "loco_hold_timeout.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-hold-timeout", path, MotionMode::Queue, 3, true))
              .ok());

  bool saw_exit = false;
  bool saw_done = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Exit) {
      saw_exit = true;
    }
    const auto found = store.findRun("loco-hold-timeout");
    if (found.ok() && found.run && found.run->state == MotionState::Done) {
      saw_done = true;
      break;
    }
  }

  REQUIRE(saw_exit);
  REQUIRE(saw_done);
  const auto done = store.findRun("loco-hold-timeout");
  REQUIRE(done.run->err == ErrorCode::Ok);
  REQUIRE(done.run->state == MotionState::Done);
  REQUIRE(done.run->loco.phase == LocoPhase::Done);
  REQUIRE(done.run->loco.reason == LocoReason::HoldTimeout);
  REQUIRE(done.run->stop_reason == StopReason::None);
}

TEST_CASE("RuntimeControlLoop loco_upper hold holds final upper target with zero lower command") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config);

  const auto path = identityQuaternionTrk(tmp, "loco_hold.trk", 3);
  const auto loaded = loadTrack(tmp.trkConfig(), path);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-hold", path, MotionMode::Queue, 3, true))
              .ok());

  bool saw_holding = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->state == MotionState::Holding &&
        snapshot.exec->loco.phase == LocoPhase::Holding && robot.write_attempts > 0) {
      saw_holding = true;
      break;
    }
  }
  REQUIRE(saw_holding);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Holding);
  REQUIRE(snapshot.exec->frame == 2);
  requireLocoUpperFrame(robot.writes.back(), composer_config, *loaded, 2);
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(6) == 0.0F);
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(7) == 0.0F);
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(8) == 0.0F);
}

TEST_CASE("RuntimeControlLoop loco_upper records holding radius limit without Passive") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  config.radius_tolerance_m = 0.05;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  HighStateSample high;
  high.fresh = true;
  high.position = {2.0F, 3.0F, 0.0F};
  robot.high_state = high;
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_hold_radius_guard.trk", 3);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-hold-radius-guard",
                                              path,
                                              MotionMode::Queue,
                                              3,
                                              true,
                                              0.5))
              .ok());

  bool reached_holding = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Holding) {
      reached_holding = true;
      break;
    }
  }
  REQUIRE(reached_holding);

  robot.high_state->position = {2.6F, 3.0F, 0.0F};
  const int writes_before_breach = robot.write_attempts;
  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::LocoUpperActive);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Holding);
  REQUIRE(snapshot.exec->loco.radius_limit_reached);
  REQUIRE(snapshot.exec->loco.reason == LocoReason::None);
  REQUIRE(snapshot.exec->loco.distance_m == Catch::Approx(0.6).margin(1.0e-5));
  REQUIRE(robot.write_attempts > writes_before_breach);
}

TEST_CASE("RuntimeControlLoop loco_upper lower policy NaN maps compact terminal reason") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(
      Vec(kLocoLowerPolicyJointDim, std::numeric_limits<float>::quiet_NaN()));
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.policy_decimation = 1;
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = identityQuaternionTrk(tmp, "loco_policy_nan.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-policy-nan", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");
  const auto failed = store.findRun("loco-policy-nan");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::ModelInferenceFailed);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(failed.run->loco.reason == LocoReason::PolicyNan);
}

TEST_CASE("RuntimeControlLoop loco_upper lower policy throw maps compact terminal reason") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  loco_lower_policy.throw_on_infer = true;
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.policy_decimation = 1;
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path = identityQuaternionTrk(tmp, "loco_policy_infer.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-policy-infer", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");
  const auto failed = store.findRun("loco-policy-infer");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::ModelInferenceFailed);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(failed.run->loco.reason == LocoReason::PolicyInfer);
}

TEST_CASE("RuntimeControlLoop loco_upper lower policy non-finite exception maps compact terminal reason") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.001;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  loco_lower_policy.throw_on_infer = true;
  loco_lower_policy.throw_message =
      "policy runtime error: actions output contains non-finite values";
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.policy_decimation = 1;
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config);

  const auto path =
      identityQuaternionTrk(tmp, "loco_policy_non_finite_throw.trk", 3);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-policy-non-finite-throw",
                                              path,
                                              MotionMode::Queue,
                                              3))
              .ok());

  loop.tick();
  loop.tick();

  const StatusSnapshot snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.ctrl == ControllerState::Fault);
  REQUIRE(snapshot.err == ErrorCode::ModelInferenceFailed);
  REQUIRE(snapshot.block == "policy_inference_failed");
  const auto failed = store.findRun("loco-policy-non-finite-throw");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->err == ErrorCode::ModelInferenceFailed);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(failed.run->loco.reason == LocoReason::PolicyNan);
}

TEST_CASE("RuntimeControlLoop loco_upper exit interpolates upper toward standby before standby velocity") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.06;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  const std::size_t logical_joint = composer_config.upper_start_joint;
  const std::size_t sdk_slot =
      static_cast<std::size_t>(composer_config.logical_to_sdk.at(logical_joint));
  composer_config.unowned_safe.q = 2.25F;
  FixStandConfig fixstand_config = fixStandConfig();
  fixstand_config.target_q.at(sdk_slot) = -1.5;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixstand_config,
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  locoLowerDeployConfig(),
                                  composer_config);

  const auto path = identityQuaternionTrk(tmp, "loco_exit_to_standby.trk", 2);
  const auto loaded = loadTrack(tmp.trkConfig(), path);
  const auto final_frame = loaded->frame(1);
  REQUIRE(final_frame.has_value());
  const float final_q = final_frame->joint_pos.ptr[logical_joint];
  const float target_q = static_cast<float>(fixstand_config.target_q.at(sdk_slot));
  REQUIRE(final_q != target_q);
  REQUIRE(target_q != composer_config.unowned_safe.q);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-exit-standby", path, MotionMode::Queue, 2))
              .ok());

  bool saw_mid_exit = false;
  bool saw_done = false;
  float mid_exit_q = final_q;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Exit &&
        !robot.writes.empty()) {
      const float q = robot.writes.back().motors.at(sdk_slot).q;
      if (q != Catch::Approx(final_q).margin(1.0e-5F) &&
          q != Catch::Approx(target_q).margin(1.0e-5F)) {
        saw_mid_exit = true;
        mid_exit_q = q;
      }
    }
    const auto found = store.findRun("loco-exit-standby");
    if (found.ok() && found.run && found.run->state == MotionState::Done) {
      saw_done = true;
      break;
    }
  }
  REQUIRE(saw_mid_exit);
  REQUIRE(std::fabs(mid_exit_q - target_q) < std::fabs(final_q - target_q));
  REQUIRE(saw_done);
  REQUIRE(robot.writes.back().motors.at(sdk_slot).q ==
          Catch::Approx(target_q).margin(1.0e-5F));

  const int writes_before_standby = robot.write_attempts;
  loop.tick();
  REQUIRE(robot.write_attempts > writes_before_standby);
  REQUIRE(robot.writes.back().motors.at(sdk_slot).q ==
          Catch::Approx(target_q).margin(1.0e-5F));
}

TEST_CASE("RuntimeControlLoop loco_upper standby uses public standby transition status") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_standby_stop.trk", 20);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-standby-stop", path, MotionMode::Queue, 20))
              .ok());
  bool saw_motion = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Motion) {
      saw_motion = true;
      break;
    }
  }
  REQUIRE(saw_motion);

  REQUIRE(bridge.standby().ok());
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::LocoUpperActive);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE_FALSE(snapshot.exec.has_value());

  bool saw_stopped = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    const auto found = store.findRun("loco-standby-stop");
    if (found.ok() && found.run && found.run->state == MotionState::Stopped) {
      saw_stopped = true;
      break;
    }
  }
  REQUIRE(saw_stopped);
  REQUIRE(store.findRun("loco-standby-stop").run->loco.phase == LocoPhase::Stopped);
  REQUIRE(store.findRun("loco-standby-stop").run->stop_reason == StopReason::Stop);
  REQUIRE(store.snapshot().active.kind == ActiveKind::None);
}

TEST_CASE("RuntimeControlLoop urgent_stop aborts loco_upper active run") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.08;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.35F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocity_config);

  const auto path = identityQuaternionTrk(tmp, "loco_stop.trk", 20);
  REQUIRE(bridge.submitQueue(locoUpperCommand("loco-stop", path, MotionMode::Queue, 20))
              .ok());
  bool saw_motion = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Motion) {
      saw_motion = true;
      break;
    }
  }
  REQUIRE(saw_motion);

  REQUIRE(bridge.configureIdle({idleMotion(validTrk(tmp, "loco_urgent_idle.trk", 3), 3)})
              .ok());
  REQUIRE(store.snapshot().idle.enabled);

  const int lower_calls_before_stop = loco_lower_policy.calls;
  const int writes_before_stop = robot.write_attempts;
  REQUIRE(bridge.urgentStop().state == ControllerState::UrgentStopping);
  REQUIRE_FALSE(store.snapshot().idle.enabled);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Stopping);
  REQUIRE(snapshot.ctrl == ControllerState::UrgentStopping);
  REQUIRE(snapshot.stop_reason == StopReason::UrgentStop);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(loco_lower_policy.calls == lower_calls_before_stop);
  REQUIRE(robot.write_attempts == writes_before_stop);
  REQUIRE(store.findRun("loco-stop").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("loco-stop").run->loco.phase == LocoPhase::Canceled);
  REQUIRE(store.findRun("loco-stop").run->stop_reason == StopReason::UrgentStop);
}

TEST_CASE("RuntimeControlLoop loco_upper bad orientation fails through safety path") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(badOrientationLowState(deploy_config, 45));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy;
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto path = identityQuaternionTrk(tmp, "loco_bad_orientation.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-bad-orientation", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  const auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  const auto failed = store.findRun("loco-bad-orientation");
  REQUIRE(failed.ok());
  REQUIRE(failed.run->state == MotionState::Failed);
  REQUIRE(failed.run->executor == MotionExecutor::LocoUpper);
  REQUIRE(failed.run->loco.phase == LocoPhase::Failed);
  REQUIRE(loco_lower_policy.calls == 0);
  REQUIRE(robot.write_attempts == 0);
}

TEST_CASE("RuntimeControlLoop GeneralTracker active run does not use loco lower policy") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.25F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  const auto standby_track = validStandbyTrack(tmp, "general_tracker_loco_runtime_ref.trk");
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  locoLowerDeployConfig(),
                                  locoUpperComposerConfig(),
                                  nullptr,
                                  standby_track);

  const auto path = validTrk(tmp, "general_tracker_not_loco.trk", 3);
  REQUIRE(bridge.submitQueue(executeCommand("general-tracker", path)).ok());
  startQueuedRun(loop, store, "general-tracker", StartQueuedRunMode::AllowStandbyTransition);

  for (int i = 0; i < 8; ++i) {
    loop.tick();
    if (tracker_policy.calls > 0) {
      break;
    }
  }
  REQUIRE(tracker_policy.calls > 0);
  REQUIRE(loco_lower_policy.calls == 0);
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->executor == MotionExecutor::GeneralTracker);
  REQUIRE(robot.write_attempts > 0);
}

TEST_CASE("RuntimeControlLoop active idle starts LocoUpper without GeneralTracker transition") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  for (const MotionMode mode : {MotionMode::Queue, MotionMode::Interrupt}) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
    auto loop = makeLocoControlLoop(config,
                                    bridge,
                                    store,
                                    tmp.trkConfig(),
                                    robot,
                                    tracker_policy,
                                    velocity_policy,
                                    loco_lower_policy,
                                    deploy_config,
                                    velocity_config,
                                    fixstand_config,
                                    ControlMode::StandbyVelocity);

    const auto idle_path =
        validTrk(tmp, std::string("loco_idle_source_") + toString(mode) + ".trk", 3);
    const auto loco_path =
        identityQuaternionTrk(tmp,
                              std::string("loco_from_idle_") + toString(mode) + ".trk",
                              4);
    REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
    loop.tick();
    loop.tick();
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
    const int tracker_calls_before_loco = tracker_policy.calls;

    if (mode == MotionMode::Interrupt) {
      REQUIRE(bridge.submitInterrupt(
                  locoUpperCommand("loco-from-idle", loco_path, mode, 4))
                  .ok());
    } else {
      REQUIRE(bridge.submitQueue(locoUpperCommand("loco-from-idle", loco_path, mode, 4))
                  .ok());
    }
    loop.tick();

    auto snapshot = store.snapshot();
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.active.kind == ActiveKind::User);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == "loco-from-idle");
    REQUIRE(snapshot.exec->executor == MotionExecutor::LocoUpper);

    bool saw_loco_lower = false;
    for (int i = 0; i < 128; ++i) {
      loop.tick();
      snapshot = store.snapshot();
      if (snapshot.exec && snapshot.exec->executor == MotionExecutor::LocoUpper &&
          loco_lower_policy.calls > 0) {
        saw_loco_lower = true;
        break;
      }
    }
    REQUIRE(saw_loco_lower);
    REQUIRE(tracker_policy.calls == tracker_calls_before_loco);
  }
}

TEST_CASE("RuntimeControlLoop GeneralTracker holding queues LocoUpper without PolicyStepRunner transition") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  const LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                transitionReadyTrk(tmp, "loco_after_gt_hold_standby.trk", 2, 20.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config,
                                  nullptr,
                                  standby_track);

  startHoldingRun(loop,
                  store,
                  bridge,
                  tmp,
                  "gt-held",
                  nullptr,
                  StartQueuedRunMode::AllowStandbyTransition);
  const int tracker_calls_before_loco = tracker_policy.calls;
  const auto loco_path = identityQuaternionTrk(tmp, "loco_after_gt_hold.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-after-gt-hold", loco_path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("gt-held").run->state == MotionState::Done);
  REQUIRE(store.findRun("gt-held").run->stop_reason == StopReason::None);

  bool saw_loco = false;
  for (int i = 0; i < 128; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->id == "loco-after-gt-hold" &&
        snapshot.exec->executor == MotionExecutor::LocoUpper &&
        loco_lower_policy.calls > 0) {
      saw_loco = true;
      break;
    }
  }
  REQUIRE(saw_loco);
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(tracker_policy.calls == tracker_calls_before_loco);
}

TEST_CASE("RuntimeControlLoop LocoUpper holding queues GeneralTracker after loco terminal") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                transitionReadyTrk(tmp, "gt_after_loco_hold_standby.trk", 2, 20.0F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocityDeployConfig(),
                                  fixStandConfig(),
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  locoLowerDeployConfig(),
                                  locoUpperComposerConfig(),
                                  nullptr,
                                  standby_track);

  const auto loco_path = identityQuaternionTrk(tmp, "loco_hold_before_gt.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-held-before-gt", loco_path, MotionMode::Queue, 3, true))
              .ok());
  bool saw_loco_holding = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->id == "loco-held-before-gt" &&
        snapshot.exec->state == MotionState::Holding &&
        snapshot.exec->loco.phase == LocoPhase::Holding) {
      saw_loco_holding = true;
      break;
    }
  }
  REQUIRE(saw_loco_holding);

  const auto gt_path = transitionReadyTrk(tmp, "gt_after_loco_hold.trk", 2, 20.0F);
  REQUIRE(bridge.submitQueue(executeCommand("gt-after-loco-hold", gt_path, MotionMode::Queue, 2))
              .ok());
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "loco-held-before-gt");
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Exit);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"gt-after-loco-hold"});
  REQUIRE(store.findRun("loco-held-before-gt").run->state == MotionState::Holding);

  bool saw_gt = false;
  bool held_done = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    const auto held = store.findRun("loco-held-before-gt");
    held_done = held_done ||
                (held.ok() && held.run && held.run->state == MotionState::Done);
    if (snapshot.exec && snapshot.exec->id == "gt-after-loco-hold" &&
        snapshot.exec->executor == MotionExecutor::GeneralTracker) {
      REQUIRE(held_done);
      saw_gt = true;
      break;
    }
  }
  REQUIRE(saw_gt);
  const auto loco_done = store.findRun("loco-held-before-gt");
  REQUIRE(loco_done.ok());
  REQUIRE(loco_done.run->state == MotionState::Done);
  REQUIRE(loco_done.run->loco.phase == LocoPhase::Done);
}

TEST_CASE("RuntimeControlLoop LocoUpper holding queues LocoUpper after bounded exit") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.1F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy);

  const auto held_path = identityQuaternionTrk(tmp, "loco_hold_before_loco.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-held-before-loco",
                               held_path,
                               MotionMode::Queue,
                               3,
                               true))
              .ok());
  bool saw_loco_holding = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->id == "loco-held-before-loco" &&
        snapshot.exec->state == MotionState::Holding &&
        snapshot.exec->loco.phase == LocoPhase::Holding) {
      saw_loco_holding = true;
      break;
    }
  }
  REQUIRE(saw_loco_holding);

  const auto next_path = identityQuaternionTrk(tmp, "loco_after_loco_hold.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-after-loco-hold",
                               next_path,
                               MotionMode::Queue,
                               3))
              .ok());
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.exec.has_value());
  REQUIRE(snapshot.exec->id == "loco-held-before-loco");
  REQUIRE(snapshot.exec->state == MotionState::Holding);
  REQUIRE(snapshot.exec->loco.phase == LocoPhase::Exit);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"loco-after-loco-hold"});
  REQUIRE(store.findRun("loco-held-before-loco").run->state ==
          MotionState::Holding);

  bool saw_next_loco = false;
  bool held_done = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    const auto held = store.findRun("loco-held-before-loco");
    held_done = held_done ||
                (held.ok() && held.run && held.run->state == MotionState::Done);
    if (snapshot.exec && snapshot.exec->id == "loco-after-loco-hold" &&
        snapshot.exec->executor == MotionExecutor::LocoUpper) {
      REQUIRE(held_done);
      saw_next_loco = true;
      break;
    }
  }
  REQUIRE(saw_next_loco);
  const auto done = store.findRun("loco-held-before-loco");
  REQUIRE(done.ok());
  REQUIRE(done.run->state == MotionState::Done);
  REQUIRE(done.run->loco.phase == LocoPhase::Done);
}

TEST_CASE("RuntimeControlLoop LocoUpper interrupt and FixStand use loco stopping") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  const DeployConfig deploy_config = deployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  SECTION("interrupt stops loco before urgent starts") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.2F));
    auto loop = makeLocoControlLoop(config,
                                    bridge,
                                    store,
                                    tmp.trkConfig(),
                                    robot,
                                    tracker_policy,
                                    velocity_policy,
                                    loco_lower_policy);

    const auto active_path = identityQuaternionTrk(tmp, "loco_interrupt_active.trk", 20);
    REQUIRE(bridge.submitQueue(
                locoUpperCommand("loco-interrupt-active", active_path, MotionMode::Queue, 20))
                .ok());
    bool saw_motion = false;
    for (int i = 0; i < 128; ++i) {
      loop.tick();
      const auto snapshot = store.snapshot();
      if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Motion) {
        saw_motion = true;
        break;
      }
    }
    REQUIRE(saw_motion);

    const auto urgent_path = identityQuaternionTrk(tmp, "loco_interrupt_urgent.trk", 4);
    REQUIRE(bridge.submitInterrupt(
                locoUpperCommand("loco-interrupt-urgent",
                                 urgent_path,
                                 MotionMode::Interrupt,
                                 4))
                .ok());
    loop.tick();
    auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::LocoUpperActive);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == "loco-interrupt-active");
    REQUIRE(snapshot.exec->state == MotionState::Stopping);
    REQUIRE(snapshot.exec->loco.phase == LocoPhase::Stopping);

    bool saw_urgent = false;
    for (int i = 0; i < 256; ++i) {
      loop.tick();
      snapshot = store.snapshot();
      if (snapshot.exec && snapshot.exec->id == "loco-interrupt-urgent" &&
          snapshot.exec->executor == MotionExecutor::LocoUpper) {
        saw_urgent = true;
        break;
      }
    }
    REQUIRE(saw_urgent);
    const auto stopped = store.findRun("loco-interrupt-active");
    REQUIRE(stopped.ok());
    REQUIRE(stopped.run->state == MotionState::Stopped);
    REQUIRE(stopped.run->stop_reason == StopReason::Interrupt);
    REQUIRE(stopped.run->loco.phase == LocoPhase::Stopped);
  }

  SECTION("fixstand immediately stops loco and enters FixStand") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(readyLowState(deploy_config));
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy;
    FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.2F));
    auto loop = makeLocoControlLoop(config,
                                    bridge,
                                    store,
                                    tmp.trkConfig(),
                                    robot,
                                    tracker_policy,
                                    velocity_policy,
                                    loco_lower_policy,
                                    deploy_config,
                                    velocityDeployConfig(),
                                    fixstand_config);

    const auto active_path = identityQuaternionTrk(tmp, "loco_fixstand_active.trk", 20);
    REQUIRE(bridge.submitQueue(
                locoUpperCommand("loco-fixstand-active", active_path, MotionMode::Queue, 20))
                .ok());
    bool saw_motion = false;
    for (int i = 0; i < 128; ++i) {
      loop.tick();
      const auto snapshot = store.snapshot();
      if (snapshot.exec && snapshot.exec->loco.phase == LocoPhase::Motion) {
        saw_motion = true;
        break;
      }
    }
    REQUIRE(saw_motion);

    REQUIRE(bridge.fixStand().ok());
    const int writes_before_fixstand = robot.write_attempts;
    loop.tick();
    auto snapshot = store.snapshot();
    REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
    REQUIRE(snapshot.ctrl == ControllerState::FixStand);
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(robot.write_attempts > writes_before_fixstand);
    REQUIRE_FALSE(robot.writes.empty());
    REQUIRE(robot.writes.back().mode_machine == kExpectedModeMachine);
    REQUIRE(robot.writes.back().motors.at(12).mode == 1);
    REQUIRE(robot.writes.back().motors.at(12).kp ==
            static_cast<float>(fixstand_config.kp.at(12)));
    REQUIRE(robot.writes.back().motors.at(12).kd ==
            static_cast<float>(fixstand_config.kd.at(12)));
    const auto stopped = store.findRun("loco-fixstand-active");
    REQUIRE(stopped.ok());
    REQUIRE(stopped.run->state == MotionState::Stopped);
    REQUIRE(stopped.run->stop_reason == StopReason::Stop);
    REQUIRE(stopped.run->loco.phase == LocoPhase::Stopped);
  }
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
                  transitionReadyTrk(
                      tmp,
                      std::string("standby_source_") + toString(mode) + ".trk",
                      3,
                      0.3F));
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
        transitionReadyTrk(tmp,
                           std::string("standby_first_user_") + toString(mode) + ".trk",
                           2,
                           0.1F);
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

TEST_CASE("RuntimeControlLoop standby no-active near-zero transition failure raw starts user") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
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
    const float endpoint_velocity =
        static_cast<float>(defaultSyntheticTransitionLimits().max_velocity + 1.0);
    const auto standby_track = loadTrack(
        tmp.trkConfig(),
        constantIdentityTrk(tmp,
                            std::string("near_zero_raw_standby_") + toString(mode) + ".trk",
                            2,
                            endpoint_velocity));
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

    const std::string id = std::string("short-raw-") + toString(mode);
    const auto user_path =
        constantIdentityTrk(tmp,
                            std::string("near_zero_raw_user_") + toString(mode) + ".trk",
                            2,
                            endpoint_velocity);
    if (mode == MotionMode::Interrupt) {
      REQUIRE(bridge.submitInterrupt(executeCommand(id, user_path, mode, 2)).ok());
    } else {
      REQUIRE(bridge.submitQueue(executeCommand(id, user_path, mode, 2)).ok());
    }

    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::User);
    REQUIRE(snapshot.active.id == id);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == id);
    REQUIRE(store.findRun(id).run->state != MotionState::Failed);
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::User);
    REQUIRE(store.snapshot().exec.has_value());
    REQUIRE(store.snapshot().exec->id == id);
  }
}

TEST_CASE("RuntimeControlLoop standby no-active yaw residual gate blocks raw start fallback") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  const auto standby_track = validStandbyTrack(tmp, "standby_yaw_residual_ref.trk");
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

  const auto user_path = validTrk(tmp, "standby_yaw_residual_user.trk", 2);
  REQUIRE(bridge.submitQueue(
              executeCommand("standby-yaw-residual-user",
                             user_path,
                             MotionMode::Queue,
                             2))
              .ok());
  loop.forceNextRootYawResidualForTest(0.06);
  loop.tick();

  requireStandbyGateFailure(loop,
                            store,
                            reference,
                            robot,
                            tracker_policy,
                            velocity_policy,
                            "standby-yaw-residual-user",
                            ErrorCode::InternalError);
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

TEST_CASE("RuntimeControlLoop standby no-active unsafe transition failure does not raw start") {
  TempTree tmp;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();

  struct Case {
    const char* id;
    std::filesystem::path (*make_path)(TempTree&, const std::string&);
  };

  const auto high_endpoint_velocity = [](TempTree& tmp,
                                         const std::string& name) {
    return jointVelocityTrk(
        tmp,
        name,
        2,
        static_cast<float>(defaultSyntheticTransitionLimits().max_velocity + 1.0));
  };
  const auto high_body_angular_velocity = [](TempTree& tmp,
                                             const std::string& name) {
    return bodyAngularVelocityTrk(
        tmp,
        name,
        2,
        static_cast<float>(defaultSyntheticTransitionLimits().max_velocity + 1.0));
  };
  const auto large_joint_gap = [](TempTree& tmp, const std::string& name) {
    const Vec large_gap(kPolicyJointDim, 1000.0F);
    return jointPosTrk(tmp, name, {large_gap, large_gap});
  };

  for (const auto& item : std::vector<Case>{
           {"standby-high-endpoint-velocity", high_endpoint_velocity},
           {"standby-high-body-angular-velocity", high_body_angular_velocity},
           {"standby-large-gap", large_joint_gap},
       }) {
    RuntimeConfig config = runtimeConfig();
    config.transition_duration_s = 0.04;
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

    const auto user_path = item.make_path(tmp, std::string(item.id) + ".trk");
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
                              ErrorCode::InternalError);
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

  const auto idle_a = transitionReadyTrk(tmp, "idle_to_idle_a.trk", 2, 0.1F);
  const auto idle_b = transitionReadyTrk(tmp, "idle_to_idle_b.trk", 2, 0.3F);
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
      REQUIRE(snapshot.idle.enabled);
      REQUIRE(snapshot.idle.n == 2);
      REQUIRE_FALSE(snapshot.idle.active);
      stopped_idle = true;
      break;
    }
  }
  REQUIRE(stopped_idle);

  loop.tick();
  const auto next = store.snapshot();
  REQUIRE(next.active.kind == ActiveKind::Idle);
  REQUIRE(next.idle.enabled);
  REQUIRE(next.idle.n == 2);
  REQUIRE(next.idle.active);
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

TEST_CASE("RuntimeControlLoop idle empty config clears active idle") {
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

  const auto idle_path = validTrk(tmp, "empty_config_clears_idle.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.enabled);
  REQUIRE(store.snapshot().idle.active);

  REQUIRE(bridge.configureIdle({}).ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 0);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.passive_reason.has_value());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);
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
  REQUIRE_FALSE(snapshot.idle.enabled);
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

TEST_CASE("RuntimeControlLoop manual gate idle-configured short execute uses controllable bridge") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.7;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  LowStateSample low_state = readyLowState(deploy_config);
  setLowStateToManualGateFirstFrame(low_state, deploy_config, 0.3F);
  FakeRobotIO robot(low_state);
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                manualGateStandbyReferenceTrk(tmp, "manual_gate_standby_ref.trk"));
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

  const auto idle_a = manualGateTrk(tmp, "manual_gate_idle_a.trk", 80, 0.1F);
  const auto idle_b = manualGateTrk(tmp, "manual_gate_idle_b.trk", 90, 0.2F);
  const auto user_path = manualGateTrk(tmp, "manual_gate_short.trk", 35, 0.3F);
  REQUIRE(bridge.configureIdle({idleMotion(idle_a, 80), idleMotion(idle_b, 90)})
              .ok());
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 2);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.active);

  REQUIRE(bridge.submitQueue(
              executeCommand("manual-gate-short", user_path, MotionMode::Queue, 35))
              .ok());
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 2);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "manual-gate-short");
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{2, 2});
  REQUIRE(store.findRun("manual-gate-short").run->state == MotionState::Queued);

  requireActiveUserAfterTransition(loop, store, "manual-gate-short");
  requireRunDoneEventually(loop, store, "manual-gate-short", 512);
}

TEST_CASE("RuntimeControlLoop manual gate held user standby uses controllable source") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.7;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  LowStateSample low_state = readyLowState(deploy_config);
  setLowStateToManualGateFirstFrame(low_state, deploy_config, 0.3F);
  FakeRobotIO robot(low_state);
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                manualGateStandbyReferenceTrk(tmp,
                                              "manual_gate_held_standby_ref.trk"));
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

  const auto user_path =
      manualGateTrk(tmp, "manual_gate_held_short.trk", 35, 0.3F);
  REQUIRE(bridge.submitQueue(
              executeCommand("manual-gate-held-short",
                             user_path,
                             MotionMode::Queue,
                             35,
                             true))
              .ok());
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);
  REQUIRE(store.snapshot().transition.target == "user");
  REQUIRE(store.snapshot().transition.target_id == "manual-gate-held-short");

  requireActiveUserAfterTransition(loop, store, "manual-gate-held-short");

  bool saw_holding = false;
  for (int i = 0; i < 512; ++i) {
    loop.tick();
    const auto snapshot = store.snapshot();
    if (snapshot.exec && snapshot.exec->id == "manual-gate-held-short" &&
        snapshot.exec->state == MotionState::Holding) {
      saw_holding = true;
      break;
    }
  }
  REQUIRE(saw_holding);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{2, 2});

  REQUIRE(bridge.standby().ok());
  loop.tick();

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{0, 0});
  REQUIRE(store.findRun("manual-gate-held-short").run->state == MotionState::Done);

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
  REQUIRE(store.findRun("manual-gate-held-short").run->state == MotionState::Done);
  REQUIRE(store.findRun("manual-gate-held-short").run->stop_reason ==
          StopReason::None);
}

TEST_CASE("RuntimeControlLoop manual gate active idle short execute reaches user") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.7;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  LowStateSample low_state = readyLowState(deploy_config);
  setLowStateToManualGateFirstFrame(low_state, deploy_config, 0.3F);
  FakeRobotIO robot(low_state);
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

  const auto idle_a = manualGateTrk(tmp, "manual_gate_active_idle_a.trk", 80, 0.1F);
  const auto idle_b = manualGateTrk(tmp, "manual_gate_active_idle_b.trk", 90, 0.2F);
  const auto user_path =
      manualGateTrk(tmp, "manual_gate_active_short.trk", 35, 0.3F);
  REQUIRE(bridge.configureIdle({idleMotion(idle_a, 80), idleMotion(idle_b, 90)})
              .ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.active);

  bool saw_contact_mismatch_frame = false;
  for (int i = 0; i < 16; ++i) {
    const auto current = store.snapshot();
    if (current.active.kind == ActiveKind::Idle && current.idle.frame == 1) {
      saw_contact_mismatch_frame = true;
      break;
    }
    loop.tick();
  }
  REQUIRE(saw_contact_mismatch_frame);

  REQUIRE(bridge.submitQueue(
              executeCommand("manual-gate-active-short",
                             user_path,
                             MotionMode::Queue,
                             35))
              .ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "manual-gate-active-short");
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("manual-gate-active-short").run->state ==
          MotionState::Queued);

  requireActiveUserAfterTransition(loop, store, "manual-gate-active-short");
  requireRunDoneEventually(loop, store, "manual-gate-active-short", 512);
}

TEST_CASE("RuntimeControlLoop existing zero-contact idle TRK preempts to same user TRK") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.7;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  LowStateSample low_state = readyLowState(deploy_config);
  setLowStateNearExistingTurnAroundFirstFrame(low_state, deploy_config);
  FakeRobotIO robot(low_state);
  HighStateSample high_state;
  high_state.fresh = true;
  high_state.age_ms = 3;
  high_state.position = {0.0F, 0.0F, 0.74F};
  high_state.linear_velocity = {0.0F, 0.0F, 0.0F};
  high_state.angular_velocity = {0.0F, 0.0F, 0.0F};
  robot.high_state = high_state;
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

  const auto turn_path = existingTurnAroundZeroContactLikeTrk(
      tmp, "et1-generated-turn_around_180_degrees-fca831f744.trk");
  const auto idle_b = existingTurnAroundZeroContactLikeTrk(
      tmp, "et1-generated-turn_right_casually-ca8d41258e.trk");
  REQUIRE(bridge.configureIdle({idleMotion(turn_path, 149), idleMotion(idle_b, 149)})
              .ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.active);

  REQUIRE(bridge.submitQueue(
              executeCommand("existing-zero-contact-user",
                             turn_path,
                             MotionMode::Queue,
                             149))
              .ok());
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "existing-zero-contact-user");
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{0, 0});
  REQUIRE(store.findRun("existing-zero-contact-user").run->state ==
          MotionState::Queued);

  requireActiveUserAfterTransition(loop, store, "existing-zero-contact-user");
  requireRunDoneEventually(loop, store, "existing-zero-contact-user", 1024);
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
    const auto standby_track =
        loadTrack(tmp.trkConfig(),
                  transitionReadyTrk(tmp, "queue_standby_ref.trk", 2, 0.1F));
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

TEST_CASE("RuntimeControlLoop failed queue preempt of idle transition preserves idle config",
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
  const auto resumed = store.snapshot();
  REQUIRE(resumed.active.kind == ActiveKind::Idle);
  REQUIRE(resumed.idle.enabled);
  REQUIRE(resumed.idle.active);
  REQUIRE_FALSE(resumed.transition.active);
  REQUIRE_FALSE(resumed.exec.has_value());
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
    const auto standby_track =
        loadTrack(tmp.trkConfig(),
                  transitionReadyTrk(tmp, "interrupt_standby_ref.trk", 2, 0.1F));
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

TEST_CASE("RuntimeControlLoop idle to user preempt uses controllable source after reference reject") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 1.0;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
  LowStateSample low_state = readyLowState(deploy_config);
  low_state.gyro = {0.0F, 0.0F, 0.0F};
  for (const int sdk_slot_raw : deploy_config.sdk_joint_ids_map) {
    REQUIRE(sdk_slot_raw >= 0);
    auto& motor = low_state.motors.at(static_cast<std::size_t>(sdk_slot_raw));
    motor.q = 0.0F;
    motor.dq = 0.0F;
  }
  FakeRobotIO robot(low_state);
  HighStateSample high_state;
  high_state.fresh = true;
  high_state.age_ms = 3;
  high_state.position = {2.0F, -3.0F, 0.82F};
  high_state.linear_velocity = {0.0F, 0.0F, 0.0F};
  high_state.angular_velocity = {0.0F, 0.0F, 0.0F};
  robot.high_state = high_state;
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
                              velocityDeployConfig(),
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_path =
      transitionReadyContactZeroVelTrk(tmp,
                                       "idle_contact_bridge_source.trk",
                                       3,
                                       2,
                                       1,
                                       1.0F);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  const auto user_path =
      transitionReadyContactZeroVelTrk(tmp,
                                       "idle_cap_bridge_user.trk",
                                       3,
                                       1,
                                       2,
                                       0.02F);
  REQUIRE(bridge.submitQueue(
              executeCommand("idle-cap-bridge-user", user_path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "user");
  REQUIRE(snapshot.transition.target_id == "idle-cap-bridge-user");
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);
  REQUIRE(reference.latest.p.at(0) == high_state.position);
  REQUIRE(reference.latest.c == std::array<std::int64_t, 2>{1, 2});
  REQUIRE(store.findRun("idle-cap-bridge-user").run->state == MotionState::Queued);

  requireActiveUserAfterTransition(loop, store, "idle-cap-bridge-user");
  REQUIRE(store.findRun("idle-cap-bridge-user").run->state == MotionState::Running);
}

TEST_CASE("RuntimeControlLoop background transition reject with nonzero gap does not raw start") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  startCompletedUserToIdleTransition(loop, store, bridge, tmp, "background-contact-bridge");
  advanceBackgroundTransition(loop, store, "idle");
  REQUIRE(store.snapshot().active.kind == ActiveKind::Transition);

  const auto user_path =
      transitionReadyContactTrk(tmp, "background_contact_bridge_user.trk", 3, 1, 0);
  REQUIRE(bridge.submitQueue(
              executeCommand("background-contact-bridge-user",
                             user_path,
                             MotionMode::Queue,
                             3))
              .ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("background-contact-bridge-user").run->state ==
          MotionState::Failed);
  REQUIRE(store.findRun("background-contact-bridge-user").run->err ==
          ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop background transition raw fallback rejects nonzero hold gap") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.04;
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  const DeployConfig deploy_config = deployConfig();
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
                              velocityDeployConfig(),
                              fixStandConfig(),
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_a = transitionReadyTrk(tmp, "background_hold_idle_a.trk", 2, 0.1F);
  const auto idle_b = transitionReadyTrk(tmp, "background_hold_idle_b.trk", 2, 0.2F);
  REQUIRE(bridge.configureIdle({idleMotion(idle_a, 2), idleMotion(idle_b, 2)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  advanceBackgroundTransition(loop, store, "idle");

  const auto user_path =
      transitionReadyTrk(tmp, "background_hold_user.trk", 3, 20.0F);
  REQUIRE(bridge.submitQueue(
              executeCommand("background-hold-user", user_path, MotionMode::Queue, 3))
              .ok());

  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(store.findRun("background-hold-user").run->state == MotionState::Failed);
  REQUIRE(store.findRun("background-hold-user").run->err == ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop background and idle yaw residual gate rejects raw start") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();

  SECTION("idle active") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

    const auto idle_path = transitionReadyTrk(tmp, "idle_yaw_residual_source.trk", 3);
    REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
    loop.tick();
    loop.tick();
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

    const auto user_path = transitionReadyTrk(tmp, "idle_yaw_residual_user.trk", 3);
    REQUIRE(bridge.submitQueue(
                executeCommand("idle-yaw-residual-user",
                               user_path,
                               MotionMode::Queue,
                               3))
                .ok());
    loop.forceNextRootYawResidualForTest(0.06);
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(snapshot.idle.enabled);
    REQUIRE(snapshot.idle.n == 1);
    REQUIRE_FALSE(snapshot.idle.active);
    REQUIRE(store.findRun("idle-yaw-residual-user").run->state == MotionState::Failed);
    REQUIRE(store.findRun("idle-yaw-residual-user").run->err ==
            ErrorCode::InternalError);
  }

  SECTION("background transition") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeReferenceSink reference;
    RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

    startCompletedUserToIdleTransition(loop, store, bridge, tmp, "background-yaw-residual");
    advanceBackgroundTransition(loop, store, "idle");

    const auto user_path =
        transitionReadyTrk(tmp, "background_yaw_residual_user.trk", 3);
    REQUIRE(bridge.submitQueue(
                executeCommand("background-yaw-residual-user",
                               user_path,
                               MotionMode::Queue,
                               3))
                .ok());
    loop.forceNextRootYawResidualForTest(0.06);
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE(store.findRun("background-yaw-residual-user").run->state ==
            MotionState::Failed);
    REQUIRE(store.findRun("background-yaw-residual-user").run->err ==
            ErrorCode::InternalError);
  }
}

TEST_CASE("RuntimeControlLoop idle interrupt yaw residual rejects raw start") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeReferenceSink reference;
  RuntimeControlLoop loop(config, bridge, store, TrkLoader(tmp.trkConfig()), &reference);

  const auto idle_path = transitionReadyTrk(tmp, "idle_interrupt_yaw_source.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);

  const auto user_path = transitionReadyTrk(tmp, "idle_interrupt_yaw_user.trk", 3);
  REQUIRE(bridge.submitInterrupt(
              executeCommand("idle-interrupt-yaw-user",
                             user_path,
                             MotionMode::Interrupt,
                             3))
              .ok());
  loop.forceNextRootYawResidualForTest(0.06);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::Idle);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE(snapshot.idle.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("idle-interrupt-yaw-user").run->state == MotionState::Failed);
  REQUIRE(store.findRun("idle-interrupt-yaw-user").run->err == ErrorCode::InternalError);
}

TEST_CASE("RuntimeControlLoop idle to user transition target failure preserves idle config") {
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
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
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
  REQUIRE(next.active.kind == ActiveKind::Idle);
  REQUIRE(next.idle.enabled);
  REQUIRE(next.idle.n == 1);
  REQUIRE(next.idle.active);
  REQUIRE_FALSE(next.transition.active);
  REQUIRE_FALSE(next.exec.has_value());
}

TEST_CASE("RuntimeControlLoop idle to user transition interrupt failure preserves idle config") {
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
  REQUIRE(snapshot.active.kind == ActiveKind::Idle);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE(snapshot.idle.active);
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
  REQUIRE(snapshot.active.kind == ActiveKind::Idle);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE(snapshot.idle.active);
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
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
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
  REQUIRE(next.active.kind == ActiveKind::Idle);
  REQUIRE(next.idle.enabled);
  REQUIRE(next.idle.n == 1);
  REQUIRE(next.idle.active);
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
  REQUIRE_FALSE(snapshot.passive_reason.has_value());

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

  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE_FALSE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.block == "bad_orientation");
  REQUIRE(snapshot.passive_reason.has_value());
  REQUIRE(snapshot.passive_reason->code == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.passive_reason->block == "bad_orientation");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

  robot.low_state = readyLowState(deploy_config);
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Passive);
  REQUIRE(snapshot.ctrl == ControllerState::Passive);
  REQUIRE(snapshot.ready);
  REQUIRE(snapshot.err == ErrorCode::Ok);
  REQUIRE(snapshot.block.empty());
  REQUIRE(snapshot.passive_reason.has_value());
  REQUIRE(snapshot.passive_reason->code == ErrorCode::RobotBadOrientation);
  REQUIRE(snapshot.passive_reason->block == "bad_orientation");

  REQUIRE(bridge.fixStand().ok());
  loop.tick();

  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE_FALSE(snapshot.passive_reason.has_value());
}

TEST_CASE("RuntimeControlLoop standby velocity default-off uses legacy velocity runner") {
  TempTree tmp;
  const RuntimeConfig config = runtimeConfig();
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
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
                              fixStandConfig(),
                              ControlMode::StandbyVelocity);

  loop.tick();

  REQUIRE(store.snapshot().ctrl == ControllerState::StandbyVelocity);
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(tracker_policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);
}

TEST_CASE("RuntimeControlLoop standby velocity with loco runtime still uses legacy velocity runner") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.04;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocity_config,
                                  fixstand_config,
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config);

  loop.tick();

  REQUIRE(store.snapshot().ctrl == ControllerState::StandbyVelocity);
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(loco_lower_policy.calls == 0);
  REQUIRE(robot.write_attempts == 1);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);

  const LowCmdFrame standby_frame = robot.writes.back();
  applyLowCmdPositionToLowState(standby_frame, *robot.low_state);
  const LowStateSample standby_low_state = *robot.low_state;
  const auto path = identityQuaternionTrk(tmp, "loco_after_standby_lower.trk", 3);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-after-standby-lower", path, MotionMode::Queue, 3))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.write_attempts == 1);

  loop.tick();

  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(loco_lower_policy.calls == 1);
  REQUIRE(robot.write_attempts == 2);
  requireLocoLowerEntryHandoffFrame(robot.writes.back(),
                                    loco_lower_config,
                                    standby_low_state,
                                    loco_lower_policy.next_raw,
                                    0.0F);
}

TEST_CASE("RuntimeControlLoop standby to loco_upper resets lower last_action before first eval") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.transition_duration_s = 0.06;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocity_config,
                                  fixstand_config,
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config);

  loop.tick();
  REQUIRE(velocity_policy.calls == 1);
  REQUIRE(loco_lower_policy.calls == 0);
  const LowCmdFrame standby_frame = robot.writes.back();
  applyLowCmdPositionToLowState(standby_frame, *robot.low_state);
  const LowStateSample standby_low_state = *robot.low_state;

  const auto path = identityQuaternionTrk(tmp, "loco_after_standby_first_lower.trk", 4);
  REQUIRE(bridge.submitQueue(
              locoUpperCommand("loco-after-standby-first-lower", path, MotionMode::Queue, 4))
              .ok());

  loop.tick();
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->loco.phase == LocoPhase::Entry);
  REQUIRE(robot.write_attempts == 1);

  loop.tick();
  REQUIRE(loco_lower_policy.calls == 1);
  REQUIRE(robot.write_attempts == 2);
  requireLocoLowerEntryHandoffFrame(robot.writes.back(),
                                    loco_lower_config,
                                    standby_low_state,
                                    loco_lower_policy.next_raw,
                                    0.0F);
  REQUIRE(loco_lower_policy.inputs_seen.back().obs.at(
              kLocoLowerPolicyObsRowWidth - kLocoLowerPolicyJointDim) == 0.0F);

  loop.tick();
  REQUIRE(loco_lower_policy.calls == 2);
  REQUIRE(robot.write_attempts == 3);
}

TEST_CASE("RuntimeControlLoop standby velocity does not run loco lower at runtime tick rate") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 1000.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  LocoLowerDeployConfig loco_lower_config = locoLowerDeployConfig();
  loco_lower_config.policy_decimation = 10;
  const LocoUpperLowCmdComposerConfig composer_config = locoUpperComposerConfig();
  const FixStandConfig fixstand_config = fixStandConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy;
  FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
  FakeVelocityPolicy loco_lower_policy(Vec(kLocoLowerPolicyJointDim, 0.5F));
  auto loop = makeLocoControlLoop(config,
                                  bridge,
                                  store,
                                  tmp.trkConfig(),
                                  robot,
                                  tracker_policy,
                                  velocity_policy,
                                  loco_lower_policy,
                                  deploy_config,
                                  velocity_config,
                                  fixstand_config,
                                  ControlMode::StandbyVelocity,
                                  passiveConfig(),
                                  loco_lower_config,
                                  composer_config);

  for (int i = 0; i < 20; ++i) {
    loop.tick();
  }

  REQUIRE(store.snapshot().ctrl == ControllerState::StandbyVelocity);
  REQUIRE(velocity_policy.calls >= 1);
  REQUIRE(loco_lower_policy.calls == 0);
  REQUIRE(robot.write_attempts == 20);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);
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

TEST_CASE("RuntimeControlLoop active tracker uses deploy step_dt for fractional policy cadence") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 60.0;
  const DeployConfig deploy_config = deployConfig();
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  FakeRobotIO robot(readyLowState(deploy_config));
  FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
  auto loop = makePolicyLoop(config,
                             bridge,
                             store,
                             tmp.trkConfig(50.0),
                             robot,
                             tracker_policy,
                             deploy_config);

  const auto path = validTrk(tmp, "fractional_step_dt_track.trk", 100);
  REQUIRE(bridge.submitQueue(
              executeCommand("fractional-step-dt-track", path, MotionMode::Queue, 100))
              .ok());
  startQueuedRun(loop, store, "fractional-step-dt-track");
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerActive);

  for (int tick = 0; tick < 60; ++tick) {
    loop.tick();
  }

  REQUIRE(tracker_policy.calls == 50);
  REQUIRE(store.snapshot().exec.has_value());
  REQUIRE(store.snapshot().exec->state == MotionState::Running);
}

TEST_CASE("RuntimeControlLoop idle tracker samples reference frames by rounded playback time") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
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
                              tmp.trkConfig(30.0),
                              robot,
                              tracker_policy,
                              velocity_policy,
                              deploy_config,
                              velocity_config,
                              fixstand_config,
                              ControlMode::StandbyVelocity,
                              passiveConfig(),
                              &reference);

  const auto idle_path = validTrk(tmp, "idle_30fps_playback_sample.trk", 8);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 8)}).ok());

  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(reference.latest.active);
  REQUIRE(reference.latest.frame == 0);

  std::vector<std::size_t> frames_at_write;
  robot.on_write = [&](const LowCmdFrame&) {
    if (reference.latest.active) {
      frames_at_write.push_back(reference.latest.frame);
    }
  };

  for (int tick = 0; tick < 6; ++tick) {
    loop.tick();
  }

  REQUIRE(frames_at_write == std::vector<std::size_t>{0, 1, 1, 2, 2, 3});
}

TEST_CASE("RuntimeControlLoop user startup hold step count is independent of track fps") {
  struct Case {
    double fps;
    const char* suffix;
  };

  for (const Case& item : std::vector<Case>{{30.0, "30"}, {50.0, "50"}, {60.0, "60"}}) {
    TempTree tmp;
    RuntimeConfig config = runtimeConfig();
    config.hz = 1000.0;
    const DeployConfig deploy_config = deployConfig();
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    LowStateSample low_state = readyLowState(deploy_config);
    setLowStateNearExistingTurnAroundFirstFrame(low_state, deploy_config);
    FakeRobotIO robot(low_state);
    HighStateSample high_state;
    high_state.fresh = true;
    high_state.age_ms = 3;
    high_state.position = {0.0F, 0.0F, 0.74F};
    high_state.linear_velocity = {0.0F, 0.0F, 0.0F};
    high_state.angular_velocity = {0.0F, 0.0F, 0.0F};
    robot.high_state = high_state;
    FakePolicy tracker_policy(floatSeq(0.0F, kPolicyJointDim));
    auto loop = makePolicyLoop(config,
                               bridge,
                               store,
                               tmp.trkConfig(item.fps),
                               robot,
                               tracker_policy,
                               deploy_config);

    const std::string id = std::string("startup-hold-fps-") + item.suffix;
    const auto path = validTrk(tmp, id + ".trk", 64);
    REQUIRE(bridge.submitQueue(executeCommand(id, path, MotionMode::Queue, 64)).ok());
    startQueuedRun(loop, store, id);

    bool saw_first_playback_frame = false;
    for (int tick = 0; tick < 2048; ++tick) {
      loop.tick();
      const auto snapshot = store.snapshot();
      REQUIRE(snapshot.exec.has_value());
      if (snapshot.exec->frame > 0) {
        saw_first_playback_frame = true;
        break;
      }
    }

    REQUIRE(saw_first_playback_frame);
    REQUIRE(tracker_policy.calls == static_cast<int>(kStartupHoldPolicyStepsAt50Fps + 1));
  }
}

TEST_CASE("RuntimeControlLoop standby preserves idle config while active user hands off to standby") {
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
  const auto standby_track =
      validStandbyTrack(tmp, "standby_active_idle_to_standby_ref.trk");
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

  const auto idle_path = validTrk(tmp, "standby_active_idle_config.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  loop.tick();
  loop.tick();
  REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
  REQUIRE(store.snapshot().idle.enabled);
  REQUIRE(store.snapshot().idle.active);

  const auto user_path = validTrk(tmp, "standby_active_idle_user.trk", 5);
  REQUIRE(bridge.submitQueue(executeCommand("active", user_path)).ok());
  startQueuedRun(loop,
                 store,
                 "active",
                 StartQueuedRunMode::AllowStandbyTransition);
  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::User);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE_FALSE(snapshot.idle.active);

  REQUIRE(bridge.standby().ok());
  REQUIRE(store.snapshot().idle.enabled);

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun(idle_path.string()).code == ErrorCode::RunNotFound);

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
  REQUIRE_FALSE(snapshot.transition.active);
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 1);
}

TEST_CASE("RuntimeControlLoop standby cancellation from idle-origin active user uses standby transition") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.hz = 50.0;
  config.transition_duration_s = 0.04;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
  const auto standby_track =
      loadTrack(tmp.trkConfig(),
                transitionReadyContactZeroVelTrk(tmp,
                                                 "idle_origin_cancel_standby_ref.trk",
                                                 3,
                                                 1,
                                                 2,
                                                 0.02F));

  auto contact_test_low_state = [&] {
    LowStateSample low_state = readyLowState(deploy_config);
    low_state.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
    low_state.gyro = {0.0F, 0.0F, 0.0F};
    for (std::size_t policy_index = 0;
         policy_index < deploy_config.sdk_joint_ids_map.size();
         ++policy_index) {
      const auto sdk_slot =
          static_cast<std::size_t>(deploy_config.sdk_joint_ids_map.at(policy_index));
      low_state.motors.at(sdk_slot).q = 0.0F;
      low_state.motors.at(sdk_slot).dq = 0.0F;
    }
    return low_state;
  };

  auto make_loop = [&](RuntimeStatusStore& store,
                       RuntimeBridge& bridge,
                       FakeRobotIO& robot,
                       FakePolicy& tracker_policy,
                       FakeVelocityPolicy& velocity_policy) {
    return makeControlLoop(config,
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
  };

  auto start_idle_origin_user = [&](RuntimeControlLoop& loop,
                                    RuntimeStatusStore& store,
                                    RuntimeBridge& bridge,
                                    const std::string& prefix) {
    const auto idle_path = transitionReadyContactZeroVelTrk(tmp,
                                                            prefix + "_idle.trk",
                                                            3,
                                                            0,
                                                            0,
                                                            0.02F);
    REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
    loop.tick();
    loop.tick();
    loop.tick();
    REQUIRE(store.snapshot().active.kind == ActiveKind::Idle);
    REQUIRE(store.snapshot().idle.enabled);
    REQUIRE(store.snapshot().idle.active);

    const auto user_path = transitionReadyContactZeroVelTrk(tmp,
                                                            prefix + "_user.trk",
                                                            3,
                                                            0,
                                                            0,
                                                            0.02F);
    REQUIRE(bridge.submitQueue(executeCommand(prefix + "-user",
                                              user_path,
                                              MotionMode::Queue,
                                              3))
                .ok());
    startQueuedRun(loop,
                   store,
                   prefix + "-user",
                   StartQueuedRunMode::AllowStandbyTransition);
    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.active.kind == ActiveKind::User);
    REQUIRE(snapshot.exec.has_value());
    REQUIRE(snapshot.exec->id == prefix + "-user");
    REQUIRE(snapshot.idle.enabled);
    REQUIRE_FALSE(snapshot.idle.active);
  };

  SECTION("standby preserves idle config but uses standby transition") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(contact_test_low_state());
    HighStateSample high_state;
    high_state.fresh = true;
    high_state.age_ms = 3;
    high_state.position = {0.0F, 0.0F, 0.0F};
    high_state.linear_velocity = {0.0F, 0.0F, 0.0F};
    high_state.angular_velocity = {0.0F, 0.0F, 0.0F};
    robot.high_state = high_state;
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    auto loop = make_loop(store, bridge, robot, tracker_policy, velocity_policy);

    start_idle_origin_user(loop, store, bridge, "standby-idle-origin");

    REQUIRE(bridge.standby().ok());
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::Running);
    REQUIRE(snapshot.active.kind == ActiveKind::Transition);
    REQUIRE(snapshot.transition.active);
    REQUIRE(snapshot.transition.target == "standby");
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE(snapshot.idle.enabled);
    REQUIRE_FALSE(snapshot.idle.active);
    REQUIRE(store.findRun("standby-idle-origin-user").run->state ==
            MotionState::Stopped);
    REQUIRE(store.findRun("standby-idle-origin-user").run->stop_reason ==
            StopReason::Stop);
  }

  SECTION("urgent_stop clears idle config and aborts without standby transition") {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    FakeRobotIO robot(contact_test_low_state());
    HighStateSample high_state;
    high_state.fresh = true;
    high_state.age_ms = 3;
    high_state.position = {0.0F, 0.0F, 0.0F};
    high_state.linear_velocity = {0.0F, 0.0F, 0.0F};
    high_state.angular_velocity = {0.0F, 0.0F, 0.0F};
    robot.high_state = high_state;
    FakePolicy tracker_policy;
    FakeVelocityPolicy velocity_policy(Vec(kVelocityPolicyJointDim, 0.25F));
    auto loop = make_loop(store, bridge, robot, tracker_policy, velocity_policy);

    start_idle_origin_user(loop, store, bridge, "stop-idle-origin");

    REQUIRE(bridge.urgentStop().state == ControllerState::UrgentStopping);
    REQUIRE_FALSE(store.snapshot().idle.enabled);
    loop.tick();

    const auto snapshot = store.snapshot();
    REQUIRE(snapshot.ctrl == ControllerState::UrgentStopping);
    REQUIRE(snapshot.robot == RobotState::Holding);
    REQUIRE(snapshot.stop_reason == StopReason::UrgentStop);
    REQUIRE(snapshot.active.kind == ActiveKind::None);
    REQUIRE_FALSE(snapshot.transition.active);
    REQUIRE_FALSE(snapshot.exec.has_value());
    REQUIRE_FALSE(snapshot.idle.enabled);
    REQUIRE_FALSE(snapshot.idle.active);
    REQUIRE(store.findRun("stop-idle-origin-user").run->state ==
            MotionState::Canceled);
    REQUIRE(store.findRun("stop-idle-origin-user").run->stop_reason ==
            StopReason::UrgentStop);
  }
}

TEST_CASE("RuntimeControlLoop standby transitions active run to StandbyVelocity") {
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

  REQUIRE(bridge.standby().ok());
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerTransition);
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);

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
  REQUIRE_FALSE(snapshot.transition.active);

  loop.tick();
  REQUIRE(velocity_policy.calls == 1);
  requireVelocityFrame(robot.writes.back(), velocity_config, velocity_policy.next_raw);
}

TEST_CASE("RuntimeControlLoop urgent_stop ignores configured stop hold") {
  TempTree tmp;
  RuntimeConfig config = runtimeConfig();
  config.stop_hold_s = 30.0;
  const DeployConfig deploy_config = deployConfig();
  const VelocityDeployConfig velocity_config = velocityDeployConfig();
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
                              fixStandConfig(),
                              ControlMode::StandbyVelocity);

  auto active_snapshot = store.snapshot();
  MotionStatus active;
  active.id = "active";
  active.path = "/tmp/active.trk";
  active.state = MotionState::Running;
  active_snapshot.ctrl = ControllerState::Running;
  active_snapshot.active = {ActiveKind::User, active.id};
  active_snapshot.exec = active;
  store.publishSnapshot(active_snapshot);

  const int writes_before_stop = robot.write_attempts;
  REQUIRE(bridge.urgentStop().state == ControllerState::UrgentStopping);
  loop.tick();
  auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::Stopping);
  REQUIRE(snapshot.ctrl == ControllerState::UrgentStopping);
  REQUIRE(snapshot.stop_reason == StopReason::UrgentStop);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());

  loop.tick();
  snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::GeneralTrackerIdle);
  REQUIRE(snapshot.ctrl == ControllerState::StandbyVelocity);
  REQUIRE(snapshot.stop_reason == StopReason::None);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(robot.write_attempts == writes_before_stop);
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

TEST_CASE("RuntimeControlLoop urgent_stop in idle FixStand clears idle and stays FixStand") {
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

  const auto idle_path = validTrk(tmp, "fixstand_urgent_idle_config.trk", 3);
  REQUIRE(bridge.configureIdle({idleMotion(idle_path, 3)}).ok());
  loop.tick();
  REQUIRE(store.snapshot().ctrl == ControllerState::FixStand);
  REQUIRE(store.snapshot().idle.enabled);
  REQUIRE(store.snapshot().idle.n == 1);
  REQUIRE(store.snapshot().active.kind == ActiveKind::None);
  REQUIRE(store.snapshot().queue.ids.empty());

  REQUIRE(bridge.urgentStop().state == ControllerState::UrgentStopping);
  REQUIRE_FALSE(store.snapshot().idle.enabled);
  loop.tick();

  const auto snapshot = store.snapshot();
  REQUIRE(loop.internalStateForTest() == RuntimeInternalState::FixStand);
  REQUIRE(snapshot.ctrl == ControllerState::FixStand);
  REQUIRE(snapshot.stop_reason == StopReason::None);
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE_FALSE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 0);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(velocity_policy.calls == 0);
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
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.active.kind == ActiveKind::Transition);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.transition.active);
  REQUIRE(snapshot.transition.target == "standby");
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"post-stop"});
  REQUIRE(store.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(store.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("old-before-stop").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-before-stop").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("post-stop").run->state == MotionState::Queued);
  REQUIRE(store.findRun("post-stop").run->stop_reason == StopReason::None);

  bool saw_fixstand = false;
  for (int i = 0; i < 256; ++i) {
    loop.tick();
    snapshot = store.snapshot();
    if (snapshot.ctrl == ControllerState::FixStand) {
      saw_fixstand = true;
      break;
    }
  }
  REQUIRE(saw_fixstand);
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
