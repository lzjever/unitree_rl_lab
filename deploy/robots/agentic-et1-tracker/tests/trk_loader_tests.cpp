#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "agentic_et1_tracker/trk/detail/scanner.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

struct TempTree {
  TempTree() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_trk_loader_tests_" + std::to_string(now));
    allowed = root / "allowed";
    outside = root / "outside";
    std::filesystem::create_directories(allowed);
    std::filesystem::create_directories(outside);
  }

  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  TrkValidationConfig config(double fps = 50.0, double max_duration_s = 120.0) const {
    TrkValidationConfig cfg;
    cfg.allowlist_dirs = {allowed};
    cfg.fps = fps;
    cfg.max_duration_s = max_duration_s;
    return cfg;
  }

  std::filesystem::path root;
  std::filesystem::path allowed;
  std::filesystem::path outside;
};

struct ArrayFixture {
  std::string name;
  TrkDtype dtype{TrkDtype::Float32};
  std::vector<std::uint64_t> shape;
  std::uint64_t byte_count_override{0};
  bool has_byte_count_override{false};
  bool write_payload{true};
  bool sparse_payload{false};
  bool invalid_contact{false};
  std::size_t invalid_contact_index{0};
  std::int64_t invalid_contact_value{3};
  bool write_shape_as_uint64{false};
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

std::uint64_t checkedBytes(const std::vector<std::uint64_t>& shape, TrkDtype dtype) {
  return elementCount(shape) * trkDtypeSize(dtype);
}

std::vector<std::uint64_t> shapeFor(const TrkRequiredArraySpec& spec, std::uint64_t frames) {
  std::vector<std::uint64_t> shape{frames};
  for (std::uint32_t i = 0; i < spec.trailing_rank; ++i) {
    shape.push_back(spec.trailing_shape[i]);
  }
  return shape;
}

TrkDtype defaultDtypeFor(const TrkRequiredArraySpec& spec) {
  return spec.dtype_family == TrkDtypeFamily::Contact ? TrkDtype::Int64 : TrkDtype::Float32;
}

std::size_t requiredIndex(const std::string& name) {
  const auto& specs = TrkSchema::kRequiredArrays;
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (specs[i].name == name) {
      return i;
    }
  }
  return specs.size() + 17;
}

bool isRequiredContact(const std::string& name) {
  const auto index = requiredIndex(name);
  return index < TrkSchema::kRequiredArrays.size() &&
         TrkSchema::kRequiredArrays[index].dtype_family == TrkDtypeFamily::Contact;
}

float expectedFloat(const std::string& name, std::size_t flat_index) {
  return static_cast<float>((requiredIndex(name) + 1) * 1000) +
         static_cast<float>(flat_index) * 0.25F;
}

std::int64_t expectedContact(const std::string& name, std::size_t flat_index) {
  return static_cast<std::int64_t>((flat_index + requiredIndex(name)) % 3);
}

std::int64_t contactValue(const ArrayFixture& array, std::size_t flat_index) {
  if (array.invalid_contact && flat_index == array.invalid_contact_index) {
    return array.invalid_contact_value;
  }
  return expectedContact(array.name, flat_index);
}

void writeSparsePayload(std::ofstream& out, std::uint64_t byte_count) {
  if (byte_count == 0) {
    return;
  }
  out.seekp(static_cast<std::streamoff>(byte_count - 1), std::ios::cur);
  const char zero = 0;
  out.write(&zero, 1);
}

void writeGeneratedPayload(std::ofstream& out,
                           const ArrayFixture& array,
                           std::uint64_t byte_count) {
  if (!array.write_payload) {
    return;
  }
  if (array.sparse_payload) {
    writeSparsePayload(out, byte_count);
    return;
  }
  const auto elements = elementCount(array.shape);
  switch (array.dtype) {
    case TrkDtype::Float32:
      for (std::size_t i = 0; i < elements; ++i) {
        writeScalar(out, expectedFloat(array.name, i));
      }
      return;
    case TrkDtype::Float64:
      for (std::size_t i = 0; i < elements; ++i) {
        writeScalar(out, static_cast<double>(expectedFloat(array.name, i)));
      }
      return;
    case TrkDtype::Int64:
      for (std::size_t i = 0; i < elements; ++i) {
        writeScalar(out, static_cast<std::int64_t>(
                             isRequiredContact(array.name) ? contactValue(array, i) : 0));
      }
      return;
    case TrkDtype::Int32:
      for (std::size_t i = 0; i < elements; ++i) {
        writeScalar(out, static_cast<std::int32_t>(
                             isRequiredContact(array.name) ? contactValue(array, i) : 0));
      }
      return;
    case TrkDtype::UInt8:
      for (std::size_t i = 0; i < elements; ++i) {
        writeScalar(out, static_cast<std::uint8_t>(
                             isRequiredContact(array.name) ? contactValue(array, i) : 0));
      }
      return;
    case TrkDtype::Int8:
      for (std::size_t i = 0; i < elements; ++i) {
        writeScalar(out, static_cast<std::int8_t>(
                             isRequiredContact(array.name) ? contactValue(array, i) : 0));
      }
      return;
    case TrkDtype::Bool:
      writeSparsePayload(out, byte_count);
      return;
  }
  writeSparsePayload(out, byte_count);
}

void writeTrk(const std::filesystem::path& path,
              const std::vector<ArrayFixture>& arrays,
              std::uint32_t version = TrkSchema::kVersion,
              std::uint32_t array_count_override = 0) {
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);

  out.write(TrkSchema::kMagic.data(), static_cast<std::streamsize>(TrkSchema::kMagic.size()));
  writeScalar(out, version);
  writeScalar(out, array_count_override == 0 ? static_cast<std::uint32_t>(arrays.size())
                                             : array_count_override);

  for (const auto& array : arrays) {
    writeScalar(out, static_cast<std::uint32_t>(array.name.size()));
    out.write(array.name.data(), static_cast<std::streamsize>(array.name.size()));
    writeScalar(out, static_cast<std::uint32_t>(array.dtype));
    writeScalar(out, static_cast<std::uint32_t>(array.shape.size()));
    for (const auto dim : array.shape) {
      if (array.write_shape_as_uint64) {
        writeScalar(out, dim);
      } else {
        REQUIRE(dim <= std::numeric_limits<std::uint32_t>::max());
        writeScalar(out, static_cast<std::uint32_t>(dim));
      }
    }
    const auto byte_count = array.has_byte_count_override
                                ? array.byte_count_override
                                : checkedBytes(array.shape, array.dtype);
    writeScalar(out, byte_count);
    writeGeneratedPayload(out, array, byte_count);
  }
}

std::vector<ArrayFixture> requiredArrays(std::uint64_t frames = 4) {
  std::vector<ArrayFixture> arrays;
  for (const auto& spec : TrkSchema::kRequiredArrays) {
    arrays.push_back({std::string(spec.name), defaultDtypeFor(spec), shapeFor(spec, frames)});
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

TrkLoadResult loadPath(const std::filesystem::path& path, const TrkValidationConfig& cfg) {
  return TrkLoader(cfg).load(path);
}

TrkValidationCode validatePath(const std::filesystem::path& path, const TrkValidationConfig& cfg) {
  return TrkValidator(cfg).validate(path).code;
}

void requireFloatArray(const TrkFloatArray& array,
                       const std::string& name,
                       const std::vector<std::uint64_t>& shape) {
  REQUIRE(array.shape == shape);
  REQUIRE(array.values.size() == elementCount(shape));
  REQUIRE(array.values.front() == expectedFloat(name, 0));
  REQUIRE(array.values.back() == expectedFloat(name, array.values.size() - 1));
}

void requireContactArray(const TrkContactArray& array,
                         const std::string& name,
                         const std::vector<std::uint64_t>& shape) {
  REQUIRE(array.shape == shape);
  REQUIRE(array.values.size() == elementCount(shape));
  REQUIRE(array.values.front() == expectedContact(name, 0));
  REQUIRE(array.values.back() == expectedContact(name, array.values.size() - 1));
}

struct ScopedScannerHook {
  ~ScopedScannerHook() {
    trk_detail::setAfterSuccessfulScanHookForTesting({});
  }
};

}  // namespace

TEST_CASE("TrkLoader loads required arrays with normalized schema and order independence") {
  TempTree tmp;
  auto arrays = requiredArrays(4);
  std::reverse(arrays.begin(), arrays.end());
  const auto path = tmp.allowed / "order_independent.trk";
  writeTrk(path, arrays);

  const auto result = loadPath(path, tmp.config());

  REQUIRE(result.ok());
  REQUIRE(result.track.has_value());
  const auto& track = *result.track;
  REQUIRE(track.metadata.canonical_path == std::filesystem::canonical(path));
  REQUIRE(track.metadata.frames == 4);
  REQUIRE(track.metadata.duration_s == 0.06);
  REQUIRE(track.metadata.fps == 50.0);
  REQUIRE(track.metadata.version == TrkSchema::kVersion);
  REQUIRE(track.metadata.array_count == TrkSchema::kRequiredArrays.size());
  REQUIRE(track.metadata.file_size == std::filesystem::file_size(path));

  requireFloatArray(track.joint_pos, "joint_pos", {4, TrkSchema::kJointDim});
  requireFloatArray(track.joint_vel, "joint_vel", {4, TrkSchema::kJointDim});
  requireFloatArray(track.body_pos_w, "body_pos_w", {4, TrkSchema::kBodyCount, 3});
  requireFloatArray(track.body_quat_w, "body_quat_w", {4, TrkSchema::kBodyCount, 4});
  requireFloatArray(track.body_lin_vel_w, "body_lin_vel_w", {4, TrkSchema::kBodyCount, 3});
  requireFloatArray(track.body_ang_vel_w, "body_ang_vel_w", {4, TrkSchema::kBodyCount, 3});
  requireContactArray(track.left_foot_contact_state, "left_foot_contact_state", {4});
  requireContactArray(track.right_foot_contact_state, "right_foot_contact_state", {4});
  requireFloatArray(track.ref_com_rel_navi, "ref_com_rel_navi", {4, 3});
  requireFloatArray(track.ref_com_vel_navi, "ref_com_vel_navi", {4, 3});
}

TEST_CASE("TrkLoader normalizes Float64 and contact dtype variants") {
  TempTree tmp;
  const std::array<TrkDtype, 4> contact_dtypes{
      TrkDtype::Int64, TrkDtype::Int32, TrkDtype::UInt8, TrkDtype::Int8};

  for (const auto dtype : contact_dtypes) {
    auto arrays = requiredArrays(3);
    for (auto& array : arrays) {
      const auto index = requiredIndex(array.name);
      if (index < TrkSchema::kRequiredArrays.size() &&
          TrkSchema::kRequiredArrays[index].dtype_family == TrkDtypeFamily::Float) {
        array.dtype = TrkDtype::Float64;
      }
    }
    arrayNamed(arrays, "left_foot_contact_state").dtype = dtype;
    arrayNamed(arrays, "right_foot_contact_state").dtype = dtype;
    const auto path = tmp.allowed /
                      ("dtype_" + std::to_string(static_cast<std::uint32_t>(dtype)) + ".trk");
    writeTrk(path, arrays);

    const auto result = loadPath(path, tmp.config());

    REQUIRE(result.ok());
    REQUIRE(result.track->joint_pos.values[1] == expectedFloat("joint_pos", 1));
    const std::vector<std::int64_t>* left_values =
        &result.track->left_foot_contact_state.values;
    const std::vector<std::int64_t>* right_values =
        &result.track->right_foot_contact_state.values;
    REQUIRE(left_values->at(1) == expectedContact("left_foot_contact_state", 1));
    REQUIRE(right_values->at(2) == expectedContact("right_foot_contact_state", 2));
  }

  auto support_state_two = requiredArrays(3);
  arrayNamed(support_state_two, "left_foot_contact_state").dtype = TrkDtype::UInt8;
  arrayNamed(support_state_two, "right_foot_contact_state").dtype = TrkDtype::Int8;
  const auto support_state_two_path = tmp.allowed / "support_state_two.trk";
  writeTrk(support_state_two_path, support_state_two);

  const auto support_state_two_result = loadPath(support_state_two_path, tmp.config());
  REQUIRE(support_state_two_result.ok());
  REQUIRE(std::find(support_state_two_result.track->left_foot_contact_state.values.begin(),
                    support_state_two_result.track->left_foot_contact_state.values.end(),
                    2) != support_state_two_result.track->left_foot_contact_state.values.end());

  auto invalid = requiredArrays(3);
  auto& left = arrayNamed(invalid, "left_foot_contact_state");
  left.dtype = TrkDtype::UInt8;
  left.invalid_contact = true;
  left.invalid_contact_index = 1;
  left.invalid_contact_value = 3;
  const auto invalid_path = tmp.allowed / "invalid_contact.trk";
  writeTrk(invalid_path, invalid);

  REQUIRE(validatePath(invalid_path, tmp.config()) == TrkValidationCode::Ok);
  const auto invalid_result = loadPath(invalid_path, tmp.config());
  REQUIRE_FALSE(invalid_result.ok());
  REQUIRE(invalid_result.code == TrkValidationCode::InvalidContactValue);
  REQUIRE_FALSE(invalid_result.track.has_value());

  for (const auto dtype : {TrkDtype::Int8, TrkDtype::Int32}) {
    auto negative = requiredArrays(3);
    auto& negative_left = arrayNamed(negative, "left_foot_contact_state");
    negative_left.dtype = dtype;
    negative_left.invalid_contact = true;
    negative_left.invalid_contact_index = 1;
    negative_left.invalid_contact_value = -1;
    const auto negative_path =
        tmp.allowed /
        ("negative_contact_" + std::to_string(static_cast<std::uint32_t>(dtype)) + ".trk");
    writeTrk(negative_path, negative);

    const auto negative_result = loadPath(negative_path, tmp.config());
    REQUIRE_FALSE(negative_result.ok());
    REQUIRE(negative_result.code == TrkValidationCode::InvalidContactValue);
  }
}

TEST_CASE("TrkLoader skips unknown payloads before between and after required arrays") {
  TempTree tmp;
  auto arrays = requiredArrays(5);
  std::vector<ArrayFixture> with_unknowns;
  with_unknowns.push_back({"unknown_before", TrkDtype::UInt8, {16ULL * 1024ULL * 1024ULL}, 0,
                           false, true, true});
  with_unknowns.insert(with_unknowns.end(), arrays.begin(), arrays.begin() + 3);
  with_unknowns.push_back({"unknown_between", TrkDtype::Float32, {2, 3}});
  with_unknowns.insert(with_unknowns.end(), arrays.begin() + 3, arrays.end());
  with_unknowns.push_back({"unknown_after", TrkDtype::UInt8, {1024}, 0, false, true, true});
  const auto path = tmp.allowed / "unknowns.trk";
  writeTrk(path, with_unknowns);

  const auto result = loadPath(path, tmp.config());

  REQUIRE(result.ok());
  REQUIRE(result.track->metadata.array_count == with_unknowns.size());
  REQUIRE(result.track->joint_pos.values.size() == 5 * TrkSchema::kJointDim);
  REQUIRE(result.track->joint_pos.values.front() == expectedFloat("joint_pos", 0));
  REQUIRE(result.track->ref_com_vel_navi.values.back() ==
          expectedFloat("ref_com_vel_navi", result.track->ref_com_vel_navi.values.size() - 1));
}

TEST_CASE("TrkLoader uses validator-compatible failures for metadata and bounds") {
  TempTree tmp;
  const auto cfg = tmp.config();

  SECTION("missing required") {
    auto arrays = requiredArrays();
    arrays.erase(arrays.begin() + 1);
    const auto path = tmp.allowed / "missing_required.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::MissingRequiredArray);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("duplicate required") {
    auto arrays = requiredArrays();
    arrays.push_back(arrays.front());
    const auto path = tmp.allowed / "duplicate_required.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::DuplicateRequiredArray);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("bad shape") {
    auto arrays = requiredArrays();
    arrayNamed(arrays, "joint_vel").shape = {4, TrkSchema::kJointDim - 1};
    const auto path = tmp.allowed / "bad_shape.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::BadRequiredShape);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("frame mismatch") {
    auto arrays = requiredArrays(4);
    arrayNamed(arrays, "body_pos_w").shape = {5, TrkSchema::kBodyCount, 3};
    const auto path = tmp.allowed / "frame_mismatch.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::FrameCountMismatch);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("bad dtype") {
    auto arrays = requiredArrays();
    arrayNamed(arrays, "joint_pos").dtype = TrkDtype::Int32;
    const auto path = tmp.allowed / "bad_dtype.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::BadRequiredDtype);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("byte count mismatch") {
    auto arrays = requiredArrays();
    auto& joint_pos = arrayNamed(arrays, "joint_pos");
    joint_pos.has_byte_count_override = true;
    joint_pos.byte_count_override = checkedBytes(joint_pos.shape, joint_pos.dtype) + 1;
    const auto path = tmp.allowed / "byte_count_mismatch.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::ByteCountMismatch);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("payload out of bounds") {
    auto arrays = requiredArrays();
    arrays.push_back({"truncated_unknown", TrkDtype::UInt8, {64}, 0, false, false});
    const auto path = tmp.allowed / "payload_out_of_bounds.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::PayloadOutOfBounds);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("short metadata read") {
    const auto path = tmp.allowed / "short_read.trk";
    std::ofstream out(path, std::ios::binary);
    REQUIRE(out);
    out.write(TrkSchema::kMagic.data(), static_cast<std::streamsize>(TrkSchema::kMagic.size()));
    writeScalar(out, TrkSchema::kVersion);
    writeScalar(out, std::uint32_t{1});
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::ReadFailed);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("single limit") {
    auto arrays = requiredArrays();
    arrays.push_back({"oversized", TrkDtype::UInt8,
                      {TrkSchema::kDefaultLimits.max_single_array_bytes + 1}, 0, false, false});
    const auto path = tmp.allowed / "single_limit.trk";
    writeTrk(path, arrays);
    REQUIRE(loadPath(path, cfg).code == TrkValidationCode::ByteCountLimitExceeded);
    REQUIRE(loadPath(path, cfg).code == validatePath(path, cfg));
  }

  SECTION("total limit") {
    auto arrays = requiredArrays();
    arrays.push_back({"unknown_a", TrkDtype::UInt8, {70}});
    arrays.push_back({"unknown_b", TrkDtype::UInt8, {70}});
    const auto path = tmp.allowed / "total_limit.trk";
    writeTrk(path, arrays);

    auto small_total = cfg;
    small_total.limits.max_total_payload_bytes = 128;
    REQUIRE(loadPath(path, small_total).code == TrkValidationCode::TotalPayloadLimitExceeded);
    REQUIRE(loadPath(path, small_total).code == validatePath(path, small_total));
  }
}

TEST_CASE("TrkLoader frame accessor returns non-copying per-frame views") {
  TempTree tmp;
  const auto path = tmp.allowed / "frames.trk";
  writeTrk(path, requiredArrays(3));

  const auto result = loadPath(path, tmp.config());
  REQUIRE(result.ok());
  const auto& track = *result.track;

  const auto frame0 = track.frame(0);
  REQUIRE(frame0.has_value());
  REQUIRE(frame0->joint_pos.ptr == track.joint_pos.values.data());
  REQUIRE(frame0->joint_pos.size == TrkSchema::kJointDim);
  REQUIRE(frame0->body_pos_w.size == TrkSchema::kBodyCount * 3);
  REQUIRE(frame0->left_foot_contact_state.ptr == track.left_foot_contact_state.values.data());
  REQUIRE(frame0->left_foot_contact_state.size == 1);
  REQUIRE(frame0->joint_pos.ptr[0] == expectedFloat("joint_pos", 0));
  REQUIRE(frame0->left_foot_contact_state.ptr[0] ==
          expectedContact("left_foot_contact_state", 0));

  const auto last = track.frame(2);
  REQUIRE(last.has_value());
  REQUIRE(last->joint_pos.ptr ==
          track.joint_pos.values.data() + 2 * static_cast<std::size_t>(TrkSchema::kJointDim));
  REQUIRE(last->right_foot_contact_state.ptr ==
          track.right_foot_contact_state.values.data() + 2);
  REQUIRE(last->ref_com_vel_navi.ptr[2] == expectedFloat("ref_com_vel_navi", 8));

  REQUIRE_FALSE(track.frame(3).has_value());
}

TEST_CASE("TrkLoader applies the same path gate as TrkValidator") {
  TempTree tmp;
  const auto outside = tmp.outside / "outside.trk";
  const auto link = tmp.allowed / "escape.trk";
  writeTrk(outside, requiredArrays());

  REQUIRE(loadPath(outside, tmp.config()).code == TrkValidationCode::PathNotAllowed);
  REQUIRE(loadPath(outside, tmp.config()).code == validatePath(outside, tmp.config()));

  std::error_code ec;
  std::filesystem::create_symlink(outside, link, ec);
  REQUIRE_FALSE(ec);

  REQUIRE(loadPath(link, tmp.config()).code == TrkValidationCode::SymlinkEscape);
  REQUIRE(loadPath(link, tmp.config()).code == validatePath(link, tmp.config()));
}

TEST_CASE("TrkLoader reads payload from the stream that scanner validated") {
  TempTree tmp;
  const auto path = tmp.allowed / "stable_handle.trk";
  writeTrk(path, requiredArrays(3));

  bool hook_called = false;
  ScopedScannerHook hook;
  trk_detail::setAfterSuccessfulScanHookForTesting([&](const TrkMetadata& metadata) {
    hook_called = true;
    const auto replacement_path = tmp.allowed / "replacement.trk";
    auto replacement = requiredArrays(3);
    auto& left = arrayNamed(replacement, "left_foot_contact_state");
    left.invalid_contact = true;
    left.invalid_contact_index = 0;
    left.invalid_contact_value = 3;
    writeTrk(replacement_path, replacement);

    std::error_code ec;
    std::filesystem::rename(replacement_path, metadata.canonical_path, ec);
    if (ec) {
      hook_called = false;
    }
  });

  const auto result = loadPath(path, tmp.config());
  trk_detail::setAfterSuccessfulScanHookForTesting({});

  REQUIRE(hook_called);
  REQUIRE(result.ok());
  REQUIRE(result.track->left_foot_contact_state.values.front() ==
          expectedContact("left_foot_contact_state", 0));

  const auto replaced_result = loadPath(path, tmp.config());
  REQUIRE(replaced_result.code == TrkValidationCode::InvalidContactValue);
}

}  // namespace agentic_et1_tracker
