#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/trk/schema.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

struct TempTree {
  TempTree() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_trk_tests_" + std::to_string(now));
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
  bool write_shape_as_uint64{false};
};

template <typename T>
void writeScalar(std::ofstream& out, T value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

std::uint64_t checkedBytes(const std::vector<std::uint64_t>& shape, TrkDtype dtype) {
  std::uint64_t elements = 1;
  for (const auto dim : shape) {
    elements *= dim;
  }
  return elements * trkDtypeSize(dtype);
}

void writePayload(std::ofstream& out, std::uint64_t byte_count) {
  if (byte_count == 0) {
    return;
  }
  out.seekp(static_cast<std::streamoff>(byte_count - 1), std::ios::cur);
  const char zero = 0;
  out.write(&zero, 1);
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
    if (array.write_payload) {
      writePayload(out, byte_count);
    }
  }
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

std::vector<ArrayFixture> validArrays(std::uint64_t frames = 3) {
  std::vector<ArrayFixture> arrays;
  for (const auto& spec : TrkSchema::kRequiredArrays) {
    arrays.push_back({std::string(spec.name), defaultDtypeFor(spec), shapeFor(spec, frames)});
  }
  return arrays;
}

TrkValidationCode validatePath(const std::filesystem::path& path, const TrkValidationConfig& cfg) {
  return TrkValidator(cfg).validate(path).code;
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

void writeMalformedNameLen(const std::filesystem::path& path, std::uint32_t name_len) {
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);
  out.write(TrkSchema::kMagic.data(), static_cast<std::streamsize>(TrkSchema::kMagic.size()));
  writeScalar(out, TrkSchema::kVersion);
  writeScalar(out, std::uint32_t{1});
  writeScalar(out, name_len);
  std::string name(name_len, 'x');
  out.write(name.data(), static_cast<std::streamsize>(name.size()));
}

}  // namespace

TEST_CASE("TrkSchema exposes the app-local ET1TRK1 GeneralTracker contract") {
  REQUIRE(TrkSchema::kMagic == std::array<char, 8>{{'E', 'T', '1', 'T', 'R', 'K', '1', '\0'}});
  REQUIRE(TrkSchema::kVersion == 1);
  REQUIRE(TrkSchema::kJointDim == 26);
  REQUIRE(TrkSchema::kBodyCount == 27);
  REQUIRE(static_cast<std::uint32_t>(TrkDtype::Float32) == 1);
  REQUIRE(static_cast<std::uint32_t>(TrkDtype::Int8) == 7);
  REQUIRE(TrkSchema::kRequiredArrays.size() == 10);
  REQUIRE(TrkSchema::kRequiredArrays.front().name == "joint_pos");
  REQUIRE(TrkSchema::kRequiredArrays.front().trailing_shape[0] == 26);
  REQUIRE(TrkSchema::kRequiredArrays.back().name == "ref_com_vel_navi");
}

TEST_CASE("TrkValidator accepts a valid trk and returns frame metadata") {
  TempTree tmp;
  const auto path = tmp.allowed / "valid.trk";
  writeTrk(path, validArrays(6));

  const auto result = TrkValidator(tmp.config(50.0, 120.0)).validate(path);

  REQUIRE(result.ok());
  REQUIRE(result.metadata.frames == 6);
  REQUIRE(result.metadata.duration_s == 0.1);
  REQUIRE(result.metadata.version == TrkSchema::kVersion);
  REQUIRE(result.metadata.array_count == TrkSchema::kRequiredArrays.size());
}

TEST_CASE("TrkValidator treats ET1TRK1 v1 shape as uint32 wire fields") {
  TempTree tmp;
  const auto valid = tmp.allowed / "uint32_shape.trk";
  writeTrk(valid, validArrays(3));

  REQUIRE(validatePath(valid, tmp.config()) == TrkValidationCode::Ok);

  auto legacy_arrays = validArrays(3);
  for (auto& array : legacy_arrays) {
    array.write_shape_as_uint64 = true;
  }
  const auto legacy = tmp.allowed / "uint64_shape.trk";
  writeTrk(legacy, legacy_arrays);

  REQUIRE(validatePath(legacy, tmp.config()) != TrkValidationCode::Ok);
}

TEST_CASE("TrkValidator accepts allowed required dtype variants") {
  TempTree tmp;
  auto arrays = validArrays();
  arrayNamed(arrays, "joint_pos").dtype = TrkDtype::Float64;
  arrayNamed(arrays, "left_foot_contact_state").dtype = TrkDtype::UInt8;
  arrayNamed(arrays, "right_foot_contact_state").dtype = TrkDtype::Int8;
  const auto path = tmp.allowed / "dtype_variants.trk";
  writeTrk(path, arrays);

  REQUIRE(validatePath(path, tmp.config()) == TrkValidationCode::Ok);
}

TEST_CASE("TrkValidator only accepts .trk extension") {
  TempTree tmp;
  const auto et1trk = tmp.allowed / "valid.et1trk";
  const auto npz = tmp.allowed / "valid.npz";
  writeTrk(et1trk, validArrays());
  writeTrk(npz, validArrays());

  REQUIRE(validatePath(et1trk, tmp.config()) == TrkValidationCode::ExtensionRejected);
  REQUIRE(validatePath(npz, tmp.config()) == TrkValidationCode::ExtensionRejected);
}

TEST_CASE("TrkValidator rejects URLs relative paths empty paths and non-trk before IO") {
  TempTree tmp;
  const auto path = tmp.allowed / "valid.trk";
  writeTrk(path, validArrays());

  const auto relative = std::filesystem::relative(path, std::filesystem::current_path());

  REQUIRE(validatePath("", tmp.config()) == TrkValidationCode::EmptyPath);
  REQUIRE(validatePath("https://example.com/motion.trk", tmp.config()) ==
          TrkValidationCode::UrlRejected);
  REQUIRE(validatePath("file:///tmp/motion.trk", tmp.config()) ==
          TrkValidationCode::UrlRejected);
  REQUIRE(validatePath(relative, tmp.config()) == TrkValidationCode::RelativePathRejected);
  REQUIRE(validatePath(tmp.allowed / "valid.npz", tmp.config()) ==
          TrkValidationCode::ExtensionRejected);
}

TEST_CASE("TrkValidator rejects missing and allowlist-external paths") {
  TempTree tmp;
  const auto missing = tmp.allowed / "missing.trk";
  const auto outside = tmp.outside / "outside.trk";
  writeTrk(outside, validArrays());

  REQUIRE(validatePath(missing, tmp.config()) == TrkValidationCode::FileNotFound);
  REQUIRE(validatePath(outside, tmp.config()) == TrkValidationCode::PathNotAllowed);
}

TEST_CASE("TrkValidator rejects symlinks that escape the allowlist") {
  TempTree tmp;
  const auto outside = tmp.outside / "outside.trk";
  const auto link = tmp.allowed / "escape.trk";
  writeTrk(outside, validArrays());

  std::error_code ec;
  std::filesystem::create_symlink(outside, link, ec);
  REQUIRE_FALSE(ec);

  REQUIRE(validatePath(link, tmp.config()) == TrkValidationCode::SymlinkEscape);
}

TEST_CASE("TrkValidator rejects bad magic and unsupported version") {
  TempTree tmp;
  const auto bad_magic = tmp.allowed / "bad_magic.trk";
  const auto bad_version = tmp.allowed / "bad_version.trk";

  {
    std::ofstream out(bad_magic, std::ios::binary);
    const std::array<char, 8> magic{{'N', 'O', 'T', 'T', 'R', 'K', '1', '\0'}};
    out.write(magic.data(), static_cast<std::streamsize>(magic.size()));
  }
  writeTrk(bad_version, validArrays(), TrkSchema::kVersion + 1);

  REQUIRE(validatePath(bad_magic, tmp.config()) == TrkValidationCode::BadMagic);
  REQUIRE(validatePath(bad_version, tmp.config()) == TrkValidationCode::UnsupportedVersion);
}

TEST_CASE("TrkValidator rejects zero frames and duration overflow") {
  TempTree tmp;
  const auto zero = tmp.allowed / "zero.trk";
  const auto too_long = tmp.allowed / "too_long.trk";
  writeTrk(zero, validArrays(0));
  writeTrk(too_long, validArrays(7));

  REQUIRE(validatePath(zero, tmp.config()) == TrkValidationCode::ZeroFrames);
  REQUIRE(validatePath(too_long, tmp.config(2.0, 2.0)) ==
          TrkValidationCode::DurationLimitExceeded);
}

TEST_CASE("TrkValidator rejects missing duplicate and malformed required arrays") {
  TempTree tmp;

  auto missing_arrays = validArrays();
  missing_arrays.erase(missing_arrays.begin() + 1);
  const auto missing = tmp.allowed / "missing_required.trk";
  writeTrk(missing, missing_arrays);
  REQUIRE(validatePath(missing, tmp.config()) == TrkValidationCode::MissingRequiredArray);

  auto duplicate_arrays = validArrays();
  duplicate_arrays.push_back(duplicate_arrays.front());
  const auto duplicate = tmp.allowed / "duplicate_required.trk";
  writeTrk(duplicate, duplicate_arrays);
  REQUIRE(validatePath(duplicate, tmp.config()) == TrkValidationCode::DuplicateRequiredArray);

  auto bad_dtype_arrays = validArrays();
  arrayNamed(bad_dtype_arrays, "joint_pos").dtype = TrkDtype::Int32;
  const auto bad_dtype = tmp.allowed / "bad_required_dtype.trk";
  writeTrk(bad_dtype, bad_dtype_arrays);
  REQUIRE(validatePath(bad_dtype, tmp.config()) == TrkValidationCode::BadRequiredDtype);

  auto bad_shape_arrays = validArrays();
  arrayNamed(bad_shape_arrays, "joint_vel").shape = {3, TrkSchema::kJointDim - 1};
  const auto bad_shape = tmp.allowed / "bad_required_shape.trk";
  writeTrk(bad_shape, bad_shape_arrays);
  REQUIRE(validatePath(bad_shape, tmp.config()) == TrkValidationCode::BadRequiredShape);

  auto mismatch_arrays = validArrays(3);
  arrayNamed(mismatch_arrays, "body_pos_w").shape = {4, TrkSchema::kBodyCount, 3};
  const auto mismatch = tmp.allowed / "frame_mismatch.trk";
  writeTrk(mismatch, mismatch_arrays);
  REQUIRE(validatePath(mismatch, tmp.config()) == TrkValidationCode::FrameCountMismatch);
}

TEST_CASE("TrkValidator enforces parser metadata limits") {
  TempTree tmp;

  const auto too_many = tmp.allowed / "too_many.trk";
  writeTrk(too_many, {}, TrkSchema::kVersion, TrkSchema::kDefaultLimits.max_array_count + 1);
  REQUIRE(validatePath(too_many, tmp.config()) == TrkValidationCode::ArrayCountLimitExceeded);

  const auto name_too_long = tmp.allowed / "name_too_long.trk";
  writeMalformedNameLen(name_too_long, TrkSchema::kDefaultLimits.max_name_len + 1);
  REQUIRE(validatePath(name_too_long, tmp.config()) == TrkValidationCode::NameTooLong);

  auto invalid_dtype_arrays = validArrays();
  invalid_dtype_arrays.push_back({"unknown_dtype", static_cast<TrkDtype>(99), {1}});
  const auto invalid_dtype = tmp.allowed / "invalid_dtype.trk";
  writeTrk(invalid_dtype, invalid_dtype_arrays);
  REQUIRE(validatePath(invalid_dtype, tmp.config()) == TrkValidationCode::InvalidDtype);

  auto too_many_dims = validArrays();
  too_many_dims.push_back({"too_many_dims", TrkDtype::Float32, {1, 1, 1, 1, 1}});
  const auto ndim = tmp.allowed / "ndim.trk";
  writeTrk(ndim, too_many_dims);
  REQUIRE(validatePath(ndim, tmp.config()) == TrkValidationCode::NdimLimitExceeded);

  auto shape_overflow = validArrays();
  shape_overflow.push_back({"shape_overflow",
                            TrkDtype::UInt8,
                            {std::numeric_limits<std::uint32_t>::max(),
                             std::numeric_limits<std::uint32_t>::max(),
                             2},
                            0,
                            true,
                            false,
                            false});
  const auto overflow = tmp.allowed / "shape_overflow.trk";
  writeTrk(overflow, shape_overflow);
  REQUIRE(validatePath(overflow, tmp.config()) == TrkValidationCode::ShapeOverflow);
}

TEST_CASE("TrkValidator enforces byte counts and payload bounds without loading payloads") {
  TempTree tmp;

  auto mismatch_arrays = validArrays();
  auto& joint_pos = arrayNamed(mismatch_arrays, "joint_pos");
  joint_pos.has_byte_count_override = true;
  joint_pos.byte_count_override = checkedBytes(joint_pos.shape, joint_pos.dtype) + 1;
  const auto mismatch = tmp.allowed / "byte_mismatch.trk";
  writeTrk(mismatch, mismatch_arrays);
  REQUIRE(validatePath(mismatch, tmp.config()) == TrkValidationCode::ByteCountMismatch);

  auto oversized_arrays = validArrays();
  oversized_arrays.push_back({"oversized",
                              TrkDtype::UInt8,
                              {TrkSchema::kDefaultLimits.max_single_array_bytes + 1},
                              0,
                              false,
                              false});
  const auto oversized = tmp.allowed / "oversized.trk";
  writeTrk(oversized, oversized_arrays);
  REQUIRE(validatePath(oversized, tmp.config()) == TrkValidationCode::ByteCountLimitExceeded);

  auto truncated_arrays = validArrays();
  truncated_arrays.push_back({"truncated", TrkDtype::UInt8, {64}, 0, false, false});
  const auto truncated = tmp.allowed / "truncated.trk";
  writeTrk(truncated, truncated_arrays);
  REQUIRE(validatePath(truncated, tmp.config()) == TrkValidationCode::PayloadOutOfBounds);

  auto accepted_arrays = validArrays();
  accepted_arrays.push_back({"large_unknown", TrkDtype::UInt8, {16ULL * 1024ULL * 1024ULL}});
  const auto accepted = tmp.allowed / "large_unknown.trk";
  writeTrk(accepted, accepted_arrays);
  REQUIRE(validatePath(accepted, tmp.config()) == TrkValidationCode::Ok);
}

TEST_CASE("TrkValidator enforces total payload limit") {
  TempTree tmp;
  auto arrays = validArrays();
  arrays.push_back({"unknown_a", TrkDtype::UInt8, {70}});
  arrays.push_back({"unknown_b", TrkDtype::UInt8, {70}});
  const auto path = tmp.allowed / "total_limit.trk";
  writeTrk(path, arrays);

  auto cfg = tmp.config();
  cfg.limits.max_total_payload_bytes = 128;

  REQUIRE(validatePath(path, cfg) == TrkValidationCode::TotalPayloadLimitExceeded);
}

}  // namespace agentic_et1_tracker
