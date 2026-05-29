#include "agentic_et1_tracker/trk/detail/scanner.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <exception>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace agentic_et1_tracker::trk_detail {
namespace {

template <typename T>
bool readScalar(std::ifstream& in, T& out) {
  in.read(reinterpret_cast<char*>(&out), sizeof(T));
  return static_cast<bool>(in);
}

bool readBytes(std::ifstream& in, char* out, std::size_t size) {
  in.read(out, static_cast<std::streamsize>(size));
  return static_cast<bool>(in);
}

TrkValidationResult validationFailure(TrkValidationCode code, std::string message = {}) {
  TrkValidationResult result;
  result.code = code;
  result.message = std::move(message);
  return result;
}

TrkScannedFile scanFailure(TrkValidationCode code, std::string message = {}) {
  TrkScannedFile result;
  result.validation = validationFailure(code, std::move(message));
  return result;
}

TrkAfterSuccessfulScanHookForTesting& afterSuccessfulScanHookForTesting() {
  static TrkAfterSuccessfulScanHookForTesting hook;
  return hook;
}

bool checkedMul(std::uint64_t lhs, std::uint64_t rhs, std::uint64_t& out) {
  if (lhs != 0 && rhs > std::numeric_limits<std::uint64_t>::max() / lhs) {
    return false;
  }
  out = lhs * rhs;
  return true;
}

bool checkedAdd(std::uint64_t lhs, std::uint64_t rhs, std::uint64_t& out) {
  if (rhs > std::numeric_limits<std::uint64_t>::max() - lhs) {
    return false;
  }
  out = lhs + rhs;
  return true;
}

bool pathStartsWith(const std::filesystem::path& path, const std::filesystem::path& base) {
  auto path_it = path.begin();
  auto base_it = base.begin();
  for (; base_it != base.end(); ++base_it, ++path_it) {
    if (path_it == path.end() || *path_it != *base_it) {
      return false;
    }
  }
  return true;
}

std::vector<std::filesystem::path> canonicalAllowDirs(
    const std::vector<std::filesystem::path>& dirs) {
  std::vector<std::filesystem::path> out;
  out.reserve(dirs.size());
  for (const auto& dir : dirs) {
    std::error_code ec;
    const auto canonical = std::filesystem::canonical(dir, ec);
    if (!ec) {
      out.push_back(canonical.lexically_normal());
    }
  }
  return out;
}

bool insideAny(const std::filesystem::path& path,
               const std::vector<std::filesystem::path>& dirs) {
  return std::any_of(dirs.begin(), dirs.end(), [&](const std::filesystem::path& dir) {
    return pathStartsWith(path, dir);
  });
}

bool isUriSchemeChar(char ch) {
  const auto uch = static_cast<unsigned char>(ch);
  return std::isalnum(uch) || ch == '+' || ch == '-' || ch == '.';
}

bool looksLikeUrl(const std::filesystem::path& path) {
  const std::string text = path.generic_string();
  const auto colon = text.find(':');
  if (colon == std::string::npos || colon == 0) {
    return false;
  }
  if (!std::isalpha(static_cast<unsigned char>(text.front()))) {
    return false;
  }
  const auto slash = text.find('/');
  if (slash != std::string::npos && slash < colon) {
    return false;
  }
  return std::all_of(text.begin() + 1, text.begin() + static_cast<std::ptrdiff_t>(colon),
                     isUriSchemeChar);
}

bool requiredShapeMatches(const TrkArrayDescriptor& array,
                          const TrkRequiredArraySpec& spec,
                          std::uint64_t expected_frames) {
  if (array.shape.size() != static_cast<std::size_t>(spec.trailing_rank + 1)) {
    return false;
  }
  if (array.shape.empty() || array.shape[0] != expected_frames) {
    return false;
  }
  for (std::uint32_t i = 0; i < spec.trailing_rank; ++i) {
    if (array.shape[static_cast<std::size_t>(i + 1)] != spec.trailing_shape[i]) {
      return false;
    }
  }
  return true;
}

TrkValidationResult validateRequiredArrays(
    const std::array<std::optional<TrkArrayDescriptor>,
                     TrkSchema::kRequiredArrays.size()>& found,
    double fps,
    double max_duration_s,
    TrkMetadata metadata) {
  const auto joint_pos_index = requiredArrayIndex("joint_pos");
  if (!joint_pos_index || !found[*joint_pos_index].has_value()) {
    return validationFailure(TrkValidationCode::MissingRequiredArray, "missing joint_pos");
  }

  const TrkArrayDescriptor& joint_pos = *found[*joint_pos_index];
  if (joint_pos.shape.size() != 2 || joint_pos.shape[1] != TrkSchema::kJointDim) {
    return validationFailure(TrkValidationCode::BadRequiredShape, "joint_pos shape mismatch");
  }

  const std::uint64_t frames = joint_pos.shape[0];
  if (frames == 0) {
    return validationFailure(TrkValidationCode::ZeroFrames, "frame count is zero");
  }

  for (std::size_t i = 0; i < found.size(); ++i) {
    if (!found[i].has_value()) {
      return validationFailure(
          TrkValidationCode::MissingRequiredArray,
          std::string("missing ") + std::string(TrkSchema::kRequiredArrays[i].name));
    }
  }

  for (std::size_t i = 0; i < found.size(); ++i) {
    const TrkArrayDescriptor& array = *found[i];
    const TrkRequiredArraySpec& spec = TrkSchema::kRequiredArrays[i];
    if (!trkDtypeAllowed(array.dtype, spec.dtype_family)) {
      return validationFailure(TrkValidationCode::BadRequiredDtype,
                               std::string("bad dtype for ") + std::string(spec.name));
    }
    if (array.shape.empty() || array.shape[0] != frames) {
      return validationFailure(TrkValidationCode::FrameCountMismatch,
                               std::string("frame count mismatch for ") +
                                   std::string(spec.name));
    }
    if (!requiredShapeMatches(array, spec, frames)) {
      return validationFailure(TrkValidationCode::BadRequiredShape,
                               std::string("shape mismatch for ") + std::string(spec.name));
    }
  }

  if (fps <= 0.0) {
    return validationFailure(TrkValidationCode::DurationLimitExceeded, "fps must be positive");
  }
  const double duration_s = static_cast<double>(frames - 1) / fps;
  if (duration_s > max_duration_s) {
    return validationFailure(TrkValidationCode::DurationLimitExceeded,
                             "track duration exceeds limit");
  }

  metadata.frames = static_cast<std::size_t>(frames);
  metadata.duration_s = duration_s;
  metadata.fps = fps;

  TrkValidationResult result;
  result.code = TrkValidationCode::Ok;
  result.metadata = std::move(metadata);
  return result;
}

TrkValidationResult readArrayMetadata(std::ifstream& in,
                                      const TrkLimits& limits,
                                      std::uint64_t file_size,
                                      std::uint64_t& total_payload_bytes,
                                      TrkArrayDescriptor& out) {
  std::uint32_t name_len = 0;
  if (!readScalar(in, name_len)) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to read name length");
  }
  if (name_len > limits.max_name_len) {
    return validationFailure(TrkValidationCode::NameTooLong, "array name is too long");
  }
  if (name_len == 0) {
    return validationFailure(TrkValidationCode::InvalidName, "array name is empty");
  }

  std::string name(name_len, '\0');
  if (!readBytes(in, name.data(), name.size())) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to read array name");
  }
  if (name.find('\0') != std::string::npos) {
    return validationFailure(TrkValidationCode::InvalidName, "array name contains null");
  }

  std::uint32_t raw_dtype = 0;
  if (!readScalar(in, raw_dtype)) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to read dtype");
  }
  if (!trkIsKnownDtype(raw_dtype)) {
    return validationFailure(TrkValidationCode::InvalidDtype, "unknown dtype");
  }

  std::uint32_t ndim = 0;
  if (!readScalar(in, ndim)) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to read ndim");
  }
  if (ndim > limits.max_ndim) {
    return validationFailure(TrkValidationCode::NdimLimitExceeded, "ndim exceeds limit");
  }

  std::vector<std::uint64_t> shape;
  shape.reserve(ndim);
  for (std::uint32_t i = 0; i < ndim; ++i) {
    std::uint32_t dim = 0;
    if (!readScalar(in, dim)) {
      return validationFailure(TrkValidationCode::ReadFailed, "failed to read shape");
    }
    shape.push_back(dim);
  }

  std::uint64_t byte_count = 0;
  if (!readScalar(in, byte_count)) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to read byte count");
  }

  std::uint64_t element_count = 1;
  for (const auto dim : shape) {
    if (!checkedMul(element_count, dim, element_count)) {
      return validationFailure(TrkValidationCode::ShapeOverflow,
                               "shape element count overflow");
    }
  }

  const TrkDtype dtype = static_cast<TrkDtype>(raw_dtype);
  std::uint64_t expected_bytes = 0;
  if (!checkedMul(element_count, trkDtypeSize(dtype), expected_bytes)) {
    return validationFailure(TrkValidationCode::ShapeOverflow, "byte count overflow");
  }
  if (byte_count != expected_bytes) {
    return validationFailure(TrkValidationCode::ByteCountMismatch,
                             "byte count does not match shape");
  }
  if (byte_count > limits.max_single_array_bytes) {
    return validationFailure(TrkValidationCode::ByteCountLimitExceeded,
                             "array byte count exceeds limit");
  }

  std::uint64_t new_total = 0;
  if (!checkedAdd(total_payload_bytes, byte_count, new_total)) {
    return validationFailure(TrkValidationCode::TotalPayloadLimitExceeded,
                             "total payload overflow");
  }
  if (new_total > limits.max_total_payload_bytes) {
    return validationFailure(TrkValidationCode::TotalPayloadLimitExceeded,
                             "total payload exceeds limit");
  }
  total_payload_bytes = new_total;

  const auto pos = in.tellg();
  if (pos < std::streampos(0)) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to get payload offset");
  }
  const auto payload_offset = static_cast<std::uint64_t>(pos);
  std::uint64_t payload_end = 0;
  if (!checkedAdd(payload_offset, byte_count, payload_end) || payload_end > file_size) {
    return validationFailure(TrkValidationCode::PayloadOutOfBounds, "payload exceeds file size");
  }

  in.seekg(static_cast<std::streamoff>(byte_count), std::ios::cur);
  if (!in) {
    return validationFailure(TrkValidationCode::ReadFailed, "failed to skip payload");
  }

  out.name = std::move(name);
  out.dtype = dtype;
  out.shape = std::move(shape);
  out.element_count = element_count;
  out.byte_count = byte_count;
  out.payload_offset = payload_offset;
  out.required_index = requiredArrayIndex(out.name);
  return TrkValidationResult{};
}

TrkScannedFile scanContents(const std::filesystem::path& canonical_path,
                            const TrkValidationConfig& config,
                            std::uint64_t file_size,
                            std::ifstream file) {
  std::ifstream& in = file;
  std::array<char, 8> magic{};
  if (!readBytes(in, magic.data(), magic.size())) {
    return scanFailure(TrkValidationCode::ReadFailed, "failed to read magic");
  }
  if (magic != TrkSchema::kMagic) {
    return scanFailure(TrkValidationCode::BadMagic, "bad magic");
  }

  std::uint32_t version = 0;
  if (!readScalar(in, version)) {
    return scanFailure(TrkValidationCode::ReadFailed, "failed to read version");
  }
  if (version != TrkSchema::kVersion) {
    return scanFailure(TrkValidationCode::UnsupportedVersion, "unsupported track version");
  }

  std::uint32_t array_count = 0;
  if (!readScalar(in, array_count)) {
    return scanFailure(TrkValidationCode::ReadFailed, "failed to read array count");
  }
  if (array_count > config.limits.max_array_count) {
    return scanFailure(TrkValidationCode::ArrayCountLimitExceeded,
                       "array count exceeds limit");
  }

  std::array<std::optional<TrkArrayDescriptor>, TrkSchema::kRequiredArrays.size()> found{};
  std::uint64_t total_payload_bytes = 0;

  for (std::uint32_t i = 0; i < array_count; ++i) {
    TrkArrayDescriptor parsed;
    auto result =
        readArrayMetadata(in, config.limits, file_size, total_payload_bytes, parsed);
    if (!result.ok()) {
      TrkScannedFile failed;
      failed.validation = std::move(result);
      return failed;
    }

    if (parsed.required_index) {
      if (found[*parsed.required_index].has_value()) {
        return scanFailure(TrkValidationCode::DuplicateRequiredArray,
                           std::string("duplicate ") + parsed.name);
      }
      found[*parsed.required_index] = std::move(parsed);
    }
  }

  TrkMetadata metadata;
  metadata.canonical_path = canonical_path;
  metadata.version = version;
  metadata.array_count = array_count;
  metadata.file_size = file_size;

  TrkScannedFile scan;
  scan.validation =
      validateRequiredArrays(found, config.fps, config.max_duration_s, std::move(metadata));
  if (!scan.validation.ok()) {
    return scan;
  }

  for (std::size_t i = 0; i < found.size(); ++i) {
    scan.required_arrays[i] = std::move(*found[i]);
  }
  scan.file = std::move(file);
  if (auto& hook = afterSuccessfulScanHookForTesting()) {
    hook(scan.validation.metadata);
  }
  return scan;
}

std::optional<std::uint64_t> streamSize(std::ifstream& file) {
  file.seekg(0, std::ios::end);
  const auto end = file.tellg();
  if (end < std::streampos(0)) {
    return std::nullopt;
  }
  file.seekg(0, std::ios::beg);
  if (!file) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(end);
}

TrkScannedFile scanTrkFileImpl(const std::filesystem::path& path,
                               const TrkValidationConfig& config) {
  if (path.empty()) {
    return scanFailure(TrkValidationCode::EmptyPath, "track path is empty");
  }
  if (looksLikeUrl(path)) {
    return scanFailure(TrkValidationCode::UrlRejected, "track path must be a local path");
  }
  if (path.extension() != ".trk") {
    return scanFailure(TrkValidationCode::ExtensionRejected, "track path must end with .trk");
  }
  if (!path.is_absolute()) {
    return scanFailure(TrkValidationCode::RelativePathRejected, "track path must be absolute");
  }

  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    return scanFailure(TrkValidationCode::FileNotFound, "track file does not exist");
  }

  const auto canonical_file = std::filesystem::canonical(path, ec);
  if (ec) {
    return scanFailure(TrkValidationCode::FileNotFound, "track file cannot be resolved");
  }

  const auto allow_dirs = canonicalAllowDirs(config.allowlist_dirs);
  if (allow_dirs.empty()) {
    return scanFailure(TrkValidationCode::PathNotAllowed, "no valid allowlist directories");
  }

  const auto canonical_normal = canonical_file.lexically_normal();
  if (!insideAny(canonical_normal, allow_dirs)) {
    const auto lexical_abs = std::filesystem::absolute(path, ec).lexically_normal();
    if (!ec && insideAny(lexical_abs, allow_dirs)) {
      return scanFailure(TrkValidationCode::SymlinkEscape, "symlink target escapes allowlist");
    }
    return scanFailure(TrkValidationCode::PathNotAllowed, "track path is outside allowlist");
  }

  std::ifstream file(canonical_normal, std::ios::binary);
  if (!file) {
    return scanFailure(TrkValidationCode::FileOpenFailed, "failed to open track file");
  }

  const auto file_size = streamSize(file);
  if (!file_size) {
    return scanFailure(TrkValidationCode::FileOpenFailed,
                       "failed to get track file size");
  }

  return scanContents(canonical_normal, config, *file_size, std::move(file));
}

}  // namespace

std::optional<std::size_t> requiredArrayIndex(std::string_view name) {
  const auto& specs = TrkSchema::kRequiredArrays;
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (specs[i].name == name) {
      return i;
    }
  }
  return std::nullopt;
}

TrkScannedFile scanTrkFile(const std::filesystem::path& path,
                           const TrkValidationConfig& config) {
  try {
    return scanTrkFileImpl(path, config);
  } catch (const std::exception& e) {
    return scanFailure(TrkValidationCode::ReadFailed, e.what());
  } catch (...) {
    return scanFailure(TrkValidationCode::ReadFailed, "unexpected scanner failure");
  }
}

void setAfterSuccessfulScanHookForTesting(TrkAfterSuccessfulScanHookForTesting hook) {
  afterSuccessfulScanHookForTesting() = std::move(hook);
}

}  // namespace agentic_et1_tracker::trk_detail
