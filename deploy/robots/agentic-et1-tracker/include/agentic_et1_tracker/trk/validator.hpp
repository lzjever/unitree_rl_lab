#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {

enum class TrkValidationCode {
  Ok,
  EmptyPath,
  UrlRejected,
  RelativePathRejected,
  ExtensionRejected,
  FileNotFound,
  PathNotAllowed,
  SymlinkEscape,
  FileOpenFailed,
  ReadFailed,
  BadMagic,
  UnsupportedVersion,
  ArrayCountLimitExceeded,
  NameTooLong,
  InvalidName,
  InvalidDtype,
  NdimLimitExceeded,
  ShapeOverflow,
  ByteCountMismatch,
  ByteCountLimitExceeded,
  TotalPayloadLimitExceeded,
  PayloadOutOfBounds,
  DuplicateRequiredArray,
  MissingRequiredArray,
  BadRequiredDtype,
  BadRequiredShape,
  FrameCountMismatch,
  ZeroFrames,
  DurationLimitExceeded,
  InvalidContactValue,
};

struct TrkValidationConfig {
  std::vector<std::filesystem::path> allowlist_dirs;
  double fps{TrkSchema::kDefaultFps};
  double max_duration_s{120.0};
  TrkLimits limits{TrkSchema::kDefaultLimits};
};

struct TrkMetadata {
  std::filesystem::path canonical_path;
  std::size_t frames{0};
  double duration_s{0.0};
  double fps{TrkSchema::kDefaultFps};
  std::uint32_t version{0};
  std::uint32_t array_count{0};
  std::uint64_t file_size{0};
};

struct TrkValidationResult {
  TrkValidationCode code{TrkValidationCode::Ok};
  TrkMetadata metadata;
  std::string message;

  bool ok() const { return code == TrkValidationCode::Ok; }
};

class TrkValidator {
 public:
  explicit TrkValidator(TrkValidationConfig config);

  TrkValidationResult validate(const std::filesystem::path& path) const;

 private:
  TrkValidationConfig config_;
};

const char* toString(TrkValidationCode code);
ErrorCode toCoreErrorCode(TrkValidationCode code);

}  // namespace agentic_et1_tracker
