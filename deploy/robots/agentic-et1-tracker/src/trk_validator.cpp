#include "agentic_et1_tracker/trk/validator.hpp"

#include <utility>

#include "agentic_et1_tracker/trk/detail/scanner.hpp"

namespace agentic_et1_tracker {

TrkValidator::TrkValidator(TrkValidationConfig config) : config_(std::move(config)) {}

TrkValidationResult TrkValidator::validate(const std::filesystem::path& path) const {
  return trk_detail::scanTrkFile(path, config_).validation;
}

const char* toString(TrkValidationCode code) {
  switch (code) {
    case TrkValidationCode::Ok:
      return "OK";
    case TrkValidationCode::EmptyPath:
      return "EMPTY_PATH";
    case TrkValidationCode::UrlRejected:
      return "URL_REJECTED";
    case TrkValidationCode::RelativePathRejected:
      return "RELATIVE_PATH_REJECTED";
    case TrkValidationCode::ExtensionRejected:
      return "EXTENSION_REJECTED";
    case TrkValidationCode::FileNotFound:
      return "FILE_NOT_FOUND";
    case TrkValidationCode::PathNotAllowed:
      return "PATH_NOT_ALLOWED";
    case TrkValidationCode::SymlinkEscape:
      return "SYMLINK_ESCAPE";
    case TrkValidationCode::FileOpenFailed:
      return "FILE_OPEN_FAILED";
    case TrkValidationCode::ReadFailed:
      return "READ_FAILED";
    case TrkValidationCode::BadMagic:
      return "BAD_MAGIC";
    case TrkValidationCode::UnsupportedVersion:
      return "UNSUPPORTED_VERSION";
    case TrkValidationCode::ArrayCountLimitExceeded:
      return "ARRAY_COUNT_LIMIT_EXCEEDED";
    case TrkValidationCode::NameTooLong:
      return "NAME_TOO_LONG";
    case TrkValidationCode::InvalidName:
      return "INVALID_NAME";
    case TrkValidationCode::InvalidDtype:
      return "INVALID_DTYPE";
    case TrkValidationCode::NdimLimitExceeded:
      return "NDIM_LIMIT_EXCEEDED";
    case TrkValidationCode::ShapeOverflow:
      return "SHAPE_OVERFLOW";
    case TrkValidationCode::ByteCountMismatch:
      return "BYTE_COUNT_MISMATCH";
    case TrkValidationCode::ByteCountLimitExceeded:
      return "BYTE_COUNT_LIMIT_EXCEEDED";
    case TrkValidationCode::TotalPayloadLimitExceeded:
      return "TOTAL_PAYLOAD_LIMIT_EXCEEDED";
    case TrkValidationCode::PayloadOutOfBounds:
      return "PAYLOAD_OUT_OF_BOUNDS";
    case TrkValidationCode::DuplicateRequiredArray:
      return "DUPLICATE_REQUIRED_ARRAY";
    case TrkValidationCode::MissingRequiredArray:
      return "MISSING_REQUIRED_ARRAY";
    case TrkValidationCode::BadRequiredDtype:
      return "BAD_REQUIRED_DTYPE";
    case TrkValidationCode::BadRequiredShape:
      return "BAD_REQUIRED_SHAPE";
    case TrkValidationCode::FrameCountMismatch:
      return "FRAME_COUNT_MISMATCH";
    case TrkValidationCode::ZeroFrames:
      return "ZERO_FRAMES";
    case TrkValidationCode::DurationLimitExceeded:
      return "DURATION_LIMIT_EXCEEDED";
    case TrkValidationCode::InvalidContactValue:
      return "INVALID_CONTACT_VALUE";
  }
  return "UNKNOWN";
}

ErrorCode toCoreErrorCode(TrkValidationCode code) {
  switch (code) {
    case TrkValidationCode::Ok:
      return ErrorCode::Ok;
    case TrkValidationCode::ExtensionRejected:
    case TrkValidationCode::EmptyPath:
    case TrkValidationCode::UrlRejected:
    case TrkValidationCode::RelativePathRejected:
      return ErrorCode::RequestInvalid;
    case TrkValidationCode::FileNotFound:
      return ErrorCode::TrkFileNotFound;
    case TrkValidationCode::PathNotAllowed:
    case TrkValidationCode::SymlinkEscape:
      return ErrorCode::TrkPathNotAllowed;
    case TrkValidationCode::FileOpenFailed:
    case TrkValidationCode::ReadFailed:
    case TrkValidationCode::BadMagic:
    case TrkValidationCode::UnsupportedVersion:
      return ErrorCode::TrkParseFailed;
    case TrkValidationCode::InvalidContactValue:
      return ErrorCode::TrkValidationFailed;
    default:
      return ErrorCode::TrkValidationFailed;
  }
}

}  // namespace agentic_et1_tracker
