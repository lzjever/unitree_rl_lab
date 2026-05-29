#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker::trk_detail {

struct TrkArrayDescriptor {
  std::string name;
  TrkDtype dtype{TrkDtype::Float32};
  std::vector<std::uint64_t> shape;
  std::uint64_t element_count{0};
  std::uint64_t byte_count{0};
  std::uint64_t payload_offset{0};
  std::optional<std::size_t> required_index;
};

struct TrkScannedFile {
  TrkValidationResult validation;
  std::array<TrkArrayDescriptor, TrkSchema::kRequiredArrays.size()> required_arrays{};
  std::ifstream file;

  bool ok() const { return validation.ok(); }
};

using TrkScanResult = TrkScannedFile;
using TrkAfterSuccessfulScanHookForTesting = std::function<void(const TrkMetadata&)>;

std::optional<std::size_t> requiredArrayIndex(std::string_view name);
TrkScannedFile scanTrkFile(const std::filesystem::path& path,
                           const TrkValidationConfig& config);
void setAfterSuccessfulScanHookForTesting(TrkAfterSuccessfulScanHookForTesting hook);

}  // namespace agentic_et1_tracker::trk_detail
