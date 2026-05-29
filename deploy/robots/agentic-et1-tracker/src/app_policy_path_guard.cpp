#include "app_policy_path_guard.hpp"

#include <string>
#include <vector>

namespace agentic_et1_tracker {
namespace app_internal {
namespace {

bool hasEt1PolicyPath(const std::filesystem::path& path) {
  const std::filesystem::path normalized = path.lexically_normal();
  std::vector<std::string> parts;
  for (const auto& part : normalized) {
    parts.push_back(part.generic_string());
  }

  constexpr const char* kEt1Policy[] = {"deploy", "robots", "et1", "config", "policy"};
  constexpr std::size_t kNeedleSize = sizeof(kEt1Policy) / sizeof(kEt1Policy[0]);
  if (parts.size() < kNeedleSize) {
    return false;
  }

  for (std::size_t i = 0; i + kNeedleSize <= parts.size(); ++i) {
    bool match = true;
    for (std::size_t j = 0; j < kNeedleSize; ++j) {
      if (parts[i + j] != kEt1Policy[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      return true;
    }
  }
  return false;
}

bool hasEt1ConfigPath(const std::filesystem::path& path) {
  const std::filesystem::path normalized = path.lexically_normal();
  std::vector<std::string> parts;
  for (const auto& part : normalized) {
    parts.push_back(part.generic_string());
  }

  constexpr const char* kEt1Config[] = {"deploy", "robots", "et1", "config",
                                        "config.yaml"};
  constexpr std::size_t kNeedleSize = sizeof(kEt1Config) / sizeof(kEt1Config[0]);
  if (parts.size() < kNeedleSize) {
    return false;
  }

  for (std::size_t i = 0; i + kNeedleSize <= parts.size(); ++i) {
    bool match = true;
    for (std::size_t j = 0; j < kNeedleSize; ++j) {
      if (parts[i + j] != kEt1Config[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      return true;
    }
  }
  return false;
}

std::filesystem::path weaklyCanonicalPath(const std::filesystem::path& path) {
  std::error_code ec;
  const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
  if (ec) {
    return path.lexically_normal();
  }
  return canonical.lexically_normal();
}

}  // namespace

bool referencesEt1PolicyTree(const std::filesystem::path& path) {
  return referencesEt1RuntimeDependency(path);
}

bool referencesEt1RuntimeDependency(const std::filesystem::path& path) {
  const std::filesystem::path canonical = weaklyCanonicalPath(path);
  return hasEt1PolicyPath(path) || hasEt1ConfigPath(path) ||
         hasEt1PolicyPath(canonical) || hasEt1ConfigPath(canonical);
}

}  // namespace app_internal
}  // namespace agentic_et1_tracker
