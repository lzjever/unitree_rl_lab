#pragma once

#include <filesystem>

namespace agentic_et1_tracker {
namespace app_internal {

bool referencesEt1PolicyTree(const std::filesystem::path& path);
bool referencesEt1RuntimeDependency(const std::filesystem::path& path);

}  // namespace app_internal
}  // namespace agentic_et1_tracker
