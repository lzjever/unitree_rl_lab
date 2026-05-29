#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>

namespace agentic_et1_tracker {

class ShortIdGenerator {
 public:
  ShortIdGenerator(std::size_t min_length = 8, std::size_t max_length = 10);

  std::string generate();

  static bool isBase62Id(const std::string& id);

 private:
  std::string encode(std::uint64_t value) const;

  std::size_t min_length_;
  std::size_t max_length_;
  std::uint64_t next_{0};
  std::unordered_set<std::string> issued_;
  std::mutex mutex_;
};

}  // namespace agentic_et1_tracker
