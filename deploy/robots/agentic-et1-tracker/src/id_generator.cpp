#include "agentic_et1_tracker/core/id_generator.hpp"

#include <algorithm>
#include <stdexcept>

namespace agentic_et1_tracker {
namespace {

constexpr char kAlphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
constexpr std::size_t kBase = 62;

}  // namespace

ShortIdGenerator::ShortIdGenerator(std::size_t min_length, std::size_t max_length)
    : min_length_(min_length), max_length_(max_length) {
  if (min_length_ == 0 || min_length_ > max_length_ || max_length_ > 10) {
    throw std::invalid_argument("short id length must be in the range 1..10");
  }
}

std::string ShortIdGenerator::generate() {
  std::lock_guard<std::mutex> lock(mutex_);
  while (true) {
    const std::string id = encode(next_++);
    if (issued_.insert(id).second) {
      return id;
    }
  }
}

bool ShortIdGenerator::isBase62Id(const std::string& id) {
  return !id.empty() &&
         std::all_of(id.begin(), id.end(), [](char c) {
           return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z');
         });
}

std::string ShortIdGenerator::encode(std::uint64_t value) const {
  std::string out;
  do {
    out.push_back(kAlphabet[value % kBase]);
    value /= kBase;
  } while (value != 0);

  std::reverse(out.begin(), out.end());
  if (out.size() < min_length_) {
    out.insert(out.begin(), min_length_ - out.size(), '0');
  }
  if (out.size() > max_length_) {
    throw std::overflow_error("short id space exhausted");
  }
  return out;
}

}  // namespace agentic_et1_tracker
