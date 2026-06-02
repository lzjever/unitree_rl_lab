#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "agentic_et1_tracker/trk/loader.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {

inline constexpr const char* kReferenceFrameSchema = "ET1REF1";
inline constexpr const char* kReferenceBodyOrder = "et1_27_v1";

struct ReferenceFrameSnapshot {
  bool active{false};
  std::string id;
  std::size_t frame{0};
  std::size_t frames{0};
  double time_s{0.0};
  double fps{TrkSchema::kDefaultFps};
  std::chrono::steady_clock::time_point updated_at{};
  std::array<std::array<float, 3>, TrkSchema::kBodyCount> p{};
  std::array<std::array<float, 4>, TrkSchema::kBodyCount> q{};
  std::array<std::int64_t, 2> c{};
  std::array<float, 3> com{};
  std::array<float, 3> comv{};
};

class ReferenceFrameSink {
 public:
  virtual ~ReferenceFrameSink() = default;
  virtual void publish(ReferenceFrameSnapshot snapshot) = 0;
  virtual void clear() = 0;
};

std::optional<ReferenceFrameSnapshot> makeReferenceFrameSnapshot(
    const std::string& id,
    const TrkTrack& track,
    std::size_t frame_index,
    std::chrono::steady_clock::time_point updated_at = std::chrono::steady_clock::now());

}  // namespace agentic_et1_tracker
