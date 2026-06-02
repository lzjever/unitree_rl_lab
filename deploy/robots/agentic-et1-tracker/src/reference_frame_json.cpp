#include "agentic_et1_tracker/reference/reference_frame_json.hpp"

namespace agentic_et1_tracker {
namespace {

template <typename ArrayT>
nlohmann::json arrayJson(const ArrayT& values) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& value : values) {
    out.push_back(value);
  }
  return out;
}

template <typename RowsT>
nlohmann::json rowsJson(const RowsT& rows) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& row : rows) {
    out.push_back(arrayJson(row));
  }
  return out;
}

}  // namespace

nlohmann::json referenceFrameSnapshotJson(
    const ReferenceFrameSnapshot& snapshot,
    std::chrono::steady_clock::time_point now) {
  if (!snapshot.active) {
    return {{"ok", true}, {"active", false}};
  }

  const auto stale = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - snapshot.updated_at);
  const auto stale_ms = stale.count() < 0 ? 0 : stale.count();
  return {
      {"ok", true},
      {"active", true},
      {"schema", kReferenceFrameSchema},
      {"body_order", kReferenceBodyOrder},
      {"id", snapshot.id},
      {"frame", snapshot.frame},
      {"frames", snapshot.frames},
      {"time_s", snapshot.time_s},
      {"fps", snapshot.fps},
      {"stale_ms", stale_ms},
      {"p", rowsJson(snapshot.p)},
      {"q", rowsJson(snapshot.q)},
      {"c", arrayJson(snapshot.c)},
      {"com", arrayJson(snapshot.com)},
      {"comv", arrayJson(snapshot.comv)},
  };
}

}  // namespace agentic_et1_tracker
