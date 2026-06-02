#include "agentic_et1_tracker/reference/reference_frame_store.hpp"

#include <atomic>
#include <memory>
#include <utility>

namespace agentic_et1_tracker {

void ReferenceFrameStore::publish(ReferenceFrameSnapshot snapshot) {
  std::shared_ptr<const ReferenceFrameSnapshot> next =
      std::make_shared<ReferenceFrameSnapshot>(std::move(snapshot));
  std::atomic_store_explicit(&latest_, std::move(next), std::memory_order_release);
}

void ReferenceFrameStore::clear() {
  std::atomic_store_explicit(&latest_,
                             std::shared_ptr<const ReferenceFrameSnapshot>{},
                             std::memory_order_release);
}

ReferenceFrameSnapshot ReferenceFrameStore::snapshot() const {
  const auto current = std::atomic_load_explicit(&latest_, std::memory_order_acquire);
  if (!current) {
    return ReferenceFrameSnapshot{};
  }
  return *current;
}

}  // namespace agentic_et1_tracker
