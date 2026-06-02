#pragma once

#include <memory>

#include "agentic_et1_tracker/reference/reference_frame.hpp"

namespace agentic_et1_tracker {

class ReferenceFrameStore final : public ReferenceFrameSink {
 public:
  void publish(ReferenceFrameSnapshot snapshot) override;
  void clear() override;
  ReferenceFrameSnapshot snapshot() const;

 private:
  std::shared_ptr<const ReferenceFrameSnapshot> latest_;
};

}  // namespace agentic_et1_tracker
