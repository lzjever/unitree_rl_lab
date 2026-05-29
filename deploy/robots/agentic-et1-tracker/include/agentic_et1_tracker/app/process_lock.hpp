#pragma once

#include <filesystem>

namespace agentic_et1_tracker {

class ProcessLock {
 public:
  explicit ProcessLock(std::filesystem::path path);
  ~ProcessLock();

  ProcessLock(const ProcessLock&) = delete;
  ProcessLock& operator=(const ProcessLock&) = delete;
  ProcessLock(ProcessLock&& other) noexcept;
  ProcessLock& operator=(ProcessLock&& other) noexcept;

  bool tryLock();
  void unlock();
  bool locked() const;

 private:
  std::filesystem::path path_;
  int fd_{-1};
};

}  // namespace agentic_et1_tracker
