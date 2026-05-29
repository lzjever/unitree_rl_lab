#include "agentic_et1_tracker/app/process_lock.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <utility>

namespace agentic_et1_tracker {

ProcessLock::ProcessLock(std::filesystem::path path) : path_(std::move(path)) {}

ProcessLock::~ProcessLock() { unlock(); }

ProcessLock::ProcessLock(ProcessLock&& other) noexcept
    : path_(std::move(other.path_)), fd_(other.fd_) {
  other.fd_ = -1;
}

ProcessLock& ProcessLock::operator=(ProcessLock&& other) noexcept {
  if (this != &other) {
    unlock();
    path_ = std::move(other.path_);
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

bool ProcessLock::tryLock() {
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(path_.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
  if (fd_ < 0) {
    return false;
  }

  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void ProcessLock::unlock() {
  if (fd_ < 0) {
    return;
  }
  ::flock(fd_, LOCK_UN);
  ::close(fd_);
  fd_ = -1;
}

bool ProcessLock::locked() const { return fd_ >= 0; }

}  // namespace agentic_et1_tracker
