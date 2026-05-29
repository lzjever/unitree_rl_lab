#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "agentic_et1_tracker/core/command_mailbox.hpp"
#include "agentic_et1_tracker/core/motion_queue.hpp"
#include "agentic_et1_tracker/core/status.hpp"
#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

struct Readiness {
  bool service_initialized{false};
  bool robot_connected{false};
  bool lowstate_fresh{false};
  bool mode_machine_ok{false};
  bool policy_ready{false};
  bool fault{false};
  std::string block;
  ErrorCode fault_code{ErrorCode::SafetyLimitTriggered};

  bool ready() const;
  ErrorCode acceptError() const;
};

using ServiceState = Readiness;

struct TrackerControllerConfig {
  std::size_t queue_limit{8};
  std::size_t recent_limit{32};
  double hz{50.0};
};

struct TrackMetadata {
  std::size_t frames{0};
  double duration_s{0.0};
  double fps{50.0};
  std::string canonical_path;
};

struct ExecuteRequest {
  std::string id;
  std::string path;
  MotionMode mode{MotionMode::Queue};
  TrackMetadata track;
  ErrorCode validation_error{ErrorCode::Ok};
};

struct ExecuteResult {
  ErrorCode code{ErrorCode::Ok};
  std::string id;
  MotionState state{MotionState::Queued};
  std::size_t q{0};

  bool ok() const { return code == ErrorCode::Ok; }
};

struct StopResult {
  ErrorCode code{ErrorCode::Ok};
  ControllerState state{ControllerState::Idle};
  StopReason stop_reason{StopReason::None};
  std::size_t cleared{0};

  bool ok() const { return code == ErrorCode::Ok; }
};

struct RunLookupResult {
  ErrorCode code{ErrorCode::RunNotFound};
  std::optional<MotionStatus> run;

  bool ok() const { return code == ErrorCode::Ok; }
};

class TrackerController {
 public:
  explicit TrackerController(TrackerControllerConfig config = {});

  void setReadiness(Readiness readiness);
  const Readiness& readiness() const;

  ExecuteResult execute(const ExecuteRequest& request);
  StopResult stop();
  void tick();

  StatusSnapshot status() const;
  RunLookupResult findRun(const std::string& id) const;

 private:
  ErrorCode acceptError(const ExecuteRequest& request) const;
  ExecuteResult acceptQueue(MotionRequest request);
  ExecuteResult acceptInterrupt(MotionRequest request);
  StopResult processStop();

  bool startGateOpen() const;
  void startNext();
  void advanceActive();
  void finishActive(MotionState state, StopReason reason, ErrorCode error);
  void enterStopping(StopReason reason);
  void syncControllerWithReadiness();

  MotionStatus toStatus(const MotionRequest& request) const;
  RobotState robotState() const;
  std::string block() const;
  ErrorCode statusError() const;

  TrackerControllerConfig config_;
  Readiness readiness_;
  MotionQueue queue_;
  CommandMailbox mailbox_;
  ControllerState ctrl_{ControllerState::Starting};
  StopReason stop_reason_{StopReason::None};
  bool stop_to_idle_pending_{false};
  std::optional<MotionRequest> active_;
};

}  // namespace agentic_et1_tracker
