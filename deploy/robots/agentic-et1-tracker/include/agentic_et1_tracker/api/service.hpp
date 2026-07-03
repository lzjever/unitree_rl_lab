#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "agentic_et1_tracker/api/json_codec.hpp"
#include "agentic_et1_tracker/core/status.hpp"
#include "agentic_et1_tracker/core/tracker_controller.hpp"
#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/loco_upper/precheck.hpp"

namespace agentic_et1_tracker {

struct AgentApiConfig {
  RuntimeMode mode{RuntimeMode::Unknown};
  std::size_t queue_limit{8};
  std::string passive_password{"galaxy"};
  LocoUpperCapability loco_upper;
};

struct ExecuteCommand {
  std::string id;
  std::string path;
  MotionExecutor executor{MotionExecutor::GeneralTracker};
  MotionMode mode{MotionMode::Queue};
  bool hold{false};
  LocoRunOptions loco_options;
  TrackMetadata track;
};

struct TrackValidation {
  ErrorCode code{ErrorCode::Ok};
  TrackMetadata metadata;
  std::string message;

  bool ok() const { return code == ErrorCode::Ok; }
};

struct HealthSnapshot {
  ServiceHealth state{ServiceHealth::Starting};
  RuntimeMode mode{RuntimeMode::Unknown};
  ErrorCode err{ErrorCode::Ok};
  std::string block;
  LocoUpperCapability loco_upper;
};

struct ApiRequest {
  std::string method;
  std::string target;
  std::string body;
};

struct ApiResponse {
  int status{200};
  nlohmann::json body;
};

struct ControlResult {
  ErrorCode code{ErrorCode::Ok};

  bool ok() const { return code == ErrorCode::Ok; }
};

struct IdleResult {
  ErrorCode code{ErrorCode::Ok};
  IdleStatus idle;

  bool ok() const { return code == ErrorCode::Ok; }
};

class ExecutionCommandSink {
 public:
  virtual ~ExecutionCommandSink() = default;
  virtual ExecuteResult submitQueue(const ExecuteCommand& command) = 0;
  virtual ExecuteResult submitInterrupt(const ExecuteCommand& command) = 0;
  virtual StopResult stop() = 0;
  virtual ControlResult passive() = 0;
  virtual ControlResult fixStand() = 0;
  virtual ControlResult standbyVelocity() = 0;
  virtual ControlResult standby() = 0;
  virtual StopResult urgentStop() = 0;
  virtual IdleResult configureIdle(std::vector<IdleMotion> motions) = 0;
};

class StatusReader {
 public:
  virtual ~StatusReader() = default;
  virtual StatusSnapshot snapshot() const = 0;
  virtual RunLookupResult findRun(const std::string& id) const = 0;
  virtual HealthSnapshot health() const = 0;
};

class TrackValidatorPort {
 public:
  virtual ~TrackValidatorPort() = default;
  virtual TrackValidation validate(const std::string& path) = 0;
};

class RunIdGenerator {
 public:
  virtual ~RunIdGenerator() = default;
  virtual std::string generate() = 0;
};

class AgentApiService {
 public:
  AgentApiService(AgentApiConfig config,
                  ExecutionCommandSink& commands,
                  StatusReader& status,
                  TrackValidatorPort& validator,
                  RunIdGenerator& ids);
  AgentApiService(AgentApiConfig config,
                  ExecutionCommandSink& commands,
                  StatusReader& status,
                  TrackValidatorPort& validator,
                  LocoUpperPrecheckPort& loco_precheck,
                  RunIdGenerator& ids);

  ApiResponse handle(const ApiRequest& request);

 private:
  ApiResponse execute(const std::string& body);
  ApiResponse executeLocoUpper(const std::string& body);
  ApiResponse idle(const std::string& body);
  ApiResponse stop(const std::string& body);
  ApiResponse urgentStop(const std::string& body);
  ApiResponse passive(const std::string& body);
  ApiResponse fixStand(const std::string& body);
  ApiResponse standby(const std::string& body);
  ApiResponse standbyVelocity(const std::string& body);
  ApiResponse renamedRoute(const std::string& message, NextAction next);
  ApiResponse status(const std::string& target);
  ApiResponse health();
  ApiResponse error(ErrorCode code);
  ApiResponse error(ErrorInfo info);
  ApiResponse controlStateConflict(ControllerState ctrl);

  ErrorCode readinessError(const StatusSnapshot& snapshot) const;

  AgentApiConfig config_;
  ExecutionCommandSink& commands_;
  StatusReader& status_;
  TrackValidatorPort& validator_;
  LocoUpperPrecheckPort& loco_precheck_;
  RunIdGenerator& ids_;
};

}  // namespace agentic_et1_tracker
