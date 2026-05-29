#pragma once

#include <cstddef>
#include <string>

#include <nlohmann/json.hpp>

#include "agentic_et1_tracker/api/json_codec.hpp"
#include "agentic_et1_tracker/core/status.hpp"
#include "agentic_et1_tracker/core/tracker_controller.hpp"
#include "agentic_et1_tracker/core/types.hpp"

namespace agentic_et1_tracker {

struct AgentApiConfig {
  RuntimeMode mode{RuntimeMode::Unknown};
  std::size_t queue_limit{8};
};

struct ExecuteCommand {
  std::string id;
  std::string path;
  MotionMode mode{MotionMode::Queue};
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

class ExecutionCommandSink {
 public:
  virtual ~ExecutionCommandSink() = default;
  virtual ExecuteResult submitQueue(const ExecuteCommand& command) = 0;
  virtual ExecuteResult submitInterrupt(const ExecuteCommand& command) = 0;
  virtual StopResult stop() = 0;
  virtual ControlResult fixStand() = 0;
  virtual ControlResult standbyVelocity() = 0;
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

  ApiResponse handle(const ApiRequest& request);

 private:
  ApiResponse execute(const std::string& body);
  ApiResponse stop(const std::string& body);
  ApiResponse fixStand(const std::string& body);
  ApiResponse standbyVelocity(const std::string& body);
  ApiResponse status(const std::string& target);
  ApiResponse health();
  ApiResponse error(ErrorCode code);

  ErrorCode readinessError(const StatusSnapshot& snapshot) const;

  AgentApiConfig config_;
  ExecutionCommandSink& commands_;
  StatusReader& status_;
  TrackValidatorPort& validator_;
  RunIdGenerator& ids_;
};

}  // namespace agentic_et1_tracker
