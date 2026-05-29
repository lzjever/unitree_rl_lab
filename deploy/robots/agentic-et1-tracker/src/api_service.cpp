#include "agentic_et1_tracker/api/service.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <string>

namespace agentic_et1_tracker {
namespace {

struct ParsedTarget {
  std::string path;
  std::string query;
};

ParsedTarget parseTarget(const std::string& target) {
  const auto pos = target.find('?');
  if (pos == std::string::npos) {
    return {target, ""};
  }
  return {target.substr(0, pos), target.substr(pos + 1)};
}

struct ParsedQueryId {
  bool present{false};
  std::string value;
};

bool blank(const std::string& value) {
  return std::all_of(value.begin(), value.end(),
                     [](unsigned char c) { return std::isspace(c) != 0; });
}

int httpStatus(ErrorCode code) {
  switch (apiErrorInfo(code).code) {
    case ErrorCode::Ok:
      return 200;
    case ErrorCode::RequestInvalid:
      return 400;
    case ErrorCode::TrkPathNotAllowed:
      return 403;
    case ErrorCode::RunNotFound:
      return 404;
    case ErrorCode::QueueFull:
    case ErrorCode::RobotDisconnected:
    case ErrorCode::RobotNotReady:
    case ErrorCode::RobotBadOrientation:
    case ErrorCode::SafetyLimitTriggered:
      return 409;
    case ErrorCode::ServiceNotReady:
    case ErrorCode::ModelNotReady:
      return 503;
    case ErrorCode::TrkFileNotFound:
    case ErrorCode::TrkParseFailed:
    case ErrorCode::TrkValidationFailed:
      return 400;
    case ErrorCode::ModelInferenceFailed:
    case ErrorCode::InternalError:
      return 500;
    case ErrorCode::RunStateConflict:
      return 500;
  }
  return 500;
}

ParsedQueryId queryId(const std::string& query) {
  std::size_t start = 0;
  while (start <= query.size()) {
    const auto end = query.find('&', start);
    const auto part = query.substr(start, end == std::string::npos ? end : end - start);
    constexpr char prefix[] = "id=";
    if (part.rfind(prefix, 0) == 0) {
      return {true, part.substr(3)};
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return {};
}

bool parseMode(const nlohmann::json& input, MotionMode& mode) {
  mode = MotionMode::Queue;
  const auto it = input.find("mode");
  if (it == input.end()) {
    return true;
  }
  if (!it->is_string()) {
    return false;
  }
  const std::string value = *it;
  if (value == "queue") {
    mode = MotionMode::Queue;
    return true;
  }
  if (value == "interrupt") {
    mode = MotionMode::Interrupt;
    return true;
  }
  return false;
}

nlohmann::json successBase() {
  return {{"ok", true}};
}

ServiceHealth projectedHealthState(const HealthSnapshot& health) {
  if (health.state != ServiceHealth::Ready) {
    return health.state;
  }
  if (health.err == ErrorCode::Ok && health.block.empty()) {
    return ServiceHealth::Ready;
  }
  switch (health.err) {
    case ErrorCode::Ok:
    case ErrorCode::ServiceNotReady:
    case ErrorCode::ModelNotReady:
    case ErrorCode::RobotDisconnected:
    case ErrorCode::RobotNotReady:
      return ServiceHealth::Starting;
    default:
      return ServiceHealth::Error;
  }
}

}  // namespace

AgentApiService::AgentApiService(AgentApiConfig config,
                                 ExecutionCommandSink& commands,
                                 StatusReader& status,
                                 TrackValidatorPort& validator,
                                 RunIdGenerator& ids)
    : config_(config),
      commands_(commands),
      status_(status),
      validator_(validator),
      ids_(ids) {}

ApiResponse AgentApiService::handle(const ApiRequest& request) {
  try {
    const auto target = parseTarget(request.target);
    if (request.method == "POST" && target.path == "/execute") {
      return execute(request.body);
    }
    if (request.method == "POST" && target.path == "/stop") {
      return stop(request.body);
    }
    if (request.method == "GET" && target.path == "/status") {
      return status(request.target);
    }
    if (request.method == "GET" && target.path == "/health") {
      return health();
    }
    return error(ErrorCode::RequestInvalid);
  } catch (const std::exception&) {
    return error(ErrorCode::InternalError);
  } catch (...) {
    return error(ErrorCode::InternalError);
  }
}

ApiResponse AgentApiService::execute(const std::string& body) {
  if (blank(body)) {
    return error(ErrorCode::RequestInvalid);
  }

  auto input = nlohmann::json::parse(body, nullptr, false);
  if (input.is_discarded() || !input.is_object()) {
    return error(ErrorCode::RequestInvalid);
  }

  const auto path_it = input.find("path");
  if (path_it == input.end() || !path_it->is_string() ||
      path_it->get<std::string>().empty()) {
    return error(ErrorCode::RequestInvalid);
  }

  MotionMode mode = MotionMode::Queue;
  if (!parseMode(input, mode)) {
    return error(ErrorCode::RequestInvalid);
  }

  const auto snapshot = status_.snapshot();
  const ErrorCode readiness = readinessError(snapshot);
  if (readiness != ErrorCode::Ok) {
    return error(readiness);
  }

  const std::string path = *path_it;
  const TrackValidation validation = validator_.validate(path);
  if (!validation.ok()) {
    return error(validation.code);
  }

  ExecuteCommand command;
  command.id = ids_.generate();
  command.path = validation.metadata.canonical_path;
  command.mode = mode;
  command.track = validation.metadata;

  const ExecuteResult result =
      mode == MotionMode::Interrupt ? commands_.submitInterrupt(command)
                                    : commands_.submitQueue(command);
  if (!result.ok()) {
    return error(result.code);
  }

  auto out = successBase();
  out["id"] = result.id;
  out["state"] = toString(result.state);
  out["q"] = result.q;
  return {200, out};
}

ApiResponse AgentApiService::stop(const std::string& body) {
  if (!blank(body)) {
    return error(ErrorCode::RequestInvalid);
  }

  const StopResult result = commands_.stop();
  if (!result.ok()) {
    return error(result.code);
  }

  auto out = successBase();
  out["state"] = "accepted";
  return {200, out};
}

ApiResponse AgentApiService::status(const std::string& target) {
  const auto parsed = parseTarget(target);
  const ParsedQueryId id = queryId(parsed.query);
  if (!id.present) {
    auto snapshot = status_.snapshot();
    if (snapshot.mode == RuntimeMode::Unknown) {
      snapshot.mode = config_.mode;
    }
    if (snapshot.queue.limit == 0) {
      snapshot.queue.limit = config_.queue_limit;
    }
    return {200, statusSnapshotJson(snapshot)};
  }
  if (id.value.empty()) {
    return error(ErrorCode::RequestInvalid);
  }

  const RunLookupResult result = status_.findRun(id.value);
  if (!result.ok() || !result.run) {
    return error(ErrorCode::RunNotFound);
  }

  auto out = motionStatusJson(*result.run);
  out["ok"] = true;
  out["robot"] = toString(status_.snapshot().robot);
  return {200, out};
}

ApiResponse AgentApiService::health() {
  const HealthSnapshot current = status_.health();
  const ServiceHealth state = projectedHealthState(current);
  return {200,
          {
              {"ok", state == ServiceHealth::Ready},
              {"state", toString(state)},
              {"mode", toString(current.mode == RuntimeMode::Unknown ? config_.mode
                                                                      : current.mode)},
          }};
}

ApiResponse AgentApiService::error(ErrorCode code) {
  const auto info = apiErrorInfo(code);
  return {httpStatus(code),
          {
              {"ok", false},
              {"error", errorJson(code)},
              {"next", toString(info.next)},
          }};
}

ErrorCode AgentApiService::readinessError(const StatusSnapshot& snapshot) const {
  if (snapshot.err != ErrorCode::Ok) {
    return snapshot.err;
  }
  if (snapshot.ready) {
    return ErrorCode::Ok;
  }
  if (snapshot.ctrl == ControllerState::Starting) {
    return ErrorCode::ServiceNotReady;
  }
  if (snapshot.ctrl == ControllerState::Fault || snapshot.robot == RobotState::Fault) {
    return ErrorCode::SafetyLimitTriggered;
  }
  if (snapshot.robot == RobotState::Disconnected) {
    return ErrorCode::RobotDisconnected;
  }
  if (snapshot.robot == RobotState::NotReady) {
    return ErrorCode::RobotNotReady;
  }
  return ErrorCode::ServiceNotReady;
}

}  // namespace agentic_et1_tracker
