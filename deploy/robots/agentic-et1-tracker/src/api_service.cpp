#include "agentic_et1_tracker/api/service.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

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
    case ErrorCode::ControlStateConflict:
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

bool parseHold(const nlohmann::json& input, bool& hold) {
  hold = false;
  const auto it = input.find("hold");
  if (it == input.end()) {
    return true;
  }
  if (!it->is_boolean()) {
    return false;
  }
  hold = it->get<bool>();
  return true;
}

bool hasOnlyKeys(const nlohmann::json& input, const std::vector<std::string>& allowed) {
  for (const auto& item : input.items()) {
    if (std::find(allowed.begin(), allowed.end(), item.key()) == allowed.end()) {
      return false;
    }
  }
  return true;
}

bool parsePassivePassword(const std::string& body, const std::string& expected) {
  if (blank(body)) {
    return false;
  }
  auto input = nlohmann::json::parse(body, nullptr, false);
  if (input.is_discarded() || !input.is_object()) {
    return false;
  }
  if (!hasOnlyKeys(input, {"password"})) {
    return false;
  }
  const auto password_it = input.find("password");
  if (password_it == input.end() || !password_it->is_string()) {
    return false;
  }
  return password_it->get<std::string>() == expected;
}

bool executeBlockedByController(ControllerState ctrl) {
  return ctrl == ControllerState::Starting || ctrl == ControllerState::Idle ||
         ctrl == ControllerState::Passive || ctrl == ControllerState::FixStand ||
         ctrl == ControllerState::Stopping || ctrl == ControllerState::Fault;
}

bool idleConfigBlockedByController(ControllerState ctrl) {
  return ctrl == ControllerState::Starting || ctrl == ControllerState::Idle ||
         ctrl == ControllerState::Passive || ctrl == ControllerState::FixStand ||
         ctrl == ControllerState::Stopping || ctrl == ControllerState::Fault;
}

bool fixStandRecoveryState(ControllerState ctrl) {
  return ctrl == ControllerState::Passive || ctrl == ControllerState::Fault;
}

ErrorCode lowCmdOccupiedReadiness(ErrorCode readiness) {
  return readiness == ErrorCode::Ok ? ErrorCode::RobotNotReady : readiness;
}

bool lowCmdOccupied(const StatusSnapshot& snapshot) {
  return snapshot.block == "lowcmd_occupied";
}

bool fixStandRecoveryAllowed(const StatusSnapshot& snapshot, ErrorCode readiness) {
  return fixStandRecoveryState(snapshot.ctrl) &&
         readiness == ErrorCode::RobotBadOrientation &&
         snapshot.block == "bad_orientation";
}

bool passiveSafetyAllowed(const StatusSnapshot& snapshot, ErrorCode readiness) {
  return readiness == ErrorCode::RobotBadOrientation &&
         snapshot.block == "bad_orientation";
}

ErrorInfo controlStateConflictInfo(ControllerState ctrl) {
  switch (ctrl) {
    case ControllerState::Starting:
      return {ErrorCode::ControlStateConflict,
              "ctrl=starting; wait /status",
              false,
              NextAction::Status};
    case ControllerState::Idle:
      return {ErrorCode::ControlStateConflict,
              "wrong ctrl; check /status",
              false,
              NextAction::Status};
    case ControllerState::Passive:
      return {ErrorCode::ControlStateConflict,
              "ctrl=passive; /fixstand then /standby_velocity",
              false,
              NextAction::FixStand};
    case ControllerState::FixStand:
      return {ErrorCode::ControlStateConflict,
              "ctrl=fixstand; /standby_velocity",
              false,
              NextAction::StandbyVelocity};
    case ControllerState::Stopping:
      return {ErrorCode::ControlStateConflict,
              "ctrl=stopping; wait /status",
              false,
              NextAction::Status};
    case ControllerState::Fault:
      return {ErrorCode::ControlStateConflict,
              "ctrl=fault; /fixstand",
              false,
              NextAction::FixStand};
    case ControllerState::StandbyVelocity:
    case ControllerState::Preparing:
    case ControllerState::Running:
      break;
  }
  return errorInfo(ErrorCode::ControlStateConflict);
}

nlohmann::json queuePositionJson(const QueueStatus& queue, const std::string& id) {
  const auto it = std::find(queue.ids.begin(), queue.ids.end(), id);
  if (it == queue.ids.end()) {
    return nullptr;
  }
  return static_cast<std::size_t>(std::distance(queue.ids.begin(), it)) + 1;
}

nlohmann::json topErrorCodeJson(ErrorCode code) {
  return code == ErrorCode::Ok ? nlohmann::json(nullptr) : nlohmann::json(toString(code));
}

nlohmann::json successBase() {
  return {{"ok", true}};
}

ErrorInfo manualReadinessInfo(ErrorCode code) {
  ErrorInfo info = apiErrorInfo(code);
  info.retryable = false;
  info.next = NextAction::Manual;
  return info;
}

nlohmann::json idleSummaryJson(const IdleStatus& idle) {
  return {
      {"enabled", idle.enabled},
      {"n", idle.n},
      {"active", idle.active},
  };
}

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

LocoUpperCapability normalizedLocoUpperCapability(LocoUpperCapability capability) {
  if (!capability.enabled) {
    capability.ready = false;
  }
  return capability;
}

LocoUpperCapability effectiveLocoUpperCapability(const LocoUpperCapability& configured,
                                                 const LocoUpperCapability& observed) {
  LocoUpperCapability capability;
  capability.enabled = configured.enabled;
  capability.ready = configured.enabled && observed.ready;
  capability.default_radius_m = configured.default_radius_m;
  capability.max_radius_m = configured.max_radius_m;
  capability.strict_pose = configured.strict_pose;
  return normalizedLocoUpperCapability(capability);
}

nlohmann::json locoUpperCapabilityJson(LocoUpperCapability capability) {
  capability = normalizedLocoUpperCapability(capability);
  return {
      {"enabled", capability.enabled},
      {"ready", capability.ready},
      {"default_radius_m", capability.default_radius_m},
      {"max_radius_m", capability.max_radius_m},
      {"strict_pose", capability.strict_pose},
  };
}

bool validLocoRadiusConfig(const LocoUpperCapability& capability) {
  return finitePositive(capability.default_radius_m) &&
         finitePositive(capability.max_radius_m) &&
         capability.default_radius_m <= capability.max_radius_m;
}

ErrorInfo locoUpperDisabledInfo() {
  return {ErrorCode::ModelNotReady, "loco_upper disabled", false, NextAction::Status};
}

class DefaultLocoUpperPrechecker final : public LocoUpperPrecheckPort {
 public:
  LocoUpperPrecheckResult precheck(
      const std::string&,
      const LocoUpperPrecheckOptions&) override {
    return {};
  }
};

LocoUpperPrecheckPort& defaultLocoUpperPrechecker() {
  static DefaultLocoUpperPrechecker prechecker;
  return prechecker;
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
    : AgentApiService(config,
                      commands,
                      status,
                      validator,
                      defaultLocoUpperPrechecker(),
                      ids) {}

AgentApiService::AgentApiService(AgentApiConfig config,
                                 ExecutionCommandSink& commands,
                                 StatusReader& status,
                                 TrackValidatorPort& validator,
                                 LocoUpperPrecheckPort& loco_precheck,
                                 RunIdGenerator& ids)
    : config_(config),
      commands_(commands),
      status_(status),
      validator_(validator),
      loco_precheck_(loco_precheck),
      ids_(ids) {}

ApiResponse AgentApiService::handle(const ApiRequest& request) {
  try {
    const auto target = parseTarget(request.target);
    if (request.method == "POST" && target.path == "/execute") {
      return execute(request.body);
    }
    if (request.method == "POST" && target.path == "/execute_loco_upper") {
      return executeLocoUpper(request.body);
    }
    if (request.method == "POST" && target.path == "/idle") {
      return idle(request.body);
    }
    if (request.method == "POST" && target.path == "/stop") {
      return stop(request.body);
    }
    if (request.method == "POST" && target.path == "/passive") {
      return passive(request.body);
    }
    if (request.method == "POST" && target.path == "/fixstand") {
      return fixStand(request.body);
    }
    if (request.method == "POST" && target.path == "/standby_velocity") {
      return standbyVelocity(request.body);
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
  if (!hasOnlyKeys(input, {"path", "mode", "hold"})) {
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
  bool hold = false;
  if (!parseHold(input, hold)) {
    return error(ErrorCode::RequestInvalid);
  }

  const auto snapshot = status_.snapshot();
  const ErrorCode readiness = readinessError(snapshot);
  if (readiness != ErrorCode::Ok) {
    if (snapshot.block == "lowcmd_occupied") {
      return error(manualReadinessInfo(readiness));
    }
    return error(readiness);
  }
  if (executeBlockedByController(snapshot.ctrl)) {
    return controlStateConflict(snapshot.ctrl);
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
  command.hold = hold;
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
  out["hold"] = hold;
  return {200, out};
}

ApiResponse AgentApiService::executeLocoUpper(const std::string& body) {
  if (blank(body)) {
    return error(ErrorCode::RequestInvalid);
  }

  auto input = nlohmann::json::parse(body, nullptr, false);
  if (input.is_discarded() || !input.is_object()) {
    return error(ErrorCode::RequestInvalid);
  }
  if (!hasOnlyKeys(input, {"path", "mode", "hold", "max_radius_m"})) {
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
  bool hold = false;
  if (!parseHold(input, hold)) {
    return error(ErrorCode::RequestInvalid);
  }

  bool explicit_radius = false;
  double requested_radius_m = 0.0;
  const auto radius_it = input.find("max_radius_m");
  if (radius_it != input.end()) {
    if (!radius_it->is_number()) {
      return error(ErrorCode::RequestInvalid);
    }
    try {
      requested_radius_m = radius_it->get<double>();
    } catch (const std::exception&) {
      return error(ErrorCode::RequestInvalid);
    }
    explicit_radius = true;
  }
  if (explicit_radius && !finitePositive(requested_radius_m)) {
    return error(ErrorCode::RequestInvalid);
  }

  const LocoUpperCapability configured =
      normalizedLocoUpperCapability(config_.loco_upper);
  if (!configured.enabled) {
    return error(locoUpperDisabledInfo());
  }
  if (!validLocoRadiusConfig(configured)) {
    return error(ErrorCode::ModelNotReady);
  }

  const auto snapshot = status_.snapshot();
  const LocoUpperCapability capability =
      effectiveLocoUpperCapability(configured, snapshot.loco_upper);
  const ErrorCode readiness = readinessError(snapshot);
  if (readiness != ErrorCode::Ok) {
    if (snapshot.block == "lowcmd_occupied") {
      return error(manualReadinessInfo(readiness));
    }
    return error(readiness);
  }
  if (!capability.ready) {
    return error(ErrorCode::ModelNotReady);
  }

  const double max_radius_m =
      explicit_radius ? std::min(requested_radius_m, capability.max_radius_m)
                      : capability.default_radius_m;
  if (!finitePositive(max_radius_m)) {
    return error(ErrorCode::RequestInvalid);
  }

  if (executeBlockedByController(snapshot.ctrl)) {
    return controlStateConflict(snapshot.ctrl);
  }

  const std::string path = *path_it;
  const TrackValidation validation = validator_.validate(path);
  if (!validation.ok()) {
    return error(validation.code);
  }

  LocoUpperPrecheckOptions precheck_options;
  precheck_options.max_radius_m = max_radius_m;
  precheck_options.strict_pose = capability.strict_pose;
  precheck_options.hold = hold;
  const LocoUpperPrecheckResult precheck =
      loco_precheck_.precheck(validation.metadata.canonical_path, precheck_options);
  if (!precheck.ok()) {
    return error(precheck.code);
  }

  ExecuteCommand command;
  command.id = ids_.generate();
  command.path = validation.metadata.canonical_path;
  command.executor = MotionExecutor::LocoUpper;
  command.mode = mode;
  command.hold = hold;
  command.loco_options.max_radius_m = max_radius_m;
  command.loco_options.hold = hold;
  command.loco_options.radius_clamped = precheck.flags.radius_clamped;
  command.loco_options.envelope_clamped = precheck.flags.envelope_clamped;
  command.loco_options.upper_clamped = precheck.flags.upper_clamped;
  command.loco_options.upper_rate_limited = precheck.flags.upper_rate_limited;
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
  out["hold"] = hold;
  out["executor"] = toString(MotionExecutor::LocoUpper);
  out["max_radius_m"] = max_radius_m;
  return {200, out};
}

ApiResponse AgentApiService::idle(const std::string& body) {
  if (blank(body)) {
    return error(ErrorCode::RequestInvalid);
  }

  auto input = nlohmann::json::parse(body, nullptr, false);
  if (input.is_discarded() || !input.is_object()) {
    return error(ErrorCode::RequestInvalid);
  }
  if (!hasOnlyKeys(input, {"paths"})) {
    return error(ErrorCode::RequestInvalid);
  }

  const auto paths_it = input.find("paths");
  if (paths_it == input.end() || !paths_it->is_array()) {
    return error(ErrorCode::RequestInvalid);
  }

  std::vector<IdleMotion> motions;
  motions.reserve(paths_it->size());
  for (const auto& item : *paths_it) {
    if (!item.is_string() || item.get<std::string>().empty()) {
      return error(ErrorCode::RequestInvalid);
    }
  }

  if (!paths_it->empty()) {
    const auto snapshot = status_.snapshot();
    if (idleConfigBlockedByController(snapshot.ctrl)) {
      return controlStateConflict(snapshot.ctrl);
    }
    const ErrorCode readiness = readinessError(snapshot);
    if (readiness != ErrorCode::Ok) {
      if (snapshot.block == "lowcmd_occupied") {
        return error(manualReadinessInfo(readiness));
      }
      return error(readiness);
    }
  }

  for (const auto& item : *paths_it) {
    const TrackValidation validation = validator_.validate(item.get<std::string>());
    if (!validation.ok()) {
      return error(validation.code);
    }
    IdleMotion motion;
    motion.path = validation.metadata.canonical_path;
    motion.track = validation.metadata;
    motions.push_back(std::move(motion));
  }

  const IdleResult result = commands_.configureIdle(std::move(motions));
  if (!result.ok()) {
    return error(result.code);
  }

  auto out = successBase();
  out["idle"] = idleSummaryJson(result.idle);
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

ApiResponse AgentApiService::passive(const std::string& body) {
  if (!parsePassivePassword(body, config_.passive_password)) {
    return error(ErrorCode::RequestInvalid);
  }

  const StatusSnapshot snapshot = status_.snapshot();
  const ErrorCode readiness = readinessError(snapshot);
  if (lowCmdOccupied(snapshot)) {
    return error(manualReadinessInfo(lowCmdOccupiedReadiness(readiness)));
  }
  if (readiness != ErrorCode::Ok && !passiveSafetyAllowed(snapshot, readiness)) {
    return error(readiness);
  }

  const ControlResult result = commands_.passive();
  if (!result.ok()) {
    return error(result.code);
  }

  auto out = successBase();
  out["state"] = "accepted";
  return {200, out};
}

ApiResponse AgentApiService::fixStand(const std::string& body) {
  if (!blank(body)) {
    return error(ErrorCode::RequestInvalid);
  }

  const StatusSnapshot snapshot = status_.snapshot();
  const ErrorCode readiness = readinessError(snapshot);
  if (lowCmdOccupied(snapshot)) {
    return error(manualReadinessInfo(lowCmdOccupiedReadiness(readiness)));
  }
  if (readiness != ErrorCode::Ok &&
      !fixStandRecoveryAllowed(snapshot, readiness)) {
    return error(readiness);
  }

  const ControlResult result = commands_.fixStand();
  if (!result.ok()) {
    return error(result.code);
  }

  auto out = successBase();
  out["state"] = "accepted";
  return {200, out};
}

ApiResponse AgentApiService::standbyVelocity(const std::string& body) {
  if (!blank(body)) {
    return error(ErrorCode::RequestInvalid);
  }

  const StatusSnapshot snapshot = status_.snapshot();
  const ErrorCode readiness = readinessError(snapshot);
  if (lowCmdOccupied(snapshot)) {
    return error(manualReadinessInfo(lowCmdOccupiedReadiness(readiness)));
  }
  if (snapshot.ctrl == ControllerState::Passive ||
      snapshot.ctrl == ControllerState::Fault) {
    return controlStateConflict(snapshot.ctrl);
  }
  if (readiness != ErrorCode::Ok) {
    return error(readiness);
  }

  const ControlResult result = commands_.standbyVelocity();
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
    snapshot.loco_upper =
        effectiveLocoUpperCapability(config_.loco_upper, snapshot.loco_upper);
    return {200, statusSnapshotJson(snapshot)};
  }
  if (id.value.empty()) {
    return error(ErrorCode::RequestInvalid);
  }

  const RunLookupResult result = status_.findRun(id.value);
  if (!result.ok() || !result.run) {
    return error(ErrorCode::RunNotFound);
  }

  const StatusSnapshot snapshot = status_.snapshot();
  auto out = motionStatusJson(*result.run);
  out["ok"] = true;
  out["robot"] = toString(snapshot.robot);
  out["ctrl"] = toString(snapshot.ctrl);
  out["ready"] = snapshot.ready;
  out["block"] = snapshot.block.empty() ? nlohmann::json(nullptr)
                                        : nlohmann::json(snapshot.block);
  out["queue_pos"] = queuePositionJson(snapshot.queue, result.run->id);
  out["top_err"] = topErrorCodeJson(snapshot.err);
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
              {"cap",
               {{"loco_upper",
                 locoUpperCapabilityJson(effectiveLocoUpperCapability(
                     config_.loco_upper, current.loco_upper))}}},
          }};
}

ApiResponse AgentApiService::error(ErrorCode code) {
  const auto info = apiErrorInfo(code);
  return error(info);
}

ApiResponse AgentApiService::error(ErrorInfo info) {
  return {httpStatus(info.code),
          {
              {"ok", false},
              {"error",
               {
                   {"code", toString(info.code)},
                   {"message", info.message},
                   {"retryable", info.retryable},
               }},
              {"next", toString(info.next)},
          }};
}

ApiResponse AgentApiService::controlStateConflict(ControllerState ctrl) {
  return error(controlStateConflictInfo(ctrl));
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
