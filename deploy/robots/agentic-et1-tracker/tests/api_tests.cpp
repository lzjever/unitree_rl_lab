#include <catch2/catch_test_macros.hpp>

#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "agentic_et1_tracker/api/service.hpp"

namespace agentic_et1_tracker {
namespace {

StatusSnapshot readySnapshot() {
  StatusSnapshot snapshot;
  snapshot.ready = true;
  snapshot.mode = RuntimeMode::Sim;
  snapshot.robot = RobotState::Idle;
  snapshot.ctrl = ControllerState::Idle;
  snapshot.queue.limit = 8;
  return snapshot;
}

class FakeSink final : public ExecutionCommandSink {
 public:
  ExecuteResult queue_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  ExecuteResult interrupt_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  StopResult stop_result{ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 0};
  ControlResult fixstand_result{ErrorCode::Ok};
  ControlResult standby_velocity_result{ErrorCode::Ok};

  ExecuteResult submitQueue(const ExecuteCommand& command) override {
    ++queue_calls;
    queue_commands.push_back(command);
    auto result = queue_result;
    result.id = command.id;
    return result;
  }

  ExecuteResult submitInterrupt(const ExecuteCommand& command) override {
    ++interrupt_calls;
    interrupt_commands.push_back(command);
    auto result = interrupt_result;
    result.id = command.id;
    return result;
  }

  StopResult stop() override {
    if (throw_stop) {
      throw std::runtime_error("stop failed");
    }
    ++stop_calls;
    return stop_result;
  }

  ControlResult fixStand() override {
    ++fixstand_calls;
    return fixstand_result;
  }

  ControlResult standbyVelocity() override {
    ++standby_velocity_calls;
    return standby_velocity_result;
  }

  int queue_calls{0};
  int interrupt_calls{0};
  int stop_calls{0};
  int fixstand_calls{0};
  int standby_velocity_calls{0};
  bool throw_stop{false};
  std::vector<ExecuteCommand> queue_commands;
  std::vector<ExecuteCommand> interrupt_commands;
};

class FakeStatus final : public StatusReader {
 public:
  StatusSnapshot snapshot() const override {
    if (throw_unknown_snapshot) {
      throw 7;
    }
    return snapshot_value;
  }

  RunLookupResult findRun(const std::string& id) const override {
    find_ids.push_back(id);
    const auto it = runs.find(id);
    if (it == runs.end()) {
      return {ErrorCode::RunNotFound, std::nullopt};
    }
    return {ErrorCode::Ok, it->second};
  }

  HealthSnapshot health() const override { return health_value; }

  StatusSnapshot snapshot_value{readySnapshot()};
  std::map<std::string, MotionStatus> runs;
  HealthSnapshot health_value{ServiceHealth::Ready, RuntimeMode::Sim, ErrorCode::Ok, ""};
  bool throw_unknown_snapshot{false};
  mutable std::vector<std::string> find_ids;
};

class FakeValidator final : public TrackValidatorPort {
 public:
  TrackValidation validate(const std::string& path) override {
    ++calls;
    paths.push_back(path);
    auto validation = result;
    if (validation.ok() && validation.metadata.canonical_path.empty()) {
      validation.metadata.canonical_path = path;
    }
    return validation;
  }

  int calls{0};
  std::vector<std::string> paths;
  TrackValidation result{ErrorCode::Ok, TrackMetadata{12, 0.22}, ""};
};

class FakeIds final : public RunIdGenerator {
 public:
  std::string generate() override {
    ++calls;
    return calls == 1 ? "a7K3p9Qx" : "B8m2Z5rT";
  }

  int calls{0};
};

struct Harness {
  AgentApiConfig config;
  FakeSink sink;
  FakeStatus status;
  FakeValidator validator;
  FakeIds ids;
  AgentApiService service{config, sink, status, validator, ids};
};

std::string errorCode(const ApiResponse& response) {
  return response.body.at("error").at("code").get<std::string>();
}

std::string nextAction(const ApiResponse& response) {
  return response.body.at("next").get<std::string>();
}

void requireFields(const nlohmann::json& body, std::set<std::string> expected) {
  REQUIRE(body.is_object());
  std::set<std::string> actual;
  for (const auto& item : body.items()) {
    actual.insert(item.key());
  }
  REQUIRE(actual == expected);
}

void requireFailure(const ApiResponse& response, const std::string& code) {
  REQUIRE(response.status >= 400);
  requireFields(response.body, {"ok", "error", "next"});
  REQUIRE(response.body.at("ok") == false);
  requireFields(response.body.at("error"), {"code", "message", "retryable"});
  REQUIRE(errorCode(response) == code);
  REQUIRE(response.body.at("error").contains("message"));
  REQUIRE(response.body.at("error").contains("retryable"));
  REQUIRE_FALSE(response.body.contains("err"));
  REQUIRE_FALSE(nextAction(response).empty());
}

bool isBase62RunId(const std::string& id) {
  static const std::regex pattern{"^[0-9A-Za-z]{8,10}$"};
  return std::regex_match(id, pattern);
}

MotionStatus run(std::string id, MotionState state) {
  MotionStatus status;
  status.id = std::move(id);
  status.path = "/tracks/" + status.id + ".trk";
  status.state = state;
  status.frames = 20;
  status.duration_s = 0.38;
  status.progress = state == MotionState::Done ? 1.0 : 0.0;
  return status;
}

}  // namespace

TEST_CASE("POST execute defaults to queue and only submits a queue command") {
  Harness h;

  const auto response =
      h.service.handle({"POST", "/execute", R"({"path":"/tracks/wave.trk"})"});

  REQUIRE(response.status == 200);
  requireFields(response.body, {"ok", "id", "state", "q"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("id") == "a7K3p9Qx");
  REQUIRE(isBase62RunId(response.body.at("id").get<std::string>()));
  REQUIRE(response.body.at("state") == "queued");
  REQUIRE(response.body.at("q") == 1);
  REQUIRE(h.sink.queue_calls == 1);
  REQUIRE(h.sink.interrupt_calls == 0);
  REQUIRE(h.sink.queue_commands.at(0).id == "a7K3p9Qx");
  REQUIRE(h.sink.queue_commands.at(0).path == "/tracks/wave.trk");
  REQUIRE(h.sink.queue_commands.at(0).mode == MotionMode::Queue);
  REQUIRE(h.sink.queue_commands.at(0).track.frames == 12);
  REQUIRE(h.validator.calls == 1);
}

TEST_CASE("POST execute submits validator canonical path instead of request path") {
  Harness h;
  h.validator.result.metadata.canonical_path = "/tracks/canonical/wave.trk";

  const auto response =
      h.service.handle({"POST", "/execute", R"({"path":"/tracks/link/wave.trk"})"});

  REQUIRE(response.status == 200);
  REQUIRE(h.validator.paths == std::vector<std::string>{"/tracks/link/wave.trk"});
  REQUIRE(h.sink.queue_calls == 1);
  REQUIRE(h.sink.queue_commands.at(0).path == "/tracks/canonical/wave.trk");
  REQUIRE(h.sink.queue_commands.at(0).track.canonical_path ==
          "/tracks/canonical/wave.trk");
}

TEST_CASE("POST execute mode interrupt only submits an interrupt command") {
  Harness h;

  const auto response = h.service.handle(
      {"POST", "/execute", R"({"path":"/tracks/urgent.trk","mode":"interrupt"})"});

  REQUIRE(response.status == 200);
  requireFields(response.body, {"ok", "id", "state", "q"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("state") == "queued");
  REQUIRE(h.sink.queue_calls == 0);
  REQUIRE(h.sink.interrupt_calls == 1);
  REQUIRE(h.sink.interrupt_commands.at(0).mode == MotionMode::Interrupt);
}

TEST_CASE("POST execute rejects invalid JSON contract before ports") {
  const std::vector<std::string> bodies{
      "",
      "not-json",
      R"({})",
      R"({"path":3})",
      R"({"path":"/tracks/a.trk","mode":null})",
      R"({"path":"/tracks/a.trk","mode":"replace"})",
  };

  for (const auto& body : bodies) {
    Harness h;
    const auto response = h.service.handle({"POST", "/execute", body});

    CAPTURE(body);
    REQUIRE(response.status == 400);
    requireFailure(response, "REQUEST_INVALID");
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST execute maps validator failures and does not submit commands") {
  const std::vector<ErrorCode> codes{
      ErrorCode::TrkPathNotAllowed,
      ErrorCode::TrkFileNotFound,
      ErrorCode::TrkParseFailed,
      ErrorCode::TrkValidationFailed,
  };

  for (const auto code : codes) {
    Harness h;
    h.validator.result.code = code;

    const auto response =
        h.service.handle({"POST", "/execute", R"({"path":"/tracks/bad.trk"})"});

    CAPTURE(toString(code));
    requireFailure(response, toString(code));
    REQUIRE(h.validator.calls == 1);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST execute reports queue full from the command sink") {
  Harness h;
  h.sink.queue_result.code = ErrorCode::QueueFull;
  h.sink.queue_result.q = 8;

  const auto response =
      h.service.handle({"POST", "/execute", R"({"path":"/tracks/full.trk"})"});

  REQUIRE(response.status == 409);
  requireFailure(response, "QUEUE_FULL");
  REQUIRE(nextAction(response) == "status");
  REQUIRE(h.sink.queue_calls == 1);
}

TEST_CASE("POST execute maps internal run-state conflict to public PRD error code") {
  Harness h;
  h.sink.queue_result.code = ErrorCode::RunStateConflict;

  const auto response =
      h.service.handle({"POST", "/execute", R"({"path":"/tracks/conflict.trk"})"});

  REQUIRE(response.status == 500);
  requireFailure(response, "INTERNAL_ERROR");
  REQUIRE(errorCode(response) != "RUN_STATE_CONFLICT");
}

TEST_CASE("POST execute gates service robot model and fault readiness before validation") {
  struct Case {
    ErrorCode code;
    StatusSnapshot snapshot;
  };

  auto service = readySnapshot();
  service.ready = false;
  service.ctrl = ControllerState::Starting;
  service.err = ErrorCode::ServiceNotReady;

  auto disconnected = readySnapshot();
  disconnected.ready = false;
  disconnected.robot = RobotState::Disconnected;
  disconnected.err = ErrorCode::RobotDisconnected;

  auto robot = readySnapshot();
  robot.ready = false;
  robot.robot = RobotState::NotReady;
  robot.err = ErrorCode::RobotNotReady;

  auto model = readySnapshot();
  model.ready = false;
  model.err = ErrorCode::ModelNotReady;
  model.block = "policy_not_loaded";

  auto fault = readySnapshot();
  fault.ready = false;
  fault.robot = RobotState::Fault;
  fault.ctrl = ControllerState::Fault;
  fault.err = ErrorCode::SafetyLimitTriggered;

  for (const auto& item : std::vector<Case>{
           {ErrorCode::ServiceNotReady, service},
           {ErrorCode::RobotDisconnected, disconnected},
           {ErrorCode::RobotNotReady, robot},
           {ErrorCode::ModelNotReady, model},
           {ErrorCode::SafetyLimitTriggered, fault},
       }) {
    Harness h;
    h.status.snapshot_value = item.snapshot;

    const auto response =
        h.service.handle({"POST", "/execute", R"({"path":"/tracks/a.trk"})"});

    CAPTURE(toString(item.code));
    requireFailure(response, toString(item.code));
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST stop with no body only submits stop even when not ready") {
  Harness h;
  h.status.snapshot_value.ready = false;
  h.status.snapshot_value.ctrl = ControllerState::Fault;
  h.status.snapshot_value.robot = RobotState::Fault;
  h.status.snapshot_value.err = ErrorCode::SafetyLimitTriggered;
  h.sink.stop_result = {ErrorCode::Ok, ControllerState::Fault, StopReason::Stop, 7};

  const auto response = h.service.handle({"POST", "/stop", ""});

  REQUIRE(response.status == 200);
  requireFields(response.body, {"ok", "state"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("state") == "accepted");
  REQUIRE(h.sink.stop_calls == 1);
  REQUIRE(h.sink.queue_calls == 0);
  REQUIRE(h.sink.interrupt_calls == 0);
  REQUIRE(h.validator.calls == 0);
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("POST control endpoints accept empty body without validator or id generator") {
  SECTION("fixstand") {
    Harness h;

    const auto response = h.service.handle({"POST", "/fixstand", ""});

    REQUIRE(response.status == 200);
    requireFields(response.body, {"ok", "state"});
    REQUIRE(response.body.at("ok") == true);
    REQUIRE(response.body.at("state") == "accepted");
    REQUIRE(h.sink.fixstand_calls == 1);
    REQUIRE(h.sink.standby_velocity_calls == 0);
    REQUIRE(h.sink.stop_calls == 0);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }

  SECTION("standby velocity") {
    Harness h;

    const auto response = h.service.handle({"POST", "/standby_velocity", ""});

    REQUIRE(response.status == 200);
    requireFields(response.body, {"ok", "state"});
    REQUIRE(response.body.at("ok") == true);
    REQUIRE(response.body.at("state") == "accepted");
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 1);
    REQUIRE(h.sink.stop_calls == 0);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST control endpoints reject non-empty bodies before ports") {
  for (const auto& target : {"/fixstand", "/standby_velocity"}) {
    Harness h;

    const auto response = h.service.handle({"POST", target, R"({})"});

    CAPTURE(target);
    REQUIRE(response.status == 400);
    requireFailure(response, "REQUEST_INVALID");
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 0);
    REQUIRE(h.sink.stop_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST standby_velocity returns conflict when controller state rejects it") {
  Harness h;
  h.sink.standby_velocity_result = {ErrorCode::ControlStateConflict};

  const auto response = h.service.handle({"POST", "/standby_velocity", ""});

  REQUIRE(response.status == 409);
  requireFailure(response, "CONTROL_STATE_CONFLICT");
  REQUIRE(h.sink.standby_velocity_calls == 1);
  REQUIRE(h.sink.fixstand_calls == 0);
  REQUIRE(h.validator.calls == 0);
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("API handle converts std exceptions from ports to internal error envelopes") {
  Harness h;
  h.sink.throw_stop = true;

  const auto response = h.service.handle({"POST", "/stop", ""});

  REQUIRE(response.status == 500);
  requireFailure(response, "INTERNAL_ERROR");
  REQUIRE(h.sink.stop_calls == 0);
}

TEST_CASE("API handle converts unknown exceptions from ports to internal error envelopes") {
  Harness h;
  h.status.throw_unknown_snapshot = true;

  const auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 500);
  requireFailure(response, "INTERNAL_ERROR");
}

TEST_CASE("GET status renders idle nulls and queue short fields") {
  Harness h;
  h.status.snapshot_value.queue = {2, 8, {"a", "b"}};

  const auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("mode") == "sim");
  REQUIRE(response.body.at("exec").is_null());
  REQUIRE(response.body.at("queue").at("n") == 2);
  REQUIRE(response.body.at("queue").at("limit") == 8);
  REQUIRE(response.body.at("queue").at("ids").at(0) == "a");
  REQUIRE(response.body.at("err").is_null());
  REQUIRE(response.body.at("stop_reason").is_null());
}

TEST_CASE("GET status renders Passive as an explicit controller state") {
  Harness h;
  h.status.snapshot_value.ctrl = ControllerState::Passive;
  h.status.snapshot_value.ready = false;
  h.status.snapshot_value.err = ErrorCode::RobotBadOrientation;
  h.status.snapshot_value.block = "bad_orientation";

  const auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("ctrl") == "passive");
  REQUIRE(response.body.at("block") == "bad_orientation");
}

TEST_CASE("GET status renders running progress and top-level stopping reason") {
  Harness h;
  MotionStatus exec = run("active", MotionState::Running);
  exec.frame = 4;
  exec.frames = 20;
  exec.progress = 0.25;
  exec.duration_s = 0.38;
  h.status.snapshot_value.ctrl = ControllerState::Running;
  h.status.snapshot_value.robot = RobotState::Running;
  h.status.snapshot_value.exec = exec;

  auto response = h.service.handle({"GET", "/status", ""});
  requireFields(response.body.at("exec"),
                {"id", "state", "frame", "frames", "time_s", "duration_s",
                 "progress", "stop_reason", "err"});
  REQUIRE(response.body.at("exec").at("frame") == 4);
  REQUIRE(response.body.at("exec").at("frames") == 20);
  REQUIRE(response.body.at("exec").at("progress") == 0.25);
  REQUIRE(response.body.at("exec").at("duration_s") == 0.38);
  REQUIRE_FALSE(response.body.at("exec").contains("path"));
  REQUIRE(response.body.at("stop_reason").is_null());

  h.status.snapshot_value.ctrl = ControllerState::Stopping;
  h.status.snapshot_value.exec.reset();
  h.status.snapshot_value.stop_reason = StopReason::Stop;

  response = h.service.handle({"GET", "/status", ""});
  REQUIRE(response.body.at("stop_reason") == "stop");
}

TEST_CASE("GET status by id covers active queued and recent run states") {
  Harness h;
  h.status.runs.emplace("active", run("active", MotionState::Running));
  h.status.runs.emplace("queued", run("queued", MotionState::Queued));
  h.status.runs.emplace("done", run("done", MotionState::Done));
  h.status.runs.emplace("stopped", run("stopped", MotionState::Stopped));
  h.status.runs.emplace("canceled", run("canceled", MotionState::Canceled));

  for (const auto& id : {"active", "queued", "done", "stopped", "canceled"}) {
    const auto response =
        h.service.handle({"GET", std::string("/status?id=") + id, ""});

    CAPTURE(id);
    REQUIRE(response.status == 200);
    REQUIRE(response.body.at("ok") == true);
    REQUIRE(response.body.at("id") == id);
    REQUIRE(response.body.at("path") == std::string("/tracks/") + id + ".trk");
    REQUIRE(response.body.at("robot") == "idle");
  }

  const auto missing = h.service.handle({"GET", "/status?id=missing", ""});
  REQUIRE(missing.status == 404);
  requireFailure(missing, "RUN_NOT_FOUND");
}

TEST_CASE("GET status rejects explicitly empty id query values") {
  for (const auto& target : {"/status?id=", "/status?x=1&id="}) {
    Harness h;

    const auto response = h.service.handle({"GET", target, ""});

    CAPTURE(target);
    REQUIRE(response.status == 400);
    requireFailure(response, "REQUEST_INVALID");
    REQUIRE(h.status.find_ids.empty());
  }
}

TEST_CASE("GET health maps service health and runtime mode strings") {
  Harness h;

  h.status.health_value = {ServiceHealth::Starting, RuntimeMode::Unknown,
                           ErrorCode::ServiceNotReady, "service_initializing"};
  auto response = h.service.handle({"GET", "/health", ""});
  REQUIRE(response.status == 200);
  requireFields(response.body, {"ok", "state", "mode"});
  REQUIRE(response.body.at("ok") == false);
  REQUIRE(response.body.at("state") == "starting");
  REQUIRE(response.body.at("mode") == "unknown");
  REQUIRE_FALSE(response.body.contains("health"));
  REQUIRE_FALSE(response.body.contains("err"));
  REQUIRE_FALSE(response.body.contains("block"));

  h.status.health_value = {ServiceHealth::Ready, RuntimeMode::Real, ErrorCode::Ok, ""};
  response = h.service.handle({"GET", "/health", ""});
  requireFields(response.body, {"ok", "state", "mode"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("state") == "ready");
  REQUIRE(response.body.at("mode") == "real");

  h.status.health_value = {ServiceHealth::Ready, RuntimeMode::Real,
                           ErrorCode::ModelNotReady, "policy_not_loaded"};
  response = h.service.handle({"GET", "/health", ""});
  requireFields(response.body, {"ok", "state", "mode"});
  REQUIRE(response.body.at("ok") == false);
  REQUIRE(response.body.at("state") == "starting");
  REQUIRE(response.body.at("mode") == "real");

  h.status.health_value = {ServiceHealth::Error, RuntimeMode::Sim,
                           ErrorCode::SafetyLimitTriggered, "safety_limit"};
  response = h.service.handle({"GET", "/health", ""});
  requireFields(response.body, {"ok", "state", "mode"});
  REQUIRE(response.body.at("ok") == false);
  REQUIRE(response.body.at("state") == "error");
  REQUIRE(response.body.at("mode") == "sim");
}

TEST_CASE("Core status exposes runtime mode and PRD block strings") {
  REQUIRE(toString(RuntimeMode::Sim) == "sim");
  REQUIRE(toString(RuntimeMode::Real) == "real");
  REQUIRE(toString(RuntimeMode::Unknown) == "unknown");

  StatusSnapshot snapshot;
  REQUIRE(snapshot.mode == RuntimeMode::Unknown);
}

}  // namespace agentic_et1_tracker
