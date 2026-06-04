#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
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
  snapshot.ctrl = ControllerState::StandbyVelocity;
  snapshot.queue.limit = 8;
  return snapshot;
}

class FakeSink final : public ExecutionCommandSink {
 public:
  ExecuteResult queue_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  ExecuteResult interrupt_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  StopResult stop_result{ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 0};
  ControlResult passive_result{ErrorCode::Ok};
  ControlResult fixstand_result{ErrorCode::Ok};
  ControlResult standby_velocity_result{ErrorCode::Ok};
  IdleResult idle_result{ErrorCode::Ok, IdleStatus{}};

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

  ControlResult passive() override {
    ++passive_calls;
    return passive_result;
  }

  ControlResult fixStand() override {
    ++fixstand_calls;
    return fixstand_result;
  }

  ControlResult standbyVelocity() override {
    ++standby_velocity_calls;
    return standby_velocity_result;
  }

  IdleResult configureIdle(std::vector<IdleMotion> motions) override {
    ++idle_calls;
    idle_motions = std::move(motions);
    auto result = idle_result;
    result.idle.enabled = !idle_motions.empty();
    result.idle.n = idle_motions.size();
    result.idle.active = false;
    return result;
  }

  int queue_calls{0};
  int interrupt_calls{0};
  int stop_calls{0};
  int passive_calls{0};
  int fixstand_calls{0};
  int standby_velocity_calls{0};
  int idle_calls{0};
  bool throw_stop{false};
  std::vector<ExecuteCommand> queue_commands;
  std::vector<ExecuteCommand> interrupt_commands;
  std::vector<IdleMotion> idle_motions;
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
    const auto override = results.find(path);
    if (override != results.end()) {
      auto validation = override->second;
      if (validation.ok() && validation.metadata.canonical_path.empty()) {
        validation.metadata.canonical_path = path;
      }
      return validation;
    }
    auto validation = result;
    if (validation.ok() && validation.metadata.canonical_path.empty()) {
      validation.metadata.canonical_path = path;
    }
    return validation;
  }

  int calls{0};
  std::vector<std::string> paths;
  TrackValidation result{ErrorCode::Ok, TrackMetadata{12, 0.22}, ""};
  std::map<std::string, TrackValidation> results;
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

std::string errorMessage(const ApiResponse& response) {
  return response.body.at("error").at("message").get<std::string>();
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
  requireFields(response.body, {"ok", "id", "state", "q", "hold"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("id") == "a7K3p9Qx");
  REQUIRE(isBase62RunId(response.body.at("id").get<std::string>()));
  REQUIRE(response.body.at("state") == "queued");
  REQUIRE(response.body.at("q") == 1);
  REQUIRE(response.body.at("hold") == false);
  REQUIRE(h.sink.queue_calls == 1);
  REQUIRE(h.sink.interrupt_calls == 0);
  REQUIRE(h.sink.queue_commands.at(0).id == "a7K3p9Qx");
  REQUIRE(h.sink.queue_commands.at(0).path == "/tracks/wave.trk");
  REQUIRE(h.sink.queue_commands.at(0).mode == MotionMode::Queue);
  REQUIRE_FALSE(h.sink.queue_commands.at(0).hold);
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
  requireFields(response.body, {"ok", "id", "state", "q", "hold"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("state") == "queued");
  REQUIRE(response.body.at("hold") == false);
  REQUIRE(h.sink.queue_calls == 0);
  REQUIRE(h.sink.interrupt_calls == 1);
  REQUIRE(h.sink.interrupt_commands.at(0).mode == MotionMode::Interrupt);
  REQUIRE_FALSE(h.sink.interrupt_commands.at(0).hold);
}

TEST_CASE("POST execute accepts boolean hold without changing mode semantics") {
  {
    Harness h;

    const auto response =
        h.service.handle({"POST", "/execute", R"({"path":"/tracks/hold.trk","hold":true})"});

    REQUIRE(response.status == 200);
    requireFields(response.body, {"ok", "id", "state", "q", "hold"});
    REQUIRE(response.body.at("hold") == true);
    REQUIRE(h.sink.queue_calls == 1);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.sink.queue_commands.at(0).mode == MotionMode::Queue);
    REQUIRE(h.sink.queue_commands.at(0).hold);
  }

  {
    Harness h;

    const auto response = h.service.handle(
        {"POST", "/execute",
         R"({"path":"/tracks/no-hold.trk","mode":"interrupt","hold":false})"});

    REQUIRE(response.status == 200);
    requireFields(response.body, {"ok", "id", "state", "q", "hold"});
    REQUIRE(response.body.at("hold") == false);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 1);
    REQUIRE(h.sink.interrupt_commands.at(0).mode == MotionMode::Interrupt);
    REQUIRE_FALSE(h.sink.interrupt_commands.at(0).hold);
  }
}

TEST_CASE("POST execute rejects invalid JSON contract before ports") {
  const std::vector<std::string> bodies{
      "",
      "not-json",
      R"({})",
      R"({"path":3})",
      R"({"path":"/tracks/a.trk","mode":null})",
      R"({"path":"/tracks/a.trk","mode":"replace"})",
      R"({"path":"/tracks/a.trk","hold":null})",
      R"({"path":"/tracks/a.trk","hold":"true"})",
      R"({"path":"/tracks/a.trk","hold":1})",
      R"({"path":"/tracks/a.trk","paths":["/tracks/b.trk"]})",
      R"({"path":"/tracks/a.trk","transition_duration_s":1.0})",
      R"({"path":"/tracks/a.trk","extra":true})",
      R"({"paths":["/tracks/a.trk"]})",
  };

  for (const auto& body : bodies) {
    Harness h;
    const auto response = h.service.handle({"POST", "/execute", body});

    CAPTURE(body);
    REQUIRE(response.status == 400);
    requireFailure(response, "REQUEST_INVALID");
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.sink.idle_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST idle rejects invalid JSON contract before ports") {
  const std::vector<std::string> bodies{
      "",
      "not-json",
      R"({})",
      R"({"paths":"/tracks/a.trk"})",
      R"({"paths":[3]})",
      R"({"paths":[""]})",
      R"({"paths":["/tracks/a.trk"],"mode":"queue"})",
      R"({"paths":["/tracks/a.trk"],"transition_duration_s":1.0})",
  };

  for (const auto& body : bodies) {
    Harness h;
    const auto response = h.service.handle({"POST", "/idle", body});

    CAPTURE(body);
    REQUIRE(response.status == 400);
    requireFailure(response, "REQUEST_INVALID");
    REQUIRE(h.sink.idle_calls == 0);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST idle validates paths atomically and never allocates run ids") {
  Harness h;
  h.validator.results["/tracks/a.trk"] =
      {ErrorCode::Ok, TrackMetadata{10, 0.2, 50.0, "/canonical/a.trk"}, ""};
  h.validator.results["/tracks/bad.trk"] =
      {ErrorCode::TrkFileNotFound, TrackMetadata{}, "missing"};

  auto response = h.service.handle(
      {"POST", "/idle", R"({"paths":["/tracks/a.trk","/tracks/bad.trk"]})"});

  REQUIRE(response.status == 400);
  requireFailure(response, "TRK_FILE_NOT_FOUND");
  REQUIRE(h.validator.paths ==
          std::vector<std::string>{"/tracks/a.trk", "/tracks/bad.trk"});
  REQUIRE(h.sink.idle_calls == 0);
  REQUIRE(h.ids.calls == 0);

  h.validator.results["/tracks/b.trk"] =
      {ErrorCode::Ok, TrackMetadata{20, 0.4, 50.0, "/canonical/b.trk"}, ""};
  response = h.service.handle(
      {"POST", "/idle", R"({"paths":["/tracks/a.trk","/tracks/b.trk"]})"});

  REQUIRE(response.status == 200);
  requireFields(response.body, {"ok", "idle"});
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("idle").at("enabled") == true);
  REQUIRE(response.body.at("idle").at("n") == 2);
  REQUIRE(response.body.at("idle").at("active") == false);
  REQUIRE(h.sink.idle_calls == 1);
  REQUIRE(h.sink.idle_motions.at(0).path == "/canonical/a.trk");
  REQUIRE(h.sink.idle_motions.at(1).path == "/canonical/b.trk");
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("POST idle accepts empty paths as an atomic clear without validation") {
  Harness h;

  const auto response = h.service.handle({"POST", "/idle", R"({"paths":[]})"});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("ok") == true);
  REQUIRE(response.body.at("idle").at("enabled") == false);
  REQUIRE(response.body.at("idle").at("n") == 0);
  REQUIRE(response.body.at("idle").at("active") == false);
  REQUIRE(h.sink.idle_calls == 1);
  REQUIRE(h.sink.idle_motions.empty());
  REQUIRE(h.validator.calls == 0);
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("POST idle clear accepts unsafe states without validation") {
  Harness h;
  h.status.snapshot_value.ready = false;
  h.status.snapshot_value.ctrl = ControllerState::Passive;
  h.status.snapshot_value.robot = RobotState::Fault;
  h.status.snapshot_value.err = ErrorCode::RobotBadOrientation;
  h.status.snapshot_value.block = "bad_orientation";

  const auto response = h.service.handle({"POST", "/idle", R"({"paths":[]})"});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("idle").at("enabled") == false);
  REQUIRE(h.validator.calls == 0);
  REQUIRE(h.sink.idle_calls == 1);
  REQUIRE(h.sink.idle_motions.empty());
  REQUIRE(h.sink.queue_calls == 0);
  REQUIRE(h.sink.interrupt_calls == 0);
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("POST idle nonempty rejects unsafe controller states before validation") {
  for (const ControllerState ctrl :
       {ControllerState::Starting,
        ControllerState::Idle,
        ControllerState::Passive,
        ControllerState::FixStand,
        ControllerState::Stopping,
        ControllerState::Fault}) {
    Harness h;
    h.status.snapshot_value.ready = true;
    h.status.snapshot_value.ctrl = ctrl;

    const auto response =
        h.service.handle({"POST", "/idle", R"({"paths":["/tracks/idle.trk"]})"});

    CAPTURE(toString(ctrl));
    REQUIRE(response.status == 409);
    requireFailure(response, "CONTROL_STATE_CONFLICT");
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.sink.idle_calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST idle nonempty accepts standby and active motion states") {
  for (const ControllerState ctrl :
       {ControllerState::StandbyVelocity,
        ControllerState::Preparing,
        ControllerState::Running}) {
    Harness h;
    h.status.snapshot_value.ctrl = ctrl;

    const auto response =
        h.service.handle({"POST", "/idle", R"({"paths":["/tracks/idle.trk"]})"});

    CAPTURE(toString(ctrl));
    REQUIRE(response.status == 200);
    REQUIRE(response.body.at("idle").at("enabled") == true);
    REQUIRE(h.validator.calls == 1);
    REQUIRE(h.sink.idle_calls == 1);
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

TEST_CASE("POST execute reports lowcmd occupancy as manual next action") {
  Harness h;
  h.status.snapshot_value.ready = false;
  h.status.snapshot_value.ctrl = ControllerState::StandbyVelocity;
  h.status.snapshot_value.robot = RobotState::NotReady;
  h.status.snapshot_value.err = ErrorCode::RobotNotReady;
  h.status.snapshot_value.block = "lowcmd_occupied";

  const auto response =
      h.service.handle({"POST", "/execute", R"({"path":"/tracks/a.trk"})"});

  REQUIRE(response.status == 409);
  requireFailure(response, "ROBOT_NOT_READY");
  REQUIRE(nextAction(response) == "manual");
  REQUIRE(h.validator.calls == 0);
  REQUIRE(h.sink.queue_calls == 0);
  REQUIRE(h.sink.interrupt_calls == 0);
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("POST control endpoints report lowcmd occupancy as manual before sink") {
  for (const auto& target : {"/passive", "/fixstand", "/standby_velocity"}) {
    for (const ControllerState ctrl :
         {ControllerState::Passive, ControllerState::Fault}) {
      Harness h;
      h.status.snapshot_value.ready = false;
      h.status.snapshot_value.ctrl = ctrl;
      h.status.snapshot_value.robot = RobotState::NotReady;
      h.status.snapshot_value.err = ErrorCode::RobotNotReady;
      h.status.snapshot_value.block = "lowcmd_occupied";

      const std::string body =
          std::string(target) == "/passive" ? R"({"password":"galaxy"})" : "";
      const auto response = h.service.handle({"POST", target, body});

      CAPTURE(target);
      CAPTURE(toString(ctrl));
      REQUIRE(response.status == 409);
      requireFailure(response, "ROBOT_NOT_READY");
      REQUIRE(nextAction(response) == "manual");
      REQUIRE(h.sink.passive_calls == 0);
      REQUIRE(h.sink.fixstand_calls == 0);
      REQUIRE(h.sink.standby_velocity_calls == 0);
      REQUIRE(h.validator.calls == 0);
      REQUIRE(h.ids.calls == 0);
    }
  }
}

TEST_CASE("POST execute rejects controller states that cannot execute queued motion") {
  for (const ControllerState ctrl :
       {ControllerState::Starting,
        ControllerState::Idle,
        ControllerState::Passive,
        ControllerState::FixStand,
        ControllerState::Stopping,
        ControllerState::Fault}) {
    Harness h;
    h.status.snapshot_value.ready = true;
    h.status.snapshot_value.ctrl = ctrl;
    h.status.snapshot_value.robot = RobotState::Holding;

    const auto response =
        h.service.handle({"POST", "/execute", R"({"path":"/tracks/a.trk"})"});

    CAPTURE(toString(ctrl));
    REQUIRE(response.status == 409);
    requireFailure(response, "CONTROL_STATE_CONFLICT");
    if (ctrl == ControllerState::FixStand) {
      REQUIRE(nextAction(response) == "standby_velocity");
      REQUIRE(errorMessage(response) == "ctrl=fixstand; /standby_velocity");
    } else if (ctrl == ControllerState::Passive) {
      REQUIRE(nextAction(response) == "fixstand");
      REQUIRE(errorMessage(response) ==
              "ctrl=passive; /fixstand then /standby_velocity");
    } else if (ctrl == ControllerState::Fault) {
      REQUIRE(nextAction(response) == "fixstand");
      REQUIRE(errorMessage(response) == "ctrl=fault; /fixstand");
    } else if (ctrl == ControllerState::Idle) {
      REQUIRE(nextAction(response) == "status");
      REQUIRE(errorMessage(response) == "wrong ctrl; check /status");
    } else {
      REQUIRE(nextAction(response) == "status");
      REQUIRE(errorMessage(response) ==
              "ctrl=" + toString(ctrl) + "; wait /status");
    }
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
    REQUIRE(h.sink.passive_calls == 0);
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
    REQUIRE(h.sink.passive_calls == 0);
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 1);
    REQUIRE(h.sink.stop_calls == 0);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST passive requires configured password before command sink") {
  struct Case {
    std::string body;
    int expected_status;
    int expected_calls;
  };

  const std::vector<Case> cases{
      {"", 400, 0},
      {R"({})", 400, 0},
      {R"({"password":"wrong"})", 400, 0},
      {R"({"password":3})", 400, 0},
      {R"({"password":"galaxy","extra":true})", 400, 0},
      {R"({"password":"galaxy"})", 200, 1},
  };

  for (const auto& item : cases) {
    Harness h;
    const auto response = h.service.handle({"POST", "/passive", item.body});

    CAPTURE(item.body);
    REQUIRE(response.status == item.expected_status);
    REQUIRE(h.sink.passive_calls == item.expected_calls);
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 0);
    REQUIRE(h.sink.stop_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
    if (item.expected_status == 200) {
      requireFields(response.body, {"ok", "state"});
      REQUIRE(response.body.at("ok") == true);
      REQUIRE(response.body.at("state") == "accepted");
    } else {
      requireFailure(response, "REQUEST_INVALID");
    }
  }
}

TEST_CASE("POST passive uses configured password") {
  AgentApiConfig config;
  config.passive_password = "secret";
  FakeSink sink;
  FakeStatus status;
  FakeValidator validator;
  FakeIds ids;
  AgentApiService service{config, sink, status, validator, ids};

  auto response = service.handle({"POST", "/passive", R"({"password":"galaxy"})"});
  REQUIRE(response.status == 400);
  requireFailure(response, "REQUEST_INVALID");
  REQUIRE(sink.passive_calls == 0);

  response = service.handle({"POST", "/passive", R"({"password":"secret"})"});
  REQUIRE(response.status == 200);
  REQUIRE(sink.passive_calls == 1);
}

TEST_CASE("POST control endpoints reject static not-ready snapshots before sink") {
  for (const auto& target : {"/passive", "/fixstand", "/standby_velocity"}) {
    Harness h;
    h.status.snapshot_value.ready = false;
    h.status.snapshot_value.ctrl = ControllerState::Starting;
    h.status.snapshot_value.robot = RobotState::NotReady;
    h.status.snapshot_value.err = ErrorCode::ModelNotReady;
    h.status.snapshot_value.block = "policy_not_loaded";

    const std::string body =
        std::string(target) == "/passive" ? R"({"password":"galaxy"})" : "";
    const auto response = h.service.handle({"POST", target, body});

    CAPTURE(target);
    REQUIRE(response.status == 503);
    requireFailure(response, "MODEL_NOT_READY");
    REQUIRE(nextAction(response) == "status");
    REQUIRE(errorMessage(response) == "policy model is not ready");
    REQUIRE(h.sink.passive_calls == 0);
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 0);
    REQUIRE(h.sink.stop_calls == 0);
    REQUIRE(h.sink.queue_calls == 0);
    REQUIRE(h.sink.interrupt_calls == 0);
    REQUIRE(h.validator.calls == 0);
    REQUIRE(h.ids.calls == 0);
  }
}

TEST_CASE("POST fixstand remains available from passive and fault recovery states") {
  struct Case {
    ControllerState ctrl;
    RobotState robot;
    ErrorCode err;
  };

  for (const auto& item : std::vector<Case>{
           {ControllerState::Passive, RobotState::NotReady,
            ErrorCode::RobotBadOrientation},
           {ControllerState::Fault, RobotState::Fault,
            ErrorCode::RobotBadOrientation},
       }) {
    Harness h;
    h.status.snapshot_value.ready = false;
    h.status.snapshot_value.ctrl = item.ctrl;
    h.status.snapshot_value.robot = item.robot;
    h.status.snapshot_value.err = item.err;
    h.status.snapshot_value.block = "bad_orientation";

    const auto response = h.service.handle({"POST", "/fixstand", ""});

    CAPTURE(toString(item.ctrl));
    REQUIRE(response.status == 200);
    requireFields(response.body, {"ok", "state"});
    REQUIRE(response.body.at("ok") == true);
    REQUIRE(response.body.at("state") == "accepted");
    REQUIRE(h.sink.fixstand_calls == 1);
    REQUIRE(h.sink.standby_velocity_calls == 0);
  }
}

TEST_CASE("POST passive remains available as bad-orientation safety sink") {
  for (const ControllerState ctrl :
       {ControllerState::Passive, ControllerState::Fault,
        ControllerState::Running}) {
    Harness h;
    h.status.snapshot_value.ready = false;
    h.status.snapshot_value.ctrl = ctrl;
    h.status.snapshot_value.robot = RobotState::Fault;
    h.status.snapshot_value.err = ErrorCode::RobotBadOrientation;
    h.status.snapshot_value.block = "bad_orientation";

    const auto response =
        h.service.handle({"POST", "/passive", R"({"password":"galaxy"})"});

    CAPTURE(toString(ctrl));
    REQUIRE(response.status == 200);
    requireFields(response.body, {"ok", "state"});
    REQUIRE(response.body.at("ok") == true);
    REQUIRE(response.body.at("state") == "accepted");
    REQUIRE(h.sink.passive_calls == 1);
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 0);
  }
}

TEST_CASE("POST fixstand rejects non-orientation recovery faults before sink") {
  for (const ErrorCode err :
       {ErrorCode::RobotNotReady, ErrorCode::SafetyLimitTriggered}) {
    Harness h;
    h.status.snapshot_value.ready = false;
    h.status.snapshot_value.ctrl = ControllerState::Fault;
    h.status.snapshot_value.robot = RobotState::Fault;
    h.status.snapshot_value.err = err;
    h.status.snapshot_value.block = "safety_limit";

    const auto response = h.service.handle({"POST", "/fixstand", ""});

    CAPTURE(toString(err));
    REQUIRE(response.status == 409);
    requireFailure(response, toString(err));
    REQUIRE(h.sink.fixstand_calls == 0);
    REQUIRE(h.sink.standby_velocity_calls == 0);
  }
}

TEST_CASE("POST control endpoints reject non-empty bodies before ports") {
  for (const auto& target : {"/fixstand", "/standby_velocity"}) {
    Harness h;

    const auto response = h.service.handle({"POST", target, R"({})"});

    CAPTURE(target);
    REQUIRE(response.status == 400);
    requireFailure(response, "REQUEST_INVALID");
    REQUIRE(h.sink.passive_calls == 0);
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
  REQUIRE(nextAction(response) == "status");
  REQUIRE(errorMessage(response) == "wrong ctrl; check /status");
  REQUIRE(h.sink.standby_velocity_calls == 1);
  REQUIRE(h.sink.fixstand_calls == 0);
  REQUIRE(h.validator.calls == 0);
  REQUIRE(h.ids.calls == 0);
}

TEST_CASE("POST standby_velocity rejects Passive and Fault before command sink") {
  for (const ControllerState ctrl : {ControllerState::Passive, ControllerState::Fault}) {
    Harness h;
    h.status.snapshot_value.ready = true;
    h.status.snapshot_value.ctrl = ctrl;

    const auto response = h.service.handle({"POST", "/standby_velocity", ""});

    CAPTURE(toString(ctrl));
    REQUIRE(response.status == 409);
    requireFailure(response, "CONTROL_STATE_CONFLICT");
    REQUIRE(nextAction(response) == "fixstand");
    if (ctrl == ControllerState::Passive) {
      REQUIRE(errorMessage(response) ==
              "ctrl=passive; /fixstand then /standby_velocity");
    } else {
      REQUIRE(errorMessage(response) == "ctrl=fault; /fixstand");
    }
    REQUIRE(h.sink.standby_velocity_calls == 0);
    REQUIRE(h.sink.fixstand_calls == 0);
  }
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
  REQUIRE(response.body.at("active").at("kind") == "none");
  REQUIRE(response.body.at("active").at("id").is_null());
  REQUIRE(response.body.at("exec").is_null());
  REQUIRE(response.body.at("queue").at("n") == 2);
  REQUIRE(response.body.at("queue").at("limit") == 8);
  REQUIRE(response.body.at("queue").at("ids").at(0) == "a");
  REQUIRE(response.body.at("idle").at("enabled") == false);
  REQUIRE(response.body.at("idle").at("n") == 0);
  REQUIRE(response.body.at("idle").at("active") == false);
  REQUIRE(response.body.at("idle").at("current").is_null());
  REQUIRE(response.body.at("err").is_null());
  REQUIRE(response.body.at("stop_reason").is_null());

  requireFields(response.body.at("transition"),
                {"active", "target", "target_id", "target_state", "frame",
                 "frames", "progress"});
  REQUIRE(response.body.at("transition").at("active") == false);
  REQUIRE(response.body.at("transition").at("target").is_null());
  REQUIRE(response.body.at("transition").at("target_id").is_null());
  REQUIRE(response.body.at("transition").at("target_state").is_null());
  REQUIRE(response.body.at("transition").at("frame") == 0);
  REQUIRE(response.body.at("transition").at("frames") == 0);
  REQUIRE(response.body.at("transition").at("progress") == 0.0);
}

TEST_CASE("GET status renders authoritative active and idle progress fields") {
  Harness h;
  h.status.snapshot_value.ctrl = ControllerState::Running;
  h.status.snapshot_value.active.kind = ActiveKind::Idle;
  h.status.snapshot_value.idle.enabled = true;
  h.status.snapshot_value.idle.n = 2;
  h.status.snapshot_value.idle.active = true;
  h.status.snapshot_value.idle.current = 0;
  h.status.snapshot_value.idle.frame = 12;
  h.status.snapshot_value.idle.frames = 120;
  h.status.snapshot_value.idle.time_s = 0.24;
  h.status.snapshot_value.idle.duration_s = 2.4;
  h.status.snapshot_value.idle.progress = 0.1;

  auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("active").at("kind") == "idle");
  REQUIRE(response.body.at("active").at("id").is_null());
  REQUIRE(response.body.at("exec").is_null());
  REQUIRE(response.body.at("idle").at("enabled") == true);
  REQUIRE(response.body.at("idle").at("n") == 2);
  REQUIRE(response.body.at("idle").at("active") == true);
  REQUIRE(response.body.at("idle").at("current") == 0);
  REQUIRE(response.body.at("idle").at("frame") == 12);
  REQUIRE(response.body.at("idle").at("frames") == 120);
  REQUIRE(response.body.at("idle").at("time_s") == 0.24);
  REQUIRE(response.body.at("idle").at("duration_s") == 2.4);
  REQUIRE(response.body.at("idle").at("progress") == 0.1);

  h.status.snapshot_value.active = {ActiveKind::User, "active"};
  h.status.snapshot_value.exec = run("active", MotionState::Running);
  h.status.snapshot_value.idle.active = false;

  response = h.service.handle({"GET", "/status", ""});
  REQUIRE(response.body.at("active").at("kind") == "user");
  REQUIRE(response.body.at("active").at("id") == "active");
  REQUIRE(response.body.at("exec").at("id") == "active");
  REQUIRE(response.body.at("idle").at("active") == false);
}

TEST_CASE("GET status renders transition active kind without controller expansion") {
  Harness h;
  h.status.snapshot_value.ctrl = ControllerState::Running;
  h.status.snapshot_value.active = {ActiveKind::Transition, "internal-transition"};

  const auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("ctrl") == "running");
  REQUIRE(response.body.at("active").at("kind") == "transition");
  REQUIRE(response.body.at("active").at("id").is_null());
  REQUIRE(response.body.at("transition").at("active") == false);
}

TEST_CASE("GET status renders active transition snapshot public contract") {
  Harness h;
  h.status.snapshot_value.ctrl = ControllerState::Running;
  h.status.snapshot_value.active = {ActiveKind::Transition, ""};
  h.status.snapshot_value.queue = {1, 8, {"queued-user"}};
  h.status.snapshot_value.transition.active = true;
  h.status.snapshot_value.transition.target = "user";
  h.status.snapshot_value.transition.target_id = "target-run";
  h.status.snapshot_value.transition.target_state = MotionState::Queued;
  h.status.snapshot_value.transition.frame = 3;
  h.status.snapshot_value.transition.frames = 12;
  h.status.snapshot_value.transition.progress = 0.25;

  const auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 200);
  REQUIRE(response.body.at("active").at("kind") == "transition");
  REQUIRE(response.body.at("active").at("id").is_null());
  REQUIRE(response.body.at("exec").is_null());
  REQUIRE(response.body.at("transition").at("active") == true);
  REQUIRE(response.body.at("transition").at("target") == "user");
  REQUIRE(response.body.at("transition").at("target_id") == "target-run");
  REQUIRE(response.body.at("transition").at("target_state") == "queued");
  REQUIRE(response.body.at("transition").at("frame") == 3);
  REQUIRE(response.body.at("transition").at("frames") == 12);
  REQUIRE(response.body.at("transition").at("progress") == 0.25);
  REQUIRE(response.body.at("queue").at("ids") == nlohmann::json::array({"queued-user"}));
  REQUIRE(std::find(response.body.at("queue").at("ids").begin(),
                    response.body.at("queue").at("ids").end(),
                    "transition") == response.body.at("queue").at("ids").end());
}

TEST_CASE("GET status renders compact pose for high-rate agent polling") {
  Harness h;
  h.status.snapshot_value.pose.q_wxyz = std::array<float, 4>{{1.0F, 0.1F, 0.2F, 0.3F}};
  h.status.snapshot_value.pose.gyro_xyz = std::array<float, 3>{{0.4F, 0.5F, 0.6F}};
  h.status.snapshot_value.pose.position_xyz = std::array<float, 3>{{1.1F, 1.2F, 1.3F}};
  h.status.snapshot_value.pose.velocity_xyz = std::array<float, 3>{{2.1F, 2.2F, 2.3F}};

  const auto response = h.service.handle({"GET", "/status", ""});

  REQUIRE(response.status == 200);
  const auto& pose = response.body.at("pose");
  REQUIRE(pose.at("q") == nlohmann::json::array({1.0F, 0.1F, 0.2F, 0.3F}));
  REQUIRE(pose.at("g") == nlohmann::json::array({0.4F, 0.5F, 0.6F}));
  REQUIRE(pose.at("p") == nlohmann::json::array({1.1F, 1.2F, 1.3F}));
  REQUIRE(pose.at("v") == nlohmann::json::array({2.1F, 2.2F, 2.3F}));

  h.status.snapshot_value.pose.position_xyz.reset();
  h.status.snapshot_value.pose.velocity_xyz.reset();
  const auto low_only = h.service.handle({"GET", "/status", ""});
  REQUIRE(low_only.body.at("pose").at("q").is_array());
  REQUIRE(low_only.body.at("pose").at("g").is_array());
  REQUIRE(low_only.body.at("pose").at("p").is_null());
  REQUIRE(low_only.body.at("pose").at("v").is_null());
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
                 "progress", "hold", "stop_reason", "err"});
  REQUIRE(response.body.at("active").at("kind") == "user");
  REQUIRE(response.body.at("active").at("id") == "active");
  REQUIRE(response.body.at("exec").at("frame") == 4);
  REQUIRE(response.body.at("exec").at("frames") == 20);
  REQUIRE(response.body.at("exec").at("progress") == 0.25);
  REQUIRE(response.body.at("exec").at("hold") == false);
  REQUIRE(response.body.at("exec").at("duration_s") == 0.38);
  REQUIRE_FALSE(response.body.at("exec").contains("path"));
  REQUIRE(response.body.at("stop_reason").is_null());

  h.status.snapshot_value.ctrl = ControllerState::Stopping;
  h.status.snapshot_value.exec.reset();
  h.status.snapshot_value.stop_reason = StopReason::Stop;

  response = h.service.handle({"GET", "/status", ""});
  REQUIRE(response.body.at("stop_reason") == "stop");
}

TEST_CASE("GET status by id exposes hold metadata for queued running and holding runs") {
  Harness h;
  auto queued = run("queued-hold", MotionState::Queued);
  queued.hold = true;
  auto running = run("running-no-hold", MotionState::Running);
  running.hold = false;
  auto holding = run("holding-hold", MotionState::Holding);
  holding.hold = true;
  holding.progress = 1.0;
  h.status.runs.emplace(queued.id, queued);
  h.status.runs.emplace(running.id, running);
  h.status.runs.emplace(holding.id, holding);

  for (const auto& item :
       std::vector<std::pair<std::string, bool>>{{queued.id, true},
                                                 {running.id, false},
                                                 {holding.id, true}}) {
    const auto response =
        h.service.handle({"GET", std::string("/status?id=") + item.first, ""});

    CAPTURE(item.first);
    REQUIRE(response.status == 200);
    REQUIRE(response.body.at("id") == item.first);
    REQUIRE(response.body.at("hold") == item.second);
  }
}

TEST_CASE("GET status by id covers active queued and recent run states") {
  Harness h;
  h.status.snapshot_value.ready = false;
  h.status.snapshot_value.ctrl = ControllerState::FixStand;
  h.status.snapshot_value.robot = RobotState::Holding;
  h.status.snapshot_value.block = "operator_wait";
  h.status.snapshot_value.err = ErrorCode::RobotNotReady;
  h.status.snapshot_value.queue = {2, 8, {"queued", "later"}};
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
    REQUIRE(response.body.at("robot") == "holding");
    REQUIRE(response.body.at("ctrl") == "fixstand");
    REQUIRE(response.body.at("ready") == false);
    REQUIRE(response.body.at("block") == "operator_wait");
    REQUIRE(response.body.at("top_err") == "ROBOT_NOT_READY");
    if (std::string(id) == "queued") {
      REQUIRE(response.body.at("queue_pos") == 1);
    } else {
      REQUIRE(response.body.at("queue_pos").is_null());
    }
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
