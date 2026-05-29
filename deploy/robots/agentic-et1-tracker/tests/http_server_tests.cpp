#include <catch2/catch_test_macros.hpp>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/http/server.hpp"

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

class FakeSink final : public ExecutionCommandSink {
 public:
  ExecuteResult submitQueue(const ExecuteCommand& command) override {
    ++queue_calls;
    queue_commands.push_back(command);
    auto out = queue_result;
    out.id = command.id;
    return out;
  }

  ExecuteResult submitInterrupt(const ExecuteCommand& command) override {
    ++interrupt_calls;
    interrupt_commands.push_back(command);
    auto out = interrupt_result;
    out.id = command.id;
    return out;
  }

  StopResult stop() override {
    ++stop_calls;
    return stop_result;
  }

  ExecuteResult queue_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  ExecuteResult interrupt_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  StopResult stop_result{ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 0};
  int queue_calls{0};
  int interrupt_calls{0};
  int stop_calls{0};
  std::vector<ExecuteCommand> queue_commands;
  std::vector<ExecuteCommand> interrupt_commands;
};

class FakeStatus final : public StatusReader {
 public:
  StatusSnapshot snapshot() const override { return snapshot_value; }

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

  TrackValidation result{ErrorCode::Ok, TrackMetadata{12, 0.22}, ""};
  int calls{0};
  std::vector<std::string> paths;
};

class FakeIds final : public RunIdGenerator {
 public:
  std::string generate() override {
    ++calls;
    return "a7K3p9Qx";
  }

  int calls{0};
};

struct Harness {
  AgentApiConfig config{RuntimeMode::Sim, 8};
  FakeSink sink;
  FakeStatus status;
  FakeValidator validator;
  FakeIds ids;
  AgentApiService service{config, sink, status, validator, ids};
  AgentHttpServer server{{"127.0.0.1", 0}, service};
};

class RunningHarness {
 public:
  RunningHarness() { REQUIRE(h.server.start()); }
  ~RunningHarness() { h.server.stop(); }

  nlohmann::json parse(const httplib::Result& result) const {
    REQUIRE(result);
    return nlohmann::json::parse(result->body);
  }

  void requireJson(const httplib::Result& result, int status) const {
    REQUIRE(result);
    REQUIRE(result->status == status);
    REQUIRE(result->get_header_value("Content-Type") == "application/json");
  }

  Harness h;
};

TEST_CASE("HTTP config defaults and clamps bounded worker thread pool size") {
  REQUIRE(normalizeHttpServerConfig({}).thread_pool_size == 4);

  HttpServerConfig low;
  low.thread_pool_size = 1;
  REQUIRE(normalizeHttpServerConfig(low).thread_pool_size == 2);

  HttpServerConfig min;
  min.thread_pool_size = 2;
  REQUIRE(normalizeHttpServerConfig(min).thread_pool_size == 2);

  HttpServerConfig mid;
  mid.thread_pool_size = 3;
  REQUIRE(normalizeHttpServerConfig(mid).thread_pool_size == 3);

  HttpServerConfig max;
  max.thread_pool_size = 4;
  REQUIRE(normalizeHttpServerConfig(max).thread_pool_size == 4);

  HttpServerConfig high;
  high.thread_pool_size = 99;
  REQUIRE(normalizeHttpServerConfig(high).thread_pool_size == 4);
}

TEST_CASE("HTTP server lifecycle binds an ephemeral loopback port and stops") {
  Harness h;

  REQUIRE(h.server.start());
  REQUIRE(h.server.isRunning());
  REQUIRE(h.server.boundPort() > 0);

  h.server.stop();
  REQUIRE_FALSE(h.server.isRunning());
  REQUIRE(h.server.boundPort() == 0);
}

TEST_CASE("GET /health returns JSON health fields") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto result = client.Get("/health");

  r.requireJson(result, 200);
  const auto body = r.parse(result);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("state") == "ready");
  REQUIRE(body.at("mode") == "sim");
}

TEST_CASE("GET /status forwards to the API service") {
  RunningHarness r;
  r.h.status.snapshot_value.queue = {2, 8, {"a", "b"}};
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto result = client.Get("/status");

  r.requireJson(result, 200);
  const auto body = r.parse(result);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("mode") == "sim");
  REQUIRE(body.at("queue").at("n") == 2);
  REQUIRE(body.at("queue").at("ids").at(1) == "b");
}

TEST_CASE("GET /status?id preserves query and returns JSON run errors") {
  RunningHarness r;
  r.h.status.runs.emplace("done", run("done", MotionState::Done));
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  auto result = client.Get("/status?id=done");
  r.requireJson(result, 200);
  auto body = r.parse(result);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("id") == "done");
  REQUIRE(r.h.status.find_ids.back() == "done");

  result = client.Get("/status?id=missing");
  r.requireJson(result, 404);
  body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "RUN_NOT_FOUND");
  REQUIRE(r.h.status.find_ids.back() == "missing");
}

TEST_CASE("GET /status with explicit empty id returns request invalid") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  for (const auto& target : {"/status?id=", "/status?x=1&id="}) {
    const auto result = client.Get(target);

    CAPTURE(target);
    r.requireJson(result, 400);
    const auto body = r.parse(result);
    REQUIRE(body.at("ok") == false);
    REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");
    REQUIRE(r.h.status.find_ids.empty());
  }
}

TEST_CASE("POST /execute queue passes raw body through to service") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto result =
      client.Post("/execute", R"({"path":"/tracks/wave.trk"})", "application/json");

  r.requireJson(result, 200);
  const auto body = r.parse(result);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("id") == "a7K3p9Qx");
  REQUIRE(body.at("state") == "queued");
  REQUIRE(body.at("q") == 1);
  REQUIRE(r.h.validator.calls == 1);
  REQUIRE(r.h.validator.paths.at(0) == "/tracks/wave.trk");
  REQUIRE(r.h.sink.queue_calls == 1);
  REQUIRE(r.h.sink.interrupt_calls == 0);
  REQUIRE(r.h.sink.queue_commands.at(0).path == "/tracks/wave.trk");
}

TEST_CASE("POST /execute interrupt is interpreted only by the API service") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto result = client.Post(
      "/execute", R"({"path":"/tracks/urgent.trk","mode":"interrupt"})", "application/json");

  r.requireJson(result, 200);
  const auto body = r.parse(result);
  REQUIRE(body.at("ok") == true);
  REQUIRE(r.h.sink.queue_calls == 0);
  REQUIRE(r.h.sink.interrupt_calls == 1);
  REQUIRE(r.h.sink.interrupt_commands.at(0).mode == MotionMode::Interrupt);
}

TEST_CASE("POST /execute malformed JSON returns request invalid before ports") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto result = client.Post("/execute", "not-json", "application/json");

  r.requireJson(result, 400);
  const auto body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");
  REQUIRE(r.h.validator.calls == 0);
  REQUIRE(r.h.sink.queue_calls == 0);
  REQUIRE(r.h.sink.interrupt_calls == 0);
  REQUIRE(r.h.ids.calls == 0);
}

TEST_CASE("POST /stop empty body stops and non-empty body is rejected") {
  RunningHarness r;
  r.h.sink.stop_result = {ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 3};
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  auto result = client.Post("/stop", "", "application/json");
  r.requireJson(result, 200);
  auto body = r.parse(result);
  REQUIRE(body.size() == 2);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("state") == "accepted");
  REQUIRE_FALSE(body.contains("cleared"));
  REQUIRE_FALSE(body.contains("stop_reason"));
  REQUIRE(r.h.sink.stop_calls == 1);

  result = client.Post("/stop", R"({})", "application/json");
  r.requireJson(result, 400);
  body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");
  REQUIRE(r.h.sink.stop_calls == 1);
}

TEST_CASE("invalid method or unknown route returns JSON request invalid") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  auto result = client.Get("/execute");
  r.requireJson(result, 400);
  auto body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");

  result = client.Get("/missing");
  r.requireJson(result, 400);
  body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");
}

}  // namespace
}  // namespace agentic_et1_tracker
