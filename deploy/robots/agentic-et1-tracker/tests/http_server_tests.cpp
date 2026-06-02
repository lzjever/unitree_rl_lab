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
#include "agentic_et1_tracker/reference/reference_frame_store.hpp"

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

  ControlResult fixStand() override {
    ++fixstand_calls;
    return fixstand_result;
  }

  ControlResult standbyVelocity() override {
    ++standby_velocity_calls;
    return standby_velocity_result;
  }

  ExecuteResult queue_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  ExecuteResult interrupt_result{ErrorCode::Ok, "", MotionState::Queued, 1};
  StopResult stop_result{ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 0};
  ControlResult fixstand_result{ErrorCode::Ok};
  ControlResult standby_velocity_result{ErrorCode::Ok};
  int queue_calls{0};
  int interrupt_calls{0};
  int stop_calls{0};
  int fixstand_calls{0};
  int standby_velocity_calls{0};
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

TEST_CASE("GET /_sim/reference_frame disabled returns 404 without affecting routes") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto hidden = client.Get("/_sim/reference_frame");
  r.requireJson(hidden, 404);
  REQUIRE(r.parse(hidden).at("ok") == false);

  const auto status = client.Get("/status");
  r.requireJson(status, 200);
  REQUIRE(r.parse(status).at("ok") == true);
}

TEST_CASE("GET /_sim/reference_frame enabled returns inactive and active snapshots") {
  Harness h;
  ReferenceFrameStore reference_store;
  AgentHttpServer server({"127.0.0.1", 0}, h.service, &reference_store);
  REQUIRE(server.start());
  httplib::Client client("127.0.0.1", server.boundPort());

  auto result = client.Get("/_sim/reference_frame");
  REQUIRE(result);
  REQUIRE(result->status == 200);
  REQUIRE(result->get_header_value("Content-Type") == "application/json");
  auto body = nlohmann::json::parse(result->body);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("active") == false);

  ReferenceFrameSnapshot snapshot;
  snapshot.active = true;
  snapshot.id = "http-ref";
  snapshot.frame = 2;
  snapshot.frames = 5;
  snapshot.time_s = 0.04;
  snapshot.fps = 50.0;
  snapshot.p.at(0) = {1.0F, 2.0F, 3.0F};
  snapshot.q.at(0) = {1.0F, 0.0F, 0.0F, 0.0F};
  snapshot.c = {1, 0};
  snapshot.com = {4.0F, 5.0F, 6.0F};
  snapshot.comv = {7.0F, 8.0F, 9.0F};
  reference_store.publish(snapshot);

  result = client.Get("/_sim/reference_frame");
  REQUIRE(result);
  REQUIRE(result->status == 200);
  body = nlohmann::json::parse(result->body);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("active") == true);
  REQUIRE(body.at("schema") == "ET1REF1");
  REQUIRE(body.at("body_order") == "et1_27_v1");
  REQUIRE(body.at("id") == "http-ref");
  REQUIRE(body.at("frame") == 2);
  REQUIRE(body.at("p").at(0).at(1) == 2.0F);
  REQUIRE(body.at("q").at(0).at(0) == 1.0F);
  REQUIRE(body.at("c").at(0) == 1);
  REQUIRE(body.at("comv").at(2) == 9.0F);

  server.stop();
}

TEST_CASE("ReferenceFrameStore publishes snapshots and clear resets to inactive") {
  ReferenceFrameStore reference_store;

  REQUIRE_FALSE(reference_store.snapshot().active);

  ReferenceFrameSnapshot first;
  first.active = true;
  first.id = "first";
  first.frame = 1;
  first.frames = 3;
  first.p.at(0) = {1.0F, 2.0F, 3.0F};
  reference_store.publish(first);

  auto snapshot = reference_store.snapshot();
  REQUIRE(snapshot.active);
  REQUIRE(snapshot.id == "first");
  REQUIRE(snapshot.frame == 1);
  REQUIRE(snapshot.p.at(0).at(2) == 3.0F);

  ReferenceFrameSnapshot second;
  second.active = true;
  second.id = "second";
  second.frame = 2;
  second.frames = 4;
  second.comv = {4.0F, 5.0F, 6.0F};
  reference_store.publish(second);

  snapshot = reference_store.snapshot();
  REQUIRE(snapshot.active);
  REQUIRE(snapshot.id == "second");
  REQUIRE(snapshot.frame == 2);
  REQUIRE(snapshot.comv.at(2) == 6.0F);

  reference_store.clear();
  snapshot = reference_store.snapshot();
  REQUIRE_FALSE(snapshot.active);
  REQUIRE(snapshot.id.empty());
}

TEST_CASE("GET /status?id preserves query and returns JSON run errors") {
  RunningHarness r;
  r.h.status.snapshot_value.ready = false;
  r.h.status.snapshot_value.ctrl = ControllerState::FixStand;
  r.h.status.snapshot_value.robot = RobotState::Holding;
  r.h.status.snapshot_value.block = "operator_wait";
  r.h.status.snapshot_value.err = ErrorCode::RobotNotReady;
  r.h.status.snapshot_value.queue = {1, 8, {"done"}};
  r.h.status.runs.emplace("done", run("done", MotionState::Done));
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  auto result = client.Get("/status?id=done");
  r.requireJson(result, 200);
  auto body = r.parse(result);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("id") == "done");
  REQUIRE(body.at("ctrl") == "fixstand");
  REQUIRE(body.at("ready") == false);
  REQUIRE(body.at("block") == "operator_wait");
  REQUIRE(body.at("top_err") == "ROBOT_NOT_READY");
  REQUIRE(body.at("queue_pos") == 1);
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

TEST_CASE("POST control routes are installed and reject non-empty bodies") {
  RunningHarness r;
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  auto result = client.Post("/fixstand", "", "application/json");
  r.requireJson(result, 200);
  auto body = r.parse(result);
  REQUIRE(body.size() == 2);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("state") == "accepted");
  REQUIRE(r.h.sink.fixstand_calls == 1);
  REQUIRE(r.h.validator.calls == 0);
  REQUIRE(r.h.ids.calls == 0);

  result = client.Post("/standby_velocity", "", "application/json");
  r.requireJson(result, 200);
  body = r.parse(result);
  REQUIRE(body.size() == 2);
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("state") == "accepted");
  REQUIRE(r.h.sink.standby_velocity_calls == 1);
  REQUIRE(r.h.validator.calls == 0);
  REQUIRE(r.h.ids.calls == 0);

  result = client.Post("/fixstand", R"({})", "application/json");
  r.requireJson(result, 400);
  body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");
  REQUIRE(r.h.sink.fixstand_calls == 1);

  result = client.Post("/standby_velocity", R"({})", "application/json");
  r.requireJson(result, 400);
  body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "REQUEST_INVALID");
  REQUIRE(r.h.sink.standby_velocity_calls == 1);
}

TEST_CASE("POST standby_velocity conflict returns compact next action") {
  RunningHarness r;
  r.h.sink.standby_velocity_result = {ErrorCode::ControlStateConflict};
  httplib::Client client("127.0.0.1", r.h.server.boundPort());

  const auto result = client.Post("/standby_velocity", "", "application/json");

  r.requireJson(result, 409);
  const auto body = r.parse(result);
  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "CONTROL_STATE_CONFLICT");
  REQUIRE(body.at("error").at("message") == "wrong ctrl; check /status");
  REQUIRE(body.at("error").at("retryable") == false);
  REQUIRE(body.at("next") == "status");
  REQUIRE(r.h.sink.standby_velocity_calls == 1);
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
