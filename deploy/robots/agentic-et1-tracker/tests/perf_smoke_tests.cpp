#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <httplib.h>

#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/app/app_runner.hpp"
#include "agentic_et1_tracker/http/server.hpp"
#include "agentic_et1_tracker/robot/robot_io.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

namespace agentic_et1_tracker {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint8_t kExpectedModeMachine = 7;
constexpr std::size_t kPolicyJointDim = TrkSchema::kJointDim;

bool enforcePrd() {
  const char* value = std::getenv("AGENTIC_ET1_PERF_ENFORCE_PRD");
  return value != nullptr && std::string(value) == "1";
}

double ms(Clock::duration duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}

double percentile(std::vector<double> values, double quantile) {
  REQUIRE_FALSE(values.empty());
  std::sort(values.begin(), values.end());
  const auto raw_index =
      static_cast<std::size_t>(std::ceil(quantile * static_cast<double>(values.size())));
  const std::size_t index = std::min(values.size() - 1, raw_index == 0 ? 0 : raw_index - 1);
  return values.at(index);
}

std::string uniqueSuffix() {
  static std::atomic<std::uint64_t> sequence{0};
  const auto now = Clock::now().time_since_epoch().count();
  const auto seq = sequence.fetch_add(1, std::memory_order_relaxed);
  return std::to_string(now) + "_" + std::to_string(seq);
}

struct TempTree {
  TempTree() {
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_perf_smoke_tests_" + uniqueSuffix());
    allowed = root / "allowed";
    std::filesystem::create_directories(allowed);
  }

  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path root;
  std::filesystem::path allowed;
};

StatusSnapshot readySnapshot() {
  StatusSnapshot snapshot;
  snapshot.ready = true;
  snapshot.mode = RuntimeMode::Sim;
  snapshot.robot = RobotState::Idle;
  snapshot.ctrl = ControllerState::Idle;
  snapshot.queue.limit = 1000000;
  snapshot.hz = 50.0;
  return snapshot;
}

class PerfSink final : public ExecutionCommandSink {
 public:
  ExecuteResult submitQueue(const ExecuteCommand& command) override {
    const auto q = queued_.fetch_add(1, std::memory_order_relaxed) + 1;
    return {ErrorCode::Ok, command.id, MotionState::Queued, q};
  }

  ExecuteResult submitInterrupt(const ExecuteCommand& command) override {
    const auto q = interrupted_.fetch_add(1, std::memory_order_relaxed) + 1;
    return {ErrorCode::Ok, command.id, MotionState::Queued, q};
  }

  StopResult stop() override {
    return {ErrorCode::Ok, ControllerState::Stopping, StopReason::Stop, 0};
  }

  ControlResult passive() override { return {ErrorCode::Ok}; }

  ControlResult fixStand() override { return {ErrorCode::Ok}; }

  ControlResult standbyVelocity() override { return {ErrorCode::Ok}; }

  ControlResult standby() override { return {ErrorCode::Ok}; }

  StopResult urgentStop() override {
    return {ErrorCode::Ok, ControllerState::UrgentStopping, StopReason::UrgentStop, 0};
  }

  IdleResult configureIdle(std::vector<IdleMotion> motions) override {
    IdleResult result;
    result.idle.enabled = !motions.empty();
    result.idle.n = motions.size();
    return result;
  }

 private:
  std::atomic<std::size_t> queued_{0};
  std::atomic<std::size_t> interrupted_{0};
};

class PerfStatus final : public StatusReader {
 public:
  StatusSnapshot snapshot() const override { return snapshot_; }

  RunLookupResult findRun(const std::string&) const override {
    return {ErrorCode::RunNotFound, std::nullopt};
  }

  HealthSnapshot health() const override { return health_; }

 private:
  StatusSnapshot snapshot_{readySnapshot()};
  HealthSnapshot health_{ServiceHealth::Ready, RuntimeMode::Sim, ErrorCode::Ok, ""};
};

class PerfValidator final : public TrackValidatorPort {
 public:
  TrackValidation validate(const std::string& path) override {
    TrackValidation validation;
    validation.metadata.frames = 120;
    validation.metadata.duration_s = 2.4;
    validation.metadata.fps = 50.0;
    validation.metadata.canonical_path = path;
    return validation;
  }
};

class PerfIds final : public RunIdGenerator {
 public:
  std::string generate() override {
    const auto id = next_.fetch_add(1, std::memory_order_relaxed);
    return "perf" + std::to_string(id);
  }

 private:
  std::atomic<std::uint64_t> next_{0};
};

class RunningHttpPerfHarness {
 public:
  RunningHttpPerfHarness() {
    REQUIRE(server.start());
    REQUIRE(server.boundPort() > 0);
  }

  ~RunningHttpPerfHarness() { server.stop(); }

  int port() const { return server.boundPort(); }

  AgentApiConfig config{RuntimeMode::Sim, 1000000};
  PerfSink sink;
  PerfStatus status;
  PerfValidator validator;
  PerfIds ids;
  AgentApiService service{config, sink, status, validator, ids};
  AgentHttpServer server{{"127.0.0.1", 0, 4}, service};
};

struct HttpRouteResult {
  std::string name;
  std::size_t successes{0};
  double p95_ms{0.0};
};

void requireHttpOk(const httplib::Result& result) {
  REQUIRE(result);
  REQUIRE(result->status == 200);
}

void warmHttpRoute(int port, const std::string& method, const std::string& target) {
  httplib::Client client("127.0.0.1", port);
  for (std::size_t i = 0; i < 100; ++i) {
    if (method == "POST") {
      requireHttpOk(client.Post(target, R"({"path":"/tmp/fake_perf.trk"})",
                                "application/json"));
    } else {
      requireHttpOk(client.Get(target));
    }
  }
}

HttpRouteResult measureGetRoute(int port, std::string target, std::string name) {
  warmHttpRoute(port, "GET", target);

  constexpr std::size_t kThreads = 4;
  constexpr std::size_t kSamplesPerThread = 250;
  std::vector<double> latencies;
  latencies.reserve(kThreads * kSamplesPerThread);
  std::mutex latencies_mutex;
  std::atomic<std::size_t> successes{0};
  std::vector<std::thread> threads;

  for (std::size_t thread_index = 0; thread_index < kThreads; ++thread_index) {
    threads.emplace_back([&, target] {
      httplib::Client client("127.0.0.1", port);
      std::vector<double> local;
      local.reserve(kSamplesPerThread);
      for (std::size_t i = 0; i < kSamplesPerThread; ++i) {
        const auto started = Clock::now();
        const auto result = client.Get(target);
        const auto elapsed = Clock::now() - started;
        if (result && result->status == 200) {
          successes.fetch_add(1, std::memory_order_relaxed);
        }
        local.push_back(ms(elapsed));
      }
      std::lock_guard<std::mutex> lock(latencies_mutex);
      latencies.insert(latencies.end(), local.begin(), local.end());
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  REQUIRE(latencies.size() == kThreads * kSamplesPerThread);
  const double p95 = percentile(latencies, 0.95);
  return {std::move(name), successes.load(std::memory_order_relaxed), p95};
}

HttpRouteResult measureExecuteRoute(int port) {
  warmHttpRoute(port, "POST", "/execute");

  constexpr std::size_t kSamples = 1000;
  httplib::Client client("127.0.0.1", port);
  std::vector<double> latencies;
  latencies.reserve(kSamples);
  std::size_t successes = 0;

  for (std::size_t i = 0; i < kSamples; ++i) {
    const auto started = Clock::now();
    const auto result =
        client.Post("/execute", R"({"path":"/tmp/fake_perf.trk"})", "application/json");
    const auto elapsed = Clock::now() - started;
    if (result && result->status == 200) {
      ++successes;
    }
    latencies.push_back(ms(elapsed));
  }

  const double p95 = percentile(latencies, 0.95);
  return {"/execute", successes, p95};
}

DeployConfig minimalDeployConfig() {
  DeployConfig config;
  config.joint_dim = kPolicyJointDim;
  for (std::size_t i = 0; i < kPolicyJointDim; ++i) {
    config.sdk_joint_ids_map.push_back(static_cast<int>(i));
    config.policy_kp.push_back(20.0);
    config.policy_kd.push_back(0.8);
  }
  return config;
}

LowStateSample readyLowState(const DeployConfig& config) {
  LowStateSample low;
  low.fresh = true;
  low.age_ms = 1;
  low.mode_machine = kExpectedModeMachine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map.at(policy_index));
    low.motors.at(sdk_slot).q = 0.1F * static_cast<float>(policy_index);
  }
  return low;
}

class TimestampRobotIO final : public RobotIO {
 public:
  explicit TimestampRobotIO(LowStateSample state) : low_state_(std::move(state)) {}

  std::optional<LowStateSample> readLowState() const override { return low_state_; }

  std::optional<HighStateSample> readHighState() const override { return std::nullopt; }

  LowCmdOccupancy lowCmdOccupancy() const override { return {}; }

  void writeLowCmd(const LowCmdFrame&) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      timestamps_.push_back(Clock::now());
    }
    cv_.notify_all();
  }

  bool waitForTimestamps(std::size_t count, std::chrono::seconds timeout) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] { return timestamps_.size() >= count; });
  }

  std::vector<Clock::time_point> timestamps() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return timestamps_;
  }

 private:
  LowStateSample low_state_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  std::vector<Clock::time_point> timestamps_;
};

class UnusedPolicy final : public PolicyInference {
 public:
  Vec infer(const PolicyInputs&) override { return Vec(kPolicyJointDim, 0.0F); }
};

PassiveConfig passiveConfig() {
  PassiveConfig config;
  config.mode = std::vector<int>(kFixStandMotorCount, 1);
  config.kd = std::vector<double>(kFixStandMotorCount, 0.5);
  return config;
}

AppRuntimeDeps makeTimestampDeps(TimestampRobotIO*& robot) {
  const DeployConfig deploy_config = minimalDeployConfig();
  auto robot_owner = std::make_unique<TimestampRobotIO>(readyLowState(deploy_config));
  robot = robot_owner.get();

  AppRuntimeDeps deps;
  deps.robot_io = std::move(robot_owner);
  deps.policy = std::make_unique<UnusedPolicy>();
  deps.deploy_config = deploy_config;
  deps.passive_config = passiveConfig();
  deps.mode = RuntimeMode::Sim;
  return deps;
}

AppConfig perfAppConfig(const TempTree& tmp) {
  AppConfig config;
  config.http.host = "127.0.0.1";
  config.http.port = 0;
  config.runtime.hz = 50.0;
  config.runtime.queue_limit = 8;
  config.trk.allowlist_dirs = {tmp.allowed};
  config.mode_machine = kExpectedModeMachine;
  config.lock_path = (tmp.root / ("tracker_" + uniqueSuffix() + ".lock")).string();
  return config;
}

}  // namespace

TEST_CASE("HTTP API p95 smoke stays finite on fake core service", "[perf]") {
  RunningHttpPerfHarness harness;

  const auto health = measureGetRoute(harness.port(), "/health", "/health");
  const auto status = measureGetRoute(harness.port(), "/status", "/status");
  const auto execute = measureExecuteRoute(harness.port());

  for (const auto& result : {health, status, execute}) {
    std::cout << "perf_smoke http " << result.name << " p95_ms=" << result.p95_ms
              << " samples=" << result.successes << '\n';
    REQUIRE(result.successes == 1000);
    REQUIRE(std::isfinite(result.p95_ms));
    REQUIRE(result.p95_ms >= 0.0);
    REQUIRE(result.p95_ms < 1000.0);
  }

  if (enforcePrd()) {
    REQUIRE(health.p95_ms < 2.0);
    REQUIRE(status.p95_ms < 5.0);
    REQUIRE(execute.p95_ms < 20.0);
  }
}

TEST_CASE("50Hz idle hold loop jitter smoke stays finite with fake RobotIO", "[perf]") {
  TempTree tmp;
  TimestampRobotIO* robot = nullptr;
  AppRunner runner(perfAppConfig(tmp), makeTimestampDeps(robot));

  REQUIRE(runner.start());
  const int port = runner.boundPort();
  REQUIRE(port > 0);

  std::atomic<bool> stop_polling{false};
  std::atomic<std::size_t> health_successes{0};
  std::atomic<std::size_t> status_successes{0};
  constexpr std::size_t kHttpPollingThreads = 4;
  std::vector<std::thread> http_threads;
  http_threads.reserve(kHttpPollingThreads);
  for (std::size_t thread_index = 0; thread_index < kHttpPollingThreads; ++thread_index) {
    http_threads.emplace_back([&, port, thread_index] {
      httplib::Client client("127.0.0.1", port);
      std::size_t request_index = thread_index;
      while (!stop_polling.load(std::memory_order_relaxed)) {
        const bool poll_health = request_index % 2 == 0;
        const auto result = client.Get(poll_health ? "/health" : "/status");
        if (result && result->status == 200) {
          if (poll_health) {
            health_successes.fetch_add(1, std::memory_order_relaxed);
          } else {
            status_successes.fetch_add(1, std::memory_order_relaxed);
          }
        }
        ++request_index;
      }
    });
  }

  constexpr std::size_t kDiscardTicks = 50;
  constexpr std::size_t kRequiredPeriods = 300;
  const std::size_t required_timestamps = kDiscardTicks + kRequiredPeriods + 1;
  const bool sampled = robot->waitForTimestamps(required_timestamps, std::chrono::seconds(10));
  stop_polling.store(true, std::memory_order_relaxed);
  for (auto& thread : http_threads) {
    thread.join();
  }
  runner.stop();

  REQUIRE(sampled);
  REQUIRE(health_successes.load(std::memory_order_relaxed) > 0);
  REQUIRE(status_successes.load(std::memory_order_relaxed) > 0);
  const auto timestamps = robot->timestamps();
  REQUIRE(timestamps.size() >= required_timestamps);

  std::vector<double> periods_ms;
  periods_ms.reserve(kRequiredPeriods);
  for (std::size_t i = kDiscardTicks + 1; i <= kDiscardTicks + kRequiredPeriods; ++i) {
    periods_ms.push_back(ms(timestamps.at(i) - timestamps.at(i - 1)));
  }
  REQUIRE(periods_ms.size() == kRequiredPeriods);

  const double avg_period_ms =
      std::accumulate(periods_ms.begin(), periods_ms.end(), 0.0) /
      static_cast<double>(periods_ms.size());
  const double avg_hz = 1000.0 / avg_period_ms;

  std::vector<double> jitter_ms;
  jitter_ms.reserve(periods_ms.size());
  for (const double period_ms : periods_ms) {
    jitter_ms.push_back(std::abs(period_ms - 20.0));
  }
  const double jitter_p95_ms = percentile(jitter_ms, 0.95);

  std::cout << "perf_smoke runtime avg_hz=" << avg_hz
            << " jitter_p95_ms=" << jitter_p95_ms
            << " periods=" << periods_ms.size()
            << " http_health_successes=" << health_successes.load(std::memory_order_relaxed)
            << " http_status_successes=" << status_successes.load(std::memory_order_relaxed)
            << '\n';

  REQUIRE(std::isfinite(avg_hz));
  REQUIRE(avg_hz > 40.0);
  REQUIRE(avg_hz < 60.0);
  REQUIRE(std::isfinite(jitter_p95_ms));
  REQUIRE(jitter_p95_ms >= 0.0);

  if (enforcePrd()) {
    REQUIRE(avg_hz >= 49.5);
    REQUIRE(avg_hz <= 50.5);
    REQUIRE(jitter_p95_ms < 5.0);
  }
}

}  // namespace agentic_et1_tracker
