#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <netinet/in.h>
#include <sys/socket.h>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "agentic_et1_tracker/app/app_runtime_factory.hpp"
#include "agentic_et1_tracker/app/app_runner.hpp"
#include "agentic_et1_tracker/core/types.hpp"
#include "agentic_et1_tracker/trk/schema.hpp"

#ifndef AGENTIC_ET1_TRACKER_REAL_FACTORY
#define AGENTIC_ET1_TRACKER_REAL_FACTORY 0
#endif

namespace agentic_et1_tracker {
namespace {

constexpr std::uint8_t kExpectedModeMachine = 7;
constexpr std::size_t kPolicyJointDim = TrkSchema::kJointDim;
constexpr const char* kDefaultFactoryMode =
    AGENTIC_ET1_TRACKER_REAL_FACTORY ? "real" : "sim";

std::string uniqueTestSuffix() {
  static std::atomic<std::uint64_t> sequence{0};
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto pid = static_cast<long long>(::getpid());
  const auto seq = sequence.fetch_add(1, std::memory_order_relaxed);
  return std::to_string(now) + "_" + std::to_string(pid) + "_" + std::to_string(seq);
}

std::filesystem::path uniqueTestLockPath() {
  return std::filesystem::temp_directory_path() /
         ("agentic_et1_tracker_app_runner_tests_" + uniqueTestSuffix() + ".lock");
}

struct TempTree {
  TempTree() {
    root = std::filesystem::temp_directory_path() /
           ("agentic_et1_tracker_app_runner_tests_" + uniqueTestSuffix());
    allowed = root / "allowed";
    outside = root / "outside";
    std::filesystem::create_directories(allowed);
    std::filesystem::create_directories(outside);
  }

  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
  }

  std::filesystem::path root;
  std::filesystem::path allowed;
  std::filesystem::path outside;
};

class TcpPortLease {
 public:
  TcpPortLease() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd_ >= 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    REQUIRE(::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    REQUIRE(::listen(fd_, 1) == 0);

    socklen_t len = sizeof(addr);
    REQUIRE(::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    port_ = ntohs(addr.sin_port);
  }

  ~TcpPortLease() { close(); }

  TcpPortLease(const TcpPortLease&) = delete;
  TcpPortLease& operator=(const TcpPortLease&) = delete;

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int port() const { return port_; }

 private:
  int fd_{-1};
  int port_{0};
};

AppConfig productionTestConfig() {
  AppConfig config;
  config.http.host = "127.0.0.1";
  config.http.port = 0;
  config.runtime.hz = 200.0;
  config.trk.allowlist_dirs = {"/tmp/agentic-et1-tracker-test-motions"};
  config.lock_path = uniqueTestLockPath().string();
  return config;
}

AppConfig productionTestConfig(const TempTree& tmp) {
  AppConfig config = productionTestConfig();
  config.trk.allowlist_dirs = {tmp.allowed};
  config.mode_machine = kExpectedModeMachine;
  config.lock_path = (tmp.root / ("tracker_" + uniqueTestSuffix() + ".lock")).string();
  return config;
}

AppConfig productionTestConfig(const TempTree& tmp, const std::filesystem::path& lock_path) {
  AppConfig config = productionTestConfig(tmp);
  config.lock_path = lock_path.string();
  return config;
}

template <typename T>
void writeScalar(std::ofstream& out, T value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

std::uint64_t elementCount(const std::vector<std::uint64_t>& shape) {
  std::uint64_t elements = 1;
  for (const auto dim : shape) {
    elements *= dim;
  }
  return elements;
}

std::vector<std::uint64_t> shapeFor(const TrkRequiredArraySpec& spec,
                                    std::uint64_t frames) {
  std::vector<std::uint64_t> shape{frames};
  for (std::uint32_t i = 0; i < spec.trailing_rank; ++i) {
    shape.push_back(spec.trailing_shape[i]);
  }
  return shape;
}

struct ArrayFixture {
  std::string name;
  TrkDtype dtype{TrkDtype::Float32};
  std::vector<std::uint64_t> shape;
};

void writePayload(std::ofstream& out, const ArrayFixture& array) {
  const auto elements = elementCount(array.shape);
  if (array.dtype == TrkDtype::Int64) {
    for (std::size_t i = 0; i < elements; ++i) {
      writeScalar(out, static_cast<std::int64_t>(i % 3));
    }
    return;
  }

  for (std::size_t i = 0; i < elements; ++i) {
    writeScalar(out, static_cast<float>(i) * 0.25F);
  }
}

std::vector<ArrayFixture> requiredArrays(std::uint64_t frames) {
  std::vector<ArrayFixture> arrays;
  for (const auto& spec : TrkSchema::kRequiredArrays) {
    arrays.push_back({std::string(spec.name),
                      spec.dtype_family == TrkDtypeFamily::Contact ? TrkDtype::Int64
                                                                    : TrkDtype::Float32,
                      shapeFor(spec, frames)});
  }
  return arrays;
}

void writeTrk(const std::filesystem::path& path, const std::vector<ArrayFixture>& arrays) {
  std::ofstream out(path, std::ios::binary);
  REQUIRE(out);

  out.write(TrkSchema::kMagic.data(), static_cast<std::streamsize>(TrkSchema::kMagic.size()));
  writeScalar(out, TrkSchema::kVersion);
  writeScalar(out, static_cast<std::uint32_t>(arrays.size()));

  for (const auto& array : arrays) {
    writeScalar(out, static_cast<std::uint32_t>(array.name.size()));
    out.write(array.name.data(), static_cast<std::streamsize>(array.name.size()));
    writeScalar(out, static_cast<std::uint32_t>(array.dtype));
    writeScalar(out, static_cast<std::uint32_t>(array.shape.size()));
    for (const auto dim : array.shape) {
      REQUIRE(dim <= std::numeric_limits<std::uint32_t>::max());
      writeScalar(out, static_cast<std::uint32_t>(dim));
    }
    writeScalar(out, elementCount(array.shape) * trkDtypeSize(array.dtype));
    writePayload(out, array);
  }
}

std::filesystem::path validTrk(const std::filesystem::path& dir,
                               const std::string& name,
                               std::uint64_t frames) {
  const auto path = dir / name;
  writeTrk(path, requiredArrays(frames));
  return path;
}

std::vector<float> floatSeq(float start, std::size_t count) {
  std::vector<float> values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(start + static_cast<float>(i));
  }
  return values;
}

std::vector<double> doubleSeq(double start, double step, std::size_t count) {
  std::vector<double> values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    values.push_back(start + step * static_cast<double>(i));
  }
  return values;
}

std::vector<int> frozenSdkMap() {
  return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
          13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30};
}

std::vector<ObservationTerm> observationTerms(
    std::initializer_list<std::pair<const char*, std::size_t>> specs) {
  std::vector<ObservationTerm> out;
  std::size_t offset = 0;
  for (const auto& spec : specs) {
    out.push_back({spec.first, spec.second, offset});
    offset += spec.second;
  }
  return out;
}

DeployConfig deployConfig() {
  DeployConfig config;
  config.joint_dim = kPolicyJointDim;
  config.sdk_joint_ids_map = frozenSdkMap();
  config.default_joint_pos = doubleSeq(0.25, 0.5, kPolicyJointDim);
  config.policy_kp = doubleSeq(10.0, 1.0, kPolicyJointDim);
  config.policy_kd = doubleSeq(0.5, 0.05, kPolicyJointDim);
  config.action_scale = doubleSeq(0.25, 0.125, kPolicyJointDim);
  config.action_offset = doubleSeq(-1.0, 0.2, kPolicyJointDim);
  config.obs_current_terms = observationTerms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kPolicyJointDim},
      {"joint_vel_rel", kPolicyJointDim},
      {"last_action", kPolicyJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_history_terms = observationTerms({
      {"command_root_ori_b", 6},
      {"command_xy_yaw_vel", 3},
      {"command_jnt_pos", kPolicyJointDim},
      {"projected_gravity", 3},
      {"base_ang_vel", 3},
      {"joint_pos_rel", kPolicyJointDim},
      {"joint_vel_rel", kPolicyJointDim},
      {"command_foot_support_state", 6},
      {"ref_com_rel_navi", 3},
      {"ref_com_vel_navi", 3},
  });
  config.obs_current_dim = 131;
  config.obs_history_width = 105;
  config.obs_history_length = 25;
  return config;
}

LowStateSample readyLowState(const DeployConfig& config,
                             std::uint8_t mode_machine = kExpectedModeMachine) {
  LowStateSample low;
  low.fresh = true;
  low.age_ms = 4;
  low.mode_machine = mode_machine;
  low.quat_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  low.gyro = {0.3F, -0.4F, 0.5F};

  for (std::size_t policy_index = 0; policy_index < config.sdk_joint_ids_map.size();
       ++policy_index) {
    const auto sdk_slot = static_cast<std::size_t>(config.sdk_joint_ids_map[policy_index]);
    low.motors.at(sdk_slot).q = 10.0F + static_cast<float>(policy_index);
    low.motors.at(sdk_slot).dq = 20.0F + static_cast<float>(policy_index);
  }
  return low;
}

class FakeRobotIO final : public RobotIO {
 public:
  explicit FakeRobotIO(LowStateSample state,
                       std::shared_ptr<std::atomic<int>> destroy_counter = nullptr)
      : low_state(std::move(state)), destroy_counter_(std::move(destroy_counter)) {}

  ~FakeRobotIO() override {
    if (destroy_counter_) {
      ++(*destroy_counter_);
    }
  }

  std::optional<LowStateSample> readLowState() const override {
    ++read_low_calls;
    return low_state;
  }

  std::optional<HighStateSample> readHighState() const override { return std::nullopt; }

  LowCmdOccupancy lowCmdOccupancy() const override { return {}; }

  void writeLowCmd(const LowCmdFrame& frame) override {
    ++write_low_cmd_calls;
    last_mode_machine.store(frame.mode_machine);
  }

  std::optional<LowStateSample> low_state;
  mutable std::atomic<int> read_low_calls{0};
  std::atomic<int> write_low_cmd_calls{0};
  std::atomic<std::uint8_t> last_mode_machine{0};

 private:
  std::shared_ptr<std::atomic<int>> destroy_counter_;
};

class FakePolicy final : public PolicyInference {
 public:
  explicit FakePolicy(std::shared_ptr<std::atomic<int>> destroy_counter = nullptr)
      : destroy_counter_(std::move(destroy_counter)) {}

  ~FakePolicy() override {
    if (destroy_counter_) {
      ++(*destroy_counter_);
    }
  }

  Vec infer(const PolicyInputs&) override {
    ++calls;
    return floatSeq(0.25F, kPolicyJointDim);
  }

  std::atomic<int> calls{0};

 private:
  std::shared_ptr<std::atomic<int>> destroy_counter_;
};

AppRuntimeDeps makeDeps(FakeRobotIO*& robot,
                        FakePolicy*& policy,
                        std::uint8_t mode_machine = kExpectedModeMachine,
                        RuntimeMode mode = RuntimeMode::Sim,
                        std::shared_ptr<std::atomic<int>> robot_destroy_counter = nullptr,
                        std::shared_ptr<std::atomic<int>> policy_destroy_counter = nullptr) {
  const DeployConfig config = deployConfig();
  auto robot_owner =
      std::make_unique<FakeRobotIO>(readyLowState(config, mode_machine),
                                    std::move(robot_destroy_counter));
  auto policy_owner = std::make_unique<FakePolicy>(std::move(policy_destroy_counter));
  robot = robot_owner.get();
  policy = policy_owner.get();

  AppRuntimeDeps deps;
  deps.robot_io = std::move(robot_owner);
  deps.policy = std::move(policy_owner);
  deps.deploy_config = config;
  deps.mode = mode;
  return deps;
}

class RunningApp {
 public:
  RunningApp() : runner(productionTestConfig()) { REQUIRE(runner.start()); }
  ~RunningApp() { runner.stop(); }

  httplib::Client client() const { return httplib::Client("127.0.0.1", runner.boundPort()); }

  static nlohmann::json body(const httplib::Result& result, int status) {
    REQUIRE(result);
    REQUIRE(result->status == status);
    REQUIRE(result->get_header_value("Content-Type") == "application/json");
    return nlohmann::json::parse(result->body);
  }

  AppRunner runner;
};

nlohmann::json body(const httplib::Result& result, int status) {
  return RunningApp::body(result, status);
}

template <typename Predicate>
nlohmann::json pollJson(httplib::Client& client, const std::string& target, Predicate done) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  nlohmann::json latest;
  while (std::chrono::steady_clock::now() < deadline) {
    latest = body(client.Get(target.c_str()), 200);
    if (done(latest)) {
      return latest;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL("timed out polling " << target << " latest=" << latest.dump());
  return latest;
}

}  // namespace

TEST_CASE("Default AppRunner starts HTTP as not-ready until policy runtime is attached") {
  RunningApp app;
  auto client = app.client();

  const auto health = RunningApp::body(client.Get("/health"), 200);
  REQUIRE(health.at("ok") == false);
  REQUIRE(health.at("state") == "starting");
  REQUIRE(health.at("mode") == kDefaultFactoryMode);

  const auto status = RunningApp::body(client.Get("/status"), 200);
  REQUIRE(status.at("ok") == true);
  REQUIRE(status.at("ready") == false);
  REQUIRE(status.at("mode") == kDefaultFactoryMode);
  REQUIRE(status.at("ctrl") == "starting");
  REQUIRE(status.at("block") == "policy_not_loaded");
  REQUIRE(status.at("err").at("code") == "MODEL_NOT_READY");
}

TEST_CASE("Default AppRunner rejects execute before TRK validation when not ready") {
  RunningApp app;
  auto client = app.client();

  const auto result =
      client.Post("/execute", R"({"path":"relative-or-invalid.trk"})", "application/json");
  const auto body = RunningApp::body(result, 503);

  REQUIRE(body.at("ok") == false);
  REQUIRE(body.at("error").at("code") == "MODEL_NOT_READY");
  REQUIRE(body.at("next") == "status");
}

TEST_CASE("AppRunner rejects concurrent start with same process lock and releases on stop") {
  TempTree tmp;
  const auto lock_path = tmp.root / "tracker.lock";

  AppRunner first(productionTestConfig(tmp, lock_path));
  REQUIRE(first.start());

  AppRunner second(productionTestConfig(tmp, lock_path));
  REQUIRE_FALSE(second.start());

  first.stop();
  REQUIRE(second.start());

  second.stop();
}

TEST_CASE("Default AppRunner creates runtime deps only after acquiring process lock") {
  TempTree tmp;
  const auto lock_path = tmp.root / "lazy_factory_tracker.lock";
  std::atomic<int> factory_calls{0};

  auto factory = [&factory_calls](const AppConfig& config) {
    ++factory_calls;
    AppRuntimeFactoryResult result;
    result.snapshot.ready = false;
    result.snapshot.mode = config.mode_machine == 0 ? RuntimeMode::Sim : RuntimeMode::Real;
    result.snapshot.robot = RobotState::NotReady;
    result.snapshot.ctrl = ControllerState::Starting;
    result.snapshot.hz = config.runtime.hz;
    result.snapshot.queue.limit = config.runtime.queue_limit;
    result.snapshot.block = "policy_not_loaded";
    result.snapshot.err = ErrorCode::ModelNotReady;
    result.health = {ServiceHealth::Starting,
                     result.snapshot.mode,
                     ErrorCode::ModelNotReady,
                     "policy_not_loaded"};
    return result;
  };

  AppRunner first(productionTestConfig(tmp, lock_path), factory);
  REQUIRE(factory_calls.load() == 0);
  REQUIRE(first.start());
  REQUIRE(factory_calls.load() == 1);

  AppRunner second(productionTestConfig(tmp, lock_path), factory);
  REQUIRE(factory_calls.load() == 1);
  REQUIRE_FALSE(second.start());
  REQUIRE(factory_calls.load() == 1);

  first.stop();
  REQUIRE(second.start());
  REQUIRE(factory_calls.load() == 2);

  second.stop();
}

TEST_CASE("Default AppRunner releases process lock when runtime factory fails") {
  TempTree tmp;
  std::atomic<int> factory_calls{0};

  auto factory = [&factory_calls](const AppConfig& config) {
    const int call = ++factory_calls;
    if (call == 1) {
      throw std::runtime_error("factory failed");
    }

    AppRuntimeFactoryResult result;
    result.snapshot.ready = false;
    result.snapshot.mode = config.mode_machine == 0 ? RuntimeMode::Sim : RuntimeMode::Real;
    result.snapshot.robot = RobotState::NotReady;
    result.snapshot.ctrl = ControllerState::Starting;
    result.snapshot.hz = config.runtime.hz;
    result.snapshot.queue.limit = config.runtime.queue_limit;
    result.snapshot.block = "policy_not_loaded";
    result.snapshot.err = ErrorCode::ModelNotReady;
    result.health = {ServiceHealth::Starting,
                     result.snapshot.mode,
                     ErrorCode::ModelNotReady,
                     "policy_not_loaded"};
    return result;
  };

  AppRunner runner(productionTestConfig(tmp), factory);
  REQUIRE_FALSE(runner.start());
  REQUIRE(factory_calls.load() == 1);
  REQUIRE(runner.start());
  REQUIRE(factory_calls.load() == 2);

  runner.stop();
}

TEST_CASE("Default AppRunner clears factory runtime deps when HTTP server start fails") {
  TempTree tmp;
  TcpPortLease occupied_port;
  REQUIRE(occupied_port.port() > 0);

  auto failing_config = productionTestConfig(tmp, tmp.root / "failing.lock");
  failing_config.http.port = occupied_port.port();

  std::atomic<int> factory_calls{0};
  const auto robot_destroyed = std::make_shared<std::atomic<int>>(0);
  const auto policy_destroyed = std::make_shared<std::atomic<int>>(0);

  auto factory = [&](const AppConfig&) {
    ++factory_calls;
    FakeRobotIO* robot = nullptr;
    FakePolicy* policy = nullptr;

    AppRuntimeFactoryResult result;
    result.deps.emplace(makeDeps(robot,
                                 policy,
                                 static_cast<std::uint8_t>(failing_config.mode_machine),
                                 RuntimeMode::Sim,
                                 robot_destroyed,
                                 policy_destroyed));
    return result;
  };

  {
    AppRunner runner(failing_config, factory);
    REQUIRE_FALSE(runner.start());
    REQUIRE(factory_calls.load() == 1);
    REQUIRE(robot_destroyed->load() == 1);
    REQUIRE(policy_destroyed->load() == 1);

    occupied_port.close();
    REQUIRE(runner.start());
    REQUIRE(factory_calls.load() == 2);

    runner.stop();
  }

  REQUIRE(robot_destroyed->load() == 2);
  REQUIRE(policy_destroyed->load() == 2);
}

TEST_CASE("AppRunner default process lock is partitioned by domain_id") {
  TempTree tmp;
  AppConfig domain0 = productionTestConfig(tmp);
  domain0.lock_path.clear();
  domain0.network = "dds0";
  domain0.mode_machine = 1;
  domain0.domain_id = 0;

  AppConfig domain1 = domain0;
  domain1.http.port = 0;
  domain1.domain_id = 1;

  FakeRobotIO* robot0 = nullptr;
  FakePolicy* policy0 = nullptr;
  AppRunner first(domain0,
                  makeDeps(robot0, policy0, static_cast<std::uint8_t>(domain0.mode_machine),
                           RuntimeMode::Real));
  REQUIRE(first.start());

  FakeRobotIO* robot1 = nullptr;
  FakePolicy* policy1 = nullptr;
  AppRunner second(domain1,
                   makeDeps(robot1, policy1, static_cast<std::uint8_t>(domain1.mode_machine),
                            RuntimeMode::Real));
  REQUIRE(second.start());

  second.stop();
  first.stop();
}

TEST_CASE("AppRunner with fake policy runtime becomes ready") {
  TempTree tmp;
  FakeRobotIO* robot = nullptr;
  FakePolicy* policy = nullptr;
  AppRunner runner(productionTestConfig(tmp), makeDeps(robot, policy));
  REQUIRE(runner.start());
  auto client = httplib::Client("127.0.0.1", runner.boundPort());

  const auto status = pollJson(client, "/status", [](const nlohmann::json& json) {
    return json.at("ready") == true;
  });
  REQUIRE(status.at("mode") == "sim");
  REQUIRE(status.at("ctrl") == "idle");
  REQUIRE(status.at("err").is_null());
  REQUIRE(status.at("block").is_null());

  const auto health = body(client.Get("/health"), 200);
  REQUIRE(health.at("ok") == true);
  REQUIRE(health.at("state") == "ready");
  REQUIRE(health.at("mode") == "sim");

  runner.stop();
}

TEST_CASE("HTTP execute reaches RuntimeControlLoop policy write") {
  TempTree tmp;
  FakeRobotIO* robot = nullptr;
  FakePolicy* policy = nullptr;
  const auto config = productionTestConfig(tmp);
  const auto path = validTrk(tmp.allowed, "policy_http.trk", 3);
  AppRunner runner(config, makeDeps(robot, policy));
  REQUIRE(runner.start());
  auto client = httplib::Client("127.0.0.1", runner.boundPort());

  pollJson(client, "/status", [](const nlohmann::json& json) {
    return json.at("ready") == true;
  });

  const auto execute =
      body(client.Post("/execute",
                       (std::string(R"({"path":")") + path.string() + R"("})").c_str(),
                       "application/json"),
           200);
  const std::string id = execute.at("id");

  const auto done = pollJson(client, "/status?id=" + id, [](const nlohmann::json& json) {
    return json.at("state") == "done";
  });
  REQUIRE(done.at("err").is_null());
  REQUIRE(policy->calls.load() >= 1);
  REQUIRE(robot->write_low_cmd_calls.load() >= 1);
  REQUIRE(robot->last_mode_machine.load() == config.mode_machine);

  runner.stop();
}

TEST_CASE("AppRunner passes sim mode_machine through to RuntimeControlLoop LowCmd") {
  TempTree tmp;
  FakeRobotIO* robot = nullptr;
  FakePolicy* policy = nullptr;
  auto config = productionTestConfig(tmp);
  config.mode_machine = 0;
  const auto path = validTrk(tmp.allowed, "policy_http_sim.trk", 3);
  AppRunner runner(config, makeDeps(robot, policy, 0, RuntimeMode::Sim));
  REQUIRE(runner.start());
  auto client = httplib::Client("127.0.0.1", runner.boundPort());

  pollJson(client, "/status", [](const nlohmann::json& json) {
    return json.at("ready") == true;
  });

  const auto execute =
      body(client.Post("/execute",
                       (std::string(R"({"path":")") + path.string() + R"("})").c_str(),
                       "application/json"),
           200);
  const std::string id = execute.at("id");

  const auto done = pollJson(client, "/status?id=" + id, [](const nlohmann::json& json) {
    return json.at("state") == "done";
  });
  REQUIRE(done.at("err").is_null());
  REQUIRE(robot->write_low_cmd_calls.load() >= 1);
  REQUIRE(robot->last_mode_machine.load() == 0);

  runner.stop();
}

TEST_CASE("Ready AppRunner rejects invalid trk before enqueue") {
  TempTree tmp;
  FakeRobotIO* robot = nullptr;
  FakePolicy* policy = nullptr;
  AppRunner runner(productionTestConfig(tmp), makeDeps(robot, policy));
  REQUIRE(runner.start());
  auto client = httplib::Client("127.0.0.1", runner.boundPort());

  pollJson(client, "/status", [](const nlohmann::json& json) {
    return json.at("ready") == true;
  });

  const auto relative =
      body(client.Post("/execute", R"({"path":"relative.trk"})", "application/json"),
           400);
  REQUIRE(relative.at("ok") == false);
  REQUIRE(relative.at("error").at("code") == "REQUEST_INVALID");

  const auto outside_path = validTrk(tmp.outside, "outside.trk", 3);
  const auto outside =
      body(client.Post("/execute",
                       (std::string(R"({"path":")") + outside_path.string() + R"("})").c_str(),
                       "application/json"),
           403);
  REQUIRE(outside.at("ok") == false);
  REQUIRE(outside.at("error").at("code") == "TRK_PATH_NOT_ALLOWED");
  REQUIRE(policy->calls.load() == 0);
  REQUIRE(robot->write_low_cmd_calls.load() >= 1);

  runner.stop();
}

TEST_CASE("AppRunner stop interrupts low-hz runtime sleep") {
  auto config = productionTestConfig();
  config.runtime.hz = 0.2;

  AppRunner runner(config);
  REQUIRE(runner.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto before_stop = std::chrono::steady_clock::now();
  runner.stop();
  const auto elapsed = std::chrono::steady_clock::now() - before_stop;

  REQUIRE(elapsed < std::chrono::milliseconds(500));
}

}  // namespace agentic_et1_tracker
