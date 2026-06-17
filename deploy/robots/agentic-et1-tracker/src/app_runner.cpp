#include "agentic_et1_tracker/app/app_runner.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "agentic_et1_tracker/app/app_runtime_factory.hpp"
#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/app/process_lock.hpp"
#include "agentic_et1_tracker/core/id_generator.hpp"
#include "agentic_et1_tracker/http/server.hpp"
#include "agentic_et1_tracker/loco_upper/command_limits.hpp"
#include "agentic_et1_tracker/loco_upper/precheck.hpp"
#include "agentic_et1_tracker/reference/reference_frame_store.hpp"
#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_control_loop.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"
#include "agentic_et1_tracker/trk/loader.hpp"
#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {
namespace {

TrackValidation toTrackValidation(const TrkValidationResult& result) {
  TrackValidation out;
  out.code = toCoreErrorCode(result.code);
  out.message = result.message;
  if (result.ok()) {
    out.metadata.frames = result.metadata.frames;
    out.metadata.duration_s = result.metadata.duration_s;
    out.metadata.fps = result.metadata.fps;
    out.metadata.canonical_path = result.metadata.canonical_path.string();
  }
  return out;
}

class AppTrackValidator final : public TrackValidatorPort {
 public:
  explicit AppTrackValidator(TrkValidationConfig config) : validator_(std::move(config)) {}

  TrackValidation validate(const std::string& path) override {
    return toTrackValidation(validator_.validate(path));
  }

 private:
  TrkValidator validator_;
};

class AppLocoUpperPrechecker final : public LocoUpperPrecheckPort {
 public:
  AppLocoUpperPrechecker(TrkValidationConfig config,
                         RuntimeConfig runtime_config,
                         bool enabled,
                         std::string limits_path,
                         std::optional<LocoLowerDeployConfig> lower_deploy_config =
                             std::nullopt)
      : loader_(std::move(config)),
        runtime_config_(runtime_config),
        enabled_(enabled) {
    if (!enabled) {
      return;
    }
    if (lower_deploy_config) {
      configureCommandLimits(*lower_deploy_config);
    }
    try {
      validation_options_ =
          loadLocoUpperJointValidationOptions(std::filesystem::path(limits_path));
    } catch (const std::exception& err) {
      load_error_ = err.what();
    }
  }

  void configureCommandLimits(const LocoLowerDeployConfig& lower_deploy_config) {
    if (!enabled_) {
      return;
    }
    command_limits_ =
        locoUpperCommandLimitsFromConfig(runtime_config_, lower_deploy_config);
  }

  LocoUpperPrecheckResult precheck(
      const std::string& canonical_path,
      const LocoUpperPrecheckOptions& options) override {
    if (!load_error_.empty()) {
      return {ErrorCode::ModelNotReady, load_error_};
    }
    if (!command_limits_) {
      return {ErrorCode::ModelNotReady, "loco_upper command limits not loaded"};
    }

    LocoUpperPrecheckOptions configured = options;
    configured.upper_joint_limits = validation_options_;
    configured.command_limits = *command_limits_;
    return precheckLocoUpperTrackFile(loader_, canonical_path, configured);
  }

 private:
  TrkLoader loader_;
  RuntimeConfig runtime_config_;
  bool enabled_{false};
  std::optional<LocoUpperCommandLimits> command_limits_;
  std::optional<LocoUpperJointValidationOptions> validation_options_;
  std::string load_error_;
};

class AppRunIdGenerator final : public RunIdGenerator {
 public:
  std::string generate() override { return ids_.generate(); }

 private:
  ShortIdGenerator ids_;
};

constexpr const char* kPolicyNotLoadedBlock = "policy_not_loaded";

RuntimeMode initialRuntimeMode(const AppConfig& config) {
  return config.mode_machine == 0 ? RuntimeMode::Sim : RuntimeMode::Real;
}

AppRuntimeFactoryResult initialNotReady(const AppConfig& config) {
  AppRuntimeFactoryResult result;
  result.snapshot.ready = false;
  result.snapshot.mode = initialRuntimeMode(config);
  result.snapshot.robot = RobotState::NotReady;
  result.snapshot.ctrl = ControllerState::Starting;
  result.snapshot.hz = config.runtime.hz;
  result.snapshot.queue.limit = config.runtime.queue_limit;
  result.snapshot.block = kPolicyNotLoadedBlock;
  result.snapshot.err = ErrorCode::ModelNotReady;
  result.health = {ServiceHealth::Starting,
                   initialRuntimeMode(config),
                   ErrorCode::ModelNotReady,
                   kPolicyNotLoadedBlock};
  return result;
}

AgentApiConfig makeAgentApiConfig(RuntimeMode mode, const AppConfig& config) {
  AgentApiConfig api;
  api.mode = mode;
  api.queue_limit = config.runtime.queue_limit;
  api.passive_password = config.passive_password;
  api.loco_upper.enabled = config.loco_upper.enabled;
  api.loco_upper.ready = false;
  api.loco_upper.default_radius_m = config.loco_upper.default_radius_m;
  api.loco_upper.max_radius_m = config.loco_upper.max_radius_m;
  api.loco_upper.strict_pose = config.loco_upper.strict_pose;
  return api;
}

AppRuntimeFactoryResult injectedRuntime(AppRuntimeDeps deps) {
  AppRuntimeFactoryResult result;
  result.deps.emplace(std::move(deps));
  return result;
}

std::optional<LocoLowerDeployConfig> lowerDeployConfigForPrecheck(
    const AppRuntimeFactoryResult& result) {
  if (!result.deps || !result.deps->loco_lower_deploy_config) {
    return std::nullopt;
  }
  return *result.deps->loco_lower_deploy_config;
}

std::string lockNamePart(std::string value) {
  for (char& ch : value) {
    const auto uch = static_cast<unsigned char>(ch);
    if (!std::isalnum(uch) && ch != '_' && ch != '-' && ch != '.') {
      ch = '_';
    }
  }
  return value;
}

std::filesystem::path processLockPath(const AppConfig& config) {
  if (!config.lock_path.empty()) {
    return config.lock_path;
  }
  return std::filesystem::temp_directory_path() /
         ("agentic-et1-tracker-" + lockNamePart(config.network) + "-" +
          std::to_string(config.domain_id) + "-" + std::to_string(config.mode_machine) +
          ".lock");
}

}  // namespace

class AppRunner::Impl {
 public:
  explicit Impl(AppConfig config)
      : Impl(std::move(config), createAppRuntimeDeps) {}

  Impl(AppConfig config, AppRuntimeFactory runtime_factory)
      : config_(std::move(config)),
        runtime_factory_(std::move(runtime_factory)),
        factory_result_(initialNotReady(config_)),
        status_(config_.runtime),
        bridge_(config_.runtime, status_),
        validator_(config_.trk),
        loco_prechecker_(config_.trk,
                         config_.runtime,
                         config_.loco_upper.enabled,
                         config_.loco_upper.limits),
        api_(makeAgentApiConfig(initialRuntimeMode(config_), config_),
             bridge_,
             status_,
             validator_,
             loco_prechecker_,
             ids_),
        reference_store_(config_.reference.enabled ? std::make_unique<ReferenceFrameStore>()
                                                   : nullptr),
        server_(config_.http, api_, reference_store_.get()) {}

  Impl(AppConfig config, AppRuntimeDeps deps)
      : Impl(std::move(config), injectedRuntime(std::move(deps))) {}

  Impl(AppConfig config, AppRuntimeFactoryResult factory_result)
      : config_(std::move(config)),
        runtime_factory_(),
        factory_result_(std::move(factory_result)),
        status_(config_.runtime),
        bridge_(config_.runtime, status_),
        validator_(config_.trk),
        loco_prechecker_(config_.trk,
                         config_.runtime,
                         config_.loco_upper.enabled,
                         config_.loco_upper.limits,
                         lowerDeployConfigForPrecheck(factory_result_)),
        api_(makeAgentApiConfig(factory_result_.deps->mode, config_),
             bridge_,
             status_,
             validator_,
             loco_prechecker_,
             ids_),
        reference_store_(config_.reference.enabled ? std::make_unique<ReferenceFrameStore>()
                                                   : nullptr),
        server_(config_.http, api_, reference_store_.get()) {
    attachRuntimeLoop(*factory_result_.deps);
  }

  ~Impl() { stop(); }

  bool start() {
    if (running_.load() || runtime_thread_.joinable()) {
      return false;
    }

    bool factory_runtime_created = false;
    process_lock_.emplace(processLockPath(config_));
    if (!process_lock_->tryLock()) {
      process_lock_.reset();
      return false;
    }

    if (!runtime_loop_ && runtime_factory_) {
      try {
        factory_result_ = runtime_factory_(config_);
        factory_runtime_created = factory_result_.deps.has_value();
        if (factory_result_.deps) {
          attachRuntimeLoop(*factory_result_.deps);
        }
      } catch (...) {
        if (factory_runtime_created) {
          clearFactoryRuntime();
        }
        process_lock_.reset();
        return false;
      }
    }

    if (!runtime_loop_) {
      publishNotReady();
    }
    running_.store(true);
    runtime_thread_ = std::thread([this] { runtimeLoop(); });

    if (!server_.start()) {
      server_.stop();
      {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        running_.store(false);
      }
      runtime_cv_.notify_all();
      if (runtime_thread_.joinable()) {
        runtime_thread_.join();
      }
      if (factory_runtime_created) {
        clearFactoryRuntime();
      }
      process_lock_.reset();
      return false;
    }
    return true;
  }

  void stop() {
    server_.stop();
    {
      std::lock_guard<std::mutex> lock(runtime_mutex_);
      running_.store(false);
    }
    runtime_cv_.notify_all();
    if (runtime_thread_.joinable()) {
      runtime_thread_.join();
    }
    process_lock_.reset();
  }

  bool isRunning() const { return server_.isRunning(); }
  int boundPort() const { return server_.boundPort(); }

 private:
  void attachRuntimeLoop(AppRuntimeDeps& deps) {
    if (deps.velocity_policy) {
      const bool enable_loco_upper_runtime =
          config_.loco_upper.enabled && deps.loco_lower_policy &&
          deps.loco_lower_deploy_config && deps.loco_upper_composer_config;
      if (enable_loco_upper_runtime) {
        loco_prechecker_.configureCommandLimits(*deps.loco_lower_deploy_config);
        runtime_loop_.emplace(config_.runtime,
                              bridge_,
                              status_,
                              TrkLoader(config_.trk),
                              *deps.robot_io,
                              *deps.policy,
                              deps.deploy_config,
                              *deps.velocity_policy,
                              deps.velocity_deploy_config,
                              deps.fixstand_config,
                              deps.passive_config,
                              deps.startup_control,
                              static_cast<std::uint8_t>(config_.mode_machine),
                              *deps.loco_lower_policy,
                              *deps.loco_lower_deploy_config,
                              *deps.loco_upper_composer_config,
                              deps.mode,
                              reference_store_.get(),
                              deps.standby_track);
        return;
      }

      runtime_loop_.emplace(config_.runtime,
                            bridge_,
                            status_,
                            TrkLoader(config_.trk),
                            *deps.robot_io,
                            *deps.policy,
                            deps.deploy_config,
                            *deps.velocity_policy,
                            deps.velocity_deploy_config,
                            deps.fixstand_config,
                            deps.passive_config,
                            deps.startup_control,
                            static_cast<std::uint8_t>(config_.mode_machine),
                            deps.mode,
                            reference_store_.get(),
                            deps.standby_track);
      return;
    }

    runtime_loop_.emplace(config_.runtime,
                          bridge_,
                          status_,
                          TrkLoader(config_.trk),
                          *deps.robot_io,
                          *deps.policy,
                          deps.deploy_config,
                          deps.passive_config,
                          static_cast<std::uint8_t>(config_.mode_machine),
                          deps.mode,
                          reference_store_.get(),
                          deps.standby_track);
  }

  void runtimeLoop() {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / config_.runtime.hz));
    auto next = std::chrono::steady_clock::now();
    while (running_.load()) {
      if (runtime_loop_) {
        runtime_loop_->tick();
      } else {
        publishNotReady();
      }
      next += period;
      std::unique_lock<std::mutex> lock(runtime_mutex_);
      runtime_cv_.wait_until(lock, next, [this] { return !running_.load(); });
    }
  }

  void publishNotReady() {
    status_.publishSnapshot(factory_result_.snapshot);
    status_.publishHealthSnapshot(factory_result_.health);
  }

  void clearFactoryRuntime() {
    runtime_loop_.reset();
    factory_result_ = initialNotReady(config_);
  }

  AppConfig config_;
  AppRuntimeFactory runtime_factory_;
  AppRuntimeFactoryResult factory_result_;
  RuntimeStatusStore status_;
  RuntimeBridge bridge_;
  AppTrackValidator validator_;
  AppLocoUpperPrechecker loco_prechecker_;
  AppRunIdGenerator ids_;
  AgentApiService api_;
  std::unique_ptr<ReferenceFrameStore> reference_store_;
  AgentHttpServer server_;
  std::optional<RuntimeControlLoop> runtime_loop_;
  std::optional<ProcessLock> process_lock_;
  std::atomic<bool> running_{false};
  std::thread runtime_thread_;
  std::mutex runtime_mutex_;
  std::condition_variable runtime_cv_;
};

AppRunner::AppRunner(AppConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

AppRunner::AppRunner(AppConfig config, AppRuntimeFactory runtime_factory)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(runtime_factory))) {}

AppRunner::AppRunner(AppConfig config, AppRuntimeDeps deps)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(deps))) {}

AppRunner::~AppRunner() = default;

bool AppRunner::start() { return impl_->start(); }

void AppRunner::stop() { impl_->stop(); }

bool AppRunner::isRunning() const { return impl_->isRunning(); }

int AppRunner::boundPort() const { return impl_->boundPort(); }

}  // namespace agentic_et1_tracker
