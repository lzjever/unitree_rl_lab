#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "agentic_et1_tracker/runtime/runtime_bridge.hpp"
#include "agentic_et1_tracker/runtime/runtime_status_store.hpp"

namespace agentic_et1_tracker {
namespace {

RuntimeConfig runtimeConfig(std::size_t queue_limit = 8) {
  RuntimeConfig config;
  config.queue_limit = queue_limit;
  config.recent_limit = 16;
  config.hz = 50.0;
  return config;
}

ExecuteCommand executeCommand(std::string id,
                              MotionMode mode = MotionMode::Queue,
                              std::size_t frames = 120,
                              double duration_s = 2.4) {
  ExecuteCommand command;
  command.id = std::move(id);
  command.path = "/tmp/" + command.id + ".trk";
  command.mode = mode;
  command.track.frames = frames;
  command.track.duration_s = duration_s;
  return command;
}

IdleMotion idleMotion(std::string path,
                      std::size_t frames = 40,
                      double duration_s = 0.8) {
  IdleMotion motion;
  motion.path = std::move(path);
  motion.track.frames = frames;
  motion.track.duration_s = duration_s;
  motion.track.fps = 50.0;
  motion.track.canonical_path = motion.path;
  return motion;
}

StatusSnapshot readySnapshot(ControllerState ctrl = ControllerState::Idle) {
  StatusSnapshot snapshot;
  snapshot.ready = true;
  snapshot.mode = RuntimeMode::Sim;
  snapshot.robot = RobotState::Idle;
  snapshot.ctrl = ctrl;
  snapshot.hz = 50.0;
  snapshot.queue.limit = 8;
  return snapshot;
}

}  // namespace

TEST_CASE("RuntimeBridge admits queue requests without touching controller state") {
  const RuntimeConfig config = runtimeConfig(2);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  const auto result = bridge.submitQueue(executeCommand("run-a"));

  REQUIRE(result.code == ErrorCode::Ok);
  REQUIRE(result.id == "run-a");
  REQUIRE(result.state == MotionState::Queued);
  REQUIRE(result.q == 1);

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.stop_reason == StopReason::None);
  REQUIRE(snapshot.queue.n == 1);
  REQUIRE(snapshot.queue.limit == 2);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"run-a"});

  const auto lookup = store.findRun("run-a");
  REQUIRE(lookup.code == ErrorCode::Ok);
  REQUIRE(lookup.run.has_value());
  REQUIRE(lookup.run->state == MotionState::Queued);
  REQUIRE(lookup.run->path == "/tmp/run-a.trk");
  REQUIRE(lookup.run->frames == 120);
  REQUIRE(lookup.run->duration_s == 2.4);
}

TEST_CASE("RuntimeBridge configureIdle publishes status without user queue state") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  const auto result =
      bridge.configureIdle({idleMotion("/tmp/idle-a.trk"), idleMotion("/tmp/idle-b.trk")});

  REQUIRE(result.code == ErrorCode::Ok);
  REQUIRE(result.idle.enabled);
  REQUIRE(result.idle.n == 2);
  REQUIRE_FALSE(result.idle.active);

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.active.kind == ActiveKind::None);
  REQUIRE_FALSE(snapshot.exec.has_value());
  REQUIRE(snapshot.queue.ids.empty());
  REQUIRE(snapshot.idle.enabled);
  REQUIRE(snapshot.idle.n == 2);
  REQUIRE_FALSE(snapshot.idle.active);
  REQUIRE(store.findRun("/tmp/idle-a.trk").code == ErrorCode::RunNotFound);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::IdleConfig);
  REQUIRE(command->idle_motions.size() == 2);
  REQUIRE(command->idle_motions.at(0).path == "/tmp/idle-a.trk");
  REQUIRE(store.snapshot().idle.enabled);
}

TEST_CASE("RuntimeBridge stop clears idle status and emits stop for idle config") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  REQUIRE(bridge.configureIdle({idleMotion("/tmp/idle.trk")}).ok());
  REQUIRE(store.snapshot().idle.enabled);

  const auto stopped = bridge.stop();

  REQUIRE(stopped.code == ErrorCode::Ok);
  REQUIRE(stopped.state == ControllerState::Idle);
  REQUIRE(stopped.stop_reason == StopReason::None);
  REQUIRE_FALSE(store.snapshot().idle.enabled);
  REQUIRE(store.snapshot().idle.n == 0);
  REQUIRE_FALSE(store.snapshot().idle.active);
  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeStatusStore lookup prefers current exec then accepted queue then recent") {
  const RuntimeConfig config = runtimeConfig(4);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  auto snapshot = readySnapshot(ControllerState::Running);
  snapshot.queue.n = 1;
  snapshot.queue.ids = {"control-queued"};
  MotionStatus active;
  active.id = "same-id";
  active.path = "/tmp/active.trk";
  active.state = MotionState::Running;
  snapshot.exec = active;
  store.publishSnapshot(snapshot);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"control-queued"});

  REQUIRE(bridge.submitQueue(executeCommand("accepted-id")).code == ErrorCode::Ok);
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"control-queued", "accepted-id"});

  REQUIRE(bridge.submitQueue(executeCommand("same-id")).code == ErrorCode::Ok);
  auto found = store.findRun("same-id");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Running);
  REQUIRE(found.run->path == "/tmp/active.trk");

  REQUIRE(bridge.submitQueue(executeCommand("recent-id")).code == ErrorCode::Ok);
  REQUIRE(bridge.submitInterrupt(executeCommand("urgent", MotionMode::Interrupt)).code ==
          ErrorCode::Ok);

  found = store.findRun("recent-id");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Canceled);
  REQUIRE(found.run->stop_reason == StopReason::Interrupt);

  found = store.findRun("urgent");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Queued);
}

TEST_CASE("RuntimeBridge enforces queue limit under concurrent submits") {
  const RuntimeConfig config = runtimeConfig(4);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  constexpr int kThreads = 16;
  std::atomic<int> ready{0};
  std::atomic<bool> start{false};
  std::vector<ErrorCode> results(kThreads, ErrorCode::InternalError);
  std::vector<std::thread> threads;
  threads.reserve(kThreads);

  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back([&, i] {
      ready.fetch_add(1);
      while (!start.load()) {
      }
      results[i] =
          bridge.submitQueue(executeCommand("run-" + std::to_string(i))).code;
    });
  }

  while (ready.load() != kThreads) {
  }
  start.store(true);
  for (auto& thread : threads) {
    thread.join();
  }

  std::size_t accepted = 0;
  std::size_t rejected = 0;
  for (const ErrorCode code : results) {
    if (code == ErrorCode::Ok) {
      ++accepted;
    } else if (code == ErrorCode::QueueFull) {
      ++rejected;
    }
  }

  REQUIRE(accepted == 4);
  REQUIRE(rejected == 12);
  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.queue.n == 4);
  REQUIRE(snapshot.queue.ids.size() == 4);
}

TEST_CASE("RuntimeBridge queue limit counts published and accepted queued ids") {
  {
    const RuntimeConfig config = runtimeConfig(2);
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);

    auto snapshot = readySnapshot();
    snapshot.queue.n = 2;
    snapshot.queue.ids = {"published-a", "published-b"};
    store.publishSnapshot(snapshot);

    const auto result = bridge.submitQueue(executeCommand("overflow"));

    REQUIRE(result.code == ErrorCode::QueueFull);
    REQUIRE(store.snapshot().queue.n == 2);
    REQUIRE(store.snapshot().queue.ids ==
            std::vector<std::string>{"published-a", "published-b"});
  }

  {
    const RuntimeConfig config = runtimeConfig(3);
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);

    auto snapshot = readySnapshot();
    snapshot.queue.n = 2;
    snapshot.queue.ids = {"published-a", "published-b"};
    store.publishSnapshot(snapshot);

    const auto accepted = bridge.submitQueue(executeCommand("accepted-c"));
    REQUIRE(accepted.code == ErrorCode::Ok);
    REQUIRE(accepted.q == 3);
    REQUIRE(store.snapshot().queue.ids ==
            std::vector<std::string>{"published-a", "published-b", "accepted-c"});

    const auto full = bridge.submitQueue(executeCommand("overflow"));
    REQUIRE(full.code == ErrorCode::QueueFull);
    REQUIRE(store.snapshot().queue.n == 3);
  }
}

TEST_CASE("RuntimeBridge keeps accepted run visible after command consumption") {
  const RuntimeConfig config = runtimeConfig(2);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  REQUIRE(bridge.submitQueue(executeCommand("dispatch-me")).code == ErrorCode::Ok);
  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Queue);
  REQUIRE(command->request.id == "dispatch-me");

  auto found = store.findRun("dispatch-me");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Queued);
  REQUIRE(found.run->path == "/tmp/dispatch-me.trk");

  auto snapshot = readySnapshot();
  snapshot.queue.n = 1;
  snapshot.queue.ids = {"dispatch-me"};
  store.publishSnapshot(snapshot);

  found = store.findRun("dispatch-me");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Queued);
  REQUIRE(found.run->frames == 120);
}

TEST_CASE("RuntimeBridge stop cancels published queued ids when consumed") {
  const RuntimeConfig config = runtimeConfig(2);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  REQUIRE(bridge.submitQueue(executeCommand("published-stop")).code == ErrorCode::Ok);
  REQUIRE(bridge.consumeNextCommand().has_value());

  auto snapshot = readySnapshot();
  snapshot.queue.n = 1;
  snapshot.queue.ids = {"published-stop"};
  store.publishSnapshot(snapshot);

  const auto stopped = bridge.stop();

  REQUIRE(stopped.code == ErrorCode::Ok);
  REQUIRE(stopped.state == ControllerState::Stopping);
  REQUIRE(stopped.stop_reason == StopReason::Stop);
  REQUIRE(stopped.cleared == 0);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"published-stop"});

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  REQUIRE(store.snapshot().queue.ids.empty());

  const auto found = store.findRun("published-stop");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Canceled);
  REQUIRE(found.run->stop_reason == StopReason::Stop);
}

TEST_CASE("RuntimeBridge interrupt cancels published queued ids before queuing new head") {
  const RuntimeConfig config = runtimeConfig(3);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  REQUIRE(bridge.submitQueue(executeCommand("published-interrupt")).code == ErrorCode::Ok);
  REQUIRE(bridge.consumeNextCommand().has_value());

  auto snapshot = readySnapshot(ControllerState::Stopping);
  snapshot.stop_reason = StopReason::Stop;
  snapshot.queue.n = 1;
  snapshot.queue.ids = {"published-interrupt"};
  store.publishSnapshot(snapshot);

  const auto interrupted =
      bridge.submitInterrupt(executeCommand("urgent", MotionMode::Interrupt));

  REQUIRE(interrupted.code == ErrorCode::Ok);
  REQUIRE(interrupted.q == 1);
  REQUIRE(store.snapshot().ctrl == ControllerState::Stopping);
  REQUIRE(store.snapshot().stop_reason == StopReason::Stop);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"urgent"});

  auto found = store.findRun("published-interrupt");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Canceled);
  REQUIRE(found.run->stop_reason == StopReason::Interrupt);

  found = store.findRun("urgent");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Queued);
}

TEST_CASE("RuntimeStatusStore terminal run publication updates current exec synchronously") {
  const RuntimeConfig config = runtimeConfig(4);
  RuntimeStatusStore store(config);

  auto snapshot = readySnapshot(ControllerState::Running);
  MotionStatus active;
  active.id = "active-terminal";
  active.path = "/tmp/active-terminal.trk";
  active.state = MotionState::Running;
  active.frame = 3;
  active.frames = 5;
  snapshot.exec = active;
  store.publishSnapshot(snapshot);

  MotionStatus terminal = active;
  terminal.state = MotionState::Done;
  terminal.frame = 4;
  terminal.progress = 1.0;
  store.publishRunStatus(terminal);

  const auto found = store.findRun("active-terminal");
  REQUIRE(found.code == ErrorCode::Ok);
  REQUIRE(found.run->state == MotionState::Done);
  REQUIRE(found.run->frame == 4);
  REQUIRE(store.snapshot().exec->state == MotionState::Done);
}

TEST_CASE("RuntimeStatusStore terminal run publication releases accepted queue slot") {
  for (const MotionState terminal_state :
       {MotionState::Done, MotionState::Failed, MotionState::Stopped,
        MotionState::Canceled}) {
    const RuntimeConfig config = runtimeConfig(1);
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    store.publishSnapshot(readySnapshot());

    const std::string id = "terminal-" + toString(terminal_state);
    REQUIRE(bridge.submitQueue(executeCommand(id)).code == ErrorCode::Ok);
    REQUIRE(bridge.consumeNextCommand().has_value());

    auto queued = store.findRun(id);
    REQUIRE(queued.code == ErrorCode::Ok);
    REQUIRE(queued.run->state == MotionState::Queued);

    MotionStatus terminal = *queued.run;
    terminal.state = terminal_state;
    terminal.stop_reason =
        terminal_state == MotionState::Stopped || terminal_state == MotionState::Canceled
            ? StopReason::Stop
            : StopReason::None;
    terminal.err = terminal_state == MotionState::Failed
                       ? ErrorCode::ModelInferenceFailed
                       : ErrorCode::Ok;
    store.publishRunStatus(terminal);

    REQUIRE(store.snapshot().queue.ids.empty());
    const auto found = store.findRun(id);
    REQUIRE(found.code == ErrorCode::Ok);
    REQUIRE(found.run->state == terminal_state);
    REQUIRE(found.run->stop_reason == terminal.stop_reason);
    REQUIRE(found.run->err == terminal.err);
    REQUIRE(bridge.submitQueue(executeCommand("next-" + toString(terminal_state))).code ==
            ErrorCode::Ok);
  }
}

TEST_CASE("RuntimeBridge interrupt replaces waiting queue and preserves top-level stop reason") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  auto stopping = readySnapshot(ControllerState::Stopping);
  stopping.stop_reason = StopReason::Stop;
  store.publishSnapshot(stopping);

  REQUIRE(bridge.submitQueue(executeCommand("old-a")).code == ErrorCode::Ok);
  REQUIRE(bridge.submitQueue(executeCommand("old-b")).code == ErrorCode::Ok);

  const auto result =
      bridge.submitInterrupt(executeCommand("urgent", MotionMode::Interrupt));

  REQUIRE(result.code == ErrorCode::Ok);
  REQUIRE(result.id == "urgent");
  REQUIRE(result.state == MotionState::Queued);
  REQUIRE(result.q == 1);

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Stopping);
  REQUIRE(snapshot.stop_reason == StopReason::Stop);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"urgent"});

  REQUIRE(store.findRun("old-a").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-a").run->stop_reason == StopReason::Interrupt);
  REQUIRE(store.findRun("old-b").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-b").run->stop_reason == StopReason::Interrupt);
  REQUIRE(store.findRun("urgent").run->state == MotionState::Queued);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Interrupt);
  REQUIRE(command->request.id == "urgent");
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeBridge stop is accepted before clearing waiting queue on consumption") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  store.publishSnapshot(readySnapshot());
  const auto idle = bridge.stop();
  REQUIRE(idle.code == ErrorCode::Ok);
  REQUIRE(idle.state == ControllerState::Idle);
  REQUIRE(idle.stop_reason == StopReason::None);
  REQUIRE(idle.cleared == 0);
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());

  REQUIRE(bridge.submitQueue(executeCommand("old-a")).code == ErrorCode::Ok);
  REQUIRE(bridge.submitQueue(executeCommand("old-b")).code == ErrorCode::Ok);

  const auto stopping = bridge.stop();
  REQUIRE(stopping.code == ErrorCode::Ok);
  REQUIRE(stopping.state == ControllerState::Stopping);
  REQUIRE(stopping.stop_reason == StopReason::Stop);
  REQUIRE(stopping.cleared == 0);

  auto snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Idle);
  REQUIRE(snapshot.stop_reason == StopReason::None);
  REQUIRE(snapshot.queue.ids == std::vector<std::string>{"old-a", "old-b"});
  REQUIRE(store.findRun("old-a").run->state == MotionState::Queued);
  REQUIRE(store.findRun("old-b").run->state == MotionState::Queued);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
  REQUIRE(store.snapshot().queue.n == 0);
  REQUIRE(store.findRun("old-a").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-a").run->stop_reason == StopReason::Stop);
  REQUIRE(store.findRun("old-b").run->state == MotionState::Canceled);
  REQUIRE(store.findRun("old-b").run->stop_reason == StopReason::Stop);

  auto running = readySnapshot(ControllerState::Running);
  MotionStatus active;
  active.id = "active";
  active.state = MotionState::Running;
  running.exec = active;
  store.publishSnapshot(running);

  const auto active_stop = bridge.stop();
  REQUIRE(active_stop.state == ControllerState::Stopping);
  REQUIRE(active_stop.stop_reason == StopReason::Stop);
  REQUIRE(active_stop.cleared == 0);
  snapshot = store.snapshot();
  REQUIRE(snapshot.ctrl == ControllerState::Running);
  REQUIRE(snapshot.stop_reason == StopReason::None);
}

TEST_CASE("RuntimeBridge stop no-op returns current passive and standby state") {
  const RuntimeConfig config = runtimeConfig(8);

  for (const ControllerState state :
       {ControllerState::Passive, ControllerState::StandbyVelocity}) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    store.publishSnapshot(readySnapshot(state));

    const auto result = bridge.stop();

    REQUIRE(result.code == ErrorCode::Ok);
    REQUIRE(result.state == state);
    REQUIRE(result.stop_reason == StopReason::None);
    REQUIRE(result.cleared == 0);
    REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
  }
}

TEST_CASE("RuntimeBridge rejects standby velocity from Passive and Fault without enqueuing") {
  const RuntimeConfig config = runtimeConfig(8);

  for (const ControllerState state : {ControllerState::Passive, ControllerState::Fault}) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    store.publishSnapshot(readySnapshot(state));

    const auto result = bridge.standbyVelocity();

    REQUIRE(result.code == ErrorCode::ControlStateConflict);
    REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
  }
}

TEST_CASE("RuntimeBridge accepts FixStand from Passive and Fault") {
  const RuntimeConfig config = runtimeConfig(8);

  for (const ControllerState state : {ControllerState::Passive, ControllerState::Fault}) {
    RuntimeStatusStore store(config);
    RuntimeBridge bridge(config, store);
    store.publishSnapshot(readySnapshot(state));

    const auto result = bridge.fixStand();

    REQUIRE(result.code == ErrorCode::Ok);
    auto command = bridge.consumeNextCommand();
    REQUIRE(command.has_value());
    REQUIRE(command->kind == CommandKind::FixStand);
    REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
  }
}

TEST_CASE("RuntimeBridge stop is accepted for run status published before snapshot exec") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  MotionStatus starting;
  starting.id = "starting";
  starting.path = "/tmp/starting.trk";
  starting.state = MotionState::Running;
  store.publishRunStatus(starting);

  const auto stopped = bridge.stop();

  REQUIRE(stopped.code == ErrorCode::Ok);
  REQUIRE(stopped.state == ControllerState::Stopping);
  REQUIRE(stopped.stop_reason == StopReason::Stop);
  REQUIRE(stopped.cleared == 0);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeBridge stop watermark preserves queue accepted after stop") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  store.publishSnapshot(readySnapshot());
  REQUIRE(bridge.submitQueue(executeCommand("old-a")).code == ErrorCode::Ok);
  REQUIRE(bridge.submitQueue(executeCommand("old-b")).code == ErrorCode::Ok);

  const auto stopping = bridge.stop();
  REQUIRE(stopping.code == ErrorCode::Ok);
  REQUIRE(stopping.state == ControllerState::Stopping);
  REQUIRE(stopping.stop_reason == StopReason::Stop);
  REQUIRE(stopping.cleared == 0);
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"old-a", "old-b"});
  REQUIRE(store.findRun("old-a").run->state == MotionState::Queued);
  REQUIRE(store.findRun("old-b").run->state == MotionState::Queued);

  REQUIRE(bridge.submitQueue(executeCommand("new-before-consume")).code == ErrorCode::Ok);
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"old-a", "old-b", "new-before-consume"});

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);

  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"new-before-consume"});
  for (const auto& id : {"old-a", "old-b"}) {
    const auto found = store.findRun(id);
    REQUIRE(found.code == ErrorCode::Ok);
    REQUIRE(found.run->state == MotionState::Canceled);
    REQUIRE(found.run->stop_reason == StopReason::Stop);
  }
  const auto kept = store.findRun("new-before-consume");
  REQUIRE(kept.code == ErrorCode::Ok);
  REQUIRE(kept.run->state == MotionState::Queued);
  REQUIRE(kept.run->stop_reason == StopReason::None);

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Queue);
  REQUIRE(command->request.id == "new-before-consume");
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeBridge fixstand after pending stop preserves stop watermark") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  store.publishSnapshot(readySnapshot());
  REQUIRE(bridge.submitQueue(executeCommand("old-before-stop")).ok());
  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitQueue(executeCommand("new-after-stop")).ok());

  const ControlResult fixstand = bridge.fixStand();
  REQUIRE(fixstand.ok());
  REQUIRE(store.snapshot().queue.ids ==
          std::vector<std::string>{"old-before-stop", "new-after-stop"});

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);

  const auto old = store.findRun("old-before-stop");
  REQUIRE(old.code == ErrorCode::Ok);
  REQUIRE(old.run->state == MotionState::Canceled);
  REQUIRE(old.run->stop_reason == StopReason::Stop);

  const auto kept = store.findRun("new-after-stop");
  REQUIRE(kept.code == ErrorCode::Ok);
  REQUIRE(kept.run->state == MotionState::Queued);
  REQUIRE(kept.run->stop_reason == StopReason::None);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"new-after-stop"});

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::FixStand);

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Queue);
  REQUIRE(command->request.id == "new-after-stop");
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeBridge stop watermark preserves interrupt accepted after stop") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  auto running = readySnapshot(ControllerState::Running);
  MotionStatus active;
  active.id = "active";
  active.state = MotionState::Running;
  running.exec = active;
  store.publishSnapshot(running);

  REQUIRE(bridge.submitQueue(executeCommand("old-before-stop")).ok());
  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitInterrupt(executeCommand("urgent", MotionMode::Interrupt)).ok());

  REQUIRE(store.findRun("old-before-stop").run->state == MotionState::Queued);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);

  const auto old = store.findRun("old-before-stop");
  REQUIRE(old.code == ErrorCode::Ok);
  REQUIRE(old.run->state == MotionState::Canceled);
  REQUIRE(old.run->stop_reason == StopReason::Stop);

  const auto kept = store.findRun("urgent");
  REQUIRE(kept.code == ErrorCode::Ok);
  REQUIRE(kept.run->state == MotionState::Queued);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"urgent"});

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Interrupt);
  REQUIRE(command->request.id == "urgent");
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeBridge later stop watermark cancels work accepted between stops") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);

  auto running = readySnapshot(ControllerState::Running);
  MotionStatus active;
  active.id = "active";
  active.state = MotionState::Running;
  running.exec = active;
  store.publishSnapshot(running);

  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitQueue(executeCommand("between-stops")).ok());
  REQUIRE(bridge.stop().state == ControllerState::Stopping);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  REQUIRE(store.findRun("between-stops").run->state == MotionState::Queued);
  REQUIRE(store.snapshot().queue.ids == std::vector<std::string>{"between-stops"});

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  const auto canceled = store.findRun("between-stops");
  REQUIRE(canceled.code == ErrorCode::Ok);
  REQUIRE(canceled.run->state == MotionState::Canceled);
  REQUIRE(canceled.run->stop_reason == StopReason::Stop);
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeBridge command consumption uses priority and FIFO for queues") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  REQUIRE(bridge.submitQueue(executeCommand("queue-a")).code == ErrorCode::Ok);
  REQUIRE(bridge.submitQueue(executeCommand("queue-b")).code == ErrorCode::Ok);

  auto command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Queue);
  REQUIRE(command->request.id == "queue-a");

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Queue);
  REQUIRE(command->request.id == "queue-b");
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());

  REQUIRE(bridge.submitQueue(executeCommand("queue-c")).code == ErrorCode::Ok);
  REQUIRE(bridge.submitInterrupt(executeCommand("interrupt-a", MotionMode::Interrupt)).code ==
          ErrorCode::Ok);
  REQUIRE(bridge.submitQueue(executeCommand("queue-d")).code == ErrorCode::Ok);
  REQUIRE(bridge.stop().state == ControllerState::Stopping);
  REQUIRE(bridge.submitInterrupt(executeCommand("interrupt-b", MotionMode::Interrupt)).code ==
          ErrorCode::Ok);

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Stop);
  REQUIRE(store.findRun("interrupt-b").run->state == MotionState::Queued);
  REQUIRE(store.findRun("interrupt-b").run->stop_reason == StopReason::None);

  command = bridge.consumeNextCommand();
  REQUIRE(command.has_value());
  REQUIRE(command->kind == CommandKind::Interrupt);
  REQUIRE(command->request.id == "interrupt-b");
  REQUIRE_FALSE(bridge.consumeNextCommand().has_value());
}

TEST_CASE("RuntimeStatusStore supports concurrent submit status lookup and publication") {
  const RuntimeConfig config = runtimeConfig(8);
  RuntimeStatusStore store(config);
  RuntimeBridge bridge(config, store);
  store.publishSnapshot(readySnapshot());

  std::atomic<bool> violation{false};
  std::atomic<bool> done{false};

  std::thread submitter([&] {
    for (int i = 0; i < 300; ++i) {
      (void)bridge.submitQueue(executeCommand("submit-" + std::to_string(i)));
      if (i % 9 == 0) {
        (void)bridge.submitInterrupt(
            executeCommand("urgent-" + std::to_string(i), MotionMode::Interrupt));
      }
    }
  });

  std::thread publisher([&] {
    for (int i = 0; i < 300; ++i) {
      auto snapshot = readySnapshot(i % 2 == 0 ? ControllerState::Idle
                                               : ControllerState::Running);
      if (snapshot.ctrl == ControllerState::Running) {
        MotionStatus active;
        active.id = "active-" + std::to_string(i);
        active.state = MotionState::Running;
        snapshot.exec = active;
      }
      store.publishSnapshot(snapshot);
      HealthSnapshot health;
      health.state = ServiceHealth::Ready;
      health.mode = RuntimeMode::Sim;
      store.publishHealthSnapshot(health);
    }
  });

  std::thread consumer([&] {
    while (!done.load()) {
      (void)bridge.consumeNextCommand();
    }
    while (bridge.consumeNextCommand().has_value()) {
    }
  });

  std::thread reader([&] {
    for (int i = 0; i < 1000; ++i) {
      const auto snapshot = store.snapshot();
      if (snapshot.queue.n != snapshot.queue.ids.size() ||
          snapshot.queue.n > config.queue_limit) {
        violation.store(true);
      }
      (void)store.findRun("submit-" + std::to_string(i % 300));
      const auto health = store.health();
      if (health.err != ErrorCode::Ok) {
        violation.store(true);
      }
    }
  });

  submitter.join();
  publisher.join();
  reader.join();
  done.store(true);
  consumer.join();

  const auto snapshot = store.snapshot();
  REQUIRE(snapshot.queue.n == snapshot.queue.ids.size());
  REQUIRE(snapshot.queue.n <= config.queue_limit);
  REQUIRE_FALSE(violation.load());
}

}  // namespace agentic_et1_tracker
