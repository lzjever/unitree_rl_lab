#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "agentic_et1_tracker/core/tracker_controller.hpp"

namespace agentic_et1_tracker {
namespace {

Readiness readyState() {
  Readiness readiness;
  readiness.service_initialized = true;
  readiness.robot_connected = true;
  readiness.lowstate_fresh = true;
  readiness.mode_machine_ok = true;
  readiness.policy_ready = true;
  return readiness;
}

TrackMetadata track(std::size_t frames = 3) {
  TrackMetadata metadata;
  metadata.frames = frames;
  metadata.duration_s = frames == 0 ? 0.0 : static_cast<double>(frames - 1) / 50.0;
  return metadata;
}

ExecuteRequest executeRequest(std::string id,
                              MotionMode mode = MotionMode::Queue,
                              std::size_t frames = 3) {
  ExecuteRequest request;
  request.id = std::move(id);
  request.path = "/tmp/" + request.id + ".trk";
  request.mode = mode;
  request.track = track(frames);
  return request;
}

TrackerController readyController(std::size_t queue_limit = 8) {
  TrackerControllerConfig config;
  config.queue_limit = queue_limit;
  TrackerController controller(config);
  controller.setReadiness(readyState());
  return controller;
}

}  // namespace

TEST_CASE("TrackerController execute rejects not-ready and invalid requests without enqueue") {
  struct Case {
    const char* name;
    Readiness readiness;
    ErrorCode expected;
  };

  auto not_initialized = readyState();
  not_initialized.service_initialized = false;
  auto disconnected = readyState();
  disconnected.robot_connected = false;
  auto stale = readyState();
  stale.lowstate_fresh = false;
  auto mode_mismatch = readyState();
  mode_mismatch.mode_machine_ok = false;
  auto model_missing = readyState();
  model_missing.policy_ready = false;
  auto faulted = readyState();
  faulted.fault = true;
  faulted.block = "safety_limit";
  faulted.fault_code = ErrorCode::SafetyLimitTriggered;

  const std::vector<Case> cases{
      {"not initialized", not_initialized, ErrorCode::ServiceNotReady},
      {"disconnected", disconnected, ErrorCode::RobotDisconnected},
      {"stale lowstate", stale, ErrorCode::RobotNotReady},
      {"mode mismatch", mode_mismatch, ErrorCode::RobotNotReady},
      {"model missing", model_missing, ErrorCode::ModelNotReady},
      {"fault", faulted, ErrorCode::SafetyLimitTriggered},
  };

  for (const auto& item : cases) {
    TrackerController controller;
    controller.setReadiness(item.readiness);

    const auto result = controller.execute(executeRequest(item.name));

    CAPTURE(item.name);
    REQUIRE(result.code == item.expected);
    REQUIRE(controller.status().queue.n == 0);
    REQUIRE(controller.findRun(item.name).code == ErrorCode::RunNotFound);
  }

  {
    auto controller = readyController();
    auto request = executeRequest("bad-trk");
    request.validation_error = ErrorCode::TrkValidationFailed;

    const auto result = controller.execute(request);

    REQUIRE(result.code == ErrorCode::TrkValidationFailed);
    REQUIRE(controller.status().queue.n == 0);
    REQUIRE(controller.findRun("bad-trk").code == ErrorCode::RunNotFound);
  }

  {
    auto controller = readyController(1);
    REQUIRE(controller.execute(executeRequest("queued")).code == ErrorCode::Ok);

    const auto full = controller.execute(executeRequest("overflow"));

    REQUIRE(full.code == ErrorCode::QueueFull);
    REQUIRE(controller.status().queue.ids == std::vector<std::string>{"queued"});
    REQUIRE(controller.findRun("overflow").code == ErrorCode::RunNotFound);
  }
}

TEST_CASE("TrackerController queues while running and starts the next run after active done") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active", MotionMode::Queue, 1)).code ==
          ErrorCode::Ok);
  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Running);
  REQUIRE(controller.status().exec->id == "active");

  REQUIRE(controller.execute(executeRequest("next")).code == ErrorCode::Ok);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"next"});

  controller.tick();
  REQUIRE(controller.findRun("active").run->state == MotionState::Done);
  REQUIRE(controller.status().ctrl == ControllerState::Idle);
  REQUIRE_FALSE(controller.status().exec.has_value());
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"next"});

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Running);
  REQUIRE(controller.status().exec->id == "next");
}

TEST_CASE("TrackerController accepts queue while stopping but waits for stop-to-idle") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active")).code == ErrorCode::Ok);
  controller.tick();

  const auto stopped = controller.stop();
  REQUIRE(stopped.code == ErrorCode::Ok);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().stop_reason == StopReason::Stop);

  REQUIRE(controller.execute(executeRequest("queued-during-stop")).code == ErrorCode::Ok);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"queued-during-stop"});
  REQUIRE_FALSE(controller.status().exec.has_value());

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Idle);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"queued-during-stop"});
  REQUIRE_FALSE(controller.status().exec.has_value());

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Running);
  REQUIRE(controller.status().exec->id == "queued-during-stop");
}

TEST_CASE("TrackerController interrupt during stop keeps stop reason and replaces queue") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active")).code == ErrorCode::Ok);
  controller.tick();

  const auto stopped = controller.stop();
  REQUIRE(stopped.code == ErrorCode::Ok);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().stop_reason == StopReason::Stop);
  REQUIRE(controller.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(controller.findRun("active").run->stop_reason == StopReason::Stop);

  REQUIRE(controller.execute(executeRequest("waiting-during-stop")).code == ErrorCode::Ok);
  const auto interrupted =
      controller.execute(executeRequest("urgent-during-stop", MotionMode::Interrupt));

  REQUIRE(interrupted.code == ErrorCode::Ok);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().stop_reason == StopReason::Stop);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"urgent-during-stop"});
  REQUIRE_FALSE(controller.status().exec.has_value());

  const auto waiting = controller.findRun("waiting-during-stop");
  REQUIRE(waiting.code == ErrorCode::Ok);
  REQUIRE(waiting.run->state == MotionState::Canceled);
  REQUIRE(waiting.run->stop_reason == StopReason::Interrupt);
  REQUIRE(controller.findRun("urgent-during-stop").run->state == MotionState::Queued);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Idle);
  REQUIRE(controller.status().stop_reason == StopReason::None);
  REQUIRE(controller.findRun("urgent-during-stop").run->state == MotionState::Queued);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Running);
  REQUIRE(controller.status().exec->id == "urgent-during-stop");
}

TEST_CASE("TrackerController interrupt stops active, cancels waiting, and waits for idle tick") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active")).code == ErrorCode::Ok);
  controller.tick();
  REQUIRE(controller.execute(executeRequest("waiting")).code == ErrorCode::Ok);

  const auto interrupted =
      controller.execute(executeRequest("urgent", MotionMode::Interrupt));

  REQUIRE(interrupted.code == ErrorCode::Ok);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().stop_reason == StopReason::Interrupt);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"urgent"});
  REQUIRE_FALSE(controller.status().exec.has_value());

  const auto active = controller.findRun("active");
  REQUIRE(active.code == ErrorCode::Ok);
  REQUIRE(active.run->state == MotionState::Stopped);
  REQUIRE(active.run->stop_reason == StopReason::Interrupt);
  const auto waiting = controller.findRun("waiting");
  REQUIRE(waiting.code == ErrorCode::Ok);
  REQUIRE(waiting.run->state == MotionState::Canceled);
  REQUIRE(waiting.run->stop_reason == StopReason::Interrupt);
  REQUIRE(controller.findRun("urgent").run->state == MotionState::Queued);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Idle);
  REQUIRE(controller.status().stop_reason == StopReason::None);
  REQUIRE(controller.findRun("urgent").run->state == MotionState::Queued);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Running);
  REQUIRE(controller.status().exec->id == "urgent");
}

TEST_CASE("TrackerController interrupt during interrupt stop replaces pending head") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active")).code == ErrorCode::Ok);
  controller.tick();
  REQUIRE(controller.execute(executeRequest("waiting")).code == ErrorCode::Ok);

  REQUIRE(controller.execute(executeRequest("urgent-a", MotionMode::Interrupt)).code ==
          ErrorCode::Ok);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().stop_reason == StopReason::Interrupt);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"urgent-a"});

  REQUIRE(controller.execute(executeRequest("urgent-b", MotionMode::Interrupt)).code ==
          ErrorCode::Ok);

  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().stop_reason == StopReason::Interrupt);
  REQUIRE(controller.status().queue.ids == std::vector<std::string>{"urgent-b"});
  const auto replaced = controller.findRun("urgent-a");
  REQUIRE(replaced.code == ErrorCode::Ok);
  REQUIRE(replaced.run->state == MotionState::Canceled);
  REQUIRE(replaced.run->stop_reason == StopReason::Interrupt);
  REQUIRE(controller.findRun("urgent-b").run->state == MotionState::Queued);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Idle);
  REQUIRE(controller.findRun("urgent-b").run->state == MotionState::Queued);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Running);
  REQUIRE(controller.status().exec->id == "urgent-b");
}

TEST_CASE("TrackerController stop is idempotent and eventually returns idle with no exec") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active")).code == ErrorCode::Ok);
  controller.tick();
  REQUIRE(controller.execute(executeRequest("queued")).code == ErrorCode::Ok);

  const auto first = controller.stop();
  const auto second = controller.stop();

  REQUIRE(first.code == ErrorCode::Ok);
  REQUIRE(first.cleared == 1);
  REQUIRE(second.code == ErrorCode::Ok);
  REQUIRE(second.cleared == 0);
  REQUIRE(controller.status().ctrl == ControllerState::Stopping);
  REQUIRE(controller.status().queue.n == 0);
  REQUIRE(controller.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(controller.findRun("active").run->stop_reason == StopReason::Stop);
  REQUIRE(controller.findRun("queued").run->state == MotionState::Canceled);
  REQUIRE(controller.findRun("queued").run->stop_reason == StopReason::Stop);

  controller.tick();
  REQUIRE(controller.status().ctrl == ControllerState::Idle);
  REQUIRE_FALSE(controller.status().exec.has_value());
  REQUIRE(controller.status().queue.n == 0);

  const auto third = controller.stop();
  REQUIRE(third.code == ErrorCode::Ok);
  REQUIRE(controller.status().queue.n == 0);
}

TEST_CASE("TrackerController bad orientation fault blocks execute and reports status block") {
  auto readiness = readyState();
  readiness.fault = true;
  readiness.block = "bad_orientation";
  readiness.fault_code = ErrorCode::RobotBadOrientation;

  TrackerController controller;
  controller.setReadiness(readiness);

  const auto result = controller.execute(executeRequest("unsafe"));

  REQUIRE(result.code == ErrorCode::RobotBadOrientation);
  REQUIRE(controller.status().ready == false);
  REQUIRE(controller.status().robot == RobotState::Fault);
  REQUIRE(controller.status().ctrl == ControllerState::Fault);
  REQUIRE(controller.status().block == "bad_orientation");
  REQUIRE(controller.status().queue.n == 0);
}

TEST_CASE("TrackerController reports PRD block strings for readiness gates") {
  struct Case {
    Readiness readiness;
    std::string block;
  };

  auto lowstate = readyState();
  lowstate.lowstate_fresh = false;
  auto mode = readyState();
  mode.mode_machine_ok = false;
  auto policy = readyState();
  policy.policy_ready = false;

  for (const auto& item : std::vector<Case>{
           {lowstate, "lowstate_timeout"},
           {mode, "mode_machine_mismatch"},
           {policy, "policy_not_loaded"},
       }) {
    TrackerController controller;
    controller.setReadiness(item.readiness);
    REQUIRE(controller.status().block == item.block);
  }
}

TEST_CASE("TrackerController id lookup covers active queued recent and not found") {
  auto controller = readyController();
  REQUIRE(controller.execute(executeRequest("active")).code == ErrorCode::Ok);
  controller.tick();
  REQUIRE(controller.execute(executeRequest("queued")).code == ErrorCode::Ok);

  auto active = controller.findRun("active");
  REQUIRE(active.code == ErrorCode::Ok);
  REQUIRE(active.run->state == MotionState::Running);
  REQUIRE(active.run->frames == 3);
  REQUIRE(active.run->progress == 1.0 / 3.0);

  auto queued = controller.findRun("queued");
  REQUIRE(queued.code == ErrorCode::Ok);
  REQUIRE(queued.run->state == MotionState::Queued);
  REQUIRE(queued.run->progress == 0.0);

  controller.stop();
  REQUIRE(controller.findRun("active").run->state == MotionState::Stopped);
  REQUIRE(controller.findRun("queued").run->state == MotionState::Canceled);
  controller.tick();

  REQUIRE(controller.execute(executeRequest("done", MotionMode::Queue, 1)).code == ErrorCode::Ok);
  controller.tick();
  REQUIRE(controller.findRun("done").run->state == MotionState::Running);
  controller.tick();
  auto done = controller.findRun("done");
  REQUIRE(done.code == ErrorCode::Ok);
  REQUIRE(done.run->state == MotionState::Done);
  REQUIRE(done.run->progress == 1.0);

  const auto missing = controller.findRun("missing");
  REQUIRE(missing.code == ErrorCode::RunNotFound);
  REQUIRE_FALSE(missing.run.has_value());
}

}  // namespace agentic_et1_tracker
