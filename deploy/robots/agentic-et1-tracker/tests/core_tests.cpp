#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <set>
#include <string>
#include <vector>

#include "agentic_et1_tracker/core/command_mailbox.hpp"
#include "agentic_et1_tracker/core/id_generator.hpp"
#include "agentic_et1_tracker/core/motion_queue.hpp"
#include "agentic_et1_tracker/core/recent_ring.hpp"
#include "agentic_et1_tracker/core/status.hpp"

namespace agentic_et1_tracker {
namespace {

MotionRequest request(std::string id) {
  MotionRequest out;
  out.id = std::move(id);
  out.path = "/tmp/" + out.id + ".trk";
  return out;
}

}  // namespace

TEST_CASE("MotionQueue preserves FIFO order and enforces default limit") {
  MotionQueue queue;

  for (int i = 0; i < 8; ++i) {
    const auto result = queue.enqueue(request("id" + std::to_string(i)));
    REQUIRE(result.code == ErrorCode::Ok);
    REQUIRE(result.queue_size == static_cast<std::size_t>(i + 1));
  }

  const auto full = queue.enqueue(request("overflow"));
  REQUIRE(full.code == ErrorCode::QueueFull);
  REQUIRE(queue.size() == 8);

  for (int i = 0; i < 8; ++i) {
    auto next = queue.popNext();
    REQUIRE(next.has_value());
    REQUIRE(next->id == "id" + std::to_string(i));
    REQUIRE(next->state == MotionState::Queued);
  }
  REQUIRE(queue.popNext() == std::nullopt);
}

TEST_CASE("MotionQueue supports a configurable limit") {
  MotionQueue queue(2);

  REQUIRE(queue.enqueue(request("a")).code == ErrorCode::Ok);
  REQUIRE(queue.enqueue(request("b")).code == ErrorCode::Ok);
  REQUIRE(queue.enqueue(request("c")).code == ErrorCode::QueueFull);
  REQUIRE(queue.limit() == 2);
  REQUIRE(queue.queuedIds() == std::vector<std::string>{"a", "b"});
}

TEST_CASE("RecentRing can look up done canceled and stopped requests") {
  RecentRing recent(3);
  auto done = request("done");
  done.state = MotionState::Done;
  auto canceled = request("canceled");
  canceled.state = MotionState::Canceled;
  auto stopped = request("stopped");
  stopped.state = MotionState::Stopped;

  recent.push(done);
  recent.push(canceled);
  recent.push(stopped);

  REQUIRE(recent.find("done")->state == MotionState::Done);
  REQUIRE(recent.find("canceled")->state == MotionState::Canceled);
  REQUIRE(recent.find("stopped")->state == MotionState::Stopped);

  auto evict = request("evict");
  evict.state = MotionState::Done;
  recent.push(evict);

  REQUIRE_FALSE(recent.find("done").has_value());
  REQUIRE(recent.find("evict")->state == MotionState::Done);
}

TEST_CASE("Interrupt clears waiting queue, records canceled requests, and queues the new head") {
  MotionQueue queue(8, 8);
  REQUIRE(queue.enqueue(request("waiting-a")).code == ErrorCode::Ok);
  REQUIRE(queue.enqueue(request("waiting-b")).code == ErrorCode::Ok);

  MotionRequest active = request("active");
  active.state = MotionState::Running;

  const auto result = queue.interruptWith(request("urgent"));

  REQUIRE(result.code == ErrorCode::Ok);
  REQUIRE(result.canceled == 2);
  REQUIRE(queue.queuedIds() == std::vector<std::string>{"urgent"});
  REQUIRE(queue.findRecent("waiting-a")->state == MotionState::Canceled);
  REQUIRE(queue.findRecent("waiting-a")->stop_reason == StopReason::Interrupt);
  REQUIRE(queue.findRecent("waiting-b")->state == MotionState::Canceled);
  REQUIRE(queue.findQueued("urgent")->state == MotionState::Queued);

  REQUIRE(active.state == MotionState::Running);
  REQUIRE(active.id == "active");
}

TEST_CASE("Stop clears queued requests, records cancellations, and is idempotent") {
  MotionQueue queue(8, 8);
  REQUIRE(queue.enqueue(request("queued-a")).code == ErrorCode::Ok);
  REQUIRE(queue.enqueue(request("queued-b")).code == ErrorCode::Ok);

  const auto first = queue.stopQueued();
  REQUIRE(first.canceled == 2);
  REQUIRE(queue.empty());
  REQUIRE(queue.findRecent("queued-a")->state == MotionState::Canceled);
  REQUIRE(queue.findRecent("queued-a")->stop_reason == StopReason::Stop);
  REQUIRE(queue.findRecent("queued-b")->state == MotionState::Canceled);

  const auto second = queue.stopQueued();
  REQUIRE(second.canceled == 0);
  REQUIRE(second.ids.empty());
  REQUIRE(queue.empty());
}

TEST_CASE("CommandMailbox consumes stop before interrupt before queue in one tick") {
  CommandMailbox mailbox;
  mailbox.submitQueue(request("queued"));
  mailbox.submitInterrupt(request("interrupt"));
  mailbox.submitStop();
  mailbox.submitQueue(request("queued-2"));

  auto first = mailbox.consumeNext();
  REQUIRE(first.has_value());
  REQUIRE(first->kind == CommandKind::Stop);

  auto second = mailbox.consumeNext();
  REQUIRE(second.has_value());
  REQUIRE(second->kind == CommandKind::Interrupt);
  REQUIRE(second->request.id == "interrupt");

  auto third = mailbox.consumeNext();
  REQUIRE(third.has_value());
  REQUIRE(third->kind == CommandKind::Queue);
  REQUIRE(third->request.id == "queued");

  auto fourth = mailbox.consumeNext();
  REQUIRE(fourth.has_value());
  REQUIRE(fourth->kind == CommandKind::Queue);
  REQUIRE(fourth->request.id == "queued-2");

  REQUIRE_FALSE(mailbox.consumeNext().has_value());
}

TEST_CASE("Progress follows PRD frame plus one contract") {
  REQUIRE(computeProgress(0, 10, MotionState::Running) == 1.0 / 10.0);
  REQUIRE(computeProgress(8, 10, MotionState::Running) == 9.0 / 10.0);
  REQUIRE(computeProgress(9, 10, MotionState::Running) == 1.0);
  REQUIRE(computeProgress(100, 10, MotionState::Running) == 1.0);
  REQUIRE(computeProgress(0, 1, MotionState::Running) == 1.0);

  REQUIRE(computeProgress(0, 10, MotionState::Queued) == 0.0);
  REQUIRE(computeProgress(9, 10, MotionState::Queued) == 0.0);
  REQUIRE(computeProgress(9, 10, MotionState::Canceled) == 0.0);
  REQUIRE(computeProgress(9, 10, MotionState::Failed) == 0.0);
  REQUIRE(computeProgress(0, 10, MotionState::Done) == 1.0);
  REQUIRE(computeProgress(0, 0, MotionState::Done) == 1.0);
  REQUIRE(computeProgress(0, 0, MotionState::Running) == 0.0);
}

TEST_CASE("ErrorInfo exposes stable code and next action contracts") {
  const auto full = errorInfo(ErrorCode::QueueFull);
  REQUIRE(toString(full.code) == "QUEUE_FULL");
  REQUIRE(full.retryable);
  REQUIRE(full.next == NextAction::Status);
  REQUIRE(toString(full.next) == "status");
}

TEST_CASE("ShortIdGenerator emits unique 8 to 10 character base62 ids") {
  ShortIdGenerator generator;
  std::set<std::string> ids;

  for (int i = 0; i < 10000; ++i) {
    const std::string id = generator.generate();
    REQUIRE(id.size() >= 8);
    REQUIRE(id.size() <= 10);
    REQUIRE(ShortIdGenerator::isBase62Id(id));
    REQUIRE(ids.insert(id).second);
  }
}

}  // namespace agentic_et1_tracker
