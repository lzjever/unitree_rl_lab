#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <vector>

#include "agentic_et1_tracker/reference/reference_frame.hpp"
#include "agentic_et1_tracker/reference/reference_frame_json.hpp"

namespace agentic_et1_tracker {
namespace {

std::vector<float> seq(float start, std::size_t count) {
  std::vector<float> out;
  out.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    out.push_back(start + static_cast<float>(i));
  }
  return out;
}

TrkTrack makeTrack(std::size_t frames = 2) {
  TrkTrack track;
  track.metadata.frames = frames;
  track.metadata.fps = 50.0;
  track.metadata.duration_s = static_cast<double>(frames) / track.metadata.fps;

  track.joint_pos.shape = {frames, TrkSchema::kJointDim};
  track.joint_pos.frame_size = TrkSchema::kJointDim;
  track.joint_pos.values = seq(100.0F, frames * TrkSchema::kJointDim);

  track.joint_vel.shape = {frames, TrkSchema::kJointDim};
  track.joint_vel.frame_size = TrkSchema::kJointDim;
  track.joint_vel.values = seq(200.0F, frames * TrkSchema::kJointDim);

  track.body_pos_w.shape = {frames, TrkSchema::kBodyCount, 3};
  track.body_pos_w.frame_size = TrkSchema::kBodyCount * 3;
  track.body_pos_w.values = seq(300.0F, frames * TrkSchema::kBodyCount * 3);

  track.body_quat_w.shape = {frames, TrkSchema::kBodyCount, 4};
  track.body_quat_w.frame_size = TrkSchema::kBodyCount * 4;
  track.body_quat_w.values = seq(400.0F, frames * TrkSchema::kBodyCount * 4);

  track.body_lin_vel_w.shape = {frames, TrkSchema::kBodyCount, 3};
  track.body_lin_vel_w.frame_size = TrkSchema::kBodyCount * 3;
  track.body_lin_vel_w.values = seq(500.0F, frames * TrkSchema::kBodyCount * 3);

  track.body_ang_vel_w.shape = {frames, TrkSchema::kBodyCount, 3};
  track.body_ang_vel_w.frame_size = TrkSchema::kBodyCount * 3;
  track.body_ang_vel_w.values = seq(600.0F, frames * TrkSchema::kBodyCount * 3);

  track.left_foot_contact_state.shape = {frames};
  track.left_foot_contact_state.frame_size = 1;
  track.left_foot_contact_state.values = {0, 1};

  track.right_foot_contact_state.shape = {frames};
  track.right_foot_contact_state.frame_size = 1;
  track.right_foot_contact_state.values = {2, 0};

  track.ref_com_rel_navi.shape = {frames, 3};
  track.ref_com_rel_navi.frame_size = 3;
  track.ref_com_rel_navi.values = seq(700.0F, frames * 3);

  track.ref_com_vel_navi.shape = {frames, 3};
  track.ref_com_vel_navi.frame_size = 3;
  track.ref_com_vel_navi.values = seq(800.0F, frames * 3);
  return track;
}

}  // namespace

TEST_CASE("Reference frame snapshot copies raw trk frame with ET1 dimensions") {
  const TrkTrack track = makeTrack();
  const auto now = std::chrono::steady_clock::time_point{} + std::chrono::seconds(5);
  const auto snapshot = makeReferenceFrameSnapshot("ref-a", track, 1, now);

  REQUIRE(snapshot.has_value());
  REQUIRE(snapshot->active);
  REQUIRE(snapshot->id == "ref-a");
  REQUIRE(snapshot->frame == 1);
  REQUIRE(snapshot->frames == 2);
  REQUIRE(snapshot->time_s == 0.02);
  REQUIRE(snapshot->fps == 50.0);
  REQUIRE(snapshot->p.size() == TrkSchema::kBodyCount);
  REQUIRE(snapshot->q.size() == TrkSchema::kBodyCount);
  REQUIRE(track.joint_pos.frame_size == TrkSchema::kJointDim);
  REQUIRE(track.joint_vel.frame_size == TrkSchema::kJointDim);

  const std::size_t pos_offset = TrkSchema::kBodyCount * 3;
  REQUIRE(snapshot->p.at(0) == std::array<float, 3>{{300.0F + pos_offset,
                                                     301.0F + pos_offset,
                                                     302.0F + pos_offset}});
  const std::size_t quat_offset = TrkSchema::kBodyCount * 4;
  REQUIRE(snapshot->q.at(26) == std::array<float, 4>{{400.0F + quat_offset + 104,
                                                      400.0F + quat_offset + 105,
                                                      400.0F + quat_offset + 106,
                                                      400.0F + quat_offset + 107}});
  REQUIRE(snapshot->c == std::array<std::int64_t, 2>{{1, 0}});
  REQUIRE(snapshot->com == std::array<float, 3>{{703.0F, 704.0F, 705.0F}});
  REQUIRE(snapshot->comv == std::array<float, 3>{{803.0F, 804.0F, 805.0F}});
}

TEST_CASE("Reference frame snapshot rejects unexpected 27/26 dimensions") {
  TrkTrack track = makeTrack();
  track.joint_pos.frame_size = TrkSchema::kJointDim - 1;
  track.joint_pos.values.pop_back();

  REQUIRE_FALSE(makeReferenceFrameSnapshot("bad", track, 0).has_value());

  track = makeTrack();
  track.body_pos_w.frame_size = (TrkSchema::kBodyCount - 1) * 3;
  track.body_pos_w.values.resize(track.body_pos_w.frame_size * track.metadata.frames);

  REQUIRE_FALSE(makeReferenceFrameSnapshot("bad", track, 0).has_value());
}

TEST_CASE("Reference frame JSON emits inactive and active short schema") {
  const auto inactive = referenceFrameSnapshotJson(ReferenceFrameSnapshot{});
  REQUIRE(inactive.dump() == R"({"active":false,"ok":true})");

  const auto now = std::chrono::steady_clock::time_point{} + std::chrono::seconds(5);
  const auto snapshot = makeReferenceFrameSnapshot("ref-json", makeTrack(), 0, now);
  REQUIRE(snapshot.has_value());

  const auto body = referenceFrameSnapshotJson(*snapshot, now + std::chrono::milliseconds(17));
  REQUIRE(body.at("ok") == true);
  REQUIRE(body.at("active") == true);
  REQUIRE(body.at("schema") == "ET1REF1");
  REQUIRE(body.at("body_order") == "et1_27_v1");
  REQUIRE(body.at("id") == "ref-json");
  REQUIRE(body.at("frame") == 0);
  REQUIRE(body.at("frames") == 2);
  REQUIRE(body.at("stale_ms") == 17);
  REQUIRE(body.at("p").size() == TrkSchema::kBodyCount);
  REQUIRE(body.at("q").size() == TrkSchema::kBodyCount);
  REQUIRE(body.at("p").at(0).size() == 3);
  REQUIRE(body.at("q").at(0).size() == 4);
  REQUIRE(body.at("c").size() == 2);
  REQUIRE(body.at("com").size() == 3);
  REQUIRE(body.at("comv").size() == 3);
}

}  // namespace agentic_et1_tracker
