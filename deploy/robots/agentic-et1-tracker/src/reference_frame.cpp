#include "agentic_et1_tracker/reference/reference_frame.hpp"

namespace agentic_et1_tracker {
namespace {

bool expectedDimensions(const TrkFrameView& frame) {
  return frame.joint_pos.size == TrkSchema::kJointDim &&
         frame.joint_vel.size == TrkSchema::kJointDim &&
         frame.body_pos_w.size == TrkSchema::kBodyCount * 3 &&
         frame.body_quat_w.size == TrkSchema::kBodyCount * 4 &&
         frame.left_foot_contact_state.size == 1 &&
         frame.right_foot_contact_state.size == 1 &&
         frame.ref_com_rel_navi.size == 3 &&
         frame.ref_com_vel_navi.size == 3;
}

}  // namespace

std::optional<ReferenceFrameSnapshot> makeReferenceFrameSnapshot(
    const std::string& id,
    const TrkTrack& track,
    std::size_t frame_index,
    std::chrono::steady_clock::time_point updated_at) {
  const std::optional<TrkFrameView> frame = track.frame(frame_index);
  if (!frame || !expectedDimensions(*frame)) {
    return std::nullopt;
  }

  ReferenceFrameSnapshot snapshot;
  snapshot.active = true;
  snapshot.id = id;
  snapshot.frame = frame_index;
  snapshot.frames = track.metadata.frames;
  snapshot.fps = track.metadata.fps;
  snapshot.time_s = snapshot.fps > 0.0 ? static_cast<double>(frame_index) / snapshot.fps : 0.0;
  snapshot.updated_at = updated_at;

  for (std::size_t i = 0; i < TrkSchema::kBodyCount; ++i) {
    snapshot.p.at(i) = {frame->body_pos_w.ptr[i * 3 + 0],
                        frame->body_pos_w.ptr[i * 3 + 1],
                        frame->body_pos_w.ptr[i * 3 + 2]};
    snapshot.q.at(i) = {frame->body_quat_w.ptr[i * 4 + 0],
                        frame->body_quat_w.ptr[i * 4 + 1],
                        frame->body_quat_w.ptr[i * 4 + 2],
                        frame->body_quat_w.ptr[i * 4 + 3]};
  }
  snapshot.c = {*frame->left_foot_contact_state.ptr,
                *frame->right_foot_contact_state.ptr};
  snapshot.com = {frame->ref_com_rel_navi.ptr[0],
                  frame->ref_com_rel_navi.ptr[1],
                  frame->ref_com_rel_navi.ptr[2]};
  snapshot.comv = {frame->ref_com_vel_navi.ptr[0],
                   frame->ref_com_vel_navi.ptr[1],
                   frame->ref_com_vel_navi.ptr[2]};
  return snapshot;
}

}  // namespace agentic_et1_tracker
