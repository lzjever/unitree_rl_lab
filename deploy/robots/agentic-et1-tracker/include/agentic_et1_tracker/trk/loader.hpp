#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "agentic_et1_tracker/trk/validator.hpp"

namespace agentic_et1_tracker {

template <typename T>
struct TrkArrayView {
  const T* ptr{nullptr};
  std::size_t size{0};
};

struct TrkFloatArray {
  std::vector<float> values;
  std::vector<std::uint64_t> shape;
  std::size_t frame_size{0};
};

struct TrkContactArray {
  std::vector<std::int64_t> values;
  std::vector<std::uint64_t> shape;
  std::size_t frame_size{0};
};

struct TrkFrameView {
  TrkArrayView<float> joint_pos;
  TrkArrayView<float> joint_vel;
  TrkArrayView<float> body_pos_w;
  TrkArrayView<float> body_quat_w;
  TrkArrayView<float> body_lin_vel_w;
  TrkArrayView<float> body_ang_vel_w;
  TrkArrayView<std::int64_t> left_foot_contact_state;
  TrkArrayView<std::int64_t> right_foot_contact_state;
  TrkArrayView<float> ref_com_rel_navi;
  TrkArrayView<float> ref_com_vel_navi;
};

struct TrkTrack {
  TrkMetadata metadata;
  TrkFloatArray joint_pos;
  TrkFloatArray joint_vel;
  TrkFloatArray body_pos_w;
  TrkFloatArray body_quat_w;
  TrkFloatArray body_lin_vel_w;
  TrkFloatArray body_ang_vel_w;
  TrkContactArray left_foot_contact_state;
  TrkContactArray right_foot_contact_state;
  TrkFloatArray ref_com_rel_navi;
  TrkFloatArray ref_com_vel_navi;

  std::optional<TrkFrameView> frame(std::size_t index) const;
};

struct TrkLoadResult {
  TrkValidationCode code{TrkValidationCode::Ok};
  std::optional<TrkTrack> track;
  std::string message;

  bool ok() const { return code == TrkValidationCode::Ok && track.has_value(); }
};

class TrkLoader {
 public:
  explicit TrkLoader(TrkValidationConfig config);

  TrkLoadResult load(const std::filesystem::path& path) const;

 private:
  TrkValidationConfig config_;
};

}  // namespace agentic_et1_tracker
