#include "agentic_et1_tracker/trk/loader.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <fstream>
#include <limits>
#include <utility>

#include "agentic_et1_tracker/trk/detail/scanner.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr std::size_t kJointPos = 0;
constexpr std::size_t kJointVel = 1;
constexpr std::size_t kBodyPosW = 2;
constexpr std::size_t kBodyQuatW = 3;
constexpr std::size_t kBodyLinVelW = 4;
constexpr std::size_t kBodyAngVelW = 5;
constexpr std::size_t kLeftFootContact = 6;
constexpr std::size_t kRightFootContact = 7;
constexpr std::size_t kRefComRelNavi = 8;
constexpr std::size_t kRefComVelNavi = 9;

struct LoadError {
  TrkValidationCode code{TrkValidationCode::Ok};
  std::string message;
};

TrkLoadResult fail(TrkValidationCode code, std::string message = {}) {
  TrkLoadResult result;
  result.code = code;
  result.message = std::move(message);
  return result;
}

TrkLoadResult fail(LoadError error) {
  return fail(error.code, std::move(error.message));
}

std::uint64_t frameElementCount(const std::vector<std::uint64_t>& shape) {
  std::uint64_t count = 1;
  for (std::size_t i = 1; i < shape.size(); ++i) {
    count *= shape[i];
  }
  return count;
}

std::optional<LoadError> prepareFloatArray(const trk_detail::TrkArrayDescriptor& descriptor,
                                           TrkFloatArray& out) {
  if (descriptor.element_count >
      static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return LoadError{TrkValidationCode::ShapeOverflow, "array is too large"};
  }
  const auto frame_size = frameElementCount(descriptor.shape);
  if (frame_size > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return LoadError{TrkValidationCode::ShapeOverflow, "frame is too large"};
  }
  out.shape = descriptor.shape;
  out.frame_size = static_cast<std::size_t>(frame_size);
  out.values.resize(static_cast<std::size_t>(descriptor.element_count));
  return std::nullopt;
}

std::optional<LoadError> prepareContactArray(
    const trk_detail::TrkArrayDescriptor& descriptor,
    TrkContactArray& out) {
  if (descriptor.element_count >
      static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return LoadError{TrkValidationCode::ShapeOverflow, "array is too large"};
  }
  const auto frame_size = frameElementCount(descriptor.shape);
  if (frame_size > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return LoadError{TrkValidationCode::ShapeOverflow, "frame is too large"};
  }
  out.shape = descriptor.shape;
  out.frame_size = static_cast<std::size_t>(frame_size);
  out.values.resize(static_cast<std::size_t>(descriptor.element_count));
  return std::nullopt;
}

std::optional<LoadError> seekPayload(std::ifstream& in,
                                     const trk_detail::TrkArrayDescriptor& descriptor) {
  in.clear();
  in.seekg(static_cast<std::streamoff>(descriptor.payload_offset), std::ios::beg);
  if (!in) {
    return LoadError{TrkValidationCode::ReadFailed, "failed to seek payload"};
  }
  return std::nullopt;
}

std::optional<LoadError> readExact(std::ifstream& in, char* out, std::uint64_t byte_count) {
  in.read(out, static_cast<std::streamsize>(byte_count));
  if (!in) {
    return LoadError{TrkValidationCode::ReadFailed, "failed to read payload"};
  }
  return std::nullopt;
}

std::optional<LoadError> readFloatArray(std::ifstream& in,
                                        const trk_detail::TrkArrayDescriptor& descriptor,
                                        TrkFloatArray& out) {
  if (auto error = prepareFloatArray(descriptor, out)) {
    return error;
  }
  if (auto error = seekPayload(in, descriptor)) {
    return error;
  }

  if (descriptor.dtype == TrkDtype::Float32) {
    return readExact(in, reinterpret_cast<char*>(out.values.data()), descriptor.byte_count);
  }

  if (descriptor.dtype != TrkDtype::Float64) {
    return LoadError{TrkValidationCode::BadRequiredDtype, "expected float payload"};
  }

  constexpr std::size_t kChunk = 4096;
  std::array<double, kChunk> chunk{};
  std::size_t written = 0;
  std::size_t remaining = out.values.size();
  while (remaining > 0) {
    const std::size_t count = std::min(remaining, chunk.size());
    in.read(reinterpret_cast<char*>(chunk.data()),
            static_cast<std::streamsize>(count * sizeof(double)));
    if (!in) {
      return LoadError{TrkValidationCode::ReadFailed, "failed to read float64 payload"};
    }
    for (std::size_t i = 0; i < count; ++i) {
      out.values[written + i] = static_cast<float>(chunk[i]);
    }
    written += count;
    remaining -= count;
  }
  return std::nullopt;
}

template <typename SourceT>
std::optional<LoadError> readContactTyped(std::ifstream& in,
                                          const trk_detail::TrkArrayDescriptor& descriptor,
                                          TrkContactArray& out) {
  constexpr std::size_t kChunk = 4096;
  std::array<SourceT, kChunk> chunk{};
  std::size_t written = 0;
  std::size_t remaining = out.values.size();
  while (remaining > 0) {
    const std::size_t count = std::min(remaining, chunk.size());
    in.read(reinterpret_cast<char*>(chunk.data()),
            static_cast<std::streamsize>(count * sizeof(SourceT)));
    if (!in) {
      return LoadError{TrkValidationCode::ReadFailed, "failed to read contact payload"};
    }
    for (std::size_t i = 0; i < count; ++i) {
      const auto value = static_cast<std::int64_t>(chunk[i]);
      if (value != 0 && value != 1 && value != 2) {
        return LoadError{TrkValidationCode::InvalidContactValue,
                         "contact value must be 0, 1, or 2"};
      }
      out.values[written + i] = value;
    }
    written += count;
    remaining -= count;
  }
  return std::nullopt;
}

std::optional<LoadError> readContactArray(std::ifstream& in,
                                          const trk_detail::TrkArrayDescriptor& descriptor,
                                          TrkContactArray& out) {
  if (auto error = prepareContactArray(descriptor, out)) {
    return error;
  }
  if (auto error = seekPayload(in, descriptor)) {
    return error;
  }

  switch (descriptor.dtype) {
    case TrkDtype::Int64:
      return readContactTyped<std::int64_t>(in, descriptor, out);
    case TrkDtype::Int32:
      return readContactTyped<std::int32_t>(in, descriptor, out);
    case TrkDtype::UInt8:
      return readContactTyped<std::uint8_t>(in, descriptor, out);
    case TrkDtype::Int8:
      return readContactTyped<std::int8_t>(in, descriptor, out);
    default:
      return LoadError{TrkValidationCode::BadRequiredDtype, "expected contact payload"};
  }
}

template <typename ArrayT, typename T>
TrkArrayView<T> frameView(const ArrayT& array, std::size_t index) {
  return {array.values.data() + index * array.frame_size, array.frame_size};
}

}  // namespace

std::optional<TrkFrameView> TrkTrack::frame(std::size_t index) const {
  if (index >= metadata.frames) {
    return std::nullopt;
  }

  TrkFrameView view;
  view.joint_pos = frameView<TrkFloatArray, float>(joint_pos, index);
  view.joint_vel = frameView<TrkFloatArray, float>(joint_vel, index);
  view.body_pos_w = frameView<TrkFloatArray, float>(body_pos_w, index);
  view.body_quat_w = frameView<TrkFloatArray, float>(body_quat_w, index);
  view.body_lin_vel_w = frameView<TrkFloatArray, float>(body_lin_vel_w, index);
  view.body_ang_vel_w = frameView<TrkFloatArray, float>(body_ang_vel_w, index);
  view.left_foot_contact_state =
      frameView<TrkContactArray, std::int64_t>(left_foot_contact_state, index);
  view.right_foot_contact_state =
      frameView<TrkContactArray, std::int64_t>(right_foot_contact_state, index);
  view.ref_com_rel_navi = frameView<TrkFloatArray, float>(ref_com_rel_navi, index);
  view.ref_com_vel_navi = frameView<TrkFloatArray, float>(ref_com_vel_navi, index);
  return view;
}

TrkLoader::TrkLoader(TrkValidationConfig config) : config_(std::move(config)) {}

TrkLoadResult TrkLoader::load(const std::filesystem::path& path) const {
  try {
    auto scan = trk_detail::scanTrkFile(path, config_);
    if (!scan.ok()) {
      return fail(scan.validation.code, scan.validation.message);
    }

    std::ifstream& in = scan.file;
    if (!in) {
      return fail(TrkValidationCode::FileOpenFailed, "scanned track stream is not open");
    }

    TrkTrack track;
    track.metadata = scan.validation.metadata;

    if (auto error = readFloatArray(in, scan.required_arrays[kJointPos], track.joint_pos)) {
      return fail(std::move(*error));
    }
    if (auto error = readFloatArray(in, scan.required_arrays[kJointVel], track.joint_vel)) {
      return fail(std::move(*error));
    }
    if (auto error = readFloatArray(in, scan.required_arrays[kBodyPosW], track.body_pos_w)) {
      return fail(std::move(*error));
    }
    if (auto error = readFloatArray(in, scan.required_arrays[kBodyQuatW], track.body_quat_w)) {
      return fail(std::move(*error));
    }
    if (auto error =
            readFloatArray(in, scan.required_arrays[kBodyLinVelW], track.body_lin_vel_w)) {
      return fail(std::move(*error));
    }
    if (auto error =
            readFloatArray(in, scan.required_arrays[kBodyAngVelW], track.body_ang_vel_w)) {
      return fail(std::move(*error));
    }
    if (auto error = readContactArray(in, scan.required_arrays[kLeftFootContact],
                                      track.left_foot_contact_state)) {
      return fail(std::move(*error));
    }
    if (auto error = readContactArray(in, scan.required_arrays[kRightFootContact],
                                      track.right_foot_contact_state)) {
      return fail(std::move(*error));
    }
    if (auto error =
            readFloatArray(in, scan.required_arrays[kRefComRelNavi], track.ref_com_rel_navi)) {
      return fail(std::move(*error));
    }
    if (auto error =
            readFloatArray(in, scan.required_arrays[kRefComVelNavi], track.ref_com_vel_navi)) {
      return fail(std::move(*error));
    }

    TrkLoadResult result;
    result.code = TrkValidationCode::Ok;
    result.track = std::move(track);
    return result;
  } catch (const std::exception& e) {
    return fail(TrkValidationCode::ReadFailed, e.what());
  } catch (...) {
    return fail(TrkValidationCode::ReadFailed, "unexpected loader failure");
  }
}

}  // namespace agentic_et1_tracker
