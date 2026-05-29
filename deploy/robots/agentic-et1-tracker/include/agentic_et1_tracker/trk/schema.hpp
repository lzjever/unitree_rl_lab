#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace agentic_et1_tracker {

enum class TrkDtype : std::uint32_t {
  Float32 = 1,
  Float64 = 2,
  Bool = 3,
  Int32 = 4,
  Int64 = 5,
  UInt8 = 6,
  Int8 = 7,
};

enum class TrkDtypeFamily {
  Float,
  Contact,
};

struct TrkRequiredArraySpec {
  std::string_view name;
  TrkDtypeFamily dtype_family{TrkDtypeFamily::Float};
  std::array<std::uint64_t, 3> trailing_shape{};
  std::uint32_t trailing_rank{0};
};

struct TrkLimits {
  std::uint32_t max_array_count{64};
  std::uint32_t max_name_len{128};
  std::uint32_t max_ndim{4};
  std::uint64_t max_single_array_bytes{256ULL * 1024ULL * 1024ULL};
  std::uint64_t max_total_payload_bytes{512ULL * 1024ULL * 1024ULL};
};

struct TrkSchema {
  static constexpr std::array<char, 8> kMagic{{'E', 'T', '1', 'T', 'R', 'K', '1', '\0'}};
  static constexpr std::uint32_t kVersion{1};
  static constexpr std::uint32_t kJointDim{26};
  static constexpr std::uint32_t kBodyCount{27};
  static constexpr double kDefaultFps{50.0};
  static constexpr TrkLimits kDefaultLimits{};

  static constexpr std::array<TrkRequiredArraySpec, 10> kRequiredArrays{{
      {"joint_pos", TrkDtypeFamily::Float, {kJointDim, 0, 0}, 1},
      {"joint_vel", TrkDtypeFamily::Float, {kJointDim, 0, 0}, 1},
      {"body_pos_w", TrkDtypeFamily::Float, {kBodyCount, 3, 0}, 2},
      {"body_quat_w", TrkDtypeFamily::Float, {kBodyCount, 4, 0}, 2},
      {"body_lin_vel_w", TrkDtypeFamily::Float, {kBodyCount, 3, 0}, 2},
      {"body_ang_vel_w", TrkDtypeFamily::Float, {kBodyCount, 3, 0}, 2},
      {"left_foot_contact_state", TrkDtypeFamily::Contact, {0, 0, 0}, 0},
      {"right_foot_contact_state", TrkDtypeFamily::Contact, {0, 0, 0}, 0},
      {"ref_com_rel_navi", TrkDtypeFamily::Float, {3, 0, 0}, 1},
      {"ref_com_vel_navi", TrkDtypeFamily::Float, {3, 0, 0}, 1},
  }};
};

constexpr std::uint64_t trkDtypeSize(TrkDtype dtype) {
  switch (dtype) {
    case TrkDtype::Float32:
      return 4;
    case TrkDtype::Float64:
      return 8;
    case TrkDtype::Bool:
      return 1;
    case TrkDtype::Int32:
      return 4;
    case TrkDtype::Int64:
      return 8;
    case TrkDtype::UInt8:
      return 1;
    case TrkDtype::Int8:
      return 1;
  }
  return 0;
}

constexpr bool trkIsKnownDtype(std::uint32_t raw) {
  return raw >= static_cast<std::uint32_t>(TrkDtype::Float32) &&
         raw <= static_cast<std::uint32_t>(TrkDtype::Int8);
}

constexpr bool trkDtypeAllowed(TrkDtype dtype, TrkDtypeFamily family) {
  switch (family) {
    case TrkDtypeFamily::Float:
      return dtype == TrkDtype::Float32 || dtype == TrkDtype::Float64;
    case TrkDtypeFamily::Contact:
      return dtype == TrkDtype::Int64 || dtype == TrkDtype::Int32 ||
             dtype == TrkDtype::UInt8 || dtype == TrkDtype::Int8;
  }
  return false;
}

}  // namespace agentic_et1_tracker
