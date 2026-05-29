#include "agentic_et1_tracker/control/fixstand.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace agentic_et1_tracker {
namespace {

FixStandConfigError error(const std::string& message) {
  return FixStandConfigError("fixstand config error: " + message);
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& err) {
    throw error(field + " has invalid value: " + err.what());
  }
}

YAML::Node requiredNode(const YAML::Node& root, const char* key, const std::string& field) {
  const YAML::Node node = root[key];
  if (!node) {
    throw error(field + " is required");
  }
  return node;
}

std::vector<double> readVector(const YAML::Node& node,
                               const std::string& field,
                               std::size_t expected_size,
                               bool require_non_negative) {
  if (!node || !node.IsSequence() || node.size() != expected_size) {
    std::ostringstream out;
    out << field << " must contain " << expected_size << " entries";
    throw error(out.str());
  }

  std::vector<double> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string element = field + "[" + std::to_string(i) + "]";
    const double value = scalarAs<double>(node[i], element);
    if (!std::isfinite(value)) {
      throw error(element + " must be finite");
    }
    if (require_non_negative && value < 0.0) {
      throw error(element + " must be non-negative");
    }
    values.push_back(value);
  }
  return values;
}

void validateConfig(const FixStandConfig& config) {
  if (config.kp.size() != kFixStandMotorCount ||
      config.kd.size() != kFixStandMotorCount ||
      config.target_q.size() != kFixStandMotorCount) {
    throw error("kp, kd, and target_q must contain 33 entries");
  }
  if (!std::isfinite(config.duration_s) || config.duration_s <= 0.0) {
    throw error("duration_s must be positive and finite");
  }
}

}  // namespace

FixStandConfig loadFixStandConfig(const std::filesystem::path& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root || !root.IsMap()) {
      throw error("root must be a map");
    }

    FixStandConfig config;
    config.kp =
        readVector(requiredNode(root, "kp", "kp"), "kp", kFixStandMotorCount, true);
    config.kd =
        readVector(requiredNode(root, "kd", "kd"), "kd", kFixStandMotorCount, true);

    const YAML::Node ts = requiredNode(root, "ts", "ts");
    if (!ts.IsSequence() || ts.size() != 2) {
      throw error("ts must contain [0, duration]");
    }
    const double t0 = scalarAs<double>(ts[0], "ts[0]");
    const double t1 = scalarAs<double>(ts[1], "ts[1]");
    if (!std::isfinite(t0) || !std::isfinite(t1) || t0 != 0.0 || t1 <= 0.0) {
      throw error("ts must contain [0, positive_duration]");
    }
    config.duration_s = t1;

    const YAML::Node qs = requiredNode(root, "qs", "qs");
    if (!qs.IsSequence() || qs.size() != 2) {
      throw error("qs must contain start and target profiles");
    }
    config.target_q = readVector(qs[1], "qs[1]", kFixStandMotorCount, false);
    validateConfig(config);
    return config;
  } catch (const FixStandConfigError&) {
    throw;
  } catch (const YAML::Exception& err) {
    throw error(err.what());
  }
}

LowCmdFrame makeFixStandLowCmdFrame(const FixStandConfig& config,
                                    const std::vector<float>& q,
                                    std::uint8_t expected_mode_machine) {
  validateConfig(config);
  if (q.size() != kFixStandMotorCount) {
    throw error("q must contain 33 entries");
  }

  LowCmdFrame frame;
  frame.mode_machine = expected_mode_machine;
  frame.mode_pr = 0;
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    if (!std::isfinite(q.at(i))) {
      throw error("q[" + std::to_string(i) + "] must be finite");
    }
    MotorCommand& motor = frame.motors.at(i);
    motor.mode = 1;
    motor.q = q.at(i);
    motor.dq = 0.0F;
    motor.kp = static_cast<float>(config.kp.at(i));
    motor.kd = static_cast<float>(config.kd.at(i));
    motor.tau = 0.0F;
  }
  return frame;
}

FixStandRunner::FixStandRunner(FixStandConfig config,
                               std::uint8_t expected_mode_machine,
                               double hz)
    : config_(std::move(config)), expected_mode_machine_(expected_mode_machine), hz_(hz) {
  validateConfig(config_);
  reset();
}

void FixStandRunner::reset() {
  q0_.clear();
  tick_ = 0;
  initialized_ = false;
}

LowCmdFrame FixStandRunner::step(const LowStateSample& low_state) {
  if (!initialized_) {
    q0_.reserve(kFixStandMotorCount);
    for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
      q0_.push_back(low_state.motors.at(i).q);
    }
    initialized_ = true;
    tick_ = 0;
  }

  const double duration_ticks = std::max(1.0, config_.duration_s * hz_);
  const double alpha = std::min(1.0, static_cast<double>(tick_) / duration_ticks);
  std::vector<float> q;
  q.reserve(kFixStandMotorCount);
  for (std::size_t i = 0; i < kFixStandMotorCount; ++i) {
    const float target = static_cast<float>(config_.target_q.at(i));
    q.push_back(q0_.at(i) + static_cast<float>(alpha) * (target - q0_.at(i)));
  }
  if (tick_ < static_cast<std::size_t>(duration_ticks)) {
    ++tick_;
  }
  return makeFixStandLowCmdFrame(config_, q, expected_mode_machine_);
}

}  // namespace agentic_et1_tracker
