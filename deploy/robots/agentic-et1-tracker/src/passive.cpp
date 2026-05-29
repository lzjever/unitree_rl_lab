#include "agentic_et1_tracker/control/passive.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace agentic_et1_tracker {
namespace {

PassiveConfigError error(const std::string& message) {
  return PassiveConfigError("passive config error: " + message);
}

template <typename T>
T scalarAs(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception& err) {
    throw error(field + " has invalid value: " + err.what());
  }
}

YAML::Node requiredNode(const YAML::Node& root, const char* key) {
  const YAML::Node node = root[key];
  if (!node) {
    throw error(std::string(key) + " is required");
  }
  return node;
}

std::vector<int> readMode(const YAML::Node& node) {
  if (!node || !node.IsSequence() || node.size() != kFixStandMotorCount) {
    std::ostringstream out;
    out << "mode must contain " << kFixStandMotorCount << " entries";
    throw error(out.str());
  }

  std::vector<int> values;
  values.reserve(kFixStandMotorCount);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string field = "mode[" + std::to_string(i) + "]";
    const int value = scalarAs<int>(node[i], field);
    if (value < 0 || value > 255) {
      throw error(field + " must fit in uint8");
    }
    values.push_back(value);
  }
  return values;
}

std::vector<double> readKd(const YAML::Node& node) {
  if (!node || !node.IsSequence() || node.size() != kFixStandMotorCount) {
    std::ostringstream out;
    out << "kd must contain " << kFixStandMotorCount << " entries";
    throw error(out.str());
  }

  std::vector<double> values;
  values.reserve(kFixStandMotorCount);
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string field = "kd[" + std::to_string(i) + "]";
    const double value = scalarAs<double>(node[i], field);
    if (!std::isfinite(value) || value < 0.0) {
      throw error(field + " must be finite and non-negative");
    }
    values.push_back(value);
  }
  return values;
}

void validateConfig(const PassiveConfig& config) {
  if (config.mode.size() != kFixStandMotorCount ||
      config.kd.size() != kFixStandMotorCount) {
    throw error("mode and kd must contain 33 entries");
  }
}

}  // namespace

PassiveConfig loadPassiveConfig(const std::filesystem::path& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root || !root.IsMap()) {
      throw error("root must be a map");
    }

    PassiveConfig config;
    config.mode = readMode(requiredNode(root, "mode"));
    config.kd = readKd(requiredNode(root, "kd"));
    validateConfig(config);
    return config;
  } catch (const PassiveConfigError&) {
    throw;
  } catch (const YAML::Exception& err) {
    throw error(err.what());
  }
}

LowCmdFrame makePassiveLowCmdFrame(const PassiveConfig& config,
                                   const LowStateSample& low_state,
                                   std::uint8_t expected_mode_machine) {
  validateConfig(config);

  LowCmdFrame frame;
  frame.mode_machine = expected_mode_machine;
  frame.mode_pr = 0;
  for (std::size_t i = 0; i < kSdkMotorCount; ++i) {
    MotorCommand& motor = frame.motors.at(i);
    motor.q = low_state.motors.at(i).q;
    motor.dq = 0.0F;
    motor.kp = 0.0F;
    motor.tau = 0.0F;
    if (i < kFixStandMotorCount) {
      motor.mode = static_cast<std::uint8_t>(config.mode.at(i));
      motor.kd = static_cast<float>(config.kd.at(i));
    }
  }
  return frame;
}

}  // namespace agentic_et1_tracker
