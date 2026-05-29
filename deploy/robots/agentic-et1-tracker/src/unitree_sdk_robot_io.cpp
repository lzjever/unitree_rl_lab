#include "agentic_et1_tracker/robot/unitree_sdk_robot_io.hpp"

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <thread>
#include <utility>

#include "unitree/dds_wrapper/common/crc.h"
#include "unitree/robot/channel/channel_factory.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr const char* kLowCmdTopic = "rt/lowcmd";
constexpr const char* kLowStateTopic = "rt/lowstate";
constexpr const char* kHighStateTopic = "rt/sportmodestate";

RobotIOError unitreeError(const std::string& message) {
  return RobotIOError("unitree sdk robot io error: " + message);
}

std::size_t ageMs(std::chrono::steady_clock::time_point sample_time,
                  std::chrono::steady_clock::time_point now) {
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - sample_time);
  return static_cast<std::size_t>(std::max<std::int64_t>(0, elapsed.count()));
}

}  // namespace

LowStateSample unitreeLowStateToSample(const unitree_hg::msg::dds_::LowState_& state,
                                       std::size_t age_ms,
                                       std::size_t timeout_ms) {
  LowStateSample sample;
  sample.fresh = age_ms <= timeout_ms;
  sample.age_ms = age_ms;
  sample.mode_machine = state.mode_machine();
  sample.mode_pr = state.mode_pr();
  sample.quat_wxyz = state.imu_state().quaternion();
  sample.gyro = state.imu_state().gyroscope();

  for (std::size_t i = 0; i < sample.motors.size(); ++i) {
    const auto& src = state.motor_state()[i];
    MotorStateSample& dst = sample.motors[i];
    dst.mode = src.mode();
    dst.q = src.q();
    dst.dq = src.dq();
    dst.tau_est = src.tau_est();
  }

  return sample;
}

HighStateSample unitreeHighStateToSample(const unitree_go::msg::dds_::SportModeState_& state,
                                         std::size_t age_ms,
                                         std::size_t timeout_ms) {
  HighStateSample sample;
  sample.fresh = age_ms <= timeout_ms;
  sample.age_ms = age_ms;
  sample.position = state.position();
  sample.quat_wxyz = state.imu_state().quaternion();
  sample.linear_velocity = state.velocity();
  sample.angular_velocity = state.imu_state().gyroscope();
  return sample;
}

unitree_hg::msg::dds_::LowCmd_ unitreeLowCmdFromFrame(const LowCmdFrame& frame) {
  unitree_hg::msg::dds_::LowCmd_ cmd;
  cmd.mode_machine(frame.mode_machine);
  cmd.mode_pr(frame.mode_pr);

  for (std::size_t i = 0; i < frame.motors.size(); ++i) {
    const MotorCommand& src = frame.motors[i];
    auto& dst = cmd.motor_cmd()[i];
    dst.mode(src.mode);
    dst.q(src.q);
    dst.dq(src.dq);
    dst.tau(src.tau);
    dst.kp(src.kp);
    dst.kd(src.kd);
    dst.reserve(0);
  }

  cmd.reserve({0, 0, 0, 0});
  cmd.crc(0);
  cmd.crc(crc32_core(reinterpret_cast<std::uint32_t*>(&cmd),
                     static_cast<std::uint32_t>((sizeof(cmd) >> 2) - 1)));
  return cmd;
}

LowCmdOwnershipTracker::LowCmdOwnershipTracker(std::size_t window_ms,
                                               std::size_t max_recent_writes)
    : window_ms_(window_ms),
      max_recent_writes_(std::max<std::size_t>(1, max_recent_writes)) {}

void LowCmdOwnershipTracker::observe(const LowCmdMsg& command, Clock::time_point now) {
  last_observed_ = command;
  last_observed_at_ = now;
}

void LowCmdOwnershipTracker::recordOwnWrite(const LowCmdMsg& command,
                                            Clock::time_point now) {
  recent_own_writes_.push_back({command, now, command.crc()});
  pruneOldWrites(now);
  while (recent_own_writes_.size() > max_recent_writes_) {
    recent_own_writes_.pop_front();
  }
}

LowCmdOccupancy LowCmdOwnershipTracker::occupancy(Clock::time_point now) const {
  LowCmdOccupancy occupancy;
  if (!last_observed_ || !last_observed_at_) {
    return occupancy;
  }

  const std::size_t observed_age = ageMs(*last_observed_at_, now);
  occupancy.sample_age_ms = observed_age;
  if (observed_age > window_ms_) {
    return occupancy;
  }

  occupancy.occupied =
      !isRecentOwnObserved(*last_observed_, *last_observed_at_, now);
  return occupancy;
}

bool LowCmdOwnershipTracker::isRecentOwnObserved(const LowCmdMsg& observed,
                                                 Clock::time_point observed_at,
                                                 Clock::time_point now) const {
  const std::uint32_t observed_crc = observed.crc();
  for (auto it = recent_own_writes_.rbegin(); it != recent_own_writes_.rend(); ++it) {
    if (observed_at < it->written_at) {
      continue;
    }
    if (ageMs(it->written_at, now) > window_ms_) {
      continue;
    }
    if (it->crc == observed_crc && it->command == observed) {
      return true;
    }
  }
  return false;
}

void LowCmdOwnershipTracker::pruneOldWrites(Clock::time_point now) {
  while (!recent_own_writes_.empty() &&
         ageMs(recent_own_writes_.front().written_at, now) > window_ms_) {
    recent_own_writes_.pop_front();
  }
}

LowCmdStartupPreflightResult checkLowCmdStartupPreflight(
    const LowCmdOwnershipTracker& tracker,
    LowCmdOwnershipTracker::Clock::time_point now) {
  LowCmdStartupPreflightResult result;
  result.occupancy = tracker.occupancy(now);
  result.ok = !result.occupancy.occupied;
  return result;
}

UnitreeSdkRobotIO::UnitreeSdkRobotIO(UnitreeSdkRobotIOConfig config)
    : config_(std::move(config)),
      lowcmd_ownership_(config_.lowcmd_occupancy_window_ms,
                        config_.lowcmd_own_write_history_size) {
  if (config_.init_channel_factory) {
    unitree::robot::ChannelFactory::Instance()->Init(config_.domain_id, config_.network);
  }

  lowcmd_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<LowCmdMsg>>(
      kLowCmdTopic, [this](const void* message) { onLowCmd(message); }, 1);
  lowcmd_subscriber_->InitChannel();
  runStartupPreflight();

  lowcmd_publisher_ =
      std::make_unique<unitree::robot::ChannelPublisher<LowCmdMsg>>(kLowCmdTopic);
  lowcmd_publisher_->InitChannel();

  lowstate_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<LowStateMsg>>(
      kLowStateTopic, [this](const void* message) { onLowState(message); }, 1);
  lowstate_subscriber_->InitChannel();

  highstate_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<HighStateMsg>>(
      kHighStateTopic, [this](const void* message) { onHighState(message); }, 1);
  highstate_subscriber_->InitChannel();
}

UnitreeSdkRobotIO::~UnitreeSdkRobotIO() {
  closeChannels();
}

void UnitreeSdkRobotIO::closeChannels() noexcept {
  try {
    if (lowcmd_subscriber_) {
      lowcmd_subscriber_->CloseChannel();
      lowcmd_subscriber_.reset();
    }
    if (lowstate_subscriber_) {
      lowstate_subscriber_->CloseChannel();
      lowstate_subscriber_.reset();
    }
    if (highstate_subscriber_) {
      highstate_subscriber_->CloseChannel();
      highstate_subscriber_.reset();
    }
    if (lowcmd_publisher_) {
      lowcmd_publisher_->CloseChannel();
      lowcmd_publisher_.reset();
    }
  } catch (...) {
  }
}

void UnitreeSdkRobotIO::runStartupPreflight() {
  if (config_.lowcmd_startup_preflight_ms == 0) {
    return;
  }

  std::this_thread::sleep_for(
      std::chrono::milliseconds(config_.lowcmd_startup_preflight_ms));

  LowCmdStartupPreflightResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = checkLowCmdStartupPreflight(lowcmd_ownership_, Clock::now());
  }
  if (!result.ok) {
    closeChannels();
    std::ostringstream out;
    out << "lowcmd owner present at startup";
    if (result.occupancy.sample_age_ms > 0) {
      out << " age_ms=" << result.occupancy.sample_age_ms;
    }
    throw LowCmdStartupPreflightError(out.str());
  }
}

std::optional<LowStateSample> UnitreeSdkRobotIO::readLowState() const {
  LowStateMsg state;
  Clock::time_point sample_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_low_state_ || !latest_low_state_time_) {
      return std::nullopt;
    }
    state = *latest_low_state_;
    sample_time = *latest_low_state_time_;
  }

  return unitreeLowStateToSample(state, ageMs(sample_time, Clock::now()), config_.low_timeout_ms);
}

std::optional<HighStateSample> UnitreeSdkRobotIO::readHighState() const {
  HighStateMsg state;
  Clock::time_point sample_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_high_state_ || !latest_high_state_time_) {
      return std::nullopt;
    }
    state = *latest_high_state_;
    sample_time = *latest_high_state_time_;
  }

  return unitreeHighStateToSample(state, ageMs(sample_time, Clock::now()),
                                  config_.high_timeout_ms);
}

LowCmdOccupancy UnitreeSdkRobotIO::lowCmdOccupancy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return lowcmd_ownership_.occupancy(Clock::now());
}

void UnitreeSdkRobotIO::writeLowCmd(const LowCmdFrame& frame) {
  if (!lowcmd_publisher_) {
    throw unitreeError("lowcmd publisher is not initialized");
  }

  const LowCmdMsg cmd = unitreeLowCmdFromFrame(frame);
  const auto write_started = Clock::now();
  if (!lowcmd_publisher_->Write(cmd, 0)) {
    throw unitreeError("lowcmd publisher Write failed");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  lowcmd_ownership_.recordOwnWrite(cmd, write_started);
}

void UnitreeSdkRobotIO::onLowCmd(const void* message) {
  if (message == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  lowcmd_ownership_.observe(*static_cast<const LowCmdMsg*>(message), Clock::now());
}

void UnitreeSdkRobotIO::onLowState(const void* message) {
  if (message == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_low_state_ = *static_cast<const LowStateMsg*>(message);
  latest_low_state_time_ = Clock::now();
}

void UnitreeSdkRobotIO::onHighState(const void* message) {
  if (message == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_high_state_ = *static_cast<const HighStateMsg*>(message);
  latest_high_state_time_ = Clock::now();
}

}  // namespace agentic_et1_tracker
