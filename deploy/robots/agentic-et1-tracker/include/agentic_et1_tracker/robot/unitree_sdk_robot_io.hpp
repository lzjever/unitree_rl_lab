#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "agentic_et1_tracker/robot/robot_io.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

namespace agentic_et1_tracker {

struct UnitreeSdkRobotIOConfig {
  std::string network{"lo"};
  int domain_id{0};
  std::size_t low_timeout_ms{200};
  std::size_t high_timeout_ms{200};
  std::size_t lowcmd_occupancy_window_ms{200};
  std::size_t lowcmd_startup_preflight_ms{200};
  std::size_t lowcmd_own_write_history_size{512};
  bool release_motion_mode_on_startup{true};
  double release_motion_mode_timeout_s{3.0};
  std::size_t release_motion_mode_max_attempts{3};
  std::size_t release_motion_mode_retry_interval_ms{500};
  bool init_channel_factory{true};
};

LowStateSample unitreeLowStateToSample(const unitree_hg::msg::dds_::LowState_& state,
                                       std::size_t age_ms,
                                       std::size_t timeout_ms);

HighStateSample unitreeHighStateToSample(const unitree_go::msg::dds_::SportModeState_& state,
                                         std::size_t age_ms,
                                         std::size_t timeout_ms);

unitree_hg::msg::dds_::LowCmd_ unitreeLowCmdFromFrame(const LowCmdFrame& frame);

class LowCmdOwnershipTracker final {
 public:
  using Clock = std::chrono::steady_clock;
  using LowCmdMsg = unitree_hg::msg::dds_::LowCmd_;

  explicit LowCmdOwnershipTracker(std::size_t window_ms = 200,
                                  std::size_t max_recent_writes = 512);

  void observe(const LowCmdMsg& command, Clock::time_point now);
  void recordOwnWrite(const LowCmdMsg& command, Clock::time_point now);
  LowCmdOccupancy occupancy(Clock::time_point now) const;

 private:
  struct StampedCommand {
    LowCmdMsg command;
    Clock::time_point written_at;
    std::uint32_t crc{0};
  };

  bool isRecentOwnObserved(const LowCmdMsg& observed,
                           Clock::time_point observed_at,
                           Clock::time_point now) const;
  void pruneOldWrites(Clock::time_point now);

  std::size_t window_ms_{200};
  std::size_t max_recent_writes_{512};
  std::optional<LowCmdMsg> last_observed_;
  std::optional<Clock::time_point> last_observed_at_;
  std::deque<StampedCommand> recent_own_writes_;
};

struct LowCmdStartupPreflightResult {
  bool ok{true};
  LowCmdOccupancy occupancy;
};

LowCmdStartupPreflightResult checkLowCmdStartupPreflight(
    const LowCmdOwnershipTracker& tracker,
    LowCmdOwnershipTracker::Clock::time_point now);

class LowCmdStartupPreflightError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class MotionModeReleaseError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct MotionModeReleaseOptions {
  std::size_t max_release_attempts{3};
  std::size_t retry_interval_ms{500};
};

struct MotionModeReleaseResult {
  bool released{false};
  std::size_t checks{0};
  std::size_t releases{0};
  std::string last_name;
};

class MotionSwitcherPort {
 public:
  virtual ~MotionSwitcherPort() = default;
  virtual int checkMode(std::string& form, std::string& name) = 0;
  virtual int releaseMode() = 0;
};

using MotionModeReleaseSleeper =
    std::function<void(std::chrono::milliseconds)>;

MotionModeReleaseResult releaseMotionModeForStartup(
    MotionSwitcherPort& switcher,
    MotionModeReleaseOptions options,
    MotionModeReleaseSleeper sleep_for = {});

class UnitreeSdkRobotIO final : public RobotIO {
 public:
  explicit UnitreeSdkRobotIO(UnitreeSdkRobotIOConfig config = {});
  ~UnitreeSdkRobotIO() override;

  std::optional<LowStateSample> readLowState() const override;
  std::optional<HighStateSample> readHighState() const override;
  LowCmdOccupancy lowCmdOccupancy() const override;
  void writeLowCmd(const LowCmdFrame& frame) override;

 private:
  using Clock = std::chrono::steady_clock;
  using LowCmdMsg = unitree_hg::msg::dds_::LowCmd_;
  using LowStateMsg = unitree_hg::msg::dds_::LowState_;
  using HighStateMsg = unitree_go::msg::dds_::SportModeState_;

  void closeChannels() noexcept;
  void runStartupPreflight();
  void onLowCmd(const void* message);
  void onLowState(const void* message);
  void onHighState(const void* message);

  UnitreeSdkRobotIOConfig config_;
  LowCmdOwnershipTracker lowcmd_ownership_;
  mutable std::mutex mutex_;
  std::optional<LowStateMsg> latest_low_state_;
  std::optional<Clock::time_point> latest_low_state_time_;
  std::optional<HighStateMsg> latest_high_state_;
  std::optional<Clock::time_point> latest_high_state_time_;

  std::unique_ptr<unitree::robot::ChannelPublisher<LowCmdMsg>> lowcmd_publisher_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<LowCmdMsg>> lowcmd_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<LowStateMsg>> lowstate_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<HighStateMsg>> highstate_subscriber_;
};

}  // namespace agentic_et1_tracker
