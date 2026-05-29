#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
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
  bool init_channel_factory{true};
};

LowStateSample unitreeLowStateToSample(const unitree_hg::msg::dds_::LowState_& state,
                                       std::size_t age_ms,
                                       std::size_t timeout_ms);

HighStateSample unitreeHighStateToSample(const unitree_go::msg::dds_::SportModeState_& state,
                                         std::size_t age_ms,
                                         std::size_t timeout_ms);

unitree_hg::msg::dds_::LowCmd_ unitreeLowCmdFromFrame(const LowCmdFrame& frame);

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
  void onLowCmd(const void* message);
  void onLowState(const void* message);
  void onHighState(const void* message);

  UnitreeSdkRobotIOConfig config_;
  mutable std::mutex mutex_;
  std::optional<LowCmdMsg> last_observed_lowcmd_;
  std::optional<Clock::time_point> last_observed_lowcmd_time_;
  std::optional<LowCmdMsg> last_written_lowcmd_;
  std::optional<Clock::time_point> last_written_lowcmd_time_;
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
