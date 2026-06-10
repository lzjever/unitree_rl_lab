#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

struct Et1DanceCommand {
    std::string cmd;
    std::string request_id;
    int dance_id = 0;
    std::string motion_path;
};

class Et1DanceBridge {
public:
    static Et1DanceBridge& Instance();

    void Init(const std::string& control_topic, const std::string& status_topic);
    bool HasPendingStart() const;
    bool ConsumeStart(Et1DanceCommand& command);
    bool ConsumeStop(Et1DanceCommand& command);
    void PublishStatus(const std::string& request_id,
                       int dance_id,
                       const std::string& status,
                       const std::string& message = "");
    void PublishStatusWithProgress(const std::string& request_id,
                                   int dance_id,
                                   const std::string& status,
                                   long long duration_ms,
                                   long long elapsed_ms,
                                   double progress,
                                   const std::string& message = "");

private:
    using DdsString = std_msgs::msg::dds_::String_;

    Et1DanceBridge() = default;

    void OnControl(const void* message);
    bool Publish(const std::string& json);

    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool has_start_ = false;
    bool has_stop_ = false;
    Et1DanceCommand start_command_;
    Et1DanceCommand stop_command_;
    std::string control_topic_;
    std::string status_topic_;
    std::unique_ptr<unitree::robot::ChannelPublisher<DdsString>> publisher_;
    std::unique_ptr<unitree::robot::ChannelSubscriber<DdsString>> subscriber_;
};
