#include "Et1DanceBridge.h"

#include <cctype>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

long long NowMilliseconds()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string JsonEscape(const std::string& input)
{
    std::string out;
    out.reserve(input.size() + 8);
    for (char ch : input) {
        switch (ch) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out.push_back(ch); break;
        }
    }
    return out;
}

std::string FindJsonString(const std::string& body, const std::string& key)
{
    const std::string pattern = "\"" + key + "\"";
    size_t pos = body.find(pattern);
    if (pos == std::string::npos) {
        return "";
    }
    pos = body.find(':', pos + pattern.size());
    if (pos == std::string::npos) {
        return "";
    }
    pos = body.find('"', pos);
    if (pos == std::string::npos) {
        return "";
    }
    std::string value;
    bool escape = false;
    for (++pos; pos < body.size(); ++pos) {
        const char ch = body[pos];
        if (escape) {
            value.push_back(ch);
            escape = false;
        } else if (ch == '\\') {
            escape = true;
        } else if (ch == '"') {
            break;
        } else {
            value.push_back(ch);
        }
    }
    return value;
}

int FindJsonInt(const std::string& body, const std::string& key)
{
    const std::string pattern = "\"" + key + "\"";
    size_t pos = body.find(pattern);
    if (pos == std::string::npos) {
        return 0;
    }
    pos = body.find(':', pos + pattern.size());
    if (pos == std::string::npos) {
        return 0;
    }
    ++pos;
    while (pos < body.size() && std::isspace(static_cast<unsigned char>(body[pos])) != 0) {
        ++pos;
    }
    const size_t start = pos;
    while (pos < body.size() && (std::isdigit(static_cast<unsigned char>(body[pos])) != 0 || body[pos] == '-')) {
        ++pos;
    }
    if (start == pos) {
        return 0;
    }
    return std::stoi(body.substr(start, pos - start));
}

std::string BuildStatusJson(const std::string& request_id,
                            int dance_id,
                            const std::string& status,
                            const std::string& message,
                            bool has_progress = false,
                            long long duration_ms = 0,
                            long long elapsed_ms = 0,
                            double progress = 0.0)
{
    std::ostringstream os;
    os << "{\"request_id\":\"" << JsonEscape(request_id)
       << "\",\"cmd\":\"dance_status\""
       << ",\"code\":200"
       << ",\"msg\":\"success\""
       << ",\"data\":{\"dance_id\":" << dance_id
       << ",\"status\":\"" << JsonEscape(status) << "\""
       << ",\"timestamp\":" << NowMilliseconds();
    if (has_progress) {
        os << ",\"duration_ms\":" << duration_ms
           << ",\"elapsed_ms\":" << elapsed_ms
           << ",\"progress\":" << std::fixed << std::setprecision(4) << progress;
    }
    if (!message.empty()) {
        os << ",\"message\":\"" << JsonEscape(message) << "\"";
    }
    os << "}}";
    return os.str();
}

} // namespace

Et1DanceBridge& Et1DanceBridge::Instance()
{
    static Et1DanceBridge bridge;
    return bridge;
}

void Et1DanceBridge::Init(const std::string& control_topic, const std::string& status_topic)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return;
    }
    control_topic_ = control_topic;
    status_topic_ = status_topic;
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<DdsString>>(status_topic_);
    subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<DdsString>>(control_topic_);
    publisher_->InitChannel();
    subscriber_->InitChannel([this](const void* message) { OnControl(message); }, 10);
    initialized_ = true;
    std::cout << "ET1 dance bridge started: control=" << control_topic_
              << ", status=" << status_topic_ << std::endl;
}

bool Et1DanceBridge::HasPendingStart() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_start_;
}

bool Et1DanceBridge::ConsumeStart(Et1DanceCommand& command)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_start_) {
        return false;
    }
    command = start_command_;
    has_start_ = false;
    return true;
}

bool Et1DanceBridge::ConsumeStop(Et1DanceCommand& command)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_stop_) {
        return false;
    }
    command = stop_command_;
    has_stop_ = false;
    return true;
}

void Et1DanceBridge::PublishStatus(const std::string& request_id,
                                   int dance_id,
                                   const std::string& status,
                                   const std::string& message)
{
    Publish(BuildStatusJson(request_id, dance_id, status, message));
}

void Et1DanceBridge::PublishStatusWithProgress(const std::string& request_id,
                                               int dance_id,
                                               const std::string& status,
                                               long long duration_ms,
                                               long long elapsed_ms,
                                               double progress,
                                               const std::string& message)
{
    Publish(BuildStatusJson(
        request_id,
        dance_id,
        status,
        message,
        true,
        duration_ms,
        elapsed_ms,
        progress));
}

void Et1DanceBridge::OnControl(const void* message)
{
    const auto* control = static_cast<const DdsString*>(message);
    if (control == nullptr) {
        return;
    }
    const std::string body = control->data();
    std::cout << "[ET1 DANCE DDS RX] " << body << std::endl;
    Et1DanceCommand command;
    command.cmd = FindJsonString(body, "cmd");
    command.request_id = FindJsonString(body, "request_id");
    command.dance_id = FindJsonInt(body, "dance_id");
    command.motion_path = FindJsonString(body, "motion_path");

    std::lock_guard<std::mutex> lock(mutex_);
    if (command.cmd == "dance_start") {
        start_command_ = command;
        has_start_ = true;
    } else if (command.cmd == "dance_stop") {
        stop_command_ = command;
        has_stop_ = true;
    }
}

bool Et1DanceBridge::Publish(const std::string& json)
{
    DdsString message;
    message.data(json);
    std::cout << "[ET1 DANCE DDS TX] " << json << std::endl;
    if (!publisher_) {
        return false;
    }
    return publisher_->Write(message);
}
