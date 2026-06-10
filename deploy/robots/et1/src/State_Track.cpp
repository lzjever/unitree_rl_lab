#include "State_Track.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <spdlog/spdlog.h>
#include <zmq.hpp>

#include "LinearInterpolator.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

std::shared_ptr<State_Track::ReferenceLoader> State_Track::reference = nullptr;
std::mutex State_Track::pending_motion_mutex_;
std::optional<std::filesystem::path> State_Track::pending_motion_file_;

namespace
{
constexpr char kLiveMagic[8] = {'E', 'T', '1', 'L', 'I', 'V', 'E', '1'};
constexpr uint32_t kLiveVersion = 1;
constexpr uint32_t kLiveFlagReset = 1u << 0;
constexpr uint32_t kLiveFlagEnd = 1u << 1;
constexpr int kUpperBodyStartIndex = 14;
constexpr int kMaxUpperBodyJointCount = 12;

struct LiveWireHeader
{
    char magic[8];
    uint32_t version;
    uint32_t flags;
    uint64_t sequence;
    uint64_t publish_time_ns;
    uint32_t float_count;
    uint32_t reserved;
};

enum class CacheDType : uint32_t
{
    Float32 = 1,
    Float64 = 2,
    Bool = 3,
    Int32 = 4,
    Int64 = 5,
    UInt8 = 6,
    Int8 = 7,
};

float quat_to_yaw(float qw, float qx, float qy, float qz)
{
    const float norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-8f) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }
    return std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

std::string timestamp_string()
{
    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream ss;
    ss << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return ss.str();
}

std::filesystem::path timestamped_dump_path(const std::filesystem::path& base_path)
{
    const auto dir = base_path.parent_path();
    const auto stem = base_path.stem().string();
    const auto ext = base_path.extension().string();
    return dir / (stem + "_" + timestamp_string() + ext);
}

std::string trim_copy(const std::string& value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string tracker_target_from_token(const std::string& token)
{
    static const std::unordered_map<std::string, std::string> aliases = {
        {"general", "GeneralTrackerCLN"},
        {"tracker", "GeneralTrackerCLN"},
        {"generaltracker", "GeneralTrackerCLN"},
        {"GeneralTracker", "GeneralTrackerCLN"},
        {"debug", "GeneralTrackerCJM"},
        {"cjm", "GeneralTrackerCJM"},
        {"general_tracker_cjm", "GeneralTrackerCJM"},
        {"generaltrackercjm", "GeneralTrackerCJM"},
        {"GeneralTrackerCJM", "GeneralTrackerCJM"},
        {"cln", "GeneralTrackerCLN"},
        {"general_tracker_cln", "GeneralTrackerCLN"},
        {"generaltrackercln", "GeneralTrackerCLN"},
        {"GeneralTrackerCLN", "GeneralTrackerCLN"},
    };
    const auto it = aliases.find(token);
    return it == aliases.end() ? "" : it->second;
}

struct TrackerRequestLine
{
    std::string target_state;
    std::string motion_file;
    bool has_profile = false;
};

TrackerRequestLine parse_tracker_request_line(const std::string& raw_line)
{
    TrackerRequestLine request;
    const auto line = trim_copy(raw_line);
    if (line.empty()) {
        return request;
    }

    std::istringstream ss(line);
    std::string first_token;
    ss >> first_token;
    request.target_state = tracker_target_from_token(first_token);
    if (request.target_state.empty()) {
        request.target_state = "GeneralTrackerCLN";
        request.motion_file = line;
        request.has_profile = false;
        return request;
    }

    std::getline(ss, request.motion_file);
    request.motion_file = trim_copy(request.motion_file);
    request.has_profile = true;
    return request;
}

std::optional<YAML::Node> find_yaml_key(const YAML::Node& node, const std::string& key)
{
    if (!node) {
        return std::nullopt;
    }
    if (node.IsMap()) {
        for (const auto& item : node) {
            if (item.first.IsScalar() && item.first.as<std::string>() == key) {
                return item.second;
            }
            if (auto found = find_yaml_key(item.second, key)) {
                return found;
            }
        }
    } else if (node.IsSequence()) {
        for (const auto& item : node) {
            if (auto found = find_yaml_key(item, key)) {
                return found;
            }
        }
    }
    return std::nullopt;
}

void write_foot_support_onehot(int left_state,
                               int right_state,
                               std::vector<float>& data,
                               size_t offset)
{
    if (data.size() < offset + State_Track::ReferenceLoader::kFootSupportStateDim) {
        throw std::runtime_error("Foot support one-hot output buffer is too small.");
    }
    std::fill(data.begin() + static_cast<std::ptrdiff_t>(offset),
              data.begin() + static_cast<std::ptrdiff_t>(offset + State_Track::ReferenceLoader::kFootSupportStateDim),
              0.0f);
    if (left_state >= 0 && left_state <= 2) {
        data[offset + static_cast<size_t>(left_state)] = 1.0f;
    }
    if (right_state >= 0 && right_state <= 2) {
        data[offset + 3 + static_cast<size_t>(right_state)] = 1.0f;
    }
}

size_t infer_future_horizon(const YAML::Node& deploy_cfg)
{
    auto future_commands = find_yaml_key(deploy_cfg["observations"], "future_commands");
    if (!future_commands) {
        future_commands = find_yaml_key(deploy_cfg["observations"], "future_command_with_foot_support_state");
    }
    if (!future_commands) {
        future_commands = find_yaml_key(deploy_cfg["observations"], "future_command_foot_support_state");
    }
    if (!future_commands) {
        return 0;
    }
    const auto params = (*future_commands)["params"];
    if (params && params["horizon"]) {
        const int configured_horizon = params["horizon"].as<int>();
        if (configured_horizon > 0) {
            return static_cast<size_t>(configured_horizon);
        }
    }
    return static_cast<size_t>(State_Track::ReferenceLoader::kDefaultFutureHorizon);
}

size_t infer_live_initial_buffer_frames(const YAML::Node& deploy_cfg)
{
    const size_t future_horizon = infer_future_horizon(deploy_cfg);
    return future_horizon > 0 ? future_horizon + 1 : 1;
}
}

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(command_root_ori_b)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_root_ori_b.");
    }
    const auto & data = State_Track::reference->command_root_ori_b();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_root_ori_b_unbiased)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_root_ori_b_unbiased.");
    }
    const auto & data = State_Track::reference->command_root_ori_b_unbiased();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_xy_yaw_vel)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_xy_yaw_vel.");
    }
    const auto & data = State_Track::reference->command_xy_yaw_vel();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_yaw)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_yaw.");
    }
    const auto& data = State_Track::reference->command_yaw();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_jnt_pos)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_jnt_pos.");
    }
    const auto & data = State_Track::reference->command_joint_pos();
    const auto& joint_ids = env->robot->data.joint_ids_map;
    if (!joint_ids.empty() && joint_ids.size() != static_cast<size_t>(data.size())) {
        const auto max_joint_id = *std::max_element(joint_ids.begin(), joint_ids.end());
        if (max_joint_id >= data.size()) {
            return std::vector<float>(data.data(), data.data() + data.size());
        }
        std::vector<float> selected;
        selected.reserve(joint_ids.size());
        for (const auto joint_id : joint_ids) {
            if (joint_id < 0) {
                throw std::runtime_error("command_jnt_pos joint_ids_map index is out of range.");
            }
            selected.push_back(data[joint_id]);
        }
        return selected;
    }
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_jnt_vel)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_jnt_vel.");
    }
    const auto & data = State_Track::reference->command_joint_vel();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(motion_command)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing motion_command.");
    }
    const auto& pos = State_Track::reference->command_joint_pos();
    const auto& vel = State_Track::reference->command_joint_vel();
    std::vector<float> data;
    data.reserve(pos.size() + vel.size());
    data.insert(data.end(), pos.data(), pos.data() + pos.size());
    data.insert(data.end(), vel.data(), vel.data() + vel.size());
    return data;
}

REGISTER_OBSERVATION(future_commands)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing future_commands.");
    }
    return State_Track::reference->future_commands();
}

REGISTER_OBSERVATION(future_command_with_foot_support_state)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing future_command_with_foot_support_state.");
    }
    const int horizon = params["horizon"] ? params["horizon"].as<int>() : State_Track::ReferenceLoader::kDefaultFutureHorizon;
    const auto& data = State_Track::reference->future_command_with_foot_support_state();
    const size_t dim = State_Track::ReferenceLoader::kFutureCommandWithFootSupportDim;
    const size_t available_horizon = data.size() / dim;
    const size_t requested_horizon = horizon > 0 ? static_cast<size_t>(horizon) : 0;
    const size_t output_horizon = std::min(requested_horizon, available_horizon);
    return std::vector<float>(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(output_horizon * dim));
}

REGISTER_OBSERVATION(future_command_foot_support_state)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing future_command_foot_support_state.");
    }
    const int horizon = params["horizon"] ? params["horizon"].as<int>() : State_Track::ReferenceLoader::kDefaultFutureHorizon;
    const auto& data = State_Track::reference->future_command_with_foot_support_state();
    const size_t dim = State_Track::ReferenceLoader::kFutureCommandWithFootSupportDim;
    const size_t foot_dim = State_Track::ReferenceLoader::kFootSupportStateDim;
    const size_t available_horizon = data.size() / dim;
    const size_t requested_horizon = horizon > 0 ? static_cast<size_t>(horizon) : 0;
    const size_t output_horizon = std::min(requested_horizon, available_horizon);
    std::vector<float> foot_support;
    foot_support.reserve(output_horizon * foot_dim);
    for (size_t i = 0; i < output_horizon; ++i) {
        const size_t offset = i * dim + State_Track::ReferenceLoader::kFutureCommandDim;
        foot_support.insert(foot_support.end(),
                            data.begin() + static_cast<std::ptrdiff_t>(offset),
                            data.begin() + static_cast<std::ptrdiff_t>(offset + foot_dim));
    }
    return foot_support;
}

REGISTER_OBSERVATION(motion_anchor_ori_b)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing motion_anchor_ori_b.");
    }
    const auto& data = State_Track::reference->command_root_ori_b();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_foot_support_state)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_foot_support_state.");
    }
    const auto& data = State_Track::reference->command_foot_support_state();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(ref_com_rel_navi)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing ref_com_rel_navi.");
    }
    const auto& data = State_Track::reference->ref_com_rel_navi();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(ref_com_vel_navi)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing ref_com_vel_navi.");
    }
    const auto& data = State_Track::reference->ref_com_vel_navi();
    return std::vector<float>(data.data(), data.data() + data.size());
}

}
}

State_Track::ReferenceLoader::ReferenceLoader(const std::filesystem::path& motion_file, float fps, size_t future_horizon)
    : fps_(fps),
      future_horizon_(future_horizon)
{
    spdlog::info("Track: initializing reference loader from '{}' at {} FPS", motion_file.string(), fps_);
    const auto cache_file = ensure_cache_file(motion_file);
    spdlog::info("Track: using cache file '{}'", cache_file.string());
    load_cache_file(cache_file);
    joint_pos_ = Eigen::VectorXf::Zero(kJointDim);
    joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
    future_commands_.assign(future_horizon_ * kFutureCommandDim, 0.0f);
    future_command_with_foot_support_state_.assign(
        future_horizon_ * kFutureCommandWithFootSupportDim,
        0.0f);
    duration_ = frame_count_ > 0 ? static_cast<float>(frame_count_ - 1) / fps_ : 0.0f;
    spdlog::info("Track: reference loaded with {} frames, duration {:.3f}s", frame_count_, duration_);
}

State_Track::ReferenceLoader::ReferenceLoader(const LiveStreamConfig& live_config, float fps, size_t future_horizon)
    : fps_(fps),
      duration_(std::numeric_limits<float>::infinity()),
      future_horizon_(future_horizon),
      live_stream_enabled_(true),
      live_config_(live_config)
{
    joint_pos_ = Eigen::VectorXf::Zero(kJointDim);
    joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
    future_commands_.assign(future_horizon_ * kFutureCommandDim, 0.0f);
    future_command_with_foot_support_state_.assign(
        future_horizon_ * kFutureCommandWithFootSupportDim,
        0.0f);
    spdlog::info("Track: initializing live reference stream from '{}' topic '{}' future_horizon={}",
                 live_config_.endpoint,
                 live_config_.topic,
                 future_horizon_);
    start_live_receiver();
}

State_Track::ReferenceLoader::~ReferenceLoader()
{
    stop_live_receiver();
}

size_t State_Track::ReferenceLoader::live_buffer_size() const
{
    std::lock_guard<std::mutex> lock(live_mutex_);
    return live_queue_.size();
}

void State_Track::ReferenceLoader::reset(const Eigen::VectorXf& default_joint_pos)
{
    default_joint_pos_ = default_joint_pos;
    joint_pos_ = Eigen::VectorXf::Zero(kJointDim);
    joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
    yaw_command_ << 1.0f, 0.0f;
    std::fill(future_commands_.begin(), future_commands_.end(), 0.0f);
    std::fill(future_command_with_foot_support_state_.begin(),
              future_command_with_foot_support_state_.end(),
              0.0f);
    ref_com_rel_navi_.setZero();
    ref_com_vel_navi_.setZero();
    initial_ref_yaw_bias_ = 0.0f;
    anchor_frame_offset_q_ = Eigen::Quaternionf::Identity();
    if (live_stream_enabled_) {
        joint_pos_ = default_joint_pos_.size() == kJointDim
            ? default_joint_pos_
            : Eigen::VectorXf::Zero(kJointDim);
        joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
        root_ori_b_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
        root_ori_b_unbiased_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
        xy_yaw_vel_.setZero();
        foot_support_state_.setZero();
        std::fill(future_commands_.begin(), future_commands_.end(), 0.0f);
        std::fill(future_command_with_foot_support_state_.begin(),
                  future_command_with_foot_support_state_.end(),
                  0.0f);
        std::lock_guard<std::mutex> lock(live_mutex_);
        live_has_last_frame_ = false;
        current_frame_index_ = 0;
        current_time_s_ = 0.0f;
        spdlog::info("Track: preserving {} pre-buffered live reference frames across reset",
                     live_queue_.size());
        return;
    }
    if (frame_count_ > 0 && body_quat_w_seq_.size() >= 4) {
        initial_ref_yaw_bias_ = quat_to_yaw(
            body_quat_w_seq_[0],
            body_quat_w_seq_[1],
            body_quat_w_seq_[2],
            body_quat_w_seq_[3]
        );
    }
    update(0.0f,
           false,
           false,
           Eigen::Vector2f::Zero(),
           0.0f,
           Eigen::Quaternionf::Identity(),
           Eigen::Quaternionf::Identity(),
           Eigen::VectorXf::Zero(kJointDim));
}

Eigen::Quaternionf State_Track::ReferenceLoader::anchor_quat_w(const Eigen::Quaternionf& root_quat,
                                                               const Eigen::VectorXf& joint_pos) const
{
    Eigen::Quaternionf q = root_quat.normalized();
    if (joint_pos.size() >= 14) {
        q = q
            * Eigen::AngleAxisf(joint_pos[12], Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(joint_pos[13], Eigen::Vector3f::UnitZ());
    }
    q.normalize();
    return q;
}

void State_Track::ReferenceLoader::calibrate_anchor_frame(float time_s,
                                                          const Eigen::VectorXf& robot_joint_pos,
                                                          const Eigen::Quaternionf& robot_root_quat,
                                                          bool yaw_only)
{
    if (live_stream_enabled_ || frame_count_ == 0) {
        anchor_frame_offset_q_ = Eigen::Quaternionf::Identity();
        return;
    }

    const float clamped_time = duration_ > 0.0f ? std::clamp(time_s, 0.0f, duration_) : 0.0f;
    const size_t frame_index = std::min(static_cast<size_t>(std::round(clamped_time * fps_)), frame_count_ - 1);
    const size_t anchor_quat_offset = (frame_index * kBodyCount + kAnchorBodyIndex) * 4;
    Eigen::Quaternionf ref_anchor_q(
        body_quat_w_seq_[anchor_quat_offset + 0],
        body_quat_w_seq_[anchor_quat_offset + 1],
        body_quat_w_seq_[anchor_quat_offset + 2],
        body_quat_w_seq_[anchor_quat_offset + 3]
    );
    ref_anchor_q.normalize();

    const Eigen::Quaternionf robot_anchor_q = anchor_quat_w(robot_root_quat, robot_joint_pos);
    Eigen::Quaternionf offset_q = robot_anchor_q * ref_anchor_q.conjugate();
    offset_q.normalize();

    if (yaw_only) {
        const float yaw_offset = quat_to_yaw(offset_q.w(), offset_q.x(), offset_q.y(), offset_q.z());
        offset_q = Eigen::Quaternionf(Eigen::AngleAxisf(yaw_offset, Eigen::Vector3f::UnitZ()));
    }

    anchor_frame_offset_q_ = offset_q.normalized();
}

Eigen::VectorXf State_Track::ReferenceLoader::sample_joint_pos(float time_s) const
{
    Eigen::VectorXf joint_pos = Eigen::VectorXf::Zero(kJointDim);
    if (live_stream_enabled_ || frame_count_ == 0 || joint_pos_seq_.empty()) {
        return joint_pos;
    }

    const float clamped_time = duration_ > 0.0f ? std::clamp(time_s, 0.0f, duration_) : 0.0f;
    const size_t frame_index = std::min(static_cast<size_t>(std::round(clamped_time * fps_)), frame_count_ - 1);
    const size_t joint_offset = frame_index * kJointDim;
    for (int i = 0; i < kJointDim; ++i) {
        joint_pos[i] = joint_pos_seq_[joint_offset + i];
    }
    return joint_pos;
}

void State_Track::ReferenceLoader::update(float time_s,
                                          bool no_global_mode,
                                          bool has_current_root_xy,
                                          const Eigen::Vector2f& current_root_xy,
                                          float current_root_yaw,
                                          const Eigen::Quaternionf& current_root_quat,
                                          const Eigen::Quaternionf& current_root_quat_unbiased,
                                          const Eigen::VectorXf& current_joint_pos,
                                          bool use_motion_root_command,
                                          bool use_motion_velocity_command,
                                          bool loop_reference)
{
    if (live_stream_enabled_) {
        LiveFrame frame;
        std::deque<LiveFrame> queue_snapshot;
        bool has_frame = false;
        {
            std::lock_guard<std::mutex> lock(live_mutex_);
            if (live_queue_.size() >= live_config_.initial_buffer_frames || live_has_last_frame_) {
                if (!live_queue_.empty()) {
                    frame = live_queue_.front();
                    live_queue_.pop_front();
                    live_last_frame_ = frame;
                    live_has_last_frame_ = true;
                    has_frame = true;
                } else if (live_has_last_frame_) {
                    frame = live_last_frame_;
                    has_frame = true;
                }
            }
            queue_snapshot = live_queue_;
        }

        if (has_frame) {
            if (frame.reset) {
                current_frame_index_ = 0;
                current_time_s_ = 0.0f;
            }
            apply_live_frame(frame,
                             no_global_mode,
                             current_root_yaw,
                             current_root_quat,
                             current_root_quat_unbiased,
                             current_joint_pos,
                             use_motion_root_command,
                             use_motion_velocity_command);
            update_live_future_commands(queue_snapshot,
                                        frame,
                                        no_global_mode,
                                        current_root_quat,
                                        use_motion_root_command,
                                        use_motion_velocity_command);
            current_frame_index_ += 1;
            current_time_s_ = static_cast<float>(current_frame_index_) / fps_;
        } else {
            if (default_joint_pos_.size() == kJointDim) {
                joint_pos_ = default_joint_pos_;
            }
            joint_vel_.setZero();
            root_ori_b_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
            root_ori_b_unbiased_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
            xy_yaw_vel_.setZero();
            foot_support_state_.setZero();
            ref_com_rel_navi_.setZero();
            ref_com_vel_navi_.setZero();
            std::fill(future_commands_.begin(), future_commands_.end(), 0.0f);
            std::fill(future_command_with_foot_support_state_.begin(),
                      future_command_with_foot_support_state_.end(),
                      0.0f);
        }
        return;
    }

    if (frame_count_ == 0) {
        return;
    }

    const float clamped_time = std::max(time_s, 0.0f);
    const float ref_time = (loop_reference && duration_ > 0.0f)
        ? std::fmod(clamped_time, duration_)
        : std::min(clamped_time, duration_);
    const size_t frame_index = std::min(static_cast<size_t>(std::round(ref_time * fps_)), frame_count_ - 1);
    current_frame_index_ = frame_index;
    current_time_s_ = ref_time;

    const size_t joint_offset = frame_index * kJointDim;
    for (int i = 0; i < kJointDim; ++i) {
        joint_pos_[i] = joint_pos_seq_[joint_offset + i];
        joint_vel_[i] = joint_vel_seq_[joint_offset + i];
    }

    const size_t root_body_offset = frame_index * kBodyCount;
    const size_t root_quat_offset = root_body_offset * 4;
    Eigen::Quaternionf ref_root_q(
        body_quat_w_seq_[root_quat_offset + 0],
        body_quat_w_seq_[root_quat_offset + 1],
        body_quat_w_seq_[root_quat_offset + 2],
        body_quat_w_seq_[root_quat_offset + 3]
    );
    ref_root_q.normalize();
    Eigen::Quaternionf ref_world_align_q = Eigen::Quaternionf::Identity();
    if (no_global_mode) {
        ref_world_align_q = anchor_frame_offset_q_;
        ref_root_q = (ref_world_align_q * ref_root_q).normalized();
    }

    if (use_motion_root_command) {
        Eigen::Quaternionf robot_root_q = current_root_quat.normalized();
        const Eigen::Matrix3f root_rot_b = (robot_root_q.conjugate() * ref_root_q).toRotationMatrix();
        root_ori_b_ << root_rot_b(0, 0), root_rot_b(0, 1),
                       root_rot_b(1, 0), root_rot_b(1, 1),
                       root_rot_b(2, 0), root_rot_b(2, 1);

        Eigen::Quaternionf robot_root_q_unbiased = current_root_quat_unbiased.normalized();
        const Eigen::Matrix3f root_rot_b_unbiased =
            (robot_root_q_unbiased.conjugate() * ref_root_q).toRotationMatrix();
        root_ori_b_unbiased_ << root_rot_b_unbiased(0, 0), root_rot_b_unbiased(0, 1),
                                root_rot_b_unbiased(1, 0), root_rot_b_unbiased(1, 1),
                                root_rot_b_unbiased(2, 0), root_rot_b_unbiased(2, 1);
    } else {
        root_ori_b_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
        root_ori_b_unbiased_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
    }

    const float yaw_ref = quat_to_yaw(ref_root_q.w(), ref_root_q.x(), ref_root_q.y(), ref_root_q.z());
    const Eigen::Quaternionf ref_yaw_q =
        Eigen::AngleAxisf(yaw_ref, Eigen::Vector3f::UnitZ()) * Eigen::Quaternionf::Identity();
    const float yaw_error = wrap_to_pi(yaw_ref - current_root_yaw);
    yaw_command_ << std::cos(yaw_error), std::sin(yaw_error);
    const size_t root_lin_vel_offset = root_body_offset * 3;
    const size_t root_ang_vel_offset = root_body_offset * 3;
    const Eigen::Vector3f ref_lin_vel_w_raw(
        body_lin_vel_w_seq_[root_lin_vel_offset + 0],
        body_lin_vel_w_seq_[root_lin_vel_offset + 1],
        body_lin_vel_w_seq_[root_lin_vel_offset + 2]
    );
    const Eigen::Vector3f ref_ang_vel_w_raw(
        body_ang_vel_w_seq_[root_ang_vel_offset + 0],
        body_ang_vel_w_seq_[root_ang_vel_offset + 1],
        body_ang_vel_w_seq_[root_ang_vel_offset + 2]
    );
    const Eigen::Vector3f ref_lin_vel_w = ref_world_align_q * ref_lin_vel_w_raw;
    const Eigen::Vector3f ref_ang_vel_w = ref_world_align_q * ref_ang_vel_w_raw;
    if (use_motion_velocity_command) {
        const Eigen::Vector3f ref_lin_vel_navi = ref_yaw_q.conjugate() * ref_lin_vel_w;
        const Eigen::Vector3f ref_ang_vel_navi = ref_yaw_q.conjugate() * ref_ang_vel_w;
        xy_yaw_vel_ << ref_lin_vel_navi.x(), ref_lin_vel_navi.y(), ref_ang_vel_navi.z();
    } else {
        xy_yaw_vel_.setZero();
    }

    if (future_commands_.size() != static_cast<size_t>(future_horizon_ * kFutureCommandDim)) {
        future_commands_.assign(future_horizon_ * kFutureCommandDim, 0.0f);
    }
    if (future_command_with_foot_support_state_.size()
        != static_cast<size_t>(future_horizon_ * kFutureCommandWithFootSupportDim)) {
        future_command_with_foot_support_state_.assign(
            future_horizon_ * kFutureCommandWithFootSupportDim,
            0.0f);
    }
    for (size_t horizon_idx = 0; horizon_idx < future_horizon_; ++horizon_idx) {
        const size_t future_frame = std::min(frame_index + static_cast<size_t>(horizon_idx + 1), frame_count_ - 1);
        const size_t future_body_offset = future_frame * kBodyCount;
        const size_t future_quat_offset = future_body_offset * 4;
        Eigen::Quaternionf future_ref_root_q(
            body_quat_w_seq_[future_quat_offset + 0],
            body_quat_w_seq_[future_quat_offset + 1],
            body_quat_w_seq_[future_quat_offset + 2],
            body_quat_w_seq_[future_quat_offset + 3]
        );
        future_ref_root_q.normalize();
        future_ref_root_q = (ref_world_align_q * future_ref_root_q).normalized();

        const Eigen::Matrix3f motion_future_root_rot_b =
            (current_root_quat.normalized().conjugate() * future_ref_root_q).toRotationMatrix();
        Eigen::Matrix<float, 6, 1> motion_future_root_ori_b;
        motion_future_root_ori_b << motion_future_root_rot_b(0, 0), motion_future_root_rot_b(0, 1),
                                    motion_future_root_rot_b(1, 0), motion_future_root_rot_b(1, 1),
                                    motion_future_root_rot_b(2, 0), motion_future_root_rot_b(2, 1);

        const float future_yaw_ref = quat_to_yaw(
            future_ref_root_q.w(),
            future_ref_root_q.x(),
            future_ref_root_q.y(),
            future_ref_root_q.z()
        );
        const Eigen::Quaternionf future_ref_yaw_q =
            Eigen::AngleAxisf(future_yaw_ref, Eigen::Vector3f::UnitZ()) * Eigen::Quaternionf::Identity();
        const size_t future_lin_vel_offset = future_body_offset * 3;
        const size_t future_ang_vel_offset = future_body_offset * 3;
        const Eigen::Vector3f future_lin_vel_w_raw(
            body_lin_vel_w_seq_[future_lin_vel_offset + 0],
            body_lin_vel_w_seq_[future_lin_vel_offset + 1],
            body_lin_vel_w_seq_[future_lin_vel_offset + 2]
        );
        const Eigen::Vector3f future_ang_vel_w_raw(
            body_ang_vel_w_seq_[future_ang_vel_offset + 0],
            body_ang_vel_w_seq_[future_ang_vel_offset + 1],
            body_ang_vel_w_seq_[future_ang_vel_offset + 2]
        );
        const Eigen::Vector3f motion_future_lin_vel_navi =
            future_ref_yaw_q.conjugate() * (ref_world_align_q * future_lin_vel_w_raw);
        const Eigen::Vector3f motion_future_ang_vel_navi =
            future_ref_yaw_q.conjugate() * (ref_world_align_q * future_ang_vel_w_raw);
        const Eigen::Vector3f motion_future_xy_yaw_vel(
            motion_future_lin_vel_navi.x(),
            motion_future_lin_vel_navi.y(),
            motion_future_ang_vel_navi.z());

        Eigen::Matrix<float, 6, 1> future_root_ori_b;
        if (use_motion_root_command) {
            future_root_ori_b = motion_future_root_ori_b;
        } else {
            future_root_ori_b << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
        }

        Eigen::Vector3f future_xy_yaw_vel = Eigen::Vector3f::Zero();
        if (use_motion_velocity_command) {
            future_xy_yaw_vel = motion_future_xy_yaw_vel;
        }

        size_t out_offset = horizon_idx * kFutureCommandDim;
        for (int i = 0; i < 6; ++i) {
            future_commands_[out_offset++] = future_root_ori_b[i];
        }
        for (int i = 0; i < 3; ++i) {
            future_commands_[out_offset++] = future_xy_yaw_vel[i];
        }
        const size_t future_joint_offset = future_frame * kJointDim;
        for (int i = 0; i < kJointDim; ++i) {
            future_commands_[out_offset++] = joint_pos_seq_[future_joint_offset + i];
        }

        const size_t command_with_foot_offset = horizon_idx * kFutureCommandWithFootSupportDim;
        size_t with_foot_offset = command_with_foot_offset;
        for (int i = 0; i < 6; ++i) {
            future_command_with_foot_support_state_[with_foot_offset++] = motion_future_root_ori_b[i];
        }
        for (int i = 0; i < 3; ++i) {
            future_command_with_foot_support_state_[with_foot_offset++] = motion_future_xy_yaw_vel[i];
        }
        for (int i = 0; i < kJointDim; ++i) {
            future_command_with_foot_support_state_[with_foot_offset++] = joint_pos_seq_[future_joint_offset + i];
        }

        int left_state = -1;
        int right_state = -1;
        if (!left_foot_contact_state_seq_.empty() && !right_foot_contact_state_seq_.empty()) {
            left_state = static_cast<int>(left_foot_contact_state_seq_[future_frame]);
            right_state = static_cast<int>(right_foot_contact_state_seq_[future_frame]);
        }
        write_foot_support_onehot(left_state,
                                  right_state,
                                  future_command_with_foot_support_state_,
                                  command_with_foot_offset + kFutureCommandDim);
    }

    foot_support_state_.setZero();
    if (!left_foot_contact_state_seq_.empty() && !right_foot_contact_state_seq_.empty()) {
        const int left_state = static_cast<int>(left_foot_contact_state_seq_[frame_index]);
        const int right_state = static_cast<int>(right_foot_contact_state_seq_[frame_index]);
        if (left_state >= 0 && left_state <= 2) {
            foot_support_state_[left_state] = 1.0f;
        }
        if (right_state >= 0 && right_state <= 2) {
            foot_support_state_[3 + right_state] = 1.0f;
        }
    }

    ref_com_rel_navi_.setZero();
    if (!ref_com_rel_navi_seq_.empty()) {
        const size_t ref_com_offset = frame_index * 3;
        ref_com_rel_navi_ << ref_com_rel_navi_seq_[ref_com_offset + 0],
                             ref_com_rel_navi_seq_[ref_com_offset + 1],
                             ref_com_rel_navi_seq_[ref_com_offset + 2];
    }

    ref_com_vel_navi_.setZero();
    if (!ref_com_vel_navi_seq_.empty()) {
        const size_t ref_com_vel_offset = frame_index * 3;
        ref_com_vel_navi_ << ref_com_vel_navi_seq_[ref_com_vel_offset + 0],
                             ref_com_vel_navi_seq_[ref_com_vel_offset + 1],
                             ref_com_vel_navi_seq_[ref_com_vel_offset + 2];
    }
}

void State_Track::ReferenceLoader::start_live_receiver()
{
    if (!live_stream_enabled_ || live_receiver_running_.load()) {
        return;
    }
    live_receiver_running_ = true;
    live_receiver_thread_ = std::thread([this] {
        live_receiver_loop();
    });
}

void State_Track::ReferenceLoader::stop_live_receiver()
{
    live_receiver_running_ = false;
    if (live_receiver_thread_.joinable()) {
        live_receiver_thread_.join();
    }
}

void State_Track::ReferenceLoader::live_receiver_loop()
{
    try {
        zmq::context_t context(1);
        zmq::socket_t socket(context, zmq::socket_type::sub);
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::rcvtimeo, live_config_.receive_timeout_ms);
        socket.set(zmq::sockopt::rcvhwm, live_config_.high_water_mark);
        socket.set(zmq::sockopt::subscribe, live_config_.topic);
        socket.connect(live_config_.endpoint);
        spdlog::info("Track: live receiver connected to '{}' topic '{}'",
                     live_config_.endpoint,
                     live_config_.topic);

        while (live_receiver_running_.load()) {
            zmq::message_t first;
            const auto first_result = socket.recv(first, zmq::recv_flags::none);
            if (!first_result) {
                continue;
            }

            zmq::message_t payload;
            const bool multipart = socket.get(zmq::sockopt::rcvmore);
            const zmq::message_t* data_msg = &first;
            if (multipart) {
                const auto payload_result = socket.recv(payload, zmq::recv_flags::none);
                if (!payload_result) {
                    continue;
                }
                data_msg = &payload;
                while (socket.get(zmq::sockopt::rcvmore)) {
                    zmq::message_t ignored;
                    const auto ignored_result = socket.recv(ignored, zmq::recv_flags::none);
                    if (!ignored_result) {
                        break;
                    }
                }
            }

            LiveFrame frame;
            if (!parse_live_message(data_msg->data(), data_msg->size(), frame)) {
                continue;
            }
            push_live_frame(frame);
        }
    } catch (const std::exception& e) {
        spdlog::error("Track: live receiver stopped after exception: {}", e.what());
    }
}

bool State_Track::ReferenceLoader::parse_live_message(const void* data, size_t size, LiveFrame& frame) const
{
    if (size < sizeof(LiveWireHeader)) {
        spdlog::warn("Track: dropped live frame smaller than header ({} bytes)", size);
        return false;
    }

    LiveWireHeader header{};
    std::memcpy(&header, data, sizeof(header));
    if (std::memcmp(header.magic, kLiveMagic, sizeof(kLiveMagic)) != 0 || header.version != kLiveVersion) {
        spdlog::warn("Track: dropped live frame with unsupported magic/version");
        return false;
    }

    const size_t payload_bytes = size - sizeof(LiveWireHeader);
    const size_t expected_bytes = static_cast<size_t>(header.float_count) * sizeof(float);
    if (payload_bytes < expected_bytes || header.float_count < 62) {
        spdlog::warn("Track: dropped live frame with invalid float_count {}", header.float_count);
        return false;
    }

    const auto* values = reinterpret_cast<const float*>(
        static_cast<const uint8_t*>(data) + sizeof(LiveWireHeader));
    size_t offset = 0;
    for (int i = 0; i < kJointDim; ++i) {
        frame.joint_pos[i] = values[offset++];
    }
    for (int i = 0; i < kJointDim; ++i) {
        frame.joint_vel[i] = values[offset++];
    }
    frame.root_quat_w = Eigen::Quaternionf(
        values[offset + 0],
        values[offset + 1],
        values[offset + 2],
        values[offset + 3]);
    frame.root_quat_w.normalize();
    offset += 4;
    frame.root_lin_vel_w << values[offset + 0], values[offset + 1], values[offset + 2];
    offset += 3;
    frame.root_ang_vel_w << values[offset + 0], values[offset + 1], values[offset + 2];
    offset += 3;
    if (header.float_count >= offset + 2) {
        frame.left_foot_contact_state = static_cast<int>(std::round(values[offset + 0]));
        frame.right_foot_contact_state = static_cast<int>(std::round(values[offset + 1]));
        offset += 2;
    }
    if (header.float_count >= offset + 3) {
        frame.ref_com_rel_navi << values[offset + 0], values[offset + 1], values[offset + 2];
        offset += 3;
    }
    if (header.float_count >= offset + 3) {
        frame.ref_com_vel_navi << values[offset + 0], values[offset + 1], values[offset + 2];
    }
    frame.sequence = header.sequence;
    frame.publish_time_ns = header.publish_time_ns;
    frame.reset = (header.flags & kLiveFlagReset) != 0;
    frame.end = (header.flags & kLiveFlagEnd) != 0;
    return true;
}

void State_Track::ReferenceLoader::push_live_frame(const LiveFrame& frame)
{
    std::lock_guard<std::mutex> lock(live_mutex_);
    if (frame.reset) {
        live_queue_.clear();
        live_has_last_frame_ = false;
        live_last_sequence_ = 0;
    }
    if (live_last_sequence_ != 0 && frame.sequence <= live_last_sequence_) {
        return;
    }
    live_last_sequence_ = frame.sequence;
    live_queue_.push_back(frame);
    while (live_queue_.size() > live_config_.max_queue_frames) {
        live_queue_.pop_front();
    }
}

void State_Track::ReferenceLoader::apply_live_frame(const LiveFrame& frame,
                                                    bool no_global_mode,
                                                    float current_root_yaw,
                                                    const Eigen::Quaternionf& current_root_quat,
                                                    const Eigen::Quaternionf& current_root_quat_unbiased,
                                                    const Eigen::VectorXf& current_joint_pos,
                                                    bool use_motion_root_command,
                                                    bool use_motion_velocity_command)
{
    if (joint_pos_.size() != kJointDim) {
        joint_pos_ = Eigen::VectorXf::Zero(kJointDim);
    }
    if (joint_vel_.size() != kJointDim) {
        joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
    }
    for (int i = 0; i < kJointDim; ++i) {
        joint_pos_[i] = frame.joint_pos[i];
        joint_vel_[i] = frame.joint_vel[i];
    }

    Eigen::Quaternionf ref_root_q = frame.root_quat_w.normalized();
    Eigen::Quaternionf ref_world_align_q = Eigen::Quaternionf::Identity();
    if (no_global_mode) {
        if (current_frame_index_ == 0 || frame.reset) {
            const Eigen::Quaternionf ref_anchor_q = anchor_quat_w(ref_root_q, joint_pos_);
            const Eigen::Quaternionf robot_anchor_q = anchor_quat_w(current_root_quat, current_joint_pos);
            Eigen::Quaternionf offset_q = robot_anchor_q * ref_anchor_q.conjugate();
            offset_q.normalize();
            const float yaw_offset = quat_to_yaw(offset_q.w(), offset_q.x(), offset_q.y(), offset_q.z());
            anchor_frame_offset_q_ = Eigen::Quaternionf(Eigen::AngleAxisf(yaw_offset, Eigen::Vector3f::UnitZ()));
        }
        ref_world_align_q = anchor_frame_offset_q_;
        ref_root_q = (ref_world_align_q * ref_root_q).normalized();
    }

    if (use_motion_root_command) {
        const Eigen::Matrix3f root_rot_b =
            (current_root_quat.normalized().conjugate() * ref_root_q).toRotationMatrix();
        root_ori_b_ << root_rot_b(0, 0), root_rot_b(0, 1),
                       root_rot_b(1, 0), root_rot_b(1, 1),
                       root_rot_b(2, 0), root_rot_b(2, 1);

        const Eigen::Matrix3f root_rot_b_unbiased =
            (current_root_quat_unbiased.normalized().conjugate() * ref_root_q).toRotationMatrix();
        root_ori_b_unbiased_ << root_rot_b_unbiased(0, 0), root_rot_b_unbiased(0, 1),
                                root_rot_b_unbiased(1, 0), root_rot_b_unbiased(1, 1),
                                root_rot_b_unbiased(2, 0), root_rot_b_unbiased(2, 1);
    } else {
        root_ori_b_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
        root_ori_b_unbiased_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
    }

    const float yaw_ref = quat_to_yaw(ref_root_q.w(), ref_root_q.x(), ref_root_q.y(), ref_root_q.z());
    const Eigen::Quaternionf ref_yaw_q =
        Eigen::AngleAxisf(yaw_ref, Eigen::Vector3f::UnitZ()) * Eigen::Quaternionf::Identity();
    const float yaw_error = wrap_to_pi(yaw_ref - current_root_yaw);
    yaw_command_ << std::cos(yaw_error), std::sin(yaw_error);
    if (use_motion_velocity_command) {
        const Eigen::Vector3f ref_lin_vel_navi = ref_yaw_q.conjugate() * (ref_world_align_q * frame.root_lin_vel_w);
        const Eigen::Vector3f ref_ang_vel_navi = ref_yaw_q.conjugate() * (ref_world_align_q * frame.root_ang_vel_w);
        xy_yaw_vel_ << ref_lin_vel_navi.x(), ref_lin_vel_navi.y(), ref_ang_vel_navi.z();
    } else {
        xy_yaw_vel_.setZero();
    }

    foot_support_state_.setZero();
    if (frame.left_foot_contact_state >= 0 && frame.left_foot_contact_state <= 2) {
        foot_support_state_[frame.left_foot_contact_state] = 1.0f;
    }
    if (frame.right_foot_contact_state >= 0 && frame.right_foot_contact_state <= 2) {
        foot_support_state_[3 + frame.right_foot_contact_state] = 1.0f;
    }
    ref_com_rel_navi_ = frame.ref_com_rel_navi;
    ref_com_vel_navi_ = frame.ref_com_vel_navi;
}

void State_Track::ReferenceLoader::update_live_future_commands(const std::deque<LiveFrame>& queue_snapshot,
                                                               const LiveFrame& fill_frame,
                                                               bool no_global_mode,
                                                               const Eigen::Quaternionf& current_root_quat,
                                                               bool use_motion_root_command,
                                                               bool use_motion_velocity_command)
{
    if (future_commands_.size() != static_cast<size_t>(future_horizon_ * kFutureCommandDim)) {
        future_commands_.assign(future_horizon_ * kFutureCommandDim, 0.0f);
    }
    if (future_command_with_foot_support_state_.size()
        != static_cast<size_t>(future_horizon_ * kFutureCommandWithFootSupportDim)) {
        future_command_with_foot_support_state_.assign(
            future_horizon_ * kFutureCommandWithFootSupportDim,
            0.0f);
    }

    const Eigen::Quaternionf ref_world_align_q = no_global_mode
        ? anchor_frame_offset_q_
        : Eigen::Quaternionf::Identity();

    for (size_t horizon_idx = 0; horizon_idx < future_horizon_; ++horizon_idx) {
        const LiveFrame& frame = horizon_idx < queue_snapshot.size()
            ? queue_snapshot[horizon_idx]
            : fill_frame;
        Eigen::Quaternionf future_ref_root_q = (ref_world_align_q * frame.root_quat_w).normalized();
        const Eigen::Matrix3f motion_future_root_rot_b =
            (current_root_quat.normalized().conjugate() * future_ref_root_q).toRotationMatrix();
        Eigen::Matrix<float, 6, 1> motion_future_root_ori_b;
        motion_future_root_ori_b << motion_future_root_rot_b(0, 0), motion_future_root_rot_b(0, 1),
                                    motion_future_root_rot_b(1, 0), motion_future_root_rot_b(1, 1),
                                    motion_future_root_rot_b(2, 0), motion_future_root_rot_b(2, 1);

        const float future_yaw_ref = quat_to_yaw(
            future_ref_root_q.w(),
            future_ref_root_q.x(),
            future_ref_root_q.y(),
            future_ref_root_q.z());
        const Eigen::Quaternionf future_ref_yaw_q =
            Eigen::AngleAxisf(future_yaw_ref, Eigen::Vector3f::UnitZ()) * Eigen::Quaternionf::Identity();
        const Eigen::Vector3f motion_future_lin_vel_navi =
            future_ref_yaw_q.conjugate() * (ref_world_align_q * frame.root_lin_vel_w);
        const Eigen::Vector3f motion_future_ang_vel_navi =
            future_ref_yaw_q.conjugate() * (ref_world_align_q * frame.root_ang_vel_w);
        Eigen::Vector3f motion_future_xy_yaw_vel;
        motion_future_xy_yaw_vel << motion_future_lin_vel_navi.x(),
                                    motion_future_lin_vel_navi.y(),
                                    motion_future_ang_vel_navi.z();

        Eigen::Matrix<float, 6, 1> future_root_ori_b;
        if (use_motion_root_command) {
            future_root_ori_b = motion_future_root_ori_b;
        } else {
            future_root_ori_b << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
        }

        Eigen::Vector3f future_xy_yaw_vel = Eigen::Vector3f::Zero();
        if (use_motion_velocity_command) {
            future_xy_yaw_vel = motion_future_xy_yaw_vel;
        }

        size_t out_offset = horizon_idx * kFutureCommandDim;
        for (int i = 0; i < 6; ++i) {
            future_commands_[out_offset++] = future_root_ori_b[i];
        }
        for (int i = 0; i < 3; ++i) {
            future_commands_[out_offset++] = future_xy_yaw_vel[i];
        }
        for (int i = 0; i < kJointDim; ++i) {
            future_commands_[out_offset++] = frame.joint_pos[i];
        }

        const size_t command_with_foot_offset = horizon_idx * kFutureCommandWithFootSupportDim;
        size_t with_foot_offset = command_with_foot_offset;
        for (int i = 0; i < 6; ++i) {
            future_command_with_foot_support_state_[with_foot_offset++] = motion_future_root_ori_b[i];
        }
        for (int i = 0; i < 3; ++i) {
            future_command_with_foot_support_state_[with_foot_offset++] = motion_future_xy_yaw_vel[i];
        }
        for (int i = 0; i < kJointDim; ++i) {
            future_command_with_foot_support_state_[with_foot_offset++] = frame.joint_pos[i];
        }
        write_foot_support_onehot(frame.left_foot_contact_state,
                                  frame.right_foot_contact_state,
                                  future_command_with_foot_support_state_,
                                  command_with_foot_offset + kFutureCommandDim);
    }
}

std::filesystem::path State_Track::ReferenceLoader::ensure_cache_file(const std::filesystem::path& motion_file) const
{
    if (motion_file.extension() != ".npz") {
        spdlog::info("Track: motion file '{}' is already in cache format", motion_file.string());
        return motion_file;
    }

    auto cache_file = motion_file;
    cache_file.replace_extension(".et1trk");

    bool regenerate = !std::filesystem::exists(cache_file)
        || std::filesystem::last_write_time(cache_file) < std::filesystem::last_write_time(motion_file);

    if (!regenerate) {
        std::ifstream in(cache_file, std::ios::binary);
        Header header{};
        in.read(reinterpret_cast<char*>(&header), sizeof(header));
        const bool header_ok = static_cast<bool>(in) && std::string(header.magic, header.magic + 7) == "ET1TRK1";
        if (!header_ok || header.version < kCacheVersion) {
            regenerate = true;
            spdlog::info("Track: cache '{}' is stale/incompatible, regenerating", cache_file.string());
        }
    }

    if (!regenerate) {
        spdlog::info("Track: reusing existing cache '{}'", cache_file.string());
        return cache_file;
    }

    const auto repo_root = std::filesystem::weakly_canonical(param::proj_dir / "../../..");
    const auto script_path = repo_root / "scripts" / "et1" / "convert_track_npz.py";

    std::ostringstream cmd;
    cmd << "python3 \"" << script_path.string() << "\""
        << " --input \"" << motion_file.string() << "\""
        << " --output \"" << cache_file.string() << "\"";

    spdlog::info("Track: converting NPZ '{}' -> '{}'", motion_file.string(), cache_file.string());
    const int ret = std::system(cmd.str().c_str());
    if (ret != 0 || !std::filesystem::exists(cache_file)) {
        throw std::runtime_error("Failed to convert track NPZ to runtime cache: " + motion_file.string());
    }
    spdlog::info("Track: cache generated successfully");
    return cache_file;
}

void State_Track::ReferenceLoader::load_cache_file(const std::filesystem::path& cache_file)
{
    spdlog::info("Track: loading cache file '{}'", cache_file.string());
    std::ifstream in(cache_file, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open track cache file: " + cache_file.string());
    }

    Header header{};
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!in || std::string(header.magic, header.magic + 7) != "ET1TRK1") {
        throw std::runtime_error("Invalid track cache header: " + cache_file.string());
    }
    if (header.version < kCacheVersion) {
        throw std::runtime_error("Track cache version is too old; regenerate cache for: " + cache_file.string());
    }

    auto dtype_item_size = [](CacheDType dtype) -> size_t {
        switch (dtype) {
            case CacheDType::Float32: return sizeof(float);
            case CacheDType::Float64: return sizeof(double);
            case CacheDType::Bool: return sizeof(bool);
            case CacheDType::Int32: return sizeof(int32_t);
            case CacheDType::Int64: return sizeof(int64_t);
            case CacheDType::UInt8: return sizeof(uint8_t);
            case CacheDType::Int8: return sizeof(int8_t);
        }
        throw std::runtime_error("Unknown cache dtype");
    };

    bool found_joint_pos = false;
    bool found_joint_vel = false;
    bool found_body_pos = false;
    bool found_body_quat = false;
    bool found_body_lin_vel = false;
    bool found_body_ang_vel = false;

    for (uint32_t array_idx = 0; array_idx < header.array_count; ++array_idx) {
        uint32_t name_len = 0;
        uint32_t dtype_code = 0;
        uint32_t ndim = 0;
        in.read(reinterpret_cast<char*>(&name_len), sizeof(name_len));
        if (!in) {
            throw std::runtime_error("Failed to read cache array name length: " + cache_file.string());
        }

        std::string name(name_len, '\0');
        in.read(name.data(), name_len);
        in.read(reinterpret_cast<char*>(&dtype_code), sizeof(dtype_code));
        in.read(reinterpret_cast<char*>(&ndim), sizeof(ndim));
        if (!in) {
            throw std::runtime_error("Failed to read cache array header: " + cache_file.string());
        }

        std::vector<uint32_t> dims(ndim, 0);
        if (ndim > 0) {
            in.read(reinterpret_cast<char*>(dims.data()), sizeof(uint32_t) * ndim);
            if (!in) {
                throw std::runtime_error("Failed to read cache array dims: " + cache_file.string());
            }
        }

        uint64_t byte_count = 0;
        in.read(reinterpret_cast<char*>(&byte_count), sizeof(byte_count));
        if (!in) {
            throw std::runtime_error("Failed to read cache array byte count: " + cache_file.string());
        }

        const auto dtype = static_cast<CacheDType>(dtype_code);
        const size_t item_size = dtype_item_size(dtype);
        size_t element_count = 1;
        for (uint32_t dim : dims) {
            element_count *= dim;
        }
        if (element_count * item_size != byte_count) {
            throw std::runtime_error("Cache array byte size mismatch for '" + name + "': " + cache_file.string());
        }

        std::vector<char> raw(byte_count);
        if (byte_count > 0) {
            in.read(raw.data(), static_cast<std::streamsize>(byte_count));
            if (!in) {
                throw std::runtime_error("Failed to read cache array payload for '" + name + "': " + cache_file.string());
            }
        }

        auto convert_to_float = [&](std::vector<float>& out) {
            out.resize(element_count);
            if (dtype == CacheDType::Float32) {
                std::memcpy(out.data(), raw.data(), byte_count);
            } else if (dtype == CacheDType::Float64) {
                const auto* src = reinterpret_cast<const double*>(raw.data());
                for (size_t i = 0; i < element_count; ++i) {
                    out[i] = static_cast<float>(src[i]);
                }
            } else {
                throw std::runtime_error("Unsupported dtype for float conversion in array '" + name + "'");
            }
        };
        auto convert_to_int64 = [&](std::vector<int64_t>& out) {
            out.resize(element_count);
            if (dtype == CacheDType::Int64) {
                std::memcpy(out.data(), raw.data(), byte_count);
            } else if (dtype == CacheDType::Int32) {
                const auto* src = reinterpret_cast<const int32_t*>(raw.data());
                for (size_t i = 0; i < element_count; ++i) {
                    out[i] = static_cast<int64_t>(src[i]);
                }
            } else if (dtype == CacheDType::UInt8) {
                const auto* src = reinterpret_cast<const uint8_t*>(raw.data());
                for (size_t i = 0; i < element_count; ++i) {
                    out[i] = static_cast<int64_t>(src[i]);
                }
            } else if (dtype == CacheDType::Int8) {
                const auto* src = reinterpret_cast<const int8_t*>(raw.data());
                for (size_t i = 0; i < element_count; ++i) {
                    out[i] = static_cast<int64_t>(src[i]);
                }
            } else {
                throw std::runtime_error("Unsupported dtype for int conversion in array '" + name + "'");
            }
        };

        if (name == "joint_pos") {
            if (dims.size() != 2 || dims[1] != kJointDim) {
                throw std::runtime_error("Unexpected joint_pos shape in cache: " + cache_file.string());
            }
            frame_count_ = dims[0];
            convert_to_float(joint_pos_seq_);
            found_joint_pos = true;
        } else if (name == "joint_vel") {
            if (dims.size() != 2 || dims[1] != kJointDim) {
                throw std::runtime_error("Unexpected joint_vel shape in cache: " + cache_file.string());
            }
            convert_to_float(joint_vel_seq_);
            found_joint_vel = true;
        } else if (name == "body_pos_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_pos_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_pos_w_seq_);
            found_body_pos = true;
        } else if (name == "body_quat_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 4) {
                throw std::runtime_error("Unexpected body_quat_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_quat_w_seq_);
            found_body_quat = true;
        } else if (name == "body_lin_vel_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_lin_vel_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_lin_vel_w_seq_);
            found_body_lin_vel = true;
        } else if (name == "body_ang_vel_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_ang_vel_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_ang_vel_w_seq_);
            found_body_ang_vel = true;
        } else if (name == "left_foot_contact_state") {
            if (dims.size() != 1) {
                throw std::runtime_error("Unexpected left_foot_contact_state shape in cache: " + cache_file.string());
            }
            convert_to_int64(left_foot_contact_state_seq_);
        } else if (name == "right_foot_contact_state") {
            if (dims.size() != 1) {
                throw std::runtime_error("Unexpected right_foot_contact_state shape in cache: " + cache_file.string());
            }
            convert_to_int64(right_foot_contact_state_seq_);
        } else if (name == "ref_com_rel_navi") {
            if (dims.size() != 2 || dims[1] != 3) {
                throw std::runtime_error("Unexpected ref_com_rel_navi shape in cache: " + cache_file.string());
            }
            convert_to_float(ref_com_rel_navi_seq_);
        } else if (name == "ref_com_vel_navi") {
            if (dims.size() != 2 || dims[1] != 3) {
                throw std::runtime_error("Unexpected ref_com_vel_navi shape in cache: " + cache_file.string());
            }
            convert_to_float(ref_com_vel_navi_seq_);
        }
    }

    if (!found_joint_pos || !found_joint_vel || !found_body_pos || !found_body_quat
        || !found_body_lin_vel || !found_body_ang_vel) {
        throw std::runtime_error("ET1 track cache missing required motion arrays: " + cache_file.string());
    }
    if ((!left_foot_contact_state_seq_.empty() && left_foot_contact_state_seq_.size() != frame_count_)
        || (!right_foot_contact_state_seq_.empty() && right_foot_contact_state_seq_.size() != frame_count_)) {
        throw std::runtime_error("Foot contact state length mismatch in cache: " + cache_file.string());
    }
    if ((!ref_com_rel_navi_seq_.empty() && ref_com_rel_navi_seq_.size() != frame_count_ * 3)
        || (!ref_com_vel_navi_seq_.empty() && ref_com_vel_navi_seq_.size() != frame_count_ * 3)) {
        throw std::runtime_error("Reference COM observation length mismatch in cache: " + cache_file.string());
    }
}

float State_Track::ReferenceLoader::wrap_to_pi(float angle) const
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

void State_Track::request_motion_file(const std::filesystem::path& motion_file)
{
    std::lock_guard<std::mutex> lock(pending_motion_mutex_);
    pending_motion_file_ = motion_file;
    spdlog::info("Track: pending motion request set to '{}'", motion_file.string());
}

bool State_Track::has_pending_motion_request()
{
    std::lock_guard<std::mutex> lock(pending_motion_mutex_);
    return pending_motion_file_.has_value();
}

std::optional<std::filesystem::path> State_Track::consume_pending_motion_file()
{
    std::lock_guard<std::mutex> lock(pending_motion_mutex_);
    auto motion_file = pending_motion_file_;
    pending_motion_file_.reset();
    return motion_file;
}

State_Track::State_Track(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    spdlog::info("Track: constructing state '{}'", state_string);
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    // Read override_joint_ids from config (joints whose positions are replaced by reference motion)
    if (cfg["override_joint_ids"].IsDefined()) {
        override_joint_ids_ = cfg["override_joint_ids"].as<std::vector<int>>();
        spdlog::info("Track: override_joint_ids = [{}]", [&]() {
            std::string s;
            for (size_t i = 0; i < override_joint_ids_.size(); ++i) {
                s += std::to_string(override_joint_ids_[i]) + (i + 1 < override_joint_ids_.size() ? ", " : "");
            }
            return s;
        }());
    } else {
        spdlog::info("Track: no override_joint_ids configured");
    }

    no_global_mode_ = cfg["no_global_mode"].as<bool>(false);
    spdlog::info("Track: no_global_mode = {}", no_global_mode_ ? "true" : "false");
    use_motion_root_command_ = cfg["use_motion_root_command"].as<bool>(false);
    use_motion_velocity_command_ = cfg["use_motion_velocity_command"].as<bool>(false);
    spdlog::info("Track: use_motion_root_command = {}, use_motion_velocity_command = {}",
                 use_motion_root_command_ ? "true" : "false",
                 use_motion_velocity_command_ ? "true" : "false");
    one_shot_mode_ = cfg["one_shot_mode"].as<bool>(false);
    require_requested_motion_ = cfg["require_requested_motion"].as<bool>(false);
    spdlog::info("Track: one_shot_mode = {}, require_requested_motion = {}",
                 one_shot_mode_ ? "true" : "false",
                 require_requested_motion_ ? "true" : "false");
    hybrid_locomotion_enabled_ = cfg["hybrid_locomotion"].as<bool>(false);
    spdlog::info("Track: hybrid_locomotion = {}", hybrid_locomotion_enabled_ ? "true" : "false");
    if (one_shot_mode_ && !hybrid_locomotion_enabled_ && FSMStringMap.right.count("Velocity")) {
        registered_checks.push_back({
            [this]() -> bool { return playback_complete_; },
            FSMStringMap.right.at("Velocity"),
            "track playback complete"
        });
    }
    debug_dump_first_frame_ = cfg["debug_dump_first_frame"].as<bool>(false);
    const std::string default_debug_dir = "debug/et1_track_first_frame";
    debug_dump_dir_ = cfg["debug_dump_dir"]
        ? std::filesystem::path(cfg["debug_dump_dir"].as<std::string>())
        : std::filesystem::path(default_debug_dir);
    if (!debug_dump_dir_.is_absolute()) {
        debug_dump_dir_ = param::proj_dir / debug_dump_dir_;
    }
    if (debug_dump_first_frame_) {
        spdlog::info("Track: first-frame debug dump enabled at '{}'", debug_dump_dir_.string());
    }

    observation_dump_enabled_ = cfg["dump_observations"].as<bool>(state_string == "Dance2");
    observation_dump_base_file_ = cfg["observation_dump_file"]
        ? std::filesystem::path(cfg["observation_dump_file"].as<std::string>())
        : std::filesystem::path("debug/dance2_pokerface_observations.txt");
    if (!observation_dump_base_file_.is_absolute()) {
        observation_dump_base_file_ = param::proj_dir / observation_dump_base_file_;
    }
    observation_dump_file_ = observation_dump_base_file_;
    if (observation_dump_enabled_) {
        spdlog::info("Track: per-frame observation dump enabled at '{}'", observation_dump_file_.string());
    }

    qpos_visualizer_dump_enabled_ = cfg["dump_qpos_visualizer"].as<bool>(false);
    qpos_visualizer_dump_base_file_ = cfg["qpos_visualizer_dump_file"]
        ? std::filesystem::path(cfg["qpos_visualizer_dump_file"].as<std::string>())
        : std::filesystem::path("debug/qpos_visualizer/et1_track_qpos_visualizer.csv");
    if (!qpos_visualizer_dump_base_file_.is_absolute()) {
        qpos_visualizer_dump_base_file_ = param::proj_dir / qpos_visualizer_dump_base_file_;
    }
    qpos_visualizer_dump_file_ = qpos_visualizer_dump_base_file_;
    if (qpos_visualizer_dump_enabled_) {
        spdlog::info("Track: qpos visualizer CSV dump enabled at '{}'", qpos_visualizer_dump_file_.string());
    }

    if (cfg["head_hold_sdk_slots"]) {
        head_hold_sdk_slots_ = cfg["head_hold_sdk_slots"].as<std::vector<int>>();
        head_hold_q_ = cfg["head_hold_q"]
            ? cfg["head_hold_q"].as<std::vector<float>>()
            : std::vector<float>(head_hold_sdk_slots_.size(), 0.0f);
        head_hold_kp_ = cfg["head_hold_kp"]
            ? cfg["head_hold_kp"].as<std::vector<float>>()
            : std::vector<float>(head_hold_sdk_slots_.size(), 0.0f);
        head_hold_kd_ = cfg["head_hold_kd"]
            ? cfg["head_hold_kd"].as<std::vector<float>>()
            : std::vector<float>(head_hold_sdk_slots_.size(), 0.0f);
        if (head_hold_q_.size() != head_hold_sdk_slots_.size()
            || head_hold_kp_.size() != head_hold_sdk_slots_.size()
            || head_hold_kd_.size() != head_hold_sdk_slots_.size()) {
            throw std::runtime_error("Track: head hold config size mismatch.");
        }
        spdlog::info("Track: head hold enabled for {} sdk joints", head_hold_sdk_slots_.size());
    }
    if (param::config["FSM"]["FixStand"]) {
        const auto fixstand_cfg = param::config["FSM"]["FixStand"];
        hybrid_idle_hold_kp_ = fixstand_cfg["kp"].as<std::vector<float>>();
        hybrid_idle_hold_kd_ = fixstand_cfg["kd"].as<std::vector<float>>();
        if (fixstand_cfg["qs"] && fixstand_cfg["qs"].IsSequence() && fixstand_cfg["qs"].size() > 0) {
            hybrid_idle_hold_q_ = fixstand_cfg["qs"][fixstand_cfg["qs"].size() - 1].as<std::vector<float>>();
        }
        if (hybrid_locomotion_enabled_) {
            spdlog::info("Track: hybrid idle hold loaded from FixStand for {} sdk slots",
                         hybrid_idle_hold_q_.size());
        }
    }

    const std::string policy_file = cfg["policy_file"] ? cfg["policy_file"].as<std::string>() : "policy.onnx";
    const std::string deploy_file = cfg["deploy_file"] ? cfg["deploy_file"].as<std::string>() : "deploy.yaml";
    const auto policy_path = policy_dir / "exported" / policy_file;
    const auto deploy_path = policy_dir / "params" / deploy_file;
    YAML::Node deploy_cfg = YAML::LoadFile(deploy_path);
    if (deploy_cfg["joint_ids_map"]) {
        const auto joint_ids_map = deploy_cfg["joint_ids_map"].as<std::vector<int>>();
        for (const int joint_id : joint_ids_map) {
            if (joint_id < 0 || joint_id >= ReferenceLoader::kJointDim) {
                throw std::runtime_error(
                    "Track: deploy joint_ids_map contains index "
                    + std::to_string(joint_id)
                    + " outside ET1 live/reference joint dimension "
                    + std::to_string(ReferenceLoader::kJointDim));
            }
        }
    }
    request_file_ = cfg["request_file"]
        ? std::filesystem::path(cfg["request_file"].as<std::string>())
        : std::filesystem::path("debug/general_tracker_request.txt");
    if (!request_file_.is_absolute()) {
        request_file_ = param::proj_dir / request_file_;
    }

    default_motion_file_ = cfg["motion_file"].as<std::string>();
    if (!default_motion_file_.is_absolute()) {
        default_motion_file_ = param::proj_dir / default_motion_file_;
    }
    const float deploy_step_dt = deploy_cfg["step_dt"].as<float>();
    reference_fps_ = deploy_step_dt > 0.0f ? 1.0f / deploy_step_dt : 50.0f;
    if (cfg["fps"]) {
        const float configured_fps = cfg["fps"].as<float>();
        if (std::abs(configured_fps - reference_fps_) > 1e-3f) {
            spdlog::warn(
                "Track: ignoring FSM fps={} because deploy step_dt={} implies fps={}",
                configured_fps,
                deploy_step_dt,
                reference_fps_);
        }
    }
    spdlog::info("Track: resolved default motion file '{}'", default_motion_file_.string());
    reference_future_horizon_ = infer_future_horizon(deploy_cfg);
    if (cfg["live_stream"] && cfg["live_stream"]["enabled"].as<bool>(false)) {
        live_stream_enabled_ = true;
        ReferenceLoader::LiveStreamConfig live_config;
        const auto live_cfg = cfg["live_stream"];
        live_config.endpoint = live_cfg["endpoint"]
            ? live_cfg["endpoint"].as<std::string>()
            : live_config.endpoint;
        live_config.topic = live_cfg["topic"]
            ? live_cfg["topic"].as<std::string>()
            : live_config.topic;
        live_config.max_queue_frames = live_cfg["max_queue_frames"]
            ? live_cfg["max_queue_frames"].as<size_t>()
            : live_config.max_queue_frames;
        live_config.initial_buffer_frames = infer_live_initial_buffer_frames(deploy_cfg);
        live_config.receive_timeout_ms = live_cfg["receive_timeout_ms"]
            ? live_cfg["receive_timeout_ms"].as<int>()
            : live_config.receive_timeout_ms;
        live_config.high_water_mark = live_cfg["high_water_mark"]
            ? live_cfg["high_water_mark"].as<int>()
            : live_config.high_water_mark;
        live_config.initial_buffer_frames = std::max<size_t>(1, live_config.initial_buffer_frames);
        live_config.max_queue_frames = std::max(live_config.initial_buffer_frames, live_config.max_queue_frames);
        live_initial_buffer_frames_ = live_config.initial_buffer_frames;
        spdlog::info("Track: live stream enabled endpoint='{}' topic='{}' queue={} initial_buffer={} future_horizon={}",
                     live_config.endpoint,
                     live_config.topic,
                     live_config.max_queue_frames,
                     live_config.initial_buffer_frames,
                     reference_future_horizon_);
        reference_ = std::make_shared<ReferenceLoader>(live_config, reference_fps_, reference_future_horizon_);
    } else {
        reference_ = std::make_shared<ReferenceLoader>(
            default_motion_file_,
            reference_fps_,
            reference_future_horizon_);
    }
    reference = reference_;
    if (require_requested_motion_) {
        spdlog::info("Track: default reference loaded for observation shape probing only; execution waits for external motion request");
    } else {
        spdlog::info("Track: reference pointer initialized");
    }

    spdlog::info("Track: loading deploy config '{}'", deploy_path.string());
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr, HighState_t::SharedPtr>>(
            FSMState::lowstate, FSMState::highstate)
    );
    policy_kp_ = env->cfg["policy_kp"].as<std::vector<float>>();
    policy_kd_ = env->cfg["policy_kd"].as<std::vector<float>>();
    configure_pd_gain_randomization();
    spdlog::info("Track: deploy config loaded, constructing ONNX session '{}'", policy_path.string());
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_path.string());
    spdlog::info("Track: ONNX session created successfully");

    if (hybrid_locomotion_enabled_) {
        const auto locomotion_policy_dir = param::parser_policy_dir(
            cfg["locomotion_policy_dir"]
                ? cfg["locomotion_policy_dir"].as<std::string>()
                : param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>());
        const std::string locomotion_policy_file = cfg["locomotion_policy_file"]
            ? cfg["locomotion_policy_file"].as<std::string>()
            : param::config["FSM"]["Velocity"]["policy_file"].as<std::string>();
        const std::string locomotion_deploy_file = cfg["locomotion_deploy_file"]
            ? cfg["locomotion_deploy_file"].as<std::string>()
            : param::config["FSM"]["Velocity"]["deploy_file"].as<std::string>();

        spdlog::info("Track: loading hybrid locomotion deploy config '{}'",
                     (locomotion_policy_dir / "params" / locomotion_deploy_file).string());
        locomotion_env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
            YAML::LoadFile(locomotion_policy_dir / "params" / locomotion_deploy_file),
            std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr, HighState_t::SharedPtr>>(
                FSMState::lowstate, FSMState::highstate)
        );
        locomotion_env_->alg = std::make_unique<isaaclab::OrtRunner>(
            (locomotion_policy_dir / "exported" / locomotion_policy_file).string());
        locomotion_policy_kp_ = locomotion_env_->cfg["policy_kp"]
            ? locomotion_env_->cfg["policy_kp"].as<std::vector<float>>()
            : locomotion_env_->robot->data.joint_stiffness;
        locomotion_policy_kd_ = locomotion_env_->cfg["policy_kd"]
            ? locomotion_env_->cfg["policy_kd"].as<std::vector<float>>()
            : locomotion_env_->robot->data.joint_damping;
        spdlog::info("Track: hybrid locomotion policy loaded");
    }

    const std::vector<std::string> tracker_targets = {
        "GeneralTrackerCJM",
        "GeneralTrackerCLN",
    };
    for (const auto& target_state : tracker_targets) {
        if (target_state == state_string || !FSMStringMap.right.count(target_state)) {
            continue;
        }
        registered_checks.push_back({
            [this, target_state]() -> bool {
                return route_profile_request_to(target_state);
            },
            FSMStringMap.right.at(target_state),
            "external " + target_state + " profile request"
        });
    }

    // this->registered_checks.emplace_back(
    //     std::make_pair(
    //         [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
    //         FSMStringMap.right.at("Passive")
    //     )
    // );
    // Temporarily disable bad_orientation-triggered state switch in Track.
}

long long State_Track::active_motion_duration_ms() const
{
    const float duration_s = reference_ ? std::max(0.0f, reference_->duration()) : 0.0f;
    return static_cast<long long>(std::round(duration_s * 1000.0f));
}

bool State_Track::consume_app_start_request()
{
    if (!param::is_app_dance_debug_mode() || getStateString() != "GeneralTrackerCLN") {
        return false;
    }

    Et1DanceCommand app_command;
    if (!Et1DanceBridge::Instance().ConsumeStart(app_command)) {
        return false;
    }

    stop_locomotion_policy_thread();
    active_app_command_ = app_command;
    active_app_request_ = true;
    active_tracking_ = false;

    try {
        active_tracking_ = start_requested_motion(app_command.motion_path);
        if (active_tracking_) {
            Et1DanceBridge::Instance().PublishStatusWithProgress(
                active_app_command_.request_id,
                active_app_command_.dance_id,
                "play_start",
                active_motion_duration_ms(),
                0,
                0.0,
                active_app_command_.motion_path);
            spdlog::info("Track: app requested motion started; waiting for playback completion before returning to idle");
        } else {
            Et1DanceBridge::Instance().PublishStatus(
                active_app_command_.request_id,
                active_app_command_.dance_id,
                "play_failed",
                "failed to load requested ET1 track motion");
            active_app_request_ = false;
        }
    } catch (const std::exception& e) {
        Et1DanceBridge::Instance().PublishStatus(
            active_app_command_.request_id,
            active_app_command_.dance_id,
            "play_failed",
            e.what());
        spdlog::error("Track: failed to load app requested motion '{}': {}",
                      app_command.motion_path,
                      e.what());
        active_app_request_ = false;
        active_tracking_ = false;
    }
    return true;
}

void State_Track::enter()
{
    spdlog::info("Track: enter");
    stop_locomotion_policy_thread();
    has_initial_yaw_bias_ = false;
    initial_yaw_bias_ = 0.0f;
    first_frame_debug_dumped_ = false;
    playback_complete_ = false;
    active_tracking_ = false;
    active_app_request_ = false;
    active_app_command_ = Et1DanceCommand{};
    startup_alignment_pending_ = false;
    startup_upper_body_interp_active_ = false;
    tracking_playback_time_ = 0.0f;

    if (consume_app_start_request()) {
        if (!active_tracking_ && !hybrid_locomotion_enabled_) {
            playback_complete_ = true;
            return;
        }
    } else if (auto requested_motion = consume_pending_motion_file()) {
        active_tracking_ = start_requested_motion(*requested_motion);
        if (!active_tracking_ && !hybrid_locomotion_enabled_) {
            playback_complete_ = true;
            return;
        }
    } else if (live_stream_enabled_) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        size_t live_buffer_size = reference_ ? reference_->live_buffer_size() : 0;
        while (live_buffer_size < live_initial_buffer_frames_
               && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            live_buffer_size = reference_ ? reference_->live_buffer_size() : 0;
        }
        if (live_buffer_size < live_initial_buffer_frames_) {
            spdlog::warn(
                "Track: live stream has only {} buffered frames, need {}; returning to Velocity without running policy",
                live_buffer_size,
                live_initial_buffer_frames_);
            playback_complete_ = true;
            return;
        }
        reference = reference_;
        active_tracking_ = true;
    } else if (require_requested_motion_ && !hybrid_locomotion_enabled_) {
        spdlog::warn("Track: no requested motion is pending; tracker will return to Velocity without running fallback motion_file");
        playback_complete_ = true;
        return;
    } else if (!hybrid_locomotion_enabled_) {
        reference = reference_;
        active_tracking_ = true;
    }

    if (active_tracking_ && !reference_) {
        spdlog::warn("Track: no reference is loaded; returning to Velocity");
        playback_complete_ = true;
        return;
    }

    open_observation_dump();
    open_qpos_visualizer_dump();
    auto* active_env = active_tracking_ ? env.get() : locomotion_env_.get();
    const auto& active_kp = active_tracking_ ? policy_kp_ : locomotion_policy_kp_;
    const auto& active_kd = active_tracking_ ? policy_kd_ : locomotion_policy_kd_;
    const bool clear_uncontrolled_motors = active_tracking_ || !hybrid_locomotion_enabled_;
    initialize_policy_motors(active_env, active_kp, active_kd, clear_uncontrolled_motors);
    apply_hybrid_idle_hold();
    apply_head_hold_command();

    if (active_tracking_) {
        reference_->reset(env->robot->data.default_joint_pos);
        spdlog::info("Track: reference reset with default joint pose of size {}", env->robot->data.default_joint_pos.size());
        env->reset();
    } else if (locomotion_env_) {
        locomotion_env_->reset();
        start_locomotion_policy_thread();
    }
    reset_pd_gain_scales();
    spdlog::info("Track: environment reset complete");

    if (no_global_mode_) {
        active_env = active_tracking_ ? env.get() : locomotion_env_.get();
        if (!active_env) {
            return;
        }
        active_env->robot->update();
        const auto& live_state = active_env->robot->data.live_state;
        initial_yaw_bias_ = quat_to_yaw(
            live_state.root_quat_w.w(),
            live_state.root_quat_w.x(),
            live_state.root_quat_w.y(),
            live_state.root_quat_w.z()
        );
        has_initial_yaw_bias_ = true;
        if (active_tracking_ && reference_) {
            reference_->calibrate_anchor_frame(
                0.0f,
                active_env->robot->data.joint_pos,
                live_state.root_quat_w,
                true);
            if (!reference_->is_live_stream()) {
                startup_alignment_pending_ = true;
                startup_alignment_start_time_s_ =
                    static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
                tracking_playback_time_ = 0.0f;
                begin_startup_upper_body_interpolation(sample_reference_joint_pos(0.0f));
                spdlog::info("Track: startup playback hold enabled for {:.3f}s", startup_alignment_duration_s_);
            }
        }
        spdlog::info("Track: no_global_mode anchor yaw alignment initialized at robot yaw {:.6f} rad", initial_yaw_bias_);
    }
}

void State_Track::run()
{
    if (playback_complete_) {
        return;
    }

    Et1DanceCommand stop_command;
    if (active_app_request_
        && Et1DanceBridge::Instance().ConsumeStop(stop_command)
        && active_tracking_) {
        Et1DanceBridge::Instance().PublishStatus(
            stop_command.request_id.empty() ? active_app_command_.request_id : stop_command.request_id,
            stop_command.dance_id != 0 ? stop_command.dance_id : active_app_command_.dance_id,
            "play_stopping");
        active_tracking_ = false;
        close_observation_dump();
        if (hybrid_locomotion_enabled_ && locomotion_env_) {
            locomotion_env_->reset();
            start_locomotion_policy_thread();
        } else {
            playback_complete_ = true;
        }
        Et1DanceBridge::Instance().PublishStatusWithProgress(
            stop_command.request_id.empty() ? active_app_command_.request_id : stop_command.request_id,
            stop_command.dance_id != 0 ? stop_command.dance_id : active_app_command_.dance_id,
            "play_stopped",
            active_motion_duration_ms(),
            active_motion_duration_ms(),
            1.0);
        active_app_request_ = false;
        return;
    }

    if (hybrid_locomotion_enabled_ && !active_tracking_) {
        if (consume_app_start_request()) {
            if (!active_tracking_ && locomotion_env_) {
                start_locomotion_policy_thread();
            }
        } else {
            poll_motion_request_file();
        }
        if (!active_tracking_ && !active_app_request_) {
            if (auto requested_motion = consume_pending_motion_file()) {
                stop_locomotion_policy_thread();
                active_tracking_ = start_requested_motion(*requested_motion);
                if (!active_tracking_ && locomotion_env_) {
                    start_locomotion_policy_thread();
                }
            }
        }
        if (!active_tracking_) {
            run_locomotion_policy();
            return;
        }
    }

    run_tracking_policy();
}

void State_Track::run_tracking_policy()
{
    env->robot->update();
    const auto& live_state = env->robot->data.live_state;
    const float current_root_yaw = quat_to_yaw(
        live_state.root_quat_w.w(),
        live_state.root_quat_w.x(),
        live_state.root_quat_w.y(),
        live_state.root_quat_w.z()
    );
    float current_root_yaw_used = current_root_yaw;
    if (no_global_mode_ && !has_initial_yaw_bias_) {
        initial_yaw_bias_ = current_root_yaw;
        has_initial_yaw_bias_ = true;
    }
    const bool has_current_root_xy = (!no_global_mode_) && live_state.has_highstate;
    Eigen::Vector2f current_root_xy = Eigen::Vector2f::Zero();
    if (has_current_root_xy) {
        current_root_xy = live_state.root_pos_w.head<2>();
    }
    Eigen::Quaternionf current_root_quat_used = live_state.root_quat_w;
    const Eigen::Quaternionf current_root_quat_unbiased_used = current_root_quat_used;
    const bool was_startup_alignment_pending = startup_alignment_pending_;
    const float track_time_s = startup_alignment_pending_ ? 0.0f : tracking_playback_time_;
    reference_->update(track_time_s,
                       no_global_mode_,
                       has_current_root_xy,
                       current_root_xy,
                       current_root_yaw_used,
                       current_root_quat_used,
                       current_root_quat_unbiased_used,
                       env->robot->data.joint_pos,
                       use_motion_root_command_,
                       use_motion_velocity_command_,
                       !one_shot_mode_);
    env->episode_length += 1;

    // Override joint positions with reference motion for specified joints.
    // Read override_joint_ids from deploy.yaml via the observation params is not possible here,
    // so we use a fixed set or read from deploy.yaml in constructor.
    // For now, use the override_joint_ids from deploy.yaml (stored during init).
    if (!override_joint_ids_.empty()) {
        const auto& ref_joint_pos = reference_->command_joint_pos();
        auto& data = env->robot->data;
        if (data.override_joint_pos.size() != data.joint_pos.size()) {
            data.override_joint_pos = Eigen::VectorXf::Zero(data.joint_pos.size());
        }
        for (int idx : override_joint_ids_) {
            if (idx >= 0 && idx < static_cast<int>(data.joint_pos.size())) {
                data.override_joint_pos[idx] = ref_joint_pos[idx];
                data.joint_pos[idx] = ref_joint_pos[idx];  // directly override
            }
        }
    }

    env->robot->update();
    const auto obs = env->observation_manager->compute();
    const auto action = env->alg->act(obs);
    env->action_manager->process_action(action);
    auto target_q = env->action_manager->processed_actions();
    if (startup_alignment_pending_) {
        apply_startup_upper_body_interpolation(target_q);
    }
    if (env->episode_length <= 5) {
        const auto& joint_pos = env->robot->data.joint_pos;
        float max_abs_target_delta = 0.0f;
        int max_delta_policy_joint = -1;
        int max_delta_sdk_slot = -1;
        const size_t joint_count = std::min(
            static_cast<size_t>(joint_pos.size()),
            target_q.size()
        );
        for (size_t i = 0; i < joint_count; ++i) {
            const float delta = std::abs(target_q[i] - joint_pos[i]);
            if (delta > max_abs_target_delta) {
                max_abs_target_delta = delta;
                max_delta_policy_joint = static_cast<int>(i);
                max_delta_sdk_slot = env->robot->data.policy_joint_to_sdk_slot(static_cast<int>(i));
            }
        }
        spdlog::info("Track: frame {} max |target_q-current_q| = {:.4f} rad at policy joint {}, sdk slot {}",
                     env->episode_length,
                     max_abs_target_delta,
                     max_delta_policy_joint,
                     max_delta_sdk_slot);
    }
    dump_control_frame(obs, action, target_q);
    dump_qpos_visualizer_frame(target_q);
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);

        auto & motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = target_q[i];
        motor.dq() = 0.0f;
        motor.kp() = policy_kp_[i] * pd_kp_scale_[i];
        motor.kd() = policy_kd_[i] * pd_kd_scale_[i];
        motor.tau() = 0.0f;
    }
    apply_head_hold_command();

    if (debug_dump_first_frame_ && !first_frame_debug_dumped_) {
        dump_first_frame_debug(obs, action, target_q);
        first_frame_debug_dumped_ = true;
    }

    if (was_startup_alignment_pending) {
        tracking_playback_time_ = 0.0f;
        const double now = static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
        if (now - startup_alignment_start_time_s_ >= startup_alignment_duration_s_) {
            env->robot->update();
            const auto& refreshed_live_state = env->robot->data.live_state;
            reference_->calibrate_anchor_frame(
                0.0f,
                env->robot->data.joint_pos,
                refreshed_live_state.root_quat_w,
                true);
            startup_alignment_pending_ = false;
            startup_upper_body_interp_active_ = false;
            spdlog::info("Track: startup playback hold complete; anchor yaw alignment refreshed");
        }
    } else {
        tracking_playback_time_ = reference_
            ? std::min(tracking_playback_time_ + env->step_dt, reference_->duration())
            : tracking_playback_time_ + env->step_dt;
    }

    if (one_shot_mode_ && reference_ && track_time_s >= reference_->duration()) {
        if (active_app_request_) {
            Et1DanceBridge::Instance().PublishStatusWithProgress(
                active_app_command_.request_id,
                active_app_command_.dance_id,
                "play_finished",
                active_motion_duration_ms(),
                active_motion_duration_ms(),
                1.0);
            active_app_request_ = false;
        }
        if (hybrid_locomotion_enabled_) {
            active_tracking_ = false;
            close_observation_dump();
            if (locomotion_env_) {
                locomotion_env_->reset();
                start_locomotion_policy_thread();
            }
            spdlog::info("Track: one-shot playback complete at {:.3f}s, returning to hybrid locomotion", track_time_s);
        } else {
            playback_complete_ = true;
            spdlog::info("Track: one-shot playback complete at {:.3f}s, returning to Velocity", track_time_s);
        }
        close_qpos_visualizer_dump();
    }
}

bool State_Track::poll_motion_request_file()
{
    if (request_file_.empty() || !std::filesystem::exists(request_file_)) {
        return false;
    }

    std::ifstream input(request_file_);
    std::string line;
    std::getline(input, line);
    input.close();

    const auto request = parse_tracker_request_line(line);
    if (request.motion_file.empty()) {
        spdlog::warn("Track: ignored empty request file '{}'", request_file_.string());
        std::error_code ec;
        std::filesystem::remove(request_file_, ec);
        return false;
    }
    if (request.has_profile && request.target_state != getStateString()) {
        return false;
    }

    std::error_code ec;
    std::filesystem::remove(request_file_, ec);

    request_motion_file(request.motion_file);
    return true;
}

bool State_Track::route_profile_request_to(const std::string& target_state)
{
    if (request_file_.empty() || !std::filesystem::exists(request_file_)) {
        return false;
    }

    std::ifstream input(request_file_);
    std::string line;
    std::getline(input, line);
    input.close();

    const auto request = parse_tracker_request_line(line);
    if (!request.has_profile || request.target_state != target_state || request.motion_file.empty()) {
        return false;
    }

    std::error_code ec;
    std::filesystem::remove(request_file_, ec);
    request_motion_file(request.motion_file);
    spdlog::info("Track: routed profile request from {} to {} with motion '{}'",
                 getStateString(),
                 target_state,
                 request.motion_file);
    return true;
}

bool State_Track::start_requested_motion(const std::filesystem::path& requested_motion)
{
    std::filesystem::path motion_file = requested_motion;
    if (!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }
    if (!std::filesystem::exists(motion_file)) {
        spdlog::warn("Track: requested motion '{}' does not exist; staying in locomotion",
                     motion_file.string());
        return false;
    }

    std::error_code motion_ec;
    std::error_code default_ec;
    const auto canonical_motion = std::filesystem::weakly_canonical(motion_file, motion_ec);
    const auto canonical_default = std::filesystem::weakly_canonical(default_motion_file_, default_ec);
    const bool matches_configured_motion = !motion_ec && !default_ec && canonical_motion == canonical_default;
    if (matches_configured_motion) {
        if (require_requested_motion_) {
            spdlog::warn("Track: requested motion '{}' equals configured fallback motion_file; rejecting request",
                         motion_file.string());
            return false;
        }
        spdlog::info("Track: requested motion '{}' equals configured motion_file; running configured motion",
                     motion_file.string());
    }

    spdlog::info("Track: loading requested motion '{}'", motion_file.string());
    reference_ = std::make_shared<ReferenceLoader>(
        motion_file,
        reference_fps_,
        reference_future_horizon_);
    reference = reference_;
    reference_->reset(env->robot->data.default_joint_pos);
    env->reset();
    reset_pd_gain_scales();
    initialize_policy_motors(env.get(), policy_kp_, policy_kd_, true);
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= static_cast<int>(lowcmd->msg_.motor_cmd().size())) {
            continue;
        }
        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.q() = env->robot->data.default_joint_pos[i];
    }
    apply_head_hold_command();
    has_initial_yaw_bias_ = false;
    initial_yaw_bias_ = 0.0f;
    startup_alignment_pending_ = false;
    startup_upper_body_interp_active_ = false;
    tracking_playback_time_ = 0.0f;
    if (no_global_mode_) {
        env->robot->update();
        const auto& live_state = env->robot->data.live_state;
        initial_yaw_bias_ = quat_to_yaw(
            live_state.root_quat_w.w(),
            live_state.root_quat_w.x(),
            live_state.root_quat_w.y(),
            live_state.root_quat_w.z()
        );
        has_initial_yaw_bias_ = true;
        reference_->calibrate_anchor_frame(
            0.0f,
            env->robot->data.joint_pos,
            live_state.root_quat_w,
            true);
        startup_alignment_pending_ = true;
        startup_alignment_start_time_s_ =
            static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
        begin_startup_upper_body_interpolation(sample_reference_joint_pos(0.0f));
        spdlog::info("Track: requested motion anchor yaw alignment reset at robot yaw {:.6f} rad", initial_yaw_bias_);
        spdlog::info("Track: requested motion startup playback hold enabled for {:.3f}s", startup_alignment_duration_s_);
    }
    open_observation_dump();
    open_qpos_visualizer_dump();
    return true;
}

void State_Track::run_locomotion_policy()
{
    if (!locomotion_env_) {
        return;
    }
    write_policy_action(
        locomotion_env_->action_manager->processed_actions(),
        locomotion_policy_kp_,
        locomotion_policy_kd_,
        locomotion_env_.get()
    );
    apply_hybrid_idle_hold();
}

void State_Track::initialize_policy_motors(isaaclab::ManagerBasedRLEnv* policy_env,
                                           const std::vector<float>& kp,
                                           const std::vector<float>& kd,
                                           bool clear_uncontrolled_motors)
{
    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    if (clear_uncontrolled_motors) {
        for (int i = 0; i < motor_cmd_count; ++i) {
            auto& motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = 0.0f;
            motor.kd() = 0.0f;
            motor.dq() = 0.0f;
            motor.tau() = 0.0f;
        }
    }

    if (!policy_env) {
        return;
    }

    const size_t joint_count = std::min({
        policy_env->robot->data.joint_ids_map.size(),
        kp.size(),
        kd.size()
    });

    for (size_t i = 0; i < joint_count; ++i) {
        const int sdk_slot = policy_env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= motor_cmd_count) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.dq() = 0.0f;
        motor.kp() = kp[i];
        motor.kd() = kd[i];
        motor.tau() = 0.0f;
    }
}

void State_Track::apply_hybrid_idle_hold()
{
    if (!hybrid_locomotion_enabled_ || active_tracking_ || !locomotion_env_) {
        return;
    }

    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    std::vector<bool> locomotion_controlled(motor_cmd_count, false);
    for (size_t i = 0; i < locomotion_env_->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = locomotion_env_->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot >= 0 && sdk_slot < motor_cmd_count) {
            locomotion_controlled[sdk_slot] = true;
        }
    }

    const size_t hold_count = std::min({
        static_cast<size_t>(motor_cmd_count),
        hybrid_idle_hold_q_.size(),
        hybrid_idle_hold_kp_.size(),
        hybrid_idle_hold_kd_.size()
    });
    for (size_t sdk_slot = 0; sdk_slot < hold_count; ++sdk_slot) {
        if (locomotion_controlled[sdk_slot]) {
            continue;
        }
        if (hybrid_idle_hold_kp_[sdk_slot] == 0.0f && hybrid_idle_hold_kd_[sdk_slot] == 0.0f) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = hybrid_idle_hold_q_[sdk_slot];
        motor.dq() = 0.0f;
        motor.kp() = hybrid_idle_hold_kp_[sdk_slot];
        motor.kd() = hybrid_idle_hold_kd_[sdk_slot];
        motor.tau() = 0.0f;
    }
}

void State_Track::write_policy_action(const std::vector<float>& action,
                                      const std::vector<float>& kp,
                                      const std::vector<float>& kd,
                                      isaaclab::ManagerBasedRLEnv* policy_env)
{
    if (!policy_env) {
        return;
    }
    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    const size_t joint_count = std::min({
        policy_env->robot->data.joint_ids_map.size(),
        action.size(),
        kp.size(),
        kd.size()
    });

    for (size_t i = 0; i < joint_count; ++i) {
        const int sdk_slot = policy_env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= motor_cmd_count) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = action[i];
        motor.dq() = 0.0f;
        motor.kp() = kp[i];
        motor.kd() = kd[i];
        motor.tau() = 0.0f;
    }
    apply_head_hold_command();
}

void State_Track::start_locomotion_policy_thread()
{
    if (!hybrid_locomotion_enabled_ || !locomotion_env_ || locomotion_policy_thread_running_) {
        return;
    }

    locomotion_policy_thread_running_ = true;
    locomotion_policy_thread_ = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(locomotion_env_->step_dt));

        auto sleep_till = clock::now() + dt;
        while (locomotion_policy_thread_running_) {
            locomotion_env_->step();
            std::this_thread::sleep_until(sleep_till);
            sleep_till += dt;
        }
    });
}

void State_Track::stop_locomotion_policy_thread()
{
    locomotion_policy_thread_running_ = false;
    if (locomotion_policy_thread_.joinable()) {
        locomotion_policy_thread_.join();
    }
}

void State_Track::exit()
{
    spdlog::info("Track: exit");
    stop_locomotion_policy_thread();
    close_observation_dump();
    close_qpos_visualizer_dump();
}

void State_Track::configure_pd_gain_randomization()
{
    const size_t joint_count = policy_kp_.size();
    pd_kp_scale_.assign(joint_count, 1.0f);
    pd_kd_scale_.assign(joint_count, 1.0f);
    pd_gain_mask_.assign(joint_count, 0);
    pd_kp_scale_range_ = {1.0f, 1.0f};
    pd_kd_scale_range_ = {1.0f, 1.0f};
    pd_gain_randomization_enabled_ = false;
    pd_gain_randomize_on_reset_ = false;

    const auto cfg = env->cfg["pd_gain_randomization"];
    if (!cfg || !cfg.IsMap()) {
        return;
    }

    pd_gain_randomization_enabled_ = cfg["enabled"] ? cfg["enabled"].as<bool>() : true;
    if (!pd_gain_randomization_enabled_) {
        return;
    }

    if (cfg["joint_mask"]) {
        pd_gain_mask_ = cfg["joint_mask"].as<std::vector<int>>();
        if (pd_gain_mask_.size() != joint_count) {
            throw std::runtime_error("Track: pd_gain_randomization.joint_mask size mismatch.");
        }
    } else {
        pd_gain_mask_.assign(joint_count, 1);
    }

    if (cfg["kp_scale_range"]) {
        pd_kp_scale_range_ = cfg["kp_scale_range"].as<std::vector<float>>();
        pd_gain_randomize_on_reset_ = true;
    } else if (cfg["kp_scale"]) {
        const float kp_scale = cfg["kp_scale"].as<float>();
        pd_kp_scale_range_ = {kp_scale, kp_scale};
    }

    if (cfg["kd_scale_range"]) {
        pd_kd_scale_range_ = cfg["kd_scale_range"].as<std::vector<float>>();
        pd_gain_randomize_on_reset_ = true;
    } else if (cfg["kd_scale"]) {
        const float kd_scale = cfg["kd_scale"].as<float>();
        pd_kd_scale_range_ = {kd_scale, kd_scale};
    }

    if (pd_kp_scale_range_.size() != 2 || pd_kd_scale_range_.size() != 2) {
        throw std::runtime_error("Track: pd gain scale ranges must have exactly 2 values.");
    }
    if (pd_kp_scale_range_[0] > pd_kp_scale_range_[1]
        || pd_kd_scale_range_[0] > pd_kd_scale_range_[1]) {
        throw std::runtime_error("Track: invalid pd gain scale range.");
    }

    pd_gain_randomize_on_reset_ = cfg["randomize_on_reset"]
        ? cfg["randomize_on_reset"].as<bool>()
        : pd_gain_randomize_on_reset_;
    reset_pd_gain_scales();
}

void State_Track::reset_pd_gain_scales()
{
    pd_kp_scale_.assign(policy_kp_.size(), 1.0f);
    pd_kd_scale_.assign(policy_kd_.size(), 1.0f);
    if (!pd_gain_randomization_enabled_) {
        return;
    }

    std::uniform_real_distribution<float> kp_dist(pd_kp_scale_range_[0], pd_kp_scale_range_[1]);
    std::uniform_real_distribution<float> kd_dist(pd_kd_scale_range_[0], pd_kd_scale_range_[1]);
    int active_joint_count = 0;
    for (size_t i = 0; i < policy_kp_.size(); ++i) {
        if (i < pd_gain_mask_.size() && pd_gain_mask_[i] != 0) {
            pd_kp_scale_[i] = pd_gain_randomize_on_reset_ ? kp_dist(pd_gain_rng_) : pd_kp_scale_range_[0];
            pd_kd_scale_[i] = pd_gain_randomize_on_reset_ ? kd_dist(pd_gain_rng_) : pd_kd_scale_range_[0];
            active_joint_count += 1;
        }
    }
    spdlog::info("Track: PD gain randomization active on {} joints; kp scale range [{:.3f}, {:.3f}], kd scale range [{:.3f}, {:.3f}]",
                 active_joint_count,
                 *std::min_element(pd_kp_scale_.begin(), pd_kp_scale_.end()),
                 *std::max_element(pd_kp_scale_.begin(), pd_kp_scale_.end()),
                 *std::min_element(pd_kd_scale_.begin(), pd_kd_scale_.end()),
                 *std::max_element(pd_kd_scale_.begin(), pd_kd_scale_.end()));
}

void State_Track::apply_head_hold_command()
{
    if (head_hold_sdk_slots_.empty()) {
        return;
    }

    const auto motor_count = lowcmd->msg_.motor_cmd().size();
    for (size_t i = 0; i < head_hold_sdk_slots_.size(); ++i) {
        const int sdk_slot = head_hold_sdk_slots_[i];
        if (sdk_slot < 0 || static_cast<size_t>(sdk_slot) >= motor_count) {
            spdlog::warn("Track: head hold sdk slot {} is out of range, skip", sdk_slot);
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = head_hold_q_[i];
        motor.dq() = 0.0f;
        motor.kp() = head_hold_kp_[i];
        motor.kd() = head_hold_kd_[i];
        motor.tau() = 0.0f;
    }
}

Eigen::VectorXf State_Track::sample_reference_joint_pos(float time_s) const
{
    if (!reference_) {
        return Eigen::VectorXf::Zero(0);
    }
    return reference_->sample_joint_pos(time_s);
}

void State_Track::begin_startup_upper_body_interpolation(const Eigen::VectorXf& target_q)
{
    const int joint_count = static_cast<int>(env->robot->data.joint_ids_map.size());
    const int upper_body_joint_count = std::clamp(
        joint_count - kUpperBodyStartIndex,
        0,
        kMaxUpperBodyJointCount);
    if (upper_body_joint_count <= 0 || target_q.size() <= kUpperBodyStartIndex) {
        startup_upper_body_interp_active_ = false;
        return;
    }

    startup_upper_body_interp_active_ = true;
    startup_upper_body_interp_t0_ = static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
    startup_upper_body_interp_ts_ = {0.0f, startup_alignment_duration_s_};
    startup_upper_body_interp_qs_.assign(2, std::vector<float>(upper_body_joint_count, 0.0f));

    const int motor_state_count = static_cast<int>(lowstate->msg_.motor_state().size());
    for (int i = 0; i < upper_body_joint_count; ++i) {
        const int policy_index = kUpperBodyStartIndex + i;
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(policy_index);
        if (sdk_slot < 0 || sdk_slot >= motor_state_count || policy_index >= target_q.size()) {
            continue;
        }

        startup_upper_body_interp_qs_[0][i] = lowstate->msg_.motor_state()[sdk_slot].q();
        startup_upper_body_interp_qs_[1][i] = target_q[policy_index];
    }
}

void State_Track::apply_startup_upper_body_interpolation(std::vector<float>& target_q)
{
    if (!startup_upper_body_interp_active_ || startup_upper_body_interp_qs_.empty()) {
        return;
    }

    const float t = static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3
        - startup_upper_body_interp_t0_;
    const auto upper_body_q = linear_interpolate(t, startup_upper_body_interp_ts_, startup_upper_body_interp_qs_);
    for (size_t i = 0; i < upper_body_q.size(); ++i) {
        const size_t policy_index = static_cast<size_t>(kUpperBodyStartIndex) + i;
        if (policy_index < target_q.size()) {
            target_q[policy_index] = upper_body_q[i];
        }
    }

    if (!startup_upper_body_interp_ts_.empty() && t >= startup_upper_body_interp_ts_.back()) {
        startup_upper_body_interp_active_ = false;
    }
}

void State_Track::open_observation_dump()
{
    observation_dump_frame_ = 0;
    if (!observation_dump_enabled_) {
        return;
    }

    try {
        observation_dump_file_ = timestamped_dump_path(observation_dump_base_file_);
        std::filesystem::create_directories(observation_dump_file_.parent_path());
        observation_dump_stream_.open(observation_dump_file_, std::ios::out | std::ios::trunc);
        if (!observation_dump_stream_) {
            throw std::runtime_error("Failed to open observation dump file: " + observation_dump_file_.string());
        }
        observation_dump_stream_ << std::setprecision(9);
        observation_dump_stream_ << "# ET1 Track observation dump\n";
        observation_dump_stream_ << "# state " << getStateString() << "\n";
        observation_dump_stream_ << "# dt " << env->step_dt << "\n";
        observation_dump_stream_ << "# format: frame <dump_index> episode_length <count> time_s <seconds> reference_frame <index> reference_time_s <seconds>\n";
        observation_dump_stream_ << "# lines per control frame: live_state, ref_command, obs groups, action raw_policy, action target_q\n";
        spdlog::info("Track: writing per-frame control dump to '{}'", observation_dump_file_.string());
    } catch (const std::exception& e) {
        observation_dump_enabled_ = false;
        spdlog::error("Track: disabling observation dump: {}", e.what());
    }
}

void State_Track::dump_control_frame(const std::unordered_map<std::string, std::vector<float>>& obs,
                                     const std::vector<float>& action,
                                     const std::vector<float>& target_q)
{
    if (!observation_dump_enabled_ || !observation_dump_stream_) {
        return;
    }

    std::vector<std::string> names;
    names.reserve(obs.size());
    for (const auto& item : obs) {
        names.push_back(item.first);
    }
    std::sort(names.begin(), names.end());

    const auto& live_state = env->robot->data.live_state;

    observation_dump_stream_ << "frame " << observation_dump_frame_
                             << " episode_length " << env->episode_length
                             << " time_s " << (env->episode_length * env->step_dt)
                             << " reference_frame " << reference_->current_frame_index()
                             << " reference_time_s " << reference_->current_time_s()
                             << "\n";

    observation_dump_stream_ << "live_state root_quat_w"
                             << " " << live_state.root_quat_w.w()
                             << " " << live_state.root_quat_w.x()
                             << " " << live_state.root_quat_w.y()
                             << " " << live_state.root_quat_w.z()
                             << " root_ang_vel_b"
                             << " " << live_state.root_gyro_b.x()
                             << " " << live_state.root_gyro_b.y()
                             << " " << live_state.root_gyro_b.z()
                             << " has_highstate " << (live_state.has_highstate ? 1 : 0)
                             << " root_pos_w"
                             << " " << live_state.root_pos_w.x()
                             << " " << live_state.root_pos_w.y()
                             << " " << live_state.root_pos_w.z()
                             << " root_lin_vel_w"
                             << " " << live_state.root_lin_vel_w.x()
                             << " " << live_state.root_lin_vel_w.y()
                             << " " << live_state.root_lin_vel_w.z()
                             << "\n";

    const auto& ref_root_ori_b = reference_->command_root_ori_b();
    observation_dump_stream_ << "ref_command root_ori_b size " << ref_root_ori_b.size() << " values";
    for (int i = 0; i < ref_root_ori_b.size(); ++i) {
        observation_dump_stream_ << " " << ref_root_ori_b[i];
    }
    observation_dump_stream_ << "\n";

    const auto& ref_xy_yaw_vel = reference_->command_xy_yaw_vel();
    observation_dump_stream_ << "ref_command xy_yaw_vel size " << ref_xy_yaw_vel.size() << " values";
    for (int i = 0; i < ref_xy_yaw_vel.size(); ++i) {
        observation_dump_stream_ << " " << ref_xy_yaw_vel[i];
    }
    observation_dump_stream_ << "\n";

    const auto& ref_joint_pos = reference_->command_joint_pos();
    observation_dump_stream_ << "ref_command joint_pos size " << ref_joint_pos.size() << " values";
    for (int i = 0; i < ref_joint_pos.size(); ++i) {
        observation_dump_stream_ << " " << ref_joint_pos[i];
    }
    observation_dump_stream_ << "\n";

    const auto& ref_joint_vel = reference_->command_joint_vel();
    observation_dump_stream_ << "ref_command joint_vel size " << ref_joint_vel.size() << " values";
    for (int i = 0; i < ref_joint_vel.size(); ++i) {
        observation_dump_stream_ << " " << ref_joint_vel[i];
    }
    observation_dump_stream_ << "\n";

    const auto& ref_foot_support = reference_->command_foot_support_state();
    observation_dump_stream_ << "ref_command foot_support_state size " << ref_foot_support.size() << " values";
    for (int i = 0; i < ref_foot_support.size(); ++i) {
        observation_dump_stream_ << " " << ref_foot_support[i];
    }
    observation_dump_stream_ << "\n";

    for (const auto& name : names) {
        const auto& values = obs.at(name);
        observation_dump_stream_ << "obs " << name << " size " << values.size() << " values";
        for (float value : values) {
            observation_dump_stream_ << " " << value;
        }
        observation_dump_stream_ << "\n";
    }

    observation_dump_stream_ << "action raw_policy size " << action.size() << " values";
    for (float value : action) {
        observation_dump_stream_ << " " << value;
    }
    observation_dump_stream_ << "\n";

    observation_dump_stream_ << "action target_q size " << target_q.size() << " values";
    for (float value : target_q) {
        observation_dump_stream_ << " " << value;
    }
    observation_dump_stream_ << "\n";

    observation_dump_stream_ << "\n";
    observation_dump_stream_.flush();
    ++observation_dump_frame_;
}

void State_Track::close_observation_dump()
{
    if (observation_dump_stream_) {
        spdlog::info("Track: closed observation dump '{}' after {} frames",
                     observation_dump_file_.string(),
                     observation_dump_frame_);
        observation_dump_stream_.close();
    }
}

void State_Track::open_qpos_visualizer_dump()
{
    qpos_visualizer_dump_frame_ = 0;
    if (!qpos_visualizer_dump_enabled_) {
        return;
    }

    try {
        qpos_visualizer_dump_file_ = timestamped_dump_path(qpos_visualizer_dump_base_file_);
        std::filesystem::create_directories(qpos_visualizer_dump_file_.parent_path());
        qpos_visualizer_dump_stream_.open(qpos_visualizer_dump_file_, std::ios::out | std::ios::trunc);
        if (!qpos_visualizer_dump_stream_) {
            throw std::runtime_error("Failed to open qpos visualizer CSV: " + qpos_visualizer_dump_file_.string());
        }

        qpos_visualizer_dump_stream_ << std::setprecision(9);
        qpos_visualizer_dump_stream_ << "time";
        for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
            qpos_visualizer_dump_stream_ << ",target_" << i;
        }
        for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
            qpos_visualizer_dump_stream_ << ",measured_" << i;
        }
        for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
            qpos_visualizer_dump_stream_ << ",motion_joint_pos_" << i;
        }
        qpos_visualizer_dump_stream_ << "\n";

        spdlog::info("Track: writing qpos visualizer CSV to '{}'", qpos_visualizer_dump_file_.string());
    } catch (const std::exception& e) {
        qpos_visualizer_dump_enabled_ = false;
        spdlog::error("Track: disabling qpos visualizer dump: {}", e.what());
    }
}

void State_Track::dump_qpos_visualizer_frame(const std::vector<float>& target_q)
{
    if (!qpos_visualizer_dump_enabled_ || !qpos_visualizer_dump_stream_) {
        return;
    }

    const float time_s = reference_ ? reference_->current_time_s() : static_cast<float>(qpos_visualizer_dump_frame_) * env->step_dt;
    qpos_visualizer_dump_stream_ << time_s;

    for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
        const float value = i < static_cast<int>(target_q.size())
            ? target_q[static_cast<size_t>(i)]
            : std::numeric_limits<float>::quiet_NaN();
        qpos_visualizer_dump_stream_ << "," << value;
    }

    const auto& measured_q = env->robot->data.joint_pos;
    for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
        const float value = i < measured_q.size()
            ? measured_q[i]
            : std::numeric_limits<float>::quiet_NaN();
        qpos_visualizer_dump_stream_ << "," << value;
    }

    if (reference_) {
        const auto& motion_q = reference_->command_joint_pos();
        for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
            const float value = i < motion_q.size()
                ? motion_q[i]
                : std::numeric_limits<float>::quiet_NaN();
            qpos_visualizer_dump_stream_ << "," << value;
        }
    } else {
        for (int i = 0; i < ReferenceLoader::kJointDim; ++i) {
            qpos_visualizer_dump_stream_ << "," << std::numeric_limits<float>::quiet_NaN();
        }
    }

    qpos_visualizer_dump_stream_ << "\n";
    ++qpos_visualizer_dump_frame_;
}

void State_Track::close_qpos_visualizer_dump()
{
    if (qpos_visualizer_dump_stream_) {
        spdlog::info("Track: closed qpos visualizer CSV '{}' after {} frames",
                     qpos_visualizer_dump_file_.string(),
                     qpos_visualizer_dump_frame_);
        qpos_visualizer_dump_stream_.close();
    }
}

void State_Track::dump_first_frame_debug(const std::unordered_map<std::string, std::vector<float>>& obs,
                                         const std::vector<float>& action,
                                         const std::vector<float>& target_q)
{
    try {
        std::filesystem::create_directories(debug_dump_dir_);

        auto write_obs = [&](const std::string& name, const std::vector<size_t>& preferred_shape) {
            auto it = obs.find(name);
            if (it == obs.end()) {
                spdlog::warn("Track debug: observation '{}' is absent, skip dump", name);
                return;
            }
            write_npy_float(debug_dump_dir_ / (name + ".npy"), it->second, preferred_shape);
        };

        write_obs("obs_current", {1, obs.count("obs_current") ? obs.at("obs_current").size() : 0});
        if (obs.count("obs_history") && obs.at("obs_history").size() % 25 == 0) {
            write_obs("obs_history", {1, 25, obs.at("obs_history").size() / 25});
        } else {
            write_obs("obs_history", {1, obs.count("obs_history") ? obs.at("obs_history").size() : 0});
        }

        write_npy_float(debug_dump_dir_ / "action.npy", action, {1, action.size()});
        write_npy_float(debug_dump_dir_ / "target_joint_pos.npy", target_q, {1, target_q.size()});

        std::vector<float> current_joint_pos(env->robot->data.joint_pos.data(),
                                             env->robot->data.joint_pos.data() + env->robot->data.joint_pos.size());
        std::vector<float> current_joint_vel(env->robot->data.joint_vel.data(),
                                             env->robot->data.joint_vel.data() + env->robot->data.joint_vel.size());
        write_npy_float(debug_dump_dir_ / "deploy_joint_pos.npy", current_joint_pos, {1, current_joint_pos.size()});
        write_npy_float(debug_dump_dir_ / "deploy_joint_vel.npy", current_joint_vel, {1, current_joint_vel.size()});

        const auto& live_state = env->robot->data.live_state;
        const std::vector<float> root_quat = {
            live_state.root_quat_w.w(),
            live_state.root_quat_w.x(),
            live_state.root_quat_w.y(),
            live_state.root_quat_w.z(),
        };
        write_npy_float(debug_dump_dir_ / "deploy_root_quat_w.npy", root_quat, {1, 4});

        const size_t motor_count = lowcmd->msg_.motor_cmd().size();
        std::vector<float> lowcmd_q(motor_count, 0.0f);
        std::vector<float> lowcmd_dq(motor_count, 0.0f);
        std::vector<float> lowcmd_kp(motor_count, 0.0f);
        std::vector<float> lowcmd_kd(motor_count, 0.0f);
        std::vector<float> lowcmd_tau(motor_count, 0.0f);
        std::vector<int64_t> lowcmd_mode(motor_count, 0);
        for (size_t i = 0; i < motor_count; ++i) {
            const auto& motor = lowcmd->msg_.motor_cmd()[i];
            lowcmd_q[i] = motor.q();
            lowcmd_dq[i] = motor.dq();
            lowcmd_kp[i] = motor.kp();
            lowcmd_kd[i] = motor.kd();
            lowcmd_tau[i] = motor.tau();
            lowcmd_mode[i] = motor.mode();
        }
        write_npy_float(debug_dump_dir_ / "lowcmd_q_idl.npy", lowcmd_q, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_dq_idl.npy", lowcmd_dq, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_kp_idl.npy", lowcmd_kp, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_kd_idl.npy", lowcmd_kd, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_tau_idl.npy", lowcmd_tau, {1, motor_count});
        write_npy_int64(debug_dump_dir_ / "lowcmd_mode_idl.npy", lowcmd_mode, {1, motor_count});

        write_npy_int64(debug_dump_dir_ / "motion_time_steps.npy", {env->episode_length}, {1});
        write_npy_int64(debug_dump_dir_ / "motion_clip_id.npy", {0}, {1});

        spdlog::info("Track debug: dumped first frame to '{}'", debug_dump_dir_.string());
    } catch (const std::exception& e) {
        spdlog::error("Track debug: failed to dump first frame: {}", e.what());
    }
}

void State_Track::write_npy_float(const std::filesystem::path& path,
                                  const std::vector<float>& data,
                                  const std::vector<size_t>& shape) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to open npy file for write: " + path.string());
    }
    write_npy_header(out, "<f4", shape);
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(float)));
}

void State_Track::write_npy_int64(const std::filesystem::path& path,
                                  const std::vector<int64_t>& data,
                                  const std::vector<size_t>& shape) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to open npy file for write: " + path.string());
    }
    write_npy_header(out, "<i8", shape);
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(int64_t)));
}

void State_Track::write_npy_header(std::ofstream& out,
                                   const std::string& descr,
                                   const std::vector<size_t>& shape) const
{
    std::ostringstream shape_ss;
    shape_ss << "(";
    for (size_t i = 0; i < shape.size(); ++i) {
        if (i > 0) {
            shape_ss << ", ";
        }
        shape_ss << shape[i];
    }
    if (shape.size() == 1) {
        shape_ss << ",";
    }
    shape_ss << ")";

    std::string header = "{'descr': '" + descr + "', 'fortran_order': False, 'shape': "
        + shape_ss.str() + ", }";
    const size_t preamble_size = 10;
    const size_t padding = 16 - ((preamble_size + header.size() + 1) % 16);
    header.append(padding, ' ');
    header.push_back('\n');

    if (header.size() > std::numeric_limits<uint16_t>::max()) {
        throw std::runtime_error("NPY header is too large.");
    }

    const char magic[] = "\x93NUMPY";
    out.write(magic, 6);
    const char version[2] = {1, 0};
    out.write(version, 2);
    const uint16_t header_len = static_cast<uint16_t>(header.size());
    out.write(reinterpret_cast<const char*>(&header_len), sizeof(header_len));
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
}
