#include "State_Velocity.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <zmq.hpp>

#include "State_Track.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "unitree_articulation.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    static const std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}},
    };

    const auto it = key_commands.find(FSMState::keyboard->key());
    return it == key_commands.end() ? std::vector<float>{0.0f, 0.0f, 0.0f} : it->second;
}

}
}

namespace
{
constexpr char kLiveMagic[8] = {'E', 'T', '1', 'L', 'I', 'V', 'E', '1'};
constexpr uint32_t kLiveVersion = 1;
constexpr uint32_t kLiveFlagReset = 1u << 0;

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

struct TrackerPolicyOption
{
    std::string key;
    std::string label;
    std::string policy_file;
    std::string deploy_file;
};

struct TrackerMotionOption
{
    std::string key;
    std::string label;
    std::filesystem::path motion_file;
};
}

bool State_Velocity::prepare_general_tracker_request()
{
    if (pending_tracker_target_state_) {
        return true;
    }
    if (State_Track::has_pending_motion_request()) {
        pending_tracker_target_state_ = "GeneralTrackerCLN";
        return true;
    }
    if (general_tracker_request_file_.empty()
        || !std::filesystem::exists(general_tracker_request_file_)) {
        return false;
    }

    std::ifstream input(general_tracker_request_file_);
    std::string line;
    std::getline(input, line);
    input.close();

    std::error_code ec;
    std::filesystem::remove(general_tracker_request_file_, ec);

    line = trim_copy(line);
    if (line.empty()) {
        spdlog::warn("Velocity: ignored empty tracker request file '{}'",
                     general_tracker_request_file_.string());
        return false;
    }

    std::istringstream ss(line);
    std::string first_token;
    ss >> first_token;
    std::string target_state = tracker_target_from_token(first_token);
    std::string motion_file;
    if (target_state.empty()) {
        target_state = "GeneralTrackerCLN";
        motion_file = line;
    } else {
        std::getline(ss, motion_file);
        motion_file = trim_copy(motion_file);
    }

    if (motion_file.empty()) {
        spdlog::warn("Velocity: ignored tracker request without motion path: '{}'", line);
        return false;
    }
    if (!FSMStringMap.right.count(target_state)) {
        spdlog::warn("Velocity: ignored tracker request for unavailable target '{}'", target_state);
        return false;
    }

    State_Track::request_motion_file(motion_file);
    pending_tracker_target_state_ = target_state;
    spdlog::info("Velocity: routed tracker request to {} with motion '{}'",
                 target_state,
                 motion_file);
    return true;
}

State_Velocity::State_Velocity(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    const std::string policy_file = cfg["policy_file"] ? cfg["policy_file"].as<std::string>() : "policy.onnx";
    const std::string deploy_file = cfg["deploy_file"] ? cfg["deploy_file"].as<std::string>() : "deploy.yaml";

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / deploy_file),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr, HighState_t::SharedPtr>>(
            FSMState::lowstate, FSMState::highstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / policy_file);
    policy_kp_ = env->cfg["policy_kp"] ? env->cfg["policy_kp"].as<std::vector<float>>() : env->robot->data.joint_stiffness;
    policy_kd_ = env->cfg["policy_kd"] ? env->cfg["policy_kd"].as<std::vector<float>>() : env->robot->data.joint_damping;

    if (FSMStringMap.right.count("GeneralTrackerCJM")) {
        const int cjm_state = FSMStringMap.right.at("GeneralTrackerCJM");
        registered_checks.erase(
            std::remove_if(
                registered_checks.begin(),
                registered_checks.end(),
                [cjm_state](const TransitionCheck& check) {
                    return check.target_state == cjm_state && check.reason == "keyboard 3";
                }),
            registered_checks.end());
        registered_checks.push_back({
            [this]() -> bool {
                if (general_tracker_cjm_prompt_ready_.exchange(false)) {
                    return true;
                }
                if (keyboard && keyboard->on_pressed && keyboard->key() == "3") {
                    start_general_tracker_cjm_prompt();
                }
                return false;
            },
            cjm_state,
            "interactive GeneralTrackerCJM policy/motion selection"
        });
    }

    if (cfg["live_stream_trigger"] && cfg["live_stream_trigger"]["enabled"].as<bool>(false)) {
        const auto trigger_cfg = cfg["live_stream_trigger"];
        live_stream_trigger_enabled_ = true;
        live_stream_trigger_endpoint_ = trigger_cfg["endpoint"]
            ? trigger_cfg["endpoint"].as<std::string>()
            : live_stream_trigger_endpoint_;
        live_stream_trigger_topic_ = trigger_cfg["topic"]
            ? trigger_cfg["topic"].as<std::string>()
            : live_stream_trigger_topic_;
        live_stream_trigger_target_state_ = trigger_cfg["target_state"]
            ? trigger_cfg["target_state"].as<std::string>()
            : live_stream_trigger_target_state_;
        live_stream_trigger_receive_timeout_ms_ = trigger_cfg["receive_timeout_ms"]
            ? trigger_cfg["receive_timeout_ms"].as<int>()
            : live_stream_trigger_receive_timeout_ms_;
        live_stream_trigger_high_water_mark_ = trigger_cfg["high_water_mark"]
            ? trigger_cfg["high_water_mark"].as<int>()
            : live_stream_trigger_high_water_mark_;
        if (!FSMStringMap.right.count(live_stream_trigger_target_state_)) {
            throw std::runtime_error(
                "Velocity: live_stream_trigger target state is not registered: "
                + live_stream_trigger_target_state_);
        }
        registered_checks.push_back({
            [this]() -> bool {
                return live_stream_trigger_pending_.exchange(false);
            },
            FSMStringMap.right.at(live_stream_trigger_target_state_),
            "ZMQ live stream reset frame"
        });
        spdlog::info(
            "Velocity: live stream trigger enabled endpoint='{}' topic='{}' target='{}'",
            live_stream_trigger_endpoint_,
            live_stream_trigger_topic_,
            live_stream_trigger_target_state_);
    }

    if (FSMStringMap.right.count("GeneralTrackerCJM") || FSMStringMap.right.count("GeneralTrackerCLN")) {
        auto tracker_cfg = param::config["FSM"]["GeneralTrackerCJM"];
        const std::string request_file = tracker_cfg["request_file"]
            ? tracker_cfg["request_file"].as<std::string>()
            : "debug/general_tracker_request.txt";
        general_tracker_request_file_ = request_file;
        if (!general_tracker_request_file_.is_absolute()) {
            general_tracker_request_file_ = param::proj_dir / general_tracker_request_file_;
        }

        const std::vector<std::string> tracker_targets = {
            "GeneralTrackerCJM",
            "GeneralTrackerCLN",
        };
        for (const auto& target_state : tracker_targets) {
            if (!FSMStringMap.right.count(target_state)) {
                continue;
            }
            registered_checks.push_back({
                [this, target_state]() -> bool {
                    if (!prepare_general_tracker_request()) {
                        return false;
                    }
                    if (pending_tracker_target_state_ != target_state) {
                        return false;
                    }
                    pending_tracker_target_state_.reset();
                    return true;
                },
                FSMStringMap.right.at(target_state),
                "external " + target_state + " motion request"
            });
        }
    }

    registered_checks.push_back({
        [&]() -> bool { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
        FSMStringMap.right.at("Passive"),
        "bad_orientation"
    });
}

void State_Velocity::start_general_tracker_cjm_prompt()
{
    if (general_tracker_cjm_prompt_running_.exchange(true)) {
        return;
    }

    std::thread([this] {
        auto finish = [this]() {
            general_tracker_cjm_prompt_running_ = false;
        };

        try {
            const auto tracker_cfg = param::config["FSM"]["GeneralTrackerCJM"];
            std::vector<TrackerPolicyOption> policy_options;
            std::vector<TrackerMotionOption> motion_options;

            if (tracker_cfg["policy_options"] && tracker_cfg["policy_options"].IsMap()) {
                for (const auto& item : tracker_cfg["policy_options"]) {
                    TrackerPolicyOption option;
                    option.key = item.first.as<std::string>();
                    option.label = item.second["label"]
                        ? item.second["label"].as<std::string>()
                        : option.key;
                    option.policy_file = item.second["policy_file"].as<std::string>();
                    option.deploy_file = item.second["deploy_file"].as<std::string>();
                    policy_options.push_back(option);
                }
            }
            if (tracker_cfg["motion_options"] && tracker_cfg["motion_options"].IsMap()) {
                for (const auto& item : tracker_cfg["motion_options"]) {
                    TrackerMotionOption option;
                    option.key = item.first.as<std::string>();
                    option.label = item.second["label"]
                        ? item.second["label"].as<std::string>()
                        : option.key;
                    option.motion_file = item.second["motion_file"].as<std::string>();
                    if (!option.motion_file.is_absolute()) {
                        option.motion_file = param::proj_dir / option.motion_file;
                    }
                    motion_options.push_back(option);
                }
            }

            if (policy_options.empty() || motion_options.empty()) {
                spdlog::warn("Velocity: GeneralTrackerCJM policy_options or motion_options are empty");
                finish();
                return;
            }

            std::cout << "\nGeneralTrackerCJM policy options:\n";
            for (const auto& option : policy_options) {
                std::cout << "  [" << option.key << "] " << option.label
                          << " (" << option.policy_file << " + " << option.deploy_file << ")\n";
            }
            const std::string policy_key = keyboard
                ? trim_copy(keyboard->getString("Select policy: "))
                : "";
            const auto policy_it = std::find_if(
                policy_options.begin(),
                policy_options.end(),
                [&policy_key](const TrackerPolicyOption& option) { return option.key == policy_key; });
            if (policy_it == policy_options.end()) {
                spdlog::warn("Velocity: unknown GeneralTrackerCJM policy selection '{}'", policy_key);
                finish();
                return;
            }

            std::cout << "\nGeneralTrackerCJM motion options:\n";
            for (const auto& option : motion_options) {
                std::cout << "  [" << option.key << "] " << option.label
                          << " (" << option.motion_file.string() << ")\n";
            }
            const std::string motion_key = keyboard
                ? trim_copy(keyboard->getString("Select motion: "))
                : "";
            const auto motion_it = std::find_if(
                motion_options.begin(),
                motion_options.end(),
                [&motion_key](const TrackerMotionOption& option) { return option.key == motion_key; });
            if (motion_it == motion_options.end()) {
                spdlog::warn("Velocity: unknown GeneralTrackerCJM motion selection '{}'", motion_key);
                finish();
                return;
            }

            State_Track::request_policy_motion(
                policy_it->policy_file,
                policy_it->deploy_file,
                motion_it->motion_file);
            general_tracker_cjm_prompt_ready_ = true;
            spdlog::info("Velocity: GeneralTrackerCJM selection ready policy '{}' motion '{}'",
                         policy_it->label,
                         motion_it->motion_file.string());
        } catch (const std::exception& e) {
            spdlog::error("Velocity: GeneralTrackerCJM selection prompt failed: {}", e.what());
        }
        finish();
    }).detach();
}

void State_Velocity::start_live_stream_trigger()
{
    if (!live_stream_trigger_enabled_ || live_stream_trigger_running_.exchange(true)) {
        return;
    }
    live_stream_trigger_pending_ = false;
    live_stream_trigger_thread_ = std::thread([this] {
        live_stream_trigger_loop();
    });
}

void State_Velocity::stop_live_stream_trigger()
{
    live_stream_trigger_running_ = false;
    if (live_stream_trigger_thread_.joinable()) {
        live_stream_trigger_thread_.join();
    }
    live_stream_trigger_pending_ = false;
}

void State_Velocity::live_stream_trigger_loop()
{
    try {
        zmq::context_t context(1);
        zmq::socket_t socket(context, zmq::socket_type::sub);
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::rcvtimeo, live_stream_trigger_receive_timeout_ms_);
        socket.set(zmq::sockopt::rcvhwm, live_stream_trigger_high_water_mark_);
        socket.set(zmq::sockopt::subscribe, live_stream_trigger_topic_);
        socket.connect(live_stream_trigger_endpoint_);
        spdlog::info(
            "Velocity: live stream trigger connected to '{}' topic '{}'",
            live_stream_trigger_endpoint_,
            live_stream_trigger_topic_);

        while (live_stream_trigger_running_.load()) {
            zmq::message_t first;
            if (!socket.recv(first, zmq::recv_flags::none)) {
                continue;
            }

            zmq::message_t payload;
            const bool multipart = socket.get(zmq::sockopt::rcvmore);
            const zmq::message_t* data_msg = &first;
            if (multipart) {
                if (!socket.recv(payload, zmq::recv_flags::none)) {
                    continue;
                }
                data_msg = &payload;
                while (socket.get(zmq::sockopt::rcvmore)) {
                    zmq::message_t ignored;
                    if (!socket.recv(ignored, zmq::recv_flags::none)) {
                        break;
                    }
                }
            }

            if (data_msg->size() < sizeof(LiveWireHeader)) {
                continue;
            }
            LiveWireHeader header{};
            std::memcpy(&header, data_msg->data(), sizeof(header));
            if (std::memcmp(header.magic, kLiveMagic, sizeof(kLiveMagic)) != 0
                || header.version != kLiveVersion
                || header.float_count < 62
                || (header.flags & kLiveFlagReset) == 0) {
                continue;
            }

            live_stream_trigger_pending_ = true;
            spdlog::info(
                "Velocity: detected ZMQ live stream reset frame sequence={}, requesting {}",
                header.sequence,
                live_stream_trigger_target_state_);
        }
    } catch (const std::exception& e) {
        spdlog::error("Velocity: live stream trigger stopped after exception: {}", e.what());
    }
}

void State_Velocity::enter()
{
    start_live_stream_trigger();
    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    const size_t joint_count = std::min({
        env->robot->data.joint_ids_map.size(),
        policy_kp_.size(),
        policy_kd_.size()
    });

    for (size_t i = 0; i < joint_count; ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= motor_cmd_count) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.dq() = 0.0f;
        motor.tau() = 0.0f;
    }

    env->robot->update();
    policy_thread_running_ = true;
    policy_thread_ = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(env->step_dt));

        auto sleep_till = clock::now() + dt;
        env->reset();

        while (policy_thread_running_) {
            env->step();
            std::this_thread::sleep_until(sleep_till);
            sleep_till += dt;
        }
    });
}

void State_Velocity::run()
{
    auto action = env->action_manager->processed_actions();
    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    const size_t joint_count = std::min({
        env->robot->data.joint_ids_map.size(),
        action.size(),
        policy_kp_.size(),
        policy_kd_.size()
    });

    for (size_t i = 0; i < joint_count; ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= motor_cmd_count) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = action[i];
        motor.dq() = 0.0f;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.tau() = 0.0f;
    }
}

void State_Velocity::exit()
{
    stop_live_stream_trigger();
    policy_thread_running_ = false;
    if (policy_thread_.joinable()) {
        policy_thread_.join();
    }
}
