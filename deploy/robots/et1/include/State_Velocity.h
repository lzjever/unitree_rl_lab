#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"

class State_Velocity : public FSMState
{
public:
    State_Velocity(int state_mode, std::string state_string = "Velocity");

    void enter();
    void run();
    void exit();

private:
    bool prepare_general_tracker_request();
    void start_general_tracker_cjm_prompt();
    void start_live_stream_trigger();
    void stop_live_stream_trigger();
    void live_stream_trigger_loop();
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::vector<float> policy_kp_;
    std::vector<float> policy_kd_;
    std::filesystem::path general_tracker_request_file_;
    std::optional<std::string> pending_tracker_target_state_;
    std::atomic_bool general_tracker_cjm_prompt_running_{false};
    std::atomic_bool general_tracker_cjm_prompt_ready_{false};
    bool live_stream_trigger_enabled_ = false;
    std::string live_stream_trigger_endpoint_ = "tcp://127.0.0.1:5557";
    std::string live_stream_trigger_topic_ = "et1_track";
    std::string live_stream_trigger_target_state_ = "GeneralTrackerCJM";
    int live_stream_trigger_receive_timeout_ms_ = 100;
    int live_stream_trigger_high_water_mark_ = 1000;
    std::atomic_bool live_stream_trigger_running_{false};
    std::atomic_bool live_stream_trigger_pending_{false};
    std::thread live_stream_trigger_thread_;

    std::thread policy_thread_;
    bool policy_thread_running_ = false;
};

REGISTER_FSM(State_Velocity)
