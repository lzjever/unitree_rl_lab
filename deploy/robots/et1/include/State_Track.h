#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "FSM/State_RLBase.h"

class State_Track : public FSMState
{
public:
    class ReferenceLoader
    {
    public:
        static constexpr int kJointDim = 26;
        static constexpr int kBodyCount = 27;
        static constexpr int kDefaultFutureHorizon = 25;
        static constexpr int kFutureCommandDim = 6 + 3 + kJointDim;
        static constexpr uint32_t kCacheVersion = 1;

        // A compact runtime cache generated from the original NPZ track.
        struct Header
        {
            char magic[8];
            uint32_t version;
            uint32_t array_count;
        };

        struct LiveStreamConfig
        {
            std::string endpoint = "tcp://127.0.0.1:5557";
            std::string topic = "et1_track";
            size_t max_queue_frames = 200;
            size_t initial_buffer_frames = 2;
            int receive_timeout_ms = 100;
            int high_water_mark = 1000;
        };

        ReferenceLoader(const std::filesystem::path& motion_file, float fps, size_t future_horizon = kDefaultFutureHorizon);
        ReferenceLoader(const LiveStreamConfig& live_config, float fps, size_t future_horizon);
        ~ReferenceLoader();

        void reset(const Eigen::VectorXf& default_joint_pos);
        void update(float time_s,
                    bool no_global_mode,
                    bool has_current_root_xy,
                    const Eigen::Vector2f& current_root_xy,
                    float current_root_yaw,
                    const Eigen::Quaternionf& current_root_quat,
                    const Eigen::Quaternionf& current_root_quat_unbiased,
                    bool use_motion_root_command = true,
                    bool use_motion_velocity_command = true,
                    bool loop_reference = true);

        const Eigen::VectorXf& command_joint_pos() const { return joint_pos_; }
        const Eigen::VectorXf& command_joint_vel() const { return joint_vel_; }
        const Eigen::Matrix<float, 6, 1>& command_root_ori_b() const { return root_ori_b_; }
        const Eigen::Matrix<float, 6, 1>& command_root_ori_b_unbiased() const { return root_ori_b_unbiased_; }
        const Eigen::Vector3f& command_xy_yaw_vel() const { return xy_yaw_vel_; }
        const Eigen::Vector2f& command_yaw() const { return yaw_command_; }
        const std::vector<float>& future_commands() const { return future_commands_; }
        const Eigen::Matrix<float, 6, 1>& command_foot_support_state() const { return foot_support_state_; }
        const Eigen::Vector3f& ref_com_rel_navi() const { return ref_com_rel_navi_; }
        const Eigen::Vector3f& ref_com_vel_navi() const { return ref_com_vel_navi_; }
        size_t current_frame_index() const { return current_frame_index_; }
        float current_time_s() const { return current_time_s_; }
        float duration() const { return duration_; }
        bool is_live_stream() const { return live_stream_enabled_; }
        size_t live_buffer_size() const;

    private:
        struct LiveFrame
        {
            uint64_t sequence = 0;
            uint64_t publish_time_ns = 0;
            bool reset = false;
            bool end = false;
            Eigen::Matrix<float, kJointDim, 1> joint_pos = Eigen::Matrix<float, kJointDim, 1>::Zero();
            Eigen::Matrix<float, kJointDim, 1> joint_vel = Eigen::Matrix<float, kJointDim, 1>::Zero();
            Eigen::Quaternionf root_quat_w = Eigen::Quaternionf::Identity();
            Eigen::Vector3f root_lin_vel_w = Eigen::Vector3f::Zero();
            Eigen::Vector3f root_ang_vel_w = Eigen::Vector3f::Zero();
            int left_foot_contact_state = -1;
            int right_foot_contact_state = -1;
            Eigen::Vector3f ref_com_rel_navi = Eigen::Vector3f::Zero();
            Eigen::Vector3f ref_com_vel_navi = Eigen::Vector3f::Zero();
        };

        std::filesystem::path ensure_cache_file(const std::filesystem::path& motion_file) const;
        void load_cache_file(const std::filesystem::path& cache_file);
        void start_live_receiver();
        void stop_live_receiver();
        void live_receiver_loop();
        bool parse_live_message(const void* data, size_t size, LiveFrame& frame) const;
        void push_live_frame(const LiveFrame& frame);
        void apply_live_frame(const LiveFrame& frame,
                              bool no_global_mode,
                              float current_root_yaw,
                              const Eigen::Quaternionf& current_root_quat,
                              const Eigen::Quaternionf& current_root_quat_unbiased,
                              bool use_motion_root_command,
                              bool use_motion_velocity_command);
        void update_live_future_commands(const std::deque<LiveFrame>& queue_snapshot,
                                         const LiveFrame& fill_frame,
                                         bool no_global_mode,
                                         const Eigen::Quaternionf& current_root_quat,
                                         bool use_motion_root_command,
                                         bool use_motion_velocity_command);

        float fps_ = 50.0f;
        float duration_ = 0.0f;
        size_t future_horizon_ = kDefaultFutureHorizon;

        std::vector<float> joint_pos_seq_;
        std::vector<float> joint_vel_seq_;
        std::vector<float> body_pos_w_seq_;
        std::vector<float> body_quat_w_seq_;
        std::vector<float> body_lin_vel_w_seq_;
        std::vector<float> body_ang_vel_w_seq_;
        std::vector<float> ref_com_rel_navi_seq_;
        std::vector<float> ref_com_vel_navi_seq_;
        std::vector<int64_t> left_foot_contact_state_seq_;
        std::vector<int64_t> right_foot_contact_state_seq_;

        Eigen::VectorXf default_joint_pos_;
        Eigen::VectorXf joint_pos_;
        Eigen::VectorXf joint_vel_;
        Eigen::Matrix<float, 6, 1> root_ori_b_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> root_ori_b_unbiased_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Vector3f xy_yaw_vel_ = Eigen::Vector3f::Zero();
        Eigen::Vector2f yaw_command_ = Eigen::Vector2f(1.0f, 0.0f);
        std::vector<float> future_commands_;
        Eigen::Matrix<float, 6, 1> foot_support_state_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Vector3f ref_com_rel_navi_ = Eigen::Vector3f::Zero();
        Eigen::Vector3f ref_com_vel_navi_ = Eigen::Vector3f::Zero();
        size_t frame_count_ = 0;
        size_t current_frame_index_ = 0;
        float current_time_s_ = 0.0f;
        float initial_ref_yaw_bias_ = 0.0f;

        bool live_stream_enabled_ = false;
        LiveStreamConfig live_config_;
        mutable std::mutex live_mutex_;
        std::deque<LiveFrame> live_queue_;
        LiveFrame live_last_frame_;
        bool live_has_last_frame_ = false;
        std::atomic_bool live_receiver_running_{false};
        std::thread live_receiver_thread_;
        uint64_t live_last_sequence_ = 0;

        float wrap_to_pi(float angle) const;
    };

    State_Track(int state_mode, std::string state_string = "Track");

    double run_dt() const override { return hybrid_locomotion_enabled_ && !active_tracking_ ? 0.001 : 0.02; }
    void enter();
    void run();
    void exit();

    static std::shared_ptr<ReferenceLoader> reference;
    static void request_motion_file(const std::filesystem::path& motion_file);
    static bool has_pending_motion_request();

private:
    static std::optional<std::filesystem::path> consume_pending_motion_file();
    void dump_first_frame_debug(const std::unordered_map<std::string, std::vector<float>>& obs,
                                const std::vector<float>& action,
                                const std::vector<float>& target_q);
    void write_npy_float(const std::filesystem::path& path,
                         const std::vector<float>& data,
                         const std::vector<size_t>& shape) const;
    void write_npy_int64(const std::filesystem::path& path,
                         const std::vector<int64_t>& data,
                         const std::vector<size_t>& shape) const;
    void write_npy_header(std::ofstream& out,
                          const std::string& descr,
                          const std::vector<size_t>& shape) const;
    void open_observation_dump();
    void dump_control_frame(const std::unordered_map<std::string, std::vector<float>>& obs,
                            const std::vector<float>& action,
                            const std::vector<float>& target_q);
    void close_observation_dump();
    void apply_head_hold_command();
    void configure_pd_gain_randomization();
    void reset_pd_gain_scales();
    bool poll_motion_request_file();
    bool route_profile_request_to(const std::string& target_state);
    bool start_requested_motion(const std::filesystem::path& motion_file);
    void run_tracking_policy();
    void run_locomotion_policy();
    void write_policy_action(const std::vector<float>& action,
                             const std::vector<float>& kp,
                             const std::vector<float>& kd,
                             isaaclab::ManagerBasedRLEnv* policy_env);
    void initialize_policy_motors(isaaclab::ManagerBasedRLEnv* policy_env,
                                  const std::vector<float>& kp,
                                  const std::vector<float>& kd,
                                  bool clear_uncontrolled_motors);
    void apply_hybrid_idle_hold();
    void start_locomotion_policy_thread();
    void stop_locomotion_policy_thread();

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> locomotion_env_;
    std::shared_ptr<ReferenceLoader> reference_;
    std::filesystem::path default_motion_file_;
    std::filesystem::path request_file_;
    float reference_fps_ = 50.0f;
    size_t reference_future_horizon_ = ReferenceLoader::kDefaultFutureHorizon;
    std::vector<int> override_joint_ids_;  // Joints whose positions are overridden by reference motion
    std::vector<float> policy_kp_;
    std::vector<float> policy_kd_;
    std::vector<float> locomotion_policy_kp_;
    std::vector<float> locomotion_policy_kd_;
    std::vector<float> hybrid_idle_hold_q_;
    std::vector<float> hybrid_idle_hold_kp_;
    std::vector<float> hybrid_idle_hold_kd_;
    std::vector<float> pd_kp_scale_;
    std::vector<float> pd_kd_scale_;
    std::vector<int> pd_gain_mask_;
    std::vector<float> pd_kp_scale_range_{1.0f, 1.0f};
    std::vector<float> pd_kd_scale_range_{1.0f, 1.0f};
    bool pd_gain_randomization_enabled_ = false;
    bool pd_gain_randomize_on_reset_ = false;
    std::mt19937 pd_gain_rng_{std::random_device{}()};
    std::vector<int> head_hold_sdk_slots_;
    std::vector<float> head_hold_q_;
    std::vector<float> head_hold_kp_;
    std::vector<float> head_hold_kd_;
    std::filesystem::path debug_dump_dir_;
    bool debug_dump_first_frame_ = false;
    bool first_frame_debug_dumped_ = false;
    bool use_motion_root_command_ = false;
    bool use_motion_velocity_command_ = false;
    bool no_global_mode_ = false;
    bool one_shot_mode_ = false;
    bool require_requested_motion_ = false;
    bool playback_complete_ = false;
    bool active_tracking_ = false;
    bool hybrid_locomotion_enabled_ = false;
    bool locomotion_policy_thread_running_ = false;
    bool has_initial_yaw_bias_ = false;
    float initial_yaw_bias_ = 0.0f;
    bool observation_dump_enabled_ = false;
    bool live_stream_enabled_ = false;
    std::filesystem::path observation_dump_base_file_;
    std::filesystem::path observation_dump_file_;
    std::ofstream observation_dump_stream_;
    size_t observation_dump_frame_ = 0;
    std::thread locomotion_policy_thread_;

    static std::mutex pending_motion_mutex_;
    static std::optional<std::filesystem::path> pending_motion_file_;
};

REGISTER_FSM(State_Track)
