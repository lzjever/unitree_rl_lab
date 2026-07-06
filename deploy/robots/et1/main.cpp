#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Velocity.h"
#include "State_Track.h"
#include "State_JointTest.h"
#include "State_JointStepTest.h"
#include "Et1DanceBridge.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <unitree/dds_wrapper/robots/go2/go2.h>

#include "r1_loco_client.hpp"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<HighState_t> FSMState::highstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void clear_stale_general_tracker_request()
{
    const auto tracker_cfg = param::config["FSM"]["GeneralTrackerCJM"];
    if (!tracker_cfg) {
        return;
    }

    std::filesystem::path request_file = tracker_cfg["request_file"]
        ? tracker_cfg["request_file"].as<std::string>()
        : "debug/general_tracker_request.txt";
    if (!request_file.is_absolute()) {
        request_file = param::proj_dir / request_file;
    }

    std::error_code ec;
    const bool removed = std::filesystem::remove(request_file, ec);
    if (ec) {
        spdlog::warn("Failed to clear stale tracker request '{}': {}",
                     request_file.string(),
                     ec.message());
    } else if (removed) {
        spdlog::info("Cleared stale tracker request '{}'", request_file.string());
    }
}

bool is_lowcmd_channel_idle()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::et1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    return lowcmd_sub->isTimeout();
}

bool wait_for_lowcmd_channel_release(double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    do {
        if (is_lowcmd_channel_idle()) {
            return true;
        }
        spdlog::warn("lowcmd channel is still busy; waiting for release...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } while (std::chrono::steady_clock::now() < deadline);
    return is_lowcmd_channel_idle();
}

void init_fsm_state()
{
    if (!wait_for_lowcmd_channel_release(8.0)) {
        spdlog::warn("lowcmd channel is still busy after LocoClient handover; releasing default controller...");
        unitree::robot::go2::shutdown();
        if (!wait_for_lowcmd_channel_release(8.0)) {
            spdlog::critical("The other process is using the lowcmd channel, please close it first.");
            std::exit(-1);
        }
    }

    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    FSMState::highstate = std::make_shared<HighState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

void release_lowcmd_channel_with_loco_handover()
{
    spdlog::info("Lowcmd handover flow: R1 LocoClient StandUp/Damp + MotionSwitcher fallback");
    spdlog::info("Initializing high-level LocoClient for lowcmd handover...");
    unitree::robot::r1::LocoClient loco_client;
    loco_client.Init();
    int fsm_id = -1;
    int32_t fsm_result = loco_client.GetFsmId(fsm_id);
    if (fsm_result == 0) {
        spdlog::info("LocoClient current FSM id before handover: {}", fsm_id);
    } else {
        spdlog::warn("LocoClient GetFsmId before handover returned {}", fsm_result);
    }

    spdlog::info("Commanding robot to StandUp before lowcmd takeover...");
    const int32_t standup_result = loco_client.StandUp();
    if (standup_result != 0) {
        spdlog::warn("LocoClient StandUp returned {}", standup_result);
    }
    sleep(5);
    fsm_result = loco_client.GetFsmId(fsm_id);
    if (fsm_result == 0) {
        spdlog::info("LocoClient FSM id after StandUp: {}", fsm_id);
    } else {
        spdlog::warn("LocoClient GetFsmId after StandUp returned {}", fsm_result);
    }

    spdlog::info("Commanding robot to Damp to release lowcmd channel...");
    const int32_t damp_result = loco_client.Damp();
    if (damp_result != 0) {
        spdlog::warn("LocoClient Damp returned {}", damp_result);
    }
    sleep(3);
    fsm_result = loco_client.GetFsmId(fsm_id);
    if (fsm_result == 0) {
        spdlog::info("LocoClient FSM id after Damp: {}", fsm_id);
    } else {
        spdlog::warn("LocoClient GetFsmId after Damp returned {}", fsm_result);
    }
}

std::optional<std::string> send_sim_control_command(int port, const std::string& command, int timeout_ms = 200)
{
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return std::nullopt;
    }

    timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    const ssize_t sent = sendto(sock, command.c_str(), command.size(), 0,
                                reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (sent < 0) {
        close(sock);
        return std::nullopt;
    }

    char buffer[256] = {};
    const ssize_t received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
    close(sock);
    if (received <= 0) {
        return std::nullopt;
    }

    return std::string(buffer, static_cast<size_t>(received));
}

bool sim_status_has_both_feet_contact(const std::string& status)
{
    return status.find("both=1") != std::string::npos;
}

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);
    clear_stale_general_tracker_request();

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     ET1 Controller \n";
    std::cout << "     Dance debug mode: " << param::dance_debug_mode << "\n";

    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());
    Et1DanceBridge::Instance().Init(param::et1_dance_control_topic, param::et1_dance_status_topic);

    if (!vm.count("sim-auto") && !vm.count("skip-loco-handover")) {
        release_lowcmd_channel_with_loco_handover();
    }

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 1; // ET1 29dof
    if (param::config["controller"]) {
        auto controller = param::config["controller"];
        if (controller["mode_pr"]) {
            FSMState::lowcmd->msg_.mode_pr() = controller["mode_pr"].as<int>();
        }
        if (controller["mode_machine"]) {
            FSMState::lowcmd->msg_.mode_machine() = controller["mode_machine"].as<int>();
        }
    }
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    if (param::is_app_dance_debug_mode()) {
        for (auto& state : fsm->states) {
            const std::string name = state->getStateString();
            if (name == "Velocity" && FSMStringMap.right.count("GeneralTrackerCLN")) {
                state->registered_checks.push_back({
                    []() -> bool { return Et1DanceBridge::Instance().HasPendingStart(); },
                    FSMStringMap.right.at("GeneralTrackerCLN"),
                    "ET1 app dance start"
                });
            } else if ((name == "Passive" || name == "FixStand") && FSMStringMap.right.count("Velocity")) {
                state->registered_checks.push_back({
                    []() -> bool { return Et1DanceBridge::Instance().HasPendingStart(); },
                    FSMStringMap.right.at("Velocity"),
                    "ET1 app dance pre-entry"
                });
            }
        }
    }
    fsm->start();

    if (vm.count("sim-auto")) {
        double fixstand_duration = 3.0;
        auto fixstand_ts = param::config["FSM"]["FixStand"]["ts"];
        if (fixstand_ts && fixstand_ts.IsSequence() && fixstand_ts.size() > 0) {
            fixstand_duration = fixstand_ts[fixstand_ts.size() - 1].as<double>();
        }

        const int sim_control_port = vm["sim-auto-port"].as<int>();
        const double contact_timeout = vm["sim-auto-contact-timeout"].as<double>();

        spdlog::info("Sim auto sequence enabled: Passive -> FixStand -> lower band -> foot contact -> Velocity -> release band");
        std::thread([fsm = fsm.get(), fixstand_duration, sim_control_port, contact_timeout]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            fsm->requestTransition("FixStand", "sim-auto");

            std::this_thread::sleep_for(std::chrono::duration<double>(fixstand_duration));
            if (!send_sim_control_command(sim_control_port, "hold")) {
                spdlog::warn("Sim auto: unitree_mujoco sim control is not responding; holding FixStand.");
                return;
            }

            spdlog::info("Sim auto: lowering elastic band until both feet contact the floor.");
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(contact_timeout);
            bool both_feet_contact = false;
            while (std::chrono::steady_clock::now() < deadline) {
                send_sim_control_command(sim_control_port, "lower");
                std::this_thread::sleep_for(std::chrono::milliseconds(250));

                auto status = send_sim_control_command(sim_control_port, "status");
                if (status && sim_status_has_both_feet_contact(*status)) {
                    both_feet_contact = true;
                    spdlog::info("Sim auto: both feet are in contact with the floor.");
                    break;
                }
            }

            if (!both_feet_contact) {
                spdlog::warn("Sim auto: timed out waiting for both feet contact; holding FixStand.");
                return;
            }

            fsm->requestTransition("Velocity", "sim-auto");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            send_sim_control_command(sim_control_port, "release");
            spdlog::info("Sim auto: elastic band released.");
        }).detach();
    }

    std::cout << "Keyboard: [1] FixStand, [2] Velocity, [3] GeneralTrackerCJM[test; prompts for policy and motion], [4] Dance1/Wind Summer, [5] Dance2/PokerFace, [6] NoHeadPokerFace, [7] JointTest, [8] JointStepTest, [9] GeneralTrackerCLN[master], [0] Passive.\n";
    std::cout << "Sim2Sim: add --sim-auto to enter FixStand, lower MuJoCo's elastic band, wait for foot contact, enter Velocity, then release the band. Real robot deployment remains manual.\n";
    std::cout << "Joystick: FixStand [LT+Up], Velocity [RB+X], GeneralTrackerCLN[master] [LT(2s)+Up], Dance1 [LT(2s)+Down], Dance2 [LT(2s)+Right], NoHeadPokerFace [LT(2s)+Left]. Tracker requests use debug/general_tracker_request.txt; prefix with cjm[test]/cln[master] to route profiles.\n";

    while (true)
    {
        sleep(1);
    }

    return 0;
}
