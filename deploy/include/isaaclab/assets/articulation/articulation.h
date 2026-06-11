// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

namespace isaaclab
{

class MotionLoader;

struct ArticulationData
{
    struct LiveState
    {
        bool has_lowstate = false;
        bool has_highstate = false;
        Eigen::Vector3f root_pos_w = Eigen::Vector3f::Zero();
        Eigen::Vector3f root_lin_vel_w = Eigen::Vector3f::Zero();
        Eigen::Quaternionf root_quat_w = Eigen::Quaternionf::Identity();
        Eigen::Vector3f root_gyro_b = Eigen::Vector3f::Zero();
        Eigen::VectorXf qpos;
        Eigen::VectorXf qvel;
    };

    Eigen::Vector3f GRAVITY_VEC_W = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f FORWARD_VEC_B = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    std::vector<float> joint_stiffness; // sdk order
    std::vector<float> joint_damping; // sdk order

    // Joint positions of all joints.
    Eigen::VectorXf joint_pos;
    
    // Default joint positions of all joints.
    Eigen::VectorXf default_joint_pos;

    // Joint velocities of all joints.
    Eigen::VectorXf joint_vel;

    // Root angular velocity in base world frame.
    Eigen::Vector3f root_ang_vel_b;

    // Projection of the gravity direction on base frame.
    Eigen::Vector3f projected_gravity_b;

    Eigen::Quaternionf root_quat_w;

    std::vector<int> joint_ids_map;
    std::vector<int> sdk_joint_ids_map;

    int policy_joint_to_sdk_slot(int policy_index) const
    {
        const int logical_joint_index = joint_ids_map[policy_index];
        if (sdk_joint_ids_map.empty()) {
            return logical_joint_index;
        }
        return sdk_joint_ids_map[logical_joint_index];
    }

    unitree::common::UnitreeJoystick* joystick = nullptr;
    LiveState live_state;
};

class Articulation
{
public:
    Articulation(){}

    virtual void update(){};

    ArticulationData data;
};

};
