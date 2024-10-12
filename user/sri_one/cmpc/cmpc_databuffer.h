//
// Created by han on 2021/5/10.
//

#ifndef AMBER_CMPC_DATABUFFER_H
#define AMBER_CMPC_DATABUFFER_H

#include <eigen3/Eigen/Core>
#include "array"

#include "FootSwingTrajectory.h"

struct LegData
{
    Eigen::Vector3d leg_base_p_;
    Eigen::Vector3d leg_end_p_;
    Eigen::Vector3d leg_base_in_local_p_;
    Eigen::Vector3d raibert_position_;

    // task space
    Eigen::Vector3d position_des_;
    Eigen::Vector3d velocity_des_;
    Eigen::Vector3d position_act_;
    Eigen::Vector3d velocity_act_;

    // joint space
    Eigen::Vector3d joint_velocity_;


    Eigen::Matrix3d kp_task_;
    Eigen::Matrix3d kd_task_;
    Eigen::Matrix3d kp_joint_;
    Eigen::Matrix3d kd_joint_;

    Eigen::Vector3d force_ff_;

    Eigen::Matrix3d J_;

    Eigen::Vector3d foot_init_position_;

    double joint_1_;
    double joint_2_;
    double joint_3_;

    Eigen::MatrixXd J_world_;
    Eigen::Vector3d dotJ_v_;
    Eigen::Vector3d velocity_world_;

    Eigen::Vector3d des_position_;
};


class CMPCDataBuffer {
public:
    double mpc_pre_update_time = 0;
    int mpc_no_update_counter = 10;

    int current_fsm_state = 0;
    double current_domain_start_time = 0.0;
    int contact_data[40];

    double torque_first[12];
    double torque_second[12];

    double roll_int = 0;
    double pitch_int = 0;

    double x_comp_integral = 0;

    bool is_first = true;

    // traj for swing leg
    bool first_swing[4];
    double swing_times[4];
    double swing_time_remaining[4];

    double dx_des_ = 0.0;
    double dy_des_ = 0.0;

    double dyaw_v_ = 0.0;

    Eigen::Vector4d swing_states;

    std::array<LegData, 4> leg_data_;

    FootSwingTrajectory<double> foot_swing_trajectory_[4];

    Eigen::Vector3d rpy_;
    // wbc data
    Vec3<double> world_position_desired_;
    // base
    Eigen::Vector3d base_position_des_;
    Eigen::Vector3d base_velocity_des_;
    //Eigen::Vector3d base_acceleration_des_;

    Eigen::Vector3d base_rpy_des_;
    Eigen::Vector3d base_angular_velocity_des_;

    bool is_first_run_ = true;
    double u_[12];

    double z_offset_[4];

};


#endif //AMBER_CMPC_DATABUFFER_H
