//
// Created by han on 5/18/23.
//

#ifndef BAER_ETHERCAT_FSM_STATE_CMPC_H
#define BAER_ETHERCAT_FSM_STATE_CMPC_H


#include "FSM_State.h"


class FSM_State_CMPC : public FSM_State {
public:
    FSM_State_CMPC(std::shared_ptr<ControlFSMData> control_data);

    // Behavior to be carried out when entering a state
    void onEnter();

    // Run the normal behavior for the state
    void run();

    // Checks for any transition triggers
    FSM_StateName checkTransition();

    // Manages state specific transitions
    TransitionData transition();

    // Behavior to be carried out when exiting a state
    void onExit();

private:
    void update_plan_contact(double current_time) const;
    // Keep track of the control iterations
    int iter_ = 0;
    double current_time_ = 0;

    double dt_ = 0.002;
    double dtMPC_;

    std::map<int, std::map<int, int>> urdf_order_to_joint_;
    std::map<int, std::map<int, int>> motor_no_;

    int n_q_;
    int n_v_;
    int n_u_;

    Eigen::Matrix3d kp_swing_;
    Eigen::Matrix3d kd_swing_;

    Eigen::Vector3d base_p_des_;

    int tick_time_;
    Eigen::Matrix3d zero_matrix_;
    Eigen::Matrix3d iden_matrix_;

    int is_stop_ = 0;
    bool is_first_ = true;

    double joint_value_lh_[3];
    double joint_value_rh_[3];
    double joint_value_ll_[3];
    double joint_value_rl_[3];

    int off_counter_ = 0;

    // for debug
    bool print_debug_info_{false};
};

#endif //BAER_ETHERCAT_FSM_STATE_CMPC_H
