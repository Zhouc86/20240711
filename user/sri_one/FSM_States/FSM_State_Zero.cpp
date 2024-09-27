//
// Created by han on 23-3-22.
//

#include "FSM_State_Zero.h"

FSM_State_Zero::FSM_State_Zero(const std::shared_ptr<ControlFSMData> &control_data)
    : FSM_State(control_data, FSM_StateName::ZERO, "ZERO") {
}

void FSM_State_Zero::onEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.zero();

    this->data_->state_no_ = K_ZERO;

    // Reset counter
    iter_ = 0;

    std::cout << "[CONTROL FSM] Enter into ZERO " << std::endl;
    data_->is_state_estimator_init_ = true;

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_Zero::run() {
    ++iter_;

    if(iter_ == 500) {
        for (int i = 0; i < 12; ++i) {
            joint_inc_[i] = (data_->qpos_init_[i] - data_->position_[7+i]) / 5000.0;
            joint_act_[i] = data_->position_[7+i];
        }
    }
    if(iter_ > 500 && iter_ <= 5500) {
        for (int i = 0; i < 12; ++i) {
            data_->kp_[i] = 100;
            data_->kd_[i] = 2;
            data_->q_des_[i] = joint_act_[i] + joint_inc_[i] * (iter_ - 500);
            data_->dq_des_[i] = 0;
            data_->tau_[i] = 0.0;
        }
    }
}


FSM_StateName FSM_State_Zero::checkTransition() {
    this->next_state_name_ = this->state_name_;

    iter_++;

    if (iter_ > 5500) {
        this->next_state_name_ = FSM_StateName::CMPC;
    }

    // Get the next state
    return this->next_state_name_;
}

TransitionData FSM_State_Zero::transition() {
    // Switch FSM control mode

    // Finish transition
    this->transition_data_.done = true;

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * Cleans up the state information on exiting the state.
 */
void FSM_State_Zero::onExit() {
    // Nothing to clean up when exiting
}
