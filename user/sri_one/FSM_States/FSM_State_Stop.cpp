//
// Created by han on 24-9-29.
//

#include "FSM_State_Stop.h"


FSM_State_Stop::FSM_State_Stop(const std::shared_ptr<ControlFSMData> &control_data)
    : FSM_State(control_data, FSM_StateName::STOP, "STOP") {
}

void FSM_State_Stop::onEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.zero();

    this->data_->state_no_ = K_STOP;

    // Reset counter
    iter_ = 0;

    std::cout << "[CONTROL FSM] Enter into Stop " << std::endl;

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_Stop::run() {
    ++iter_;


    for (int i = 0; i < 12; ++i) {
        data_->kp_[i] = 100;
        data_->kd_[i] = 2;
        data_->q_des_[i] = data_->fsm_stop_q_des_[i];
        data_->dq_des_[i] = 0;
        data_->tau_[i] = 0.0;
    }
}


FSM_StateName FSM_State_Stop::checkTransition() {
    this->next_state_name_ = this->state_name_;

    iter_++;


    // Get the next state
    return this->next_state_name_;
}

TransitionData FSM_State_Stop::transition() {
    // Switch FSM control mode

    // Finish transition
    this->transition_data_.done = true;

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * Cleans up the state information on exiting the state.
 */
void FSM_State_Stop::onExit() {
    // Nothing to clean up when exiting
}
