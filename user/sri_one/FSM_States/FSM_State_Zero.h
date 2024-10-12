//
// Created by han on 23-3-22.
//

#ifndef BAER_ETHERCAT_FSM_STATE_ZERO_H
#define BAER_ETHERCAT_FSM_STATE_ZERO_H

#include "FSM_State.h"

class FSM_State_Zero : public FSM_State {
public:
    FSM_State_Zero(const std::shared_ptr<ControlFSMData>& control_data);

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
    // Keep track of the control iterations
    int iter_ = 0;
    Byte8 byte8_;

    double joint_inc_[12];
    double joint_act_[12];

};

#endif //BAER_ETHERCAT_FSM_STATE_ZERO_H
