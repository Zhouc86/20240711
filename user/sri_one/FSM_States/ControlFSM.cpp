//
// Created by han on 23-2-16.
//

#include <chrono>
#include "ControlFSM.h"


ControlFSM::ControlFSM(const std::shared_ptr<ControlFSMData>& data){
    data_ = data;

    states_list_.invalid = nullptr;
    states_list_.passive = std::make_shared<FSM_State_Passive>(data);
    states_list_.zero = std::make_shared<FSM_State_Zero>(data);
    states_list_.cmpc = std::make_shared<FSM_State_CMPC>(data);

    initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */

void ControlFSM::initialize() {
    // for test
    current_state_ = states_list_.passive;

    current_state_->onEnter();

    next_state_ = current_state_;

    operating_mode_ = FSM_OperatingMode::NORMAL;

}

void ControlFSM::runFSM() {
    // Publish state estimator data to other computer
    //for(size_t i(0); i<3; ++i){
    //_state_estimator.p[i] = data._stateEstimator->getResult().position[i];
    //_state_estimator.quat[i] = data._stateEstimator->getResult().orientation[i];
    //}
    //_state_estimator.quat[3] = data._stateEstimator->getResult().orientation[3];
    //state_estimator_lcm.publish("state_estimator_ctrl_pc", &_state_estimator);

    operating_mode_ = safetyPreCheck();

    if(is_first_){
        auto const now = std::chrono::high_resolution_clock::now();
        auto const second =
                std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch()
                );

        data_->begin_time_ = static_cast<double>(second.count()) / 1000000.0;
        is_first_ = false;
    }
    // recode time;
    auto const now = std::chrono::high_resolution_clock::now();
    auto const second =
            std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch()
            );

    double current_time = static_cast<double>(second.count()) / 1000000.0;
    data_->time_ = current_time - data_->begin_time_;

    if (operating_mode_ != FSM_OperatingMode::ESTOP) {

        if (operating_mode_ == FSM_OperatingMode::NORMAL) {
            next_state_name_ = current_state_->checkTransition();

            if (next_state_name_ != current_state_->state_name_) {
                operating_mode_ = FSM_OperatingMode::TRANSITIONING;
                next_state_ = getNextState(next_state_name_);


            } else {
                current_state_->run();
            }
        }

        if (operating_mode_ == FSM_OperatingMode::TRANSITIONING) {
            transition_data_ = current_state_->transition();

            safetyPostCheck();

            if (transition_data_.done) {
                current_state_->onExit();

                current_state_ = next_state_;

                current_state_->onEnter();

                operating_mode_ = FSM_OperatingMode::NORMAL;
            }
        } else {
            safetyPostCheck();
        }

    } else { // if ESTOP
        current_state_ = states_list_.passive;
        current_state_->onEnter();
        next_state_name_ = current_state_->state_name_;
    }
}

FSM_OperatingMode ControlFSM::safetyPreCheck() {
    // Check for safe orientation if the current state requires it
    /*if (currentState->checkSafeOrientation && data.controlParameters->control_mode != K_RECOVERY_STAND) {
        if (!safetyChecker->checkSafeOrientation()) {
            operating_mode_Mode = FSM_OperatingMode::ESTOP;
            std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
        }
    }*/

    // Default is to return the current operating mode
    return operating_mode_;
}

FSM_OperatingMode ControlFSM::safetyPostCheck()
{
    // Check for safe desired foot positions
    /*if (currentState->checkPDesFoot) {
        safetyChecker->checkPDesFoot();
    }

    // Check for safe desired feedforward forces
    if (currentState->checkForceFeedForward) {
        safetyChecker->checkForceFeedForward();
    }*/

    // Default is to return the current operating mode
    return operating_mode_;
}

std::shared_ptr<FSM_State> ControlFSM::getNextState(FSM_StateName stateName)
{
    switch (stateName) {
        case FSM_StateName::INVALID:
            return states_list_.invalid;

        case FSM_StateName::PASSIVE:
            return states_list_.passive;

        case FSM_StateName::ZERO:
            return states_list_.zero;

        case FSM_StateName::CMPC:
            return states_list_.cmpc;

        /*case FSM_StateName::JOINT_PD:
            return states_list_.jointPD;*/

        /*case FSM_StateName::IMPEDANCE_CONTROL:
            return statesList.impedanceControl;

        case FSM_StateName::STAND_UP:
            return statesList.standUp;*/

        default:
            return states_list_.invalid;
    }
}

