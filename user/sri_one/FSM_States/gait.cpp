//
// Created by han on 24-10-10.
//

#include "gait.h"


Gait::Gait(double dt, int mpc_discrete_tick, int mpc_update_tick) {
    dt_ = dt;
    mpc_discrete_tick_ = mpc_discrete_tick;
    mpc_update_tick_ = mpc_update_tick;
    counter_ = 0;
    helf_gait_tick_ = mpc_discrete_tick_*5;
    gait_tick_ = mpc_discrete_tick_*10;
}

void Gait::run() {
    // std::cout << "run Gait" << std::endl;
    counter_++;

}

void Gait::get_contact_table(int *contact_table) {
    uint64_t begin_tick = counter_ % gait_tick_;
    for (int i=0; i!=10; ++i) {
        uint64_t current_tick = begin_tick + i*mpc_discrete_tick_;
        if(current_tick >= gait_tick_) {
            current_tick = current_tick - gait_tick_;
        }
        if(current_tick < helf_gait_tick_) {
            contact_table[0+i*4] = 1;
            contact_table[1+i*4] = 0;
            contact_table[2+i*4] = 0;
            contact_table[3+i*4] = 1;
        } else {
            contact_table[0+i*4] = 0;
            contact_table[1+i*4] = 1;
            contact_table[2+i*4] = 1;
            contact_table[3+i*4] = 0;
        }
    }
}

Eigen::Vector4d Gait::get_swing_states() {
    uint64_t tmp_tick = counter_ % gait_tick_;
    if(tmp_tick < helf_gait_tick_) {
        return Eigen::Vector4d{0, 1, 1, 0};
    } else {
        return Eigen::Vector4d{1, 0, 0, 1};
    }
}