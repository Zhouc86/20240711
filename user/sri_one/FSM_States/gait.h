//
// Created by han on 24-10-10.
//

#ifndef GAIT_H
#define GAIT_H

#include "cppTypes.h"

// only for tort
class Gait {
public:
    Gait(double dt, int mpc_discrete_tick, int mpc_update_tick);
    void run();
    void get_contact_table(int *contact_table);
    Eigen::Vector4d get_swing_states();

private:
    double dt_;
    int mpc_discrete_tick_;
    int mpc_update_tick_;
    uint64_t counter_;
    uint64_t helf_gait_tick_;
    uint64_t gait_tick_;
};



#endif //GAIT_H
