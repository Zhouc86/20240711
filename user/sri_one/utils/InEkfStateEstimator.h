//
// Created by han on 24-10-12.
//

#ifndef INEKFSTATEESTIMATOR_H
#define INEKFSTATEESTIMATOR_H

#include "../FSM_States/ControlFSMData.h"

#include "InEKF.h"

class InEkfStateEstimator {

public:
    InEkfStateEstimator(std::shared_ptr<ControlFSMData> data, int type);

    void init();

    void run();


    std::shared_ptr<ControlFSMData> data_;
    std::shared_ptr<inekf::InEKF> position_filter_;
    int type_;

    bool is_first_;

    // EKF encoder noise
    Eigen::Matrix<double, 3, 3> cov_w_;

    std::map<int, std::map<int, int>> urdf_order_to_joint_;

};



#endif //INEKFSTATEESTIMATOR_H
