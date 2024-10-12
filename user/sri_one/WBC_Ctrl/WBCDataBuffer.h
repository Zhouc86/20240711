//
// Created by han on 5/22/23.
//

#ifndef AMBER_C_WBCDATABUFFER_H
#define AMBER_C_WBCDATABUFFER_H

#include "cppTypes.h"

class LocomotionCtrlData{
public:
    Vec3<float> pBody_des;
    Vec3<float> vBody_des;
    Vec3<float> aBody_des;
    Vec3<float> pBody_RPY_des;
    Vec3<float> vBody_Ori_des;

    Vec3<float> pFoot_des[4];
    Vec3<float> vFoot_des[4];
    Vec3<float> aFoot_des[4];
    Vec3<float> Fr_des[4];

    Vec4<float> contact_state;
};


struct WBCDataBuffer{
    WBCDataBuffer(){
        Jc_ = Eigen::MatrixXf::Zero(12, 18);
        JcDotQdot_ = Eigen::MatrixXf::Zero(3, 4);
        foot_position_ = Eigen::MatrixXf::Zero(3, 4);
        foot_velocity_ = Eigen::MatrixXf::Zero(3, 4);
        A_ = Eigen::MatrixXf::Zero(18, 18);
        grav_ = Eigen::MatrixXf::Zero(18, 1);
        coriolis_ = Eigen::MatrixXf::Zero(18, 1);
        q_ = Eigen::VectorXf(18);
        foot_acc_ = Eigen::MatrixXf::Zero(3, 4);
        fr_des_ = Eigen::MatrixXf::Zero(3, 4);

        foot_position_des_ = Eigen::MatrixXf::Zero(3, 4);
        foot_velocity_des_ = Eigen::MatrixXf::Zero(3, 4);
        foot_acc_des_ = Eigen::MatrixXf::Zero(3, 4);
    };

    Eigen::MatrixXf Jc_;
    Eigen::MatrixXf JcDotQdot_;

    Quat<float> bodyOrientation;
    Vec3<float> bodyPosition;
    SVec<float> bodyVelocity;

    Eigen::MatrixXf foot_position_;
    Eigen::MatrixXf foot_velocity_;
    Eigen::MatrixXf foot_acc_;
    Eigen::MatrixXf fr_des_;

    Eigen::MatrixXf foot_position_des_;
    Eigen::MatrixXf foot_velocity_des_;
    Eigen::MatrixXf foot_acc_des_;

    LocomotionCtrlData robot_des_info_;

    Eigen::MatrixXf A_;
    Eigen::VectorXf grav_;
    Eigen::VectorXf coriolis_;

    Eigen::VectorXf q_;

    bool is_first_ = true;
    bool is_first_print = true;
};

#endif //AMBER_C_WBCDATABUFFER_H
