//
// Created by han on 24-9-10.
//

#include "sri_kin.h"


SriKin::SriKin()
{
    LF_ = Eigen::Matrix4d::Identity();
    RF_ = Eigen::Matrix4d::Identity();
    LH_ = Eigen::Matrix4d::Identity();
    RH_ = Eigen::Matrix4d::Identity();

    lf_se3_0_ << 0, 0, -1, 0,
                 -1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;

    lf_se3_1_ << -2.4078e-01, -7.3234e-02, -9.6781e-01, -0.018388,
                 -9.2594e-01, -2.8161e-01, 2.5167e-01,  0.0047816,
                 -2.9098e-01, 9.5673e-01,  -3.5143e-06, -0.063,
                    0, 0, 0, 1;

    lf_se3_2_  << 0.0, 1.0, 0.0, 0.0,
                  -1.0, 0.0, 0.0, 0.24425,
                  0.0, 0.0, 1.0, 0.087825,
                  0.0, 0.0, 0.0, 1.0;

    lf_se3_3_ << 1.0, 0.0, 0.0, 0.23,
                0.0, 1.0, 0.0, 0.07123,
                0.0, 0.0, 1.0, 0.000525,
                0.0, 0.0, 0.0, 1.0;

    rf_se3_0_ << 0, 0, -1, 0,
                 -1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;

    rf_se3_1_ << 2.5094e-01,   7.6318e-02,  -9.6499e-01, 0.018388,
                 -9.2323e-01, -2.8079e-01,  -2.6229e-01,  0.0049841,
                 -2.9098e-01,  9.5673e-01,  -3.5143e-06, -0.063,
                 0, 0, 0, 1;

    rf_se3_2_ << 0.0, 1.0, 0.0, 0.0,
                 -1.0, 0.0, 0.0, 0.24425,
                 0.0, 0.0, 1.0, -0.088575,
                 0.0, 0.0, 0.0, 1.0;

    rf_se3_3_ << 1.0, 0.0, 0.0, 0.23,
                 0.0, 1.0, 0.0, 0.07123,
                 0.0, 0.0, 1.0, 0.000525,
                 0.0, 0.0, 0.0, 1.0;

    lh_se3_0_ << 0, 0, -1, 0,
                -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 0, 1;

    lh_se3_1_ << -2.4078e-01, -7.3234e-02, -9.6781e-01, -0.018388,
                -9.2594e-01,  -2.8161e-01,  2.5167e-01,  0.0047816,
                -2.9098e-01,   9.5673e-01,  -3.5143e-06, 0.063,
                0, 0, 0, 1;

    lh_se3_2_ << 0.2910, 0.9567, 0.0, 0.0,
                -0.9567, 0.2910, 0.0, 0.24425,
                0.0, 0.0, 1.0, 0.08695,
                0.0, 0.0, 0.0, 1.0;

    lh_se3_3_ << 1.0, 0.0, 0.0, 0.24078,
                0.0, 1.0, 0.0, 0.0012216,
                0.0, 0.0, 1.0, 0.000525,
                0.0, 0.0, 0.0, 1.0;

    rh_se3_0_ << 0, 0, -1, 0,
                -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 0, 1;

    rh_se3_1_ << 2.5046e-01,  7.7890e-02,  -9.6499e-01, 0.018335,
                -9.2145e-01,  -2.8658e-01,  -2.6229e-01,  0.0049841,
                -2.9697e-01,  9.5489e-01,  -3.5075e-06, 0.063,
                0, 0, 0, 1;

    rh_se3_2_ << 0, 1, 0.0, 0.0,
                -1, 0, 0.0, 0.24425,
                0.0, 0.0, 1.0, -0.088575,
                0.0, 0.0, 0.0, 1.0;

    rh_se3_3_ << 1.0, 0.0, 0.0, 0.23,
                0.0, 1.0, 0.0, 0.07123,
                0.0, 0.0, 1.0, -0.00007,
                0.0, 0.0, 0.0, 1.0;

    lf_in_base_ << 0, 0, -1, 0.191,
                 -1, 0, 0, 0.05475,
                 0, 1, 0, 0.058,
                 0, 0, 0, 1;

    rf_in_base_ << 0, 0, -1, 0.191,
                 -1, 0, 0, -0.05475,
                 0, 1, 0, 0.058,
                 0, 0, 0, 1;

    lh_in_base_ << 0, 0, -1, -0.191,
                 -1, 0, 0, 0.05475,
                 0, 1, 0, 0.058,
                 0, 0, 0, 1;

    rh_in_base_ << 0, 0, -1, -0.191,
                 -1, 0, 0, -0.05475,
                 0, 1, 0, 0.058,
                 0, 0, 0, 1;
}



void SriKin::fk(const Eigen::Vector3d &q, int leg_no) {
    joint_0_ << cos(q(0)), -sin(q(0)), 0, 0,
                sin(q(0)), cos(q(0)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    joint_1_ << cos(q(1)), -sin(q(1)), 0, 0,
                sin(q(1)), cos(q(1)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    joint_2_ << cos(q(2)), -sin(q(2)), 0, 0,
                sin(q(2)), cos(q(2)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    if(leg_no == 1) {
        LF_ = lf_se3_0_ * joint_0_ * lf_se3_1_ * joint_1_ * lf_se3_2_ * joint_2_ * lf_se3_3_;
    }else if(leg_no == 2) {
        RF_ = rf_se3_0_ * joint_0_ * rf_se3_1_ * joint_1_ * rf_se3_2_ * joint_2_ * rf_se3_3_;
    }else if(leg_no == 3) {
        LH_ = lh_se3_0_ * joint_0_ * lh_se3_1_ * joint_1_ * lh_se3_2_ * joint_2_ * lh_se3_3_;
    }else if(leg_no == 4) {
        RH_ = rh_se3_0_ * joint_0_ * rh_se3_1_ * joint_1_ * rh_se3_2_ * joint_2_ * rh_se3_3_;
    }
}

void SriKin::jac(const Eigen::Vector3d &q, int leg_no) {
    joint_0_ << cos(q(0)), -sin(q(0)), 0, 0,
                sin(q(0)), cos(q(0)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    joint_1_ << cos(q(1)), -sin(q(1)), 0, 0,
                sin(q(1)), cos(q(1)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    joint_2_ << cos(q(2)), -sin(q(2)), 0, 0,
                sin(q(2)), cos(q(2)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    if(leg_no == 1) {
        tmp_3_ = lf_se3_0_ * joint_0_ * lf_se3_1_ * joint_1_ * lf_se3_2_ * joint_2_ * lf_se3_3_;
        tmp_2_ = lf_se3_0_ * joint_0_ * lf_se3_1_ * joint_1_ * lf_se3_2_;
        tmp_1_ = lf_se3_0_ * joint_0_ * lf_se3_1_;
        tmp_0_ = lf_se3_0_;

        Eigen::Vector3d t0 = tmp_0_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t1 = tmp_1_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t2 = tmp_2_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);

        Eigen::Vector3d t00 = tmp_3_.block(0, 3, 3, 1) - tmp_0_.block(0, 3, 3, 1);
        Eigen::Vector3d t11 = tmp_3_.block(0, 3, 3, 1) - tmp_1_.block(0, 3, 3, 1);
        Eigen::Vector3d t22 = tmp_3_.block(0, 3, 3, 1) - tmp_2_.block(0, 3, 3, 1);

        lf_jac_.block(0, 0, 3, 1) =
            t0.cross(t00);
        lf_jac_.block(0, 1, 3, 1) =
            t1.cross(t11);
        lf_jac_.block(0, 2, 3, 1) =
            t2.cross(t22);
    }else if (leg_no == 2) {
        tmp_3_ = rf_se3_0_ * joint_0_ * rf_se3_1_ * joint_1_ * rf_se3_2_ * joint_2_ * rf_se3_3_;
        tmp_2_ = rf_se3_0_ * joint_0_ * rf_se3_1_ * joint_1_ * rf_se3_2_;
        tmp_1_ = rf_se3_0_ * joint_0_ * rf_se3_1_;
        tmp_0_ = rf_se3_0_;

        Eigen::Vector3d t0 = tmp_0_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t1 = tmp_1_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t2 = tmp_2_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);

        Eigen::Vector3d t00 = tmp_3_.block(0, 3, 3, 1) - tmp_0_.block(0, 3, 3, 1);
        Eigen::Vector3d t11 = tmp_3_.block(0, 3, 3, 1) - tmp_1_.block(0, 3, 3, 1);
        Eigen::Vector3d t22 = tmp_3_.block(0, 3, 3, 1) - tmp_2_.block(0, 3, 3, 1);

        rf_jac_.block(0, 0, 3, 1) =
            t0.cross(t00);
        rf_jac_.block(0, 1, 3, 1) =
            t1.cross(t11);
        rf_jac_.block(0, 2, 3, 1) =
            t2.cross(t22);
    }else if(leg_no == 3) {
        tmp_3_ = lh_se3_0_ * joint_0_ * lh_se3_1_ * joint_1_ * lh_se3_2_ * joint_2_ * lh_se3_3_;
        tmp_2_ = lh_se3_0_ * joint_0_ * lh_se3_1_ * joint_1_ * lh_se3_2_;
        tmp_1_ = lh_se3_0_ * joint_0_ * lh_se3_1_;
        tmp_0_ = lh_se3_0_;

        Eigen::Vector3d t0 = tmp_0_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t1 = tmp_1_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t2 = tmp_2_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);

        Eigen::Vector3d t00 = tmp_3_.block(0, 3, 3, 1) - tmp_0_.block(0, 3, 3, 1);
        Eigen::Vector3d t11 = tmp_3_.block(0, 3, 3, 1) - tmp_1_.block(0, 3, 3, 1);
        Eigen::Vector3d t22 = tmp_3_.block(0, 3, 3, 1) - tmp_2_.block(0, 3, 3, 1);

        lh_jac_.block(0, 0, 3, 1) =
            t0.cross(t00);
        lh_jac_.block(0, 1, 3, 1) =
            t1.cross(t11);
        lh_jac_.block(0, 2, 3, 1) =
            t2.cross(t22);
    }else if(leg_no == 4) {
        tmp_3_ = rh_se3_0_ * joint_0_ * rh_se3_1_ * joint_1_ * rh_se3_2_ * joint_2_ * rh_se3_3_;
        tmp_2_ = rh_se3_0_ * joint_0_ * rh_se3_1_ * joint_1_ * rh_se3_2_;
        tmp_1_ = rh_se3_0_ * joint_0_ * rh_se3_1_;
        tmp_0_ = rh_se3_0_;

        Eigen::Vector3d t0 = tmp_0_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t1 = tmp_1_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d t2 = tmp_2_.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 1);

        Eigen::Vector3d t00 = tmp_3_.block(0, 3, 3, 1) - tmp_0_.block(0, 3, 3, 1);
        Eigen::Vector3d t11 = tmp_3_.block(0, 3, 3, 1) - tmp_1_.block(0, 3, 3, 1);
        Eigen::Vector3d t22 = tmp_3_.block(0, 3, 3, 1) - tmp_2_.block(0, 3, 3, 1);

        rh_jac_.block(0, 0, 3, 1) =
            t0.cross(t00);
        rh_jac_.block(0, 1, 3, 1) =
            t1.cross(t11);
        rh_jac_.block(0, 2, 3, 1) =
            t2.cross(t22);
    }
}


void SriKin::fk_ext(const Eigen::Vector3d &base_pos, const Eigen::Matrix3d &base_ori, int leg_no) {
    Eigen::Matrix4d base_tf = Eigen::Matrix4d::Identity();
    base_tf.block(0, 0, 3, 3) = base_ori;
    base_tf.block(0, 3, 3, 1) = base_pos;

    Eigen::Matrix4d tmp_var;
    tmp_var << 0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1;

    if(leg_no == 1) {
        lf_base_in_world_ = base_tf * lf_in_base_;
        lf_in_world_ = base_tf * lf_in_base_*tmp_var*LF_;
    }else if(leg_no == 2) {
        rf_base_in_world_ = base_tf * rf_in_base_;
        rf_in_world_ = base_tf * rf_in_base_*tmp_var*RF_;
    }else if(leg_no == 3) {
        lh_base_in_world_ = base_tf * lh_in_base_;
        lh_in_world_ = base_tf * lh_in_base_*tmp_var*LH_;
    }else if(leg_no == 4) {
        rh_base_in_world_ = base_tf * rh_in_base_;
        rh_in_world_ = base_tf * rh_in_base_*tmp_var*RH_;
    }
}