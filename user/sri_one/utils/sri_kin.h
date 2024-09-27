//
// Created by han on 24-9-10.
//

#ifndef SRI_KIN_H
#define SRI_KIN_H

#include <Eigen/Dense>


class SriKin {
public:
    SriKin();
    ~SriKin() = default;

    // 1: LF, 2: RF, 3: LH, 4: RH in local coord
    void fk(const Eigen::Vector3d &q, int leg_no);
    void jac(const Eigen::Vector3d &q, int leg_no);

    void fk_ext(const Eigen::Vector3d &base_pos, const Eigen::Matrix3d &base_ori, int leg_no);

    Eigen::Matrix4d LF_;
    Eigen::Matrix4d RF_;
    Eigen::Matrix4d LH_;
    Eigen::Matrix4d RH_;

    Eigen::Matrix4d lf_se3_0_;
    Eigen::Matrix4d lf_se3_1_;
    Eigen::Matrix4d lf_se3_2_;
    Eigen::Matrix4d lf_se3_3_;

    Eigen::Matrix4d rf_se3_0_;
    Eigen::Matrix4d rf_se3_1_;
    Eigen::Matrix4d rf_se3_2_;
    Eigen::Matrix4d rf_se3_3_;

    Eigen::Matrix4d lh_se3_0_;
    Eigen::Matrix4d lh_se3_1_;
    Eigen::Matrix4d lh_se3_2_;
    Eigen::Matrix4d lh_se3_3_;

    Eigen::Matrix4d rh_se3_0_;
    Eigen::Matrix4d rh_se3_1_;
    Eigen::Matrix4d rh_se3_2_;
    Eigen::Matrix4d rh_se3_3_;

    Eigen::Matrix4d joint_0_;
    Eigen::Matrix4d joint_1_;
    Eigen::Matrix4d joint_2_;

    Eigen::Matrix4d tmp_0_;
    Eigen::Matrix4d tmp_1_;
    Eigen::Matrix4d tmp_2_;
    Eigen::Matrix4d tmp_3_;

    Eigen::Matrix3d lf_jac_;
    Eigen::Matrix3d rf_jac_;
    Eigen::Matrix3d lh_jac_;
    Eigen::Matrix3d rh_jac_;

    // for external fk
    Eigen::Matrix4d lf_in_world_;
    Eigen::Matrix4d rf_in_world_;
    Eigen::Matrix4d lh_in_world_;
    Eigen::Matrix4d rh_in_world_;

    Eigen::Matrix4d lf_in_base_;
    Eigen::Matrix4d rf_in_base_;
    Eigen::Matrix4d lh_in_base_;
    Eigen::Matrix4d rh_in_base_;

    Eigen::Matrix4d lf_base_in_world_;
    Eigen::Matrix4d rf_base_in_world_;
    Eigen::Matrix4d lh_base_in_world_;
    Eigen::Matrix4d rh_base_in_world_;
};



#endif //SRI_KIN_H
