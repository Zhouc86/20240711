//
// Created by han on 24-10-12.
//

#include "InEkfStateEstimator.h"


InEkfStateEstimator::InEkfStateEstimator(std::shared_ptr<ControlFSMData> data, int type)
    : data_(data), type_(type), is_first_(true) {
}


void InEkfStateEstimator::init() {
    if (type_ == 1) {
        //  ---- Initialize invariant extended Kalman filter ----- //
        inekf::RobotState initial_state;

        // Initialize state mean
        Eigen::Matrix3d R0;
        Eigen::Vector3d v0, p0, bg0, ba0;
        R0 << 1, 0, 0, // initial orientation
                0, 1, 0, // IMU frame is rotated 90deg about the x-axis
                0, 0, 1;
        v0 << 0, 0, 0; // initial velocity
        p0 << 0, 0, 0.0; // initial position
        bg0 << 0, 0, 0; // initial gyroscope bias
        ba0 << 0, 0, 0; // initial accelerometer bias
        initial_state.setRotation(R0);
        initial_state.setVelocity(v0);
        initial_state.setPosition(p0);
        initial_state.setGyroscopeBias(bg0);
        initial_state.setAccelerometerBias(ba0);
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(15, 15);
        P.block<3, 3>(0, 0) = 0.0001 * Eigen::MatrixXd::Identity(3, 3); // rotation
        P.block<3, 3>(3, 3) = 0.01 * Eigen::MatrixXd::Identity(3, 3); // velocity
        P.block<3, 3>(6, 6) = 0.0001 * Eigen::MatrixXd::Identity(3, 3); // position
        P.block<3, 3>(9, 9) = 0.0001 * Eigen::MatrixXd::Identity(3, 3); // gyro bias
        P.block<3, 3>(12, 12) = 0.01 * Eigen::MatrixXd::Identity(3, 3); // accel bias

        initial_state.setP(P);

        // Initialize state covariance
        inekf::NoiseParams noise_params;
        noise_params.setGyroscopeNoise(0.002);
        noise_params.setAccelerometerNoise(0.04);
        noise_params.setGyroscopeBiasNoise(0.001);
        noise_params.setAccelerometerBiasNoise(0.001);
        noise_params.setContactNoise(0.05);
        

        position_filter_ = std::make_shared<inekf::InEKF>(initial_state, noise_params);

        //cov_w_ = 0.0174533 * Eigen::MatrixXd::Identity(3, 3);
        cov_w_ = 0.00001 * Eigen::MatrixXd::Identity(3, 3);

        data_->inekf_est_pos_(0) = 0.0;
        data_->inekf_est_pos_(1) = 0.0;
        data_->inekf_est_pos_(2) = 0.0;

        data_->inekf_est_rot_(0) = 1;
        data_->inekf_est_rot_(1) = 0;
        data_->inekf_est_rot_(2) = 0;
        data_->inekf_est_rot_(3) = 0;

        data_->inekf_est_lin_vel_.setZero();
        data_->inekf_est_ang_vel_.setZero();

        //foot order is LF RF LH RH

        urdf_order_to_joint_[0][0] = 7;
        urdf_order_to_joint_[0][1] = 8;
        urdf_order_to_joint_[0][2] = 9;
        urdf_order_to_joint_[1][0] = 10;
        urdf_order_to_joint_[1][1] = 11;
        urdf_order_to_joint_[1][2] = 12;
        urdf_order_to_joint_[2][0] = 13;
        urdf_order_to_joint_[2][1] = 14;
        urdf_order_to_joint_[2][2] = 15;
        urdf_order_to_joint_[3][0] = 16;
        urdf_order_to_joint_[3][1] = 17;
        urdf_order_to_joint_[3][2] = 18;
    }
}

void InEkfStateEstimator::run() {
    if (type_ == 1) {
        // 2. propagate step
        Eigen::VectorXd imu_measurement(6);
        imu_measurement << data_->imu_gyr_(0), data_->imu_gyr_(1), data_->imu_gyr_(2),
                data_->imu_acc_(0), data_->imu_acc_(1), data_->imu_acc_(2);

        position_filter_->Propagate(imu_measurement, 0.001);

        // 3. contact
        // lf -> 0, rf -> 1, lh -> 2, rh -> 3
        std::vector<std::pair<int, bool> > contacts;
        if (data_->contact_est_[0] == 1) {
            contacts.emplace_back(0, true);
        } else {
            contacts.emplace_back(0, false);
        }
        if (data_->contact_est_[1] == 1) {
            contacts.emplace_back(1, true);
        } else {
            contacts.emplace_back(1, false);
        }
        if (data_->contact_est_[2] == 1) {
            contacts.emplace_back(2, true);
        } else {
            contacts.emplace_back(2, false);
        }
        if (data_->contact_est_[3] == 1) {
            contacts.emplace_back(3, true);
        } else {
            contacts.emplace_back(3, false);
        }

        position_filter_->setContacts(contacts);

        // 4. leg kinematics

        inekf::vectorKinematics measured_kinematics;
        // lf -> 0, rf -> 1, lh -> 2, rh -> 3

        // update joint val
        // foot order is LF RF LH RH

        VectorXd v_(data_->plant_->num_velocities());
        v_ << data_->velocity_;
        VectorXd x_(data_->plant_->num_positions() + data_->plant_->num_velocities());
        x_ << data_->position_, data_->velocity_;
        for (int i = 0; i < 4; ++i) {
            data_->data_buffer_.leg_data_[i].joint_1_ = x_(urdf_order_to_joint_.at(i).at(0));
            data_->data_buffer_.leg_data_[i].joint_2_ = x_(urdf_order_to_joint_.at(i).at(1));
            data_->data_buffer_.leg_data_[i].joint_3_ = x_(urdf_order_to_joint_.at(i).at(2));

            data_->data_buffer_.leg_data_[i].joint_velocity_(0) = v_((urdf_order_to_joint_.at(i).at(0) - 1));
            data_->data_buffer_.leg_data_[i].joint_velocity_(1) = v_((urdf_order_to_joint_.at(i).at(1) - 1));
            data_->data_buffer_.leg_data_[i].joint_velocity_(2) = v_((urdf_order_to_joint_.at(i).at(2) - 1));
        }

        // update leg position and jac in base frame
        Eigen::Vector3d tmp_q;
        Eigen::Vector3d zero_position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d zero_rotation = Eigen::Matrix3d::Identity();

        // 1. LF
        tmp_q(0) = data_->data_buffer_.leg_data_[0].joint_1_;
        tmp_q(1) = data_->data_buffer_.leg_data_[0].joint_2_;
        tmp_q(2) = data_->data_buffer_.leg_data_[0].joint_3_;
        data_->sri_kin_->fk(tmp_q, 1);
        data_->sri_kin_->fk_ext(zero_position, zero_rotation, 1);
        data_->sri_kin_->jac(tmp_q, 1);
        data_->data_buffer_.leg_data_[0].J_ = data_->sri_kin_->lf_jac_;

        data_->lf_pos_ = data_->sri_kin_->lf_in_world_.block<3, 1>(0, 3);


        Eigen::Matrix4d lf_toe_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> lf_covariance = Eigen::MatrixXd::Identity(6, 6);

        lf_toe_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        lf_toe_pose.block<3, 1>(0, 3) = data_->sri_kin_->lf_in_world_.block<3, 1>(0, 3);
        lf_covariance.block<3, 3>(3, 3) = data_->data_buffer_.leg_data_[0].J_*cov_w_*data_->data_buffer_.leg_data_[0].J_.transpose();

        inekf::Kinematics lf_frame(0, lf_toe_pose, lf_covariance);
        measured_kinematics.push_back(lf_frame);

        //2. RF
        tmp_q(0) = data_->data_buffer_.leg_data_[1].joint_1_;
        tmp_q(1) = data_->data_buffer_.leg_data_[1].joint_2_;
        tmp_q(2) = data_->data_buffer_.leg_data_[1].joint_3_;
        data_->sri_kin_->fk(tmp_q, 2);
        data_->sri_kin_->fk_ext(zero_position, zero_rotation, 2);
        data_->sri_kin_->jac(tmp_q, 2);
        data_->data_buffer_.leg_data_[1].J_ = data_->sri_kin_->rf_jac_;

        data_->rf_pos_ = data_->sri_kin_->rf_in_world_.block<3, 1>(0, 3);

        Eigen::Matrix4d rf_toe_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> rf_covariance = Eigen::MatrixXd::Identity(6, 6);

        rf_toe_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        rf_toe_pose.block<3, 1>(0, 3) = data_->sri_kin_->rf_in_world_.block<3, 1>(0, 3);
        rf_covariance.block<3, 3>(3, 3) = data_->data_buffer_.leg_data_[1].J_*cov_w_*data_->data_buffer_.leg_data_[1].J_.transpose();

        inekf::Kinematics rf_frame(1, rf_toe_pose, rf_covariance);
        measured_kinematics.push_back(rf_frame);

        //3. LH
        tmp_q(0) = data_->data_buffer_.leg_data_[2].joint_1_;
        tmp_q(1) = data_->data_buffer_.leg_data_[2].joint_2_;
        tmp_q(2) = data_->data_buffer_.leg_data_[2].joint_3_;
        data_->sri_kin_->fk(tmp_q, 3);
        data_->sri_kin_->fk_ext(zero_position, zero_rotation, 3);
        data_->sri_kin_->jac(tmp_q, 3);
        data_->data_buffer_.leg_data_[2].J_ = data_->sri_kin_->lh_jac_;

        data_->lh_pos_ = data_->sri_kin_->lh_in_world_.block<3, 1>(0, 3);

        Eigen::Matrix4d lh_toe_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> lh_covariance = Eigen::MatrixXd::Identity(6, 6);

        lh_toe_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        lh_toe_pose.block<3, 1>(0, 3) = data_->sri_kin_->lh_in_world_.block<3, 1>(0, 3);
        lh_covariance.block<3, 3>(3, 3) = data_->data_buffer_.leg_data_[2].J_*cov_w_*data_->data_buffer_.leg_data_[2].J_.transpose();


        inekf::Kinematics lh_frame(2, lh_toe_pose, lh_covariance);
        measured_kinematics.push_back(lh_frame);

        //4. RH
        tmp_q(0) = data_->data_buffer_.leg_data_[3].joint_1_;
        tmp_q(1) = data_->data_buffer_.leg_data_[3].joint_2_;
        tmp_q(2) = data_->data_buffer_.leg_data_[3].joint_3_;
        data_->sri_kin_->fk(tmp_q, 4);
        data_->sri_kin_->fk_ext(zero_position, zero_rotation, 4);
        data_->sri_kin_->jac(tmp_q, 4);
        data_->data_buffer_.leg_data_[3].J_ = data_->sri_kin_->rh_jac_;

        data_->rh_pos_ = data_->sri_kin_->rh_in_world_.block<3, 1>(0, 3);

        Eigen::Matrix4d rh_toe_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> rh_covariance = Eigen::MatrixXd::Identity(6, 6);

        rh_toe_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        rh_toe_pose.block<3, 1>(0, 3) = data_->sri_kin_->rh_in_world_.block<3, 1>(0, 3);
        rh_covariance.block<3, 3>(3, 3) = data_->data_buffer_.leg_data_[3].J_*cov_w_*data_->data_buffer_.leg_data_[3].J_.transpose();

        inekf::Kinematics rh_frame(3, rh_toe_pose, rh_covariance);
        measured_kinematics.push_back(rh_frame);

        position_filter_->CorrectKinematics(measured_kinematics);

        data_->inekf_est_pos_ = position_filter_->getState().getPosition();
        Eigen::Quaterniond q = Eigen::Quaterniond(position_filter_->getState().getRotation()).normalized();
        data_->inekf_est_rot_(0) = q.w();
        data_->inekf_est_rot_(1) = q.x();
        data_->inekf_est_rot_(2) = q.y();
        data_->inekf_est_rot_(3) = q.z();

        data_->inekf_est_ang_vel_ = position_filter_->getState().getRotation()*imu_measurement.head(3);
        data_->inekf_est_lin_vel_= position_filter_->getState().getVelocity();


    }
}
