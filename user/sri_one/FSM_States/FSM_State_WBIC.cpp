//
// Created by han on 5/24/23.
//

#include "FSM_State_WBIC.h"
#include "../cmpc/convexMPC_interface.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

FSM_State_WBIC::FSM_State_WBIC(std::shared_ptr<ControlFSMData> control_data)
        : FSM_State(control_data, FSM_StateName::WBIC, "wbic") {

    n_q_ = data_->plant_->num_positions();
    n_v_ = data_->plant_->num_velocities();
    n_u_ = data_->plant_->num_actuators();

    tick_time_ = 70;

    data_->data_buffer_.mpc_no_update_counter = tick_time_;

    dtMPC_ = dt_ * tick_time_;

    for (int i = 0; i < 4; ++i) {
        data_->data_buffer_.swing_times[i] = dtMPC_ * 5;
        data_->data_buffer_.first_swing[i] = true;
        data_->data_buffer_.swing_time_remaining[i] = dtMPC_ * 5;
    }

    // foot order is 1. LF 2. RF 3. LH 4. RH
    data_->data_buffer_.leg_data_[0].leg_base_in_local_p_ = Eigen::Vector3d(0.191, 0.05475,0.058);
    data_->data_buffer_.leg_data_[1].leg_base_in_local_p_ = Eigen::Vector3d(0.191, -0.05475, 0.058);
    data_->data_buffer_.leg_data_[2].leg_base_in_local_p_ = Eigen::Vector3d(-0.191, 0.05475, 0.058);
    data_->data_buffer_.leg_data_[3].leg_base_in_local_p_ = Eigen::Vector3d(-0.191, -0.05475, 0.058);

    zero_matrix_.setZero();
    iden_matrix_.setIdentity();

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

    //foot order is LF RF LH RH
    // to hardware  hardware order lh -> rh -> ll -> rl
    motor_no_[0][0] = 0;
    motor_no_[0][1] = 1;
    motor_no_[0][2] = 2;
    motor_no_[1][0] = 3;
    motor_no_[1][1] = 4;
    motor_no_[1][2] = 5;
    motor_no_[2][0] = 6;
    motor_no_[2][1] = 7;
    motor_no_[2][2] = 8;
    motor_no_[3][0] = 9;
    motor_no_[3][1] = 10;
    motor_no_[3][2] = 11;

    setup_problem(dtMPC_, 10, 0.4, 200);

    wbc_data_ = std::make_shared<WBCDataBuffer>();
    wbc_ctrl_ = std::make_shared<LocomotionCtrl>(wbc_data_);

    gait_ = std::make_shared<Gait>(dt_, tick_time_, 10);
}

void FSM_State_WBIC::onEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.zero();

    this->data_->state_no_ = K_WBIC;

    // Reset counter
    iter_ = 0;
    current_time_ = 0;

    //this->data_->is_work_ = true;

    std::cout << "[CONTROL FSM] Enter into WBIC" << std::endl;


    base_p_des_ = Eigen::Vector3d(0, 0, 0.3);
    /*data_->only_run_once_counter_ = 0;
    data_->stand_ = false;
    data_->wbic_working_ = 1;*/
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_WBIC::run() {

    current_time_ = iter_ * dt_;
    VectorXd u_sol(n_u_);
    VectorXd q_sol(n_u_);
    VectorXd dq_sol(n_u_);
    ++iter_;

    data_->current_time_ = current_time_;

     VectorXd q_(data_->plant_->num_positions());
    q_ << data_->position_;

    VectorXd v_(data_->plant_->num_velocities());
    v_ << data_->velocity_;

    VectorXd x_(data_->plant_->num_positions() + data_->plant_->num_velocities());
    x_ << data_->position_, data_->velocity_;

    data_->plant_->SetPositions(data_->context_.get(), q_);
    data_->plant_->SetVelocities(data_->context_.get(), v_);


    Eigen::Quaternion<double> base_ori(x_(0), x_(1), x_(2), x_(3));

    drake::math::RollPitchYawd base_o(base_ori);

    //std::cout<<base_o.pitch_angle()<<" "<<base_o.roll_angle()<<std::endl;

    if(walking_counter_ == 50) {
        data_->data_buffer_.dx_des_ = 0.0;
    }
    Vector3d v_des_robot(data_->data_buffer_.dx_des_, data_->data_buffer_.dy_des_, 0);
    Vector3d v_des_world = base_ori.toRotationMatrix().matrix()*v_des_robot;

    if(abs(v_(3)) > 0.2){
        data_->data_buffer_.pitch_int += (dt_*(0-base_o.pitch_angle())/v_(3));
    }

    if(abs(v_(4)) > 0.1){
        data_->data_buffer_.roll_int += (dt_*(0-base_o.roll_angle())/v_(4));
    }

    data_->data_buffer_.pitch_int = fmin(fmax(data_->data_buffer_.pitch_int, -0.25), 0.25);
    data_->data_buffer_.roll_int = fmin(fmax(data_->data_buffer_.roll_int, -0.25), 0.25);

    double roll_comp = v_(4)*data_->data_buffer_.roll_int;
    double pitch_comp = v_(3)*data_->data_buffer_.pitch_int;

    data_->roll_compensate_ = roll_comp;
    data_->pitch_compensate_ = pitch_comp;

    // update leg controller data


    // update joint val
    // foot order is LF RF LH RH
    for (int i = 0; i < 4; ++i) {
        data_->data_buffer_.leg_data_[i].joint_1_ = x_(urdf_order_to_joint_.at(i).at(0));
        data_->data_buffer_.leg_data_[i].joint_2_ = x_(urdf_order_to_joint_.at(i).at(1));
        data_->data_buffer_.leg_data_[i].joint_3_ = x_(urdf_order_to_joint_.at(i).at(2));

        data_->data_buffer_.leg_data_[i].joint_velocity_(0) = v_((urdf_order_to_joint_.at(i).at(0)-1));
        data_->data_buffer_.leg_data_[i].joint_velocity_(1) = v_((urdf_order_to_joint_.at(i).at(1)-1));
        data_->data_buffer_.leg_data_[i].joint_velocity_(2) = v_((urdf_order_to_joint_.at(i).at(2)-1));
    }

    // update leg position and jac in base frame
    Eigen::Vector3d tmp_q;

    // 1. LF
    tmp_q(0) = data_->data_buffer_.leg_data_[0].joint_1_;
    tmp_q(1) = data_->data_buffer_.leg_data_[0].joint_2_;
    tmp_q(2) = data_->data_buffer_.leg_data_[0].joint_3_;
    data_->sri_kin_->fk(tmp_q, 1);
    data_->data_buffer_.leg_data_[0].position_act_ = data_->sri_kin_->LF_.block(0, 3, 3, 1);
    data_->sri_kin_->jac(tmp_q, 1);
    data_->data_buffer_.leg_data_[0].J_ = data_->sri_kin_->lf_jac_;
    data_->data_buffer_.leg_data_[0].velocity_act_ = data_->data_buffer_.leg_data_[0].J_ * tmp_q;

    //2. RF
    tmp_q(0) = data_->data_buffer_.leg_data_[1].joint_1_;
    tmp_q(1) = data_->data_buffer_.leg_data_[1].joint_2_;
    tmp_q(2) = data_->data_buffer_.leg_data_[1].joint_3_;
    data_->sri_kin_->fk(tmp_q, 2);
    data_->data_buffer_.leg_data_[1].position_act_ = data_->sri_kin_->RF_.block(0, 3, 3, 1);
    data_->sri_kin_->jac(tmp_q, 2);
    data_->data_buffer_.leg_data_[1].J_ = data_->sri_kin_->rf_jac_;
    data_->data_buffer_.leg_data_[1].velocity_act_ = data_->data_buffer_.leg_data_[1].J_ * tmp_q;

    //3. LH
    tmp_q(0) = data_->data_buffer_.leg_data_[2].joint_1_;
    tmp_q(1) = data_->data_buffer_.leg_data_[2].joint_2_;
    tmp_q(2) = data_->data_buffer_.leg_data_[2].joint_3_;
    data_->sri_kin_->fk(tmp_q, 3);
    data_->data_buffer_.leg_data_[2].position_act_ = data_->sri_kin_->LH_.block(0, 3, 3, 1);
    data_->sri_kin_->jac(tmp_q, 3);
    data_->data_buffer_.leg_data_[2].J_ = data_->sri_kin_->lh_jac_;
    data_->data_buffer_.leg_data_[2].velocity_act_ = data_->data_buffer_.leg_data_[2].J_ * tmp_q;

    //4. RH
    tmp_q(0) = data_->data_buffer_.leg_data_[3].joint_1_;
    tmp_q(1) = data_->data_buffer_.leg_data_[3].joint_2_;
    tmp_q(2) = data_->data_buffer_.leg_data_[3].joint_3_;
    data_->sri_kin_->fk(tmp_q, 4);
    data_->data_buffer_.leg_data_[3].position_act_ = data_->sri_kin_->RH_.block(0, 3, 3, 1);
    data_->sri_kin_->jac(tmp_q, 4);
    data_->data_buffer_.leg_data_[3].J_ = data_->sri_kin_->rh_jac_;
    data_->data_buffer_.leg_data_[3].velocity_act_ = data_->data_buffer_.leg_data_[3].J_ * tmp_q;

    // update dynamic data using drake
    // LF RF LH RH
    Eigen::MatrixXd J_tmp(3, n_v_);
    Vector3d begin_point;
    Vector3d end_point;
    Eigen::Matrix3d J_t;
    data_->plant_->CalcJacobianTranslationalVelocity(*(data_->context_), drake::multibody::JacobianWrtVariable::kV,
                                                     data_->plant_->GetFrameByName("LF_FOOT"),
                                                     Eigen::Vector3d::Zero(),
                                                     data_->plant_->world_frame(), data_->plant_->world_frame(),
                                                     &J_tmp);

    data_->data_buffer_.leg_data_[0].dotJ_v_ = data_->plant_->CalcBiasTranslationalAcceleration(*(data_->context_),
                                                                                                drake::multibody::JacobianWrtVariable::kV,
                                                                                                data_->plant_->GetFrameByName(
                                                                                                        "LF_FOOT"),
                                                                                                Eigen::Vector3d::Zero(),
                                                                                                data_->plant_->world_frame(),
                                                                                                data_->plant_->world_frame());
    data_->data_buffer_.leg_data_[0].J_world_ = J_tmp;
    data_->data_buffer_.leg_data_[0].velocity_world_ = J_tmp * v_;

    data_->plant_->CalcJacobianTranslationalVelocity(*(data_->context_), drake::multibody::JacobianWrtVariable::kV,
                                                     data_->plant_->GetFrameByName("RF_FOOT"),
                                                     Eigen::Vector3d::Zero(),
                                                     data_->plant_->world_frame(), data_->plant_->world_frame(),
                                                     &J_tmp);

    data_->data_buffer_.leg_data_[1].dotJ_v_ = data_->plant_->CalcBiasTranslationalAcceleration(*(data_->context_),
                                                                                                drake::multibody::JacobianWrtVariable::kV,
                                                                                                data_->plant_->GetFrameByName(
                                                                                                        "RF_FOOT"),
                                                                                                Eigen::Vector3d::Zero(),
                                                                                                data_->plant_->world_frame(),
                                                                                                data_->plant_->world_frame());
    data_->data_buffer_.leg_data_[1].J_world_ = J_tmp;
    data_->data_buffer_.leg_data_[1].velocity_world_ = J_tmp * v_;

    data_->plant_->CalcJacobianTranslationalVelocity(*(data_->context_), drake::multibody::JacobianWrtVariable::kV,
                                                     data_->plant_->GetFrameByName("LH_FOOT"),
                                                     Eigen::Vector3d::Zero(),
                                                     data_->plant_->world_frame(), data_->plant_->world_frame(),
                                                     &J_tmp);

    data_->data_buffer_.leg_data_[2].dotJ_v_ = data_->plant_->CalcBiasTranslationalAcceleration(*(data_->context_),
                                                                                                drake::multibody::JacobianWrtVariable::kV,
                                                                                                data_->plant_->GetFrameByName(
                                                                                                        "LH_FOOT"),
                                                                                                Eigen::Vector3d::Zero(),
                                                                                                data_->plant_->world_frame(),
                                                                                                data_->plant_->world_frame());
    data_->data_buffer_.leg_data_[2].J_world_ = J_tmp;
    data_->data_buffer_.leg_data_[2].velocity_world_ = J_tmp * v_;

    data_->plant_->CalcJacobianTranslationalVelocity(*(data_->context_), drake::multibody::JacobianWrtVariable::kV,
                                                     data_->plant_->GetFrameByName("RH_FOOT"),
                                                     Eigen::Vector3d::Zero(),
                                                     data_->plant_->world_frame(), data_->plant_->world_frame(),
                                                     &J_tmp);

    data_->data_buffer_.leg_data_[3].dotJ_v_ = data_->plant_->CalcBiasTranslationalAcceleration(*(data_->context_),
                                                                                                drake::multibody::JacobianWrtVariable::kV,
                                                                                                data_->plant_->GetFrameByName(
                                                                                                        "RH_FOOT"),
                                                                                                Eigen::Vector3d::Zero(),
                                                                                                data_->plant_->world_frame(),
                                                                                                data_->plant_->world_frame());
    data_->data_buffer_.leg_data_[3].J_world_ = J_tmp;
    data_->data_buffer_.leg_data_[3].velocity_world_ = J_tmp * v_;





    // 1.5 fsm
    if ((current_time_ >= (data_->data_buffer_.current_domain_start_time + dtMPC_ * 5 - 0.005)) or
        data_->data_buffer_.is_first) {
        if (data_->data_buffer_.current_fsm_state == 1) {
            data_->data_buffer_.current_fsm_state = 0;
        } else {
            data_->data_buffer_.current_fsm_state = 1;
        }
        data_->data_buffer_.current_domain_start_time = current_time_;
        data_->data_buffer_.is_first = false;
        ++walking_counter_;
    }

    // 2. get foot position

    // foot order is LF RF LH RH

    Eigen::Matrix3d base_ori_mat = base_ori.toRotationMatrix();
    Eigen::Vector3d base_pos(x_(4), x_(5), x_(6));

    data_->sri_kin_->fk_ext(base_pos, base_ori_mat, 1);
    data_->data_buffer_.leg_data_[0].leg_end_p_ = data_->sri_kin_->lf_in_world_.block(0, 3, 3, 1);
    data_->data_buffer_.leg_data_[0].leg_base_p_ = data_->sri_kin_->lf_base_in_world_.block(0, 3, 3, 1);

    data_->sri_kin_->fk_ext(base_pos, base_ori_mat, 2);
    data_->data_buffer_.leg_data_[1].leg_end_p_ = data_->sri_kin_->rf_in_world_.block(0, 3, 3, 1);
    data_->data_buffer_.leg_data_[1].leg_base_p_ = data_->sri_kin_->rf_base_in_world_.block(0, 3, 3, 1);

    data_->sri_kin_->fk_ext(base_pos, base_ori_mat, 3);
    data_->data_buffer_.leg_data_[2].leg_end_p_ = data_->sri_kin_->lh_in_world_.block(0, 3, 3, 1);
    data_->data_buffer_.leg_data_[2].leg_base_p_ = data_->sri_kin_->lh_base_in_world_.block(0, 3, 3, 1);

    data_->sri_kin_->fk_ext(base_pos, base_ori_mat, 4);
    data_->data_buffer_.leg_data_[3].leg_end_p_ = data_->sri_kin_->rh_in_world_.block(0, 3, 3, 1);
    data_->data_buffer_.leg_data_[3].leg_base_p_ = data_->sri_kin_->rh_base_in_world_.block(0, 3, 3, 1);



    if (data_->data_buffer_.is_first_run_) {
        for (int i = 0; i < 4; i++) {
            data_->data_buffer_.foot_swing_trajectory_[i].setHeight(0.08);
            data_->data_buffer_.foot_swing_trajectory_[i].setInitialPosition(
                    data_->data_buffer_.leg_data_[i].leg_end_p_);
            data_->data_buffer_.foot_swing_trajectory_[i].setFinalPosition(data_->data_buffer_.leg_data_[i].leg_end_p_);

            wbc_data_->foot_position_.block<3, 1>(0, i) = data_->data_buffer_.leg_data_[i].leg_end_p_.cast<float>();
            wbc_data_->foot_velocity_.block<3, 1>(0, i) = Eigen::Vector3f::Zero();
            wbc_data_->foot_acc_.block<3, 1>(0, i) = Eigen::Vector3f::Zero();

            wbc_data_->foot_position_des_.block<3, 1>(0, i) = data_->data_buffer_.leg_data_[i].leg_end_p_.cast<float>();
            wbc_data_->foot_velocity_des_.block<3, 1>(0, i) = Eigen::Vector3f::Zero();
            wbc_data_->foot_acc_des_.block<3, 1>(0, i) = Eigen::Vector3f::Zero();

        }
        data_->data_buffer_.world_position_desired_[0] = q_(4);
        data_->data_buffer_.world_position_desired_[1] = q_(5);
        data_->data_buffer_.world_position_desired_[2] = data_->data_buffer_.rpy_(2);
        data_->data_buffer_.is_first_run_ = false;
    }

    data_->data_buffer_.world_position_desired_ += 0.001 * Vec3<double>(v_des_world[0], v_des_world[1], 0);

    // 3. raibert controller
    double side_sign[4] = {1, -1, 1, -1};
    double interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
    double interleave_gain = 0.0;
    double v_abs = std::fabs(v_des_robot(0));

    //std::cout<<"~~~~~~~~~~~~~"<<std::endl;
    for(int i = 0; i < 4; ++i){
        if(data_->data_buffer_.first_swing[i]) {
            data_->data_buffer_.swing_time_remaining[i] = data_->data_buffer_.swing_times[i];
        } else {
            data_->data_buffer_.swing_time_remaining[i] -= dt_;
        }

        Eigen::Vector3d offset(0, side_sign[i] * .10, 0);
        Eigen::Vector3d pRobotFrame = (data_->data_buffer_.leg_data_[i].leg_base_in_local_p_ + offset);

        // pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;

        double yaw_ang = (-data_->data_buffer_.dyaw_v_*dtMPC_*5)/2.0;
        Eigen::Matrix3d rotz;
        rotz << cos(yaw_ang), -sin(yaw_ang), 0,
                sin(yaw_ang), cos(yaw_ang), 0,
                0, 0, 1;

        Eigen::Vector3d pYawCorrected = rotz*pRobotFrame;

        Eigen::Vector3d des_vel(data_->data_buffer_.dx_des_, data_->data_buffer_.dy_des_, 0.0);

        Eigen::Vector3d current_position;

        current_position(0) = x_(4);
        current_position(1) = x_(5);
        current_position(2) = x_(6);

        // ff
        Eigen::Vector3d Pf = current_position + base_ori.toRotationMatrix().matrix()*(pYawCorrected
                                                                                      + des_vel * data_->data_buffer_.swing_time_remaining[i]);

        //Eigen::Vector3d Pf = current_position + base_ori.toRotationMatrix().matrix()*(pRobotFrame);

        double p_rel_max = 0.5f;

        // fb
        // Using the estimated velocity is correct
        // todo: should be body frame ?
        double pfx_rel = v_(3) * 0.5 * dtMPC_*5 +
                         0.03*(v_(3)-v_des_world[0]); /* +
                        (0.5*q_(6)/9.81) * (v_(4)*data_->data_buffer_.dyaw_v_);*/

        double pfy_rel = v_(4) * 0.5 * dtMPC_*5 * dtMPC_ +
                         0.03*(v_(4)-v_des_world[1]); /*+
                        (0.5*q_(6)/9.81) * (-v_(3)*data_->data_buffer_.dyaw_v_);*/
        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        //std::cout<<pfx_rel<<" "<<pfy_rel<<" "<<Pf(0)<<" "<<Pf(1)<<std::endl;

        Pf(0) +=  pfx_rel;
        Pf(1) +=  pfy_rel;
        Pf(2) = 0.01;
        data_->data_buffer_.leg_data_[i].raibert_position_ = Pf;
        //std::cout<<Pf(1)<<std::endl;

    }

    bool is_need_mpc_update = false;

    if (data_->data_buffer_.mpc_no_update_counter >= 5) {
        is_need_mpc_update = true;
        data_->data_buffer_.mpc_pre_update_time = current_time_;
        data_->data_buffer_.mpc_no_update_counter = 0;
    }
    data_->data_buffer_.mpc_no_update_counter++;

    if (is_need_mpc_update) {
        update_plan_contact(current_time_);

        double pz_err = x_(6) - 0.3;

        Eigen::Vector3d current_position;
        Eigen::Vector3d point_position;

        //setup_problem(dtMPC_, 10, 0.8, 200);

        update_x_drag(static_cast<float>(data_->data_buffer_.x_comp_integral));

        if(abs(v_(3)) > 0.3){
            data_->data_buffer_.x_comp_integral += 3*pz_err*dtMPC_/v_(3);
        }
        data_->x_comp_integral_ = data_->data_buffer_.x_comp_integral;


        float Q[12] = {10, 10, 10, 0.5, 0.5, 50, 0.0, 0.0, 1, 1, 1, 1};
        float alpha = 1e-6;

        //float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
        //float alpha = 4e-5;

        float p[3];
        p[0] = x_(4);
        p[1] = x_(5);
        p[2] = x_(6);

        current_position(0) = x_(4);
        current_position(1) = x_(5);
        current_position(2) = x_(6);

        float v[3];
        v[0] = v_(3);
        v[1] = v_(4);
        v[2] = v_(5);

        float q[4];
        q[0] = x_(0);
        q[1] = x_(1);
        q[2] = x_(2);
        q[3] = x_(3);

        float w[3];
        w[0] = v_(0);
        w[1] = v_(1);
        w[2] = v_(2);

        // this get foot position
        float r[12];

        // foot order is LF RF LH RH
        ;
        r[0] = static_cast<float>(data_->data_buffer_.leg_data_[0].leg_end_p_(0) - current_position(0));
        r[4] = static_cast<float>(data_->data_buffer_.leg_data_[0].leg_end_p_(1)  - current_position(1));
        r[8] = static_cast<float>(data_->data_buffer_.leg_data_[0].leg_end_p_(2)  - current_position(2));

        r[1] = static_cast<float>(data_->data_buffer_.leg_data_[1].leg_end_p_(0) - current_position(0));
        r[5] = static_cast<float>(data_->data_buffer_.leg_data_[1].leg_end_p_(1) - current_position(1));
        r[9] = static_cast<float>(data_->data_buffer_.leg_data_[1].leg_end_p_(2) - current_position(2));

        r[2] = static_cast<float>(data_->data_buffer_.leg_data_[2].leg_end_p_(0) - current_position(0));
        r[6] = static_cast<float>(data_->data_buffer_.leg_data_[2].leg_end_p_(1) - current_position(1));
        r[10] = static_cast<float>(data_->data_buffer_.leg_data_[2].leg_end_p_(2) - current_position(2));

        r[3] = static_cast<float>(data_->data_buffer_.leg_data_[3].leg_end_p_(0) - current_position(0));
        r[7] = static_cast<float>(data_->data_buffer_.leg_data_[3].leg_end_p_(1) - current_position(1));
        r[11] = static_cast<float>(data_->data_buffer_.leg_data_[3].leg_end_p_(2) - current_position(2));

        if(print_debug_info_) {
            std::cout<<r[0]<<" "<<r[4]<<" "<<r[8]<<std::endl;
            std::cout<<r[1]<<" "<<r[5]<<" "<<r[9]<<std::endl;
            std::cout<<r[2]<<" "<<r[6]<<" "<<r[10]<<std::endl;
            std::cout<<r[3]<<" "<<r[7]<<" "<<r[11]<<std::endl;
        }
        /*for (int k = 0; k < 12; ++k) {
            std::cout<<r[k]<<std::endl;
        }*/

        float trajAll[120];


        float trajInitial[12] = {(float)roll_comp,  // 0
                                 (float)pitch_comp,    // 1
                                 0.0f,    // 2
                //yawStart,    // 2
                                 0,                                   // 3
                                 0,                                   // 4
                                 0.3f,      // 5
                                 0,                                        // 6
                                 0,                                        // 7
                                 0,  // 8
                                 (float)v_des_world[0],                           // 9
                                 (float)v_des_world[1],                           // 10
                                 0};

        for(int i = 0; i < 10; i++)
        {
            for(int j = 0; j < 12; j++)
                trajAll[12*i+j] = trajInitial[j];

            if(i == 0) // start at current position  TODO consider not doing this
            {
                trajAll[3] = (float)x_(4);
                trajAll[4] = (float)x_(5);
                trajAll[2] = base_o.yaw_angle();
            }
            else
            {
                trajAll[12*i + 3] = (float)x_(4);
                trajAll[12*i + 4] = (float)x_(5);
                trajAll[12*i + 5] = 0.3f;
                trajAll[12*i + 2] = 0.0f;
            }
        }

        update_problem_data_floats(p, v, q, w, r, base_o.yaw_angle(), Q, trajAll, alpha, data_->data_buffer_.contact_data);

        /*for (int i = 0; i < 12; ++i) {
            data_->data_buffer_.torque_first[i] = get_solution(i);
            //std::cout<<get_solution(i)<<std::endl;
        }*/
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d tmp_ff(get_solution(i * 3), get_solution(i * 3 + 1), get_solution(i * 3 + 2));
            wbc_data_->fr_des_.block<3, 1>(0, i) = tmp_ff.cast<float>();
        }
    }

    // process current status
    // process current status
    /*if(data_->data_buffer_.current_fsm_state == 0){
        data_->data_buffer_.swing_states(0) = 0;
        data_->data_buffer_.swing_states(1) = 1;
        data_->data_buffer_.swing_states(2) = 1;
        data_->data_buffer_.swing_states(3) = 0;
    }else{
        data_->data_buffer_.swing_states(0) = 1;
        data_->data_buffer_.swing_states(1) = 0;
        data_->data_buffer_.swing_states(2) = 0;
        data_->data_buffer_.swing_states(3) = 1;
    }*/
    data_->data_buffer_.swing_states = gait_->get_swing_states();
    // check for debug
    if(walking_counter_ <= 50) {
        data_->data_buffer_.swing_states(0) = 0;
        data_->data_buffer_.swing_states(1) = 0;
        data_->data_buffer_.swing_states(2) = 0;
        data_->data_buffer_.swing_states(3) = 0;
    }
    /*data_->data_buffer_.swing_states(0) = 0;
    data_->data_buffer_.swing_states(1) = 0;
    data_->data_buffer_.swing_states(2) = 0;
    data_->data_buffer_.swing_states(3) = 0;*/



    for (int foot = 0; foot < 4; foot++) {
        if ((data_->data_buffer_.swing_states(foot) > 0.5)) {
            if (data_->data_buffer_.first_swing[foot]) {
                data_->data_buffer_.first_swing[foot] = false;
                data_->data_buffer_.leg_data_[foot].foot_init_position_ = data_->data_buffer_.leg_data_[foot].leg_end_p_;
            }

            double time_nom = (current_time_ - data_->data_buffer_.current_domain_start_time) / (dtMPC_ * 5);
            data_->data_buffer_.foot_swing_trajectory_[foot].computeSwingTrajectoryBezier(time_nom, (dtMPC_ * 5));
            wbc_data_->foot_position_des_.block<3, 1>(0,
                                                      foot) = data_->data_buffer_.foot_swing_trajectory_[foot].getPosition().cast<float>();
            wbc_data_->foot_velocity_des_.block<3, 1>(0,
                                                      foot) = data_->data_buffer_.foot_swing_trajectory_[foot].getVelocity().cast<float>();
            wbc_data_->foot_acc_des_.block<3, 1>(0,
                                                 foot) = data_->data_buffer_.foot_swing_trajectory_[foot].getAcceleration().cast<float>();


        } else {
            data_->data_buffer_.first_swing[foot] = true;

        }
    }

    if ((not is_need_mpc_update) || wbc_data_->is_first_) {
        wbc_data_->is_first_ = false;
        // get base cmd
        wbc_data_->robot_des_info_.pBody_des[0] = (float) data_->data_buffer_.world_position_desired_[0];
        wbc_data_->robot_des_info_.pBody_des[1] = (float) data_->data_buffer_.world_position_desired_[1];
        wbc_data_->robot_des_info_.pBody_des[2] = (float) base_p_des_(2);


        wbc_data_->robot_des_info_.vBody_des[0] = (float) v_des_world[0];
        wbc_data_->robot_des_info_.vBody_des[1] = (float) v_des_world[1];
        wbc_data_->robot_des_info_.vBody_des[2] = 0.0f;

        wbc_data_->robot_des_info_.aBody_des.setZero();

        wbc_data_->robot_des_info_.pBody_RPY_des.setZero();
        wbc_data_->robot_des_info_.vBody_Ori_des.setZero();

        for (int i = 0; i < 4; ++i) {
            wbc_data_->robot_des_info_.Fr_des[i] = wbc_data_->fr_des_.block<3, 1>(0, i);
            wbc_data_->robot_des_info_.pFoot_des[i] = wbc_data_->foot_position_des_.block<3, 1>(0, i);
            wbc_data_->robot_des_info_.vFoot_des[i] = wbc_data_->foot_velocity_des_.block<3, 1>(0, i);
            wbc_data_->robot_des_info_.aFoot_des[i] = wbc_data_->foot_acc_des_.block<3, 1>(0, i);
            if (data_->data_buffer_.swing_states(i) > 0.5) {
                wbc_data_->robot_des_info_.contact_state[i] = 0.0;
            } else {
                wbc_data_->robot_des_info_.contact_state[i] = 1.0;
            }
            // for test
            data_->data_buffer_.leg_data_[i].des_position_ = wbc_data_->foot_position_des_.block<3, 1>(0,
                                                                                                       i).cast<double>();
        }

        wbc_data_->bodyPosition = x_.segment(4, 3).cast<float>();
        wbc_data_->bodyOrientation = x_.head(4).cast<float>();
        Eigen::Vector3f v_body = (base_ori.toRotationMatrix().matrix().inverse() * v_.segment(3, 3)).cast<float>();
        Eigen::Vector3f omega_body = (base_ori.toRotationMatrix().matrix().inverse() * v_.head(3)).cast<float>();
        wbc_data_->bodyVelocity.head(3) = omega_body;
        wbc_data_->bodyVelocity.segment(3, 3) = v_body;

        // update dynamic data;

        // Get M, f_cg, B matrices of the manipulator equation
        MatrixXd B = data_->plant_->MakeActuationMatrix();
        MatrixXd M(n_v_, n_v_);
        data_->plant_->CalcMassMatrix(*(data_->context_), &M);
        VectorXd bias(n_v_);
        data_->plant_->CalcBiasTerm(*(data_->context_), &bias);
        drake::multibody::MultibodyForces<double> f_app(*data_->plant_);
        data_->plant_->CalcForceElementsContribution(*(data_->context_), &f_app);
        VectorXd grav = data_->plant_->CalcGravityGeneralizedForces(*(data_->context_));
        bias = bias - grav;

        wbc_data_->A_ = M.cast<float>();
        wbc_data_->coriolis_ = bias.cast<float>();
        wbc_data_->grav_ = grav.cast<float>();

        Eigen::VectorXf tmp_q(18);
        tmp_q.setZero();
        for (int i = 0; i < 12; ++i) {
            tmp_q(6 + i) = (float) q_(7 + i);
        }
        wbc_data_->q_ = tmp_q;

        // recode leg act position
        for (int i = 0; i < 4; ++i) {
            wbc_data_->foot_position_.block<3, 1>(0, i) = data_->data_buffer_.leg_data_[i].leg_end_p_.cast<float>();
            wbc_data_->foot_velocity_.block<3, 1>(0,
                                                  i) = data_->data_buffer_.leg_data_[i].velocity_world_.cast<float>();
            wbc_data_->Jc_.block<3, 18>(i * 3, 0) = data_->data_buffer_.leg_data_[i].J_world_.cast<float>();
            wbc_data_->JcDotQdot_.block<3, 1>(0, i) = data_->data_buffer_.leg_data_[i].dotJ_v_.cast<float>();
        }


        wbc_ctrl_->run(wbc_data_);
        // printf data
        data_->wbic_q_des_ = wbc_ctrl_->_des_jpos.cast<double>();

        for (int i = 0; i < 12; ++i) {
            data_->wbic_torque_[i] = wbc_ctrl_->torque_[i];
            data_->wbic_qdes_[i] = wbc_ctrl_->q_des_[i];
            data_->wbic_dqdes_[i] = wbc_ctrl_->dotq_des_[i];
        }
    }

    data_->wbic_q_des_ = q_sol;
    data_->wbic_dq_des_ = dq_sol;

    if (is_stop_ == 0) {
        for (int i = 0; i < 12; ++i) {
            this->data_->kp_[i] = 8;
            this->data_->kd_[i] = 0.5;
            this->data_->q_des_[i] = data_->wbic_qdes_[i];
            this->data_->dq_des_[i] = data_->wbic_dqdes_[i];
            this->data_->tau_[i] = data_->wbic_torque_[i];
        }
    }

    gait_->run();

}


FSM_StateName FSM_State_WBIC::checkTransition() {
    this->next_state_name_ = this->state_name_;

    iter_++;
    if (walking_counter_ > walking_max_step_) {
        for (int i=0; i<12; i++) {
            data_->fsm_stop_q_des_[i] = data_->position_(i+7);
        }
        next_state_name_ = FSM_StateName::STOP;
    }

    // Get the next state
    return this->next_state_name_;
}

TransitionData FSM_State_WBIC::transition() {
    // Switch FSM control mode
    // Finish transition
    this->transition_data_.done = true;

    // Return the transition data to the FSM
    return this->transition_data_;
}

void FSM_State_WBIC::onExit() {
    // Nothing to clean up when exiting
}

void FSM_State_WBIC::update_plan_contact(double current_time) const {

    gait_->get_contact_table(data_->data_buffer_.contact_data);
    // for debug
    if(walking_counter_ <= 50) {
        for (int i = 0; i < 10; ++i) {
            data_->data_buffer_.contact_data[0+i*4] = 1;
            data_->data_buffer_.contact_data[1+i*4] = 1;
            data_->data_buffer_.contact_data[2+i*4] = 1;
            data_->data_buffer_.contact_data[3+i*4] = 1;
        }
    }
}
