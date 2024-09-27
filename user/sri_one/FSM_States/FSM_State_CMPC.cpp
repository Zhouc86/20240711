//
// Created by han on 5/18/23.
//

#include "FSM_State_CMPC.h"
#include "../cmpc/convexMPC_interface.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "stdio.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;


FSM_State_CMPC::FSM_State_CMPC(std::shared_ptr<ControlFSMData> control_data)
        : FSM_State(control_data, FSM_StateName::CMPC, "cmpc"){

    n_q_ = data_->plant_->num_positions();
    n_v_ = data_->plant_->num_velocities();
    n_u_ = data_->plant_->num_actuators();

    tick_time_ = 30;

    data_->data_buffer_.mpc_no_update_counter = tick_time_;

    kp_swing_ << 700, 0, 0,
            0, 700, 0,
            0, 0, 150;

    kd_swing_ << 14, 0, 0,
            0, 14, 0,
            0, 0, 14;

    dtMPC_ = dt_*tick_time_;

    for (int i = 0; i < 4; ++i) {
        data_->data_buffer_.swing_times[i] = dtMPC_*5;
        data_->data_buffer_.first_swing[i] = true;
        data_->data_buffer_.swing_time_remaining[i] = dtMPC_*5;
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

    setup_problem(dtMPC_, 10, 0.6, 300);
}

void FSM_State_CMPC::onEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.zero();

    this->data_->state_no_ = K_CMPC;

    // Reset counter
    iter_ = 0;
    current_time_ = 0;

    //this->data_->is_work_ = true;

    std::cout << "[CONTROL FSM] Enter into CMPC"<<std::endl;

    /*data_->is_contact_est_working_ = true;

    data_->estimate_force_rl_(2) = 100;
    data_->estimate_force_ll_(2) = 100;
    data_->estimate_force_rh_(2) = 100;
    data_->estimate_force_lh_(2) = 100;

    base_p_des_ = data_->est_position_;*/
    base_p_des_ = Eigen::Vector3d(0, 0, 0.3);

    // data_->is_state_estimator_init_ = true;

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_CMPC::run() {

    current_time_ = iter_*dt_;
    VectorXd u_sol(n_u_);
    ++iter_;

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



    // 1.5 fsm
    if ((current_time_ >= (data_->data_buffer_.current_domain_start_time + dtMPC_*5 - 0.001)) or data_->data_buffer_.is_first) {
        if (data_->data_buffer_.current_fsm_state == 1) {
            data_->data_buffer_.current_fsm_state = 0;
        }else{
            data_->data_buffer_.current_fsm_state = 1;
        }
        data_->data_buffer_.current_domain_start_time = current_time_;
        data_->data_buffer_.is_first = false;
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

        Eigen::Vector3d offset(0, side_sign[i] * .115, 0);
        Eigen::Vector3d pRobotFrame = (data_->data_buffer_.leg_data_[i].leg_base_in_local_p_ + offset);

        pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;

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
                         0.05*(v_(3)-v_des_world[0]); /* +
                        (0.5*q_(6)/9.81) * (v_(4)*data_->data_buffer_.dyaw_v_);*/

        double pfy_rel = v_(4) * 0.5 * dtMPC_*5 * dtMPC_ +
                         0.03*(v_(4)-v_des_world[1]); /*+
                        (0.5*q_(6)/9.81) * (-v_(3)*data_->data_buffer_.dyaw_v_);*/
        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        //std::cout<<pfx_rel<<" "<<pfy_rel<<" "<<Pf(0)<<" "<<Pf(1)<<std::endl;

        Pf(0) +=  pfx_rel;
        Pf(1) +=  pfy_rel;
        Pf(2) = -0.003;
        data_->data_buffer_.leg_data_[i].raibert_position_ = Pf;
        //std::cout<<Pf(1)<<std::endl;

    }

    //std::cout<<"!!!!!!!!"<<std::endl;
    bool is_need_mpc_update = false;

    if (data_->data_buffer_.mpc_no_update_counter >= tick_time_) {
        is_need_mpc_update = true;
        data_->data_buffer_.mpc_pre_update_time = current_time_;
        data_->data_buffer_.mpc_no_update_counter = 0;
    }
    data_->data_buffer_.mpc_no_update_counter++;

    if(is_need_mpc_update){
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


        float Q[12] = {1, 1, 1, 0.5, 0.5, 50, 0.0, 0.0, 0.0, 1, 1, 1};
        float alpha = 1e-6;

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
                                 (float)base_o.yaw_angle(),    // 2
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
                trajAll[5] = (float)x_(6);
                trajAll[2] = (float)base_o.yaw_angle();
            }
            else
            {
                trajAll[12*i + 3] = base_p_des_(0);
                trajAll[12*i + 4] = base_p_des_(1);
                trajAll[12*i + 5] = 0.3f;
                trajAll[12*i + 2] = 0;
            }
        }

        update_problem_data_floats(p, v, q, w, r, base_o.yaw_angle(), Q, trajAll, alpha, data_->data_buffer_.contact_data);

        for (int i = 0; i < 12; ++i) {
            data_->data_buffer_.torque_first[i] = get_solution(i);
            //std::cout<<get_solution(i)<<std::endl;
        }
    }

    if(print_debug_info_) {
        std::cout<<q_.transpose()<<std::endl;
        std::cout<<v_.transpose()<<std::endl;
        std::cout<< data_->data_buffer_.torque_first[0]<<" "<< data_->data_buffer_.torque_first[1]<<" "<< data_->data_buffer_.torque_first[2]<<std::endl;
        std::cout<< data_->data_buffer_.torque_first[3]<<" "<< data_->data_buffer_.torque_first[4]<<" "<< data_->data_buffer_.torque_first[5]<<std::endl;
        std::cout<< data_->data_buffer_.torque_first[6]<<" "<< data_->data_buffer_.torque_first[7]<<" "<< data_->data_buffer_.torque_first[8]<<std::endl;
        std::cout<< data_->data_buffer_.torque_first[9]<<" "<< data_->data_buffer_.torque_first[10]<<" "<< data_->data_buffer_.torque_first[11]<<std::endl;
        std::cout<<std::endl;
    }


    // process current status
    // if(data_->data_buffer_.current_fsm_state == 0){
    //     data_->data_buffer_.swing_states(0) = 0;
    //     data_->data_buffer_.swing_states(1) = 1;
    //     data_->data_buffer_.swing_states(2) = 1;
    //     data_->data_buffer_.swing_states(3) = 0;
    // }else{
    //     data_->data_buffer_.swing_states(0) = 1;
    //     data_->data_buffer_.swing_states(1) = 0;
    //     data_->data_buffer_.swing_states(2) = 0;
    //     data_->data_buffer_.swing_states(3) = 1;
    // }


    // std::vector<double> T_way_point = {data_->data_buffer_.current_domain_start_time,
    //                                    data_->data_buffer_.current_domain_start_time+dtMPC_*2.5,
    //                                    data_->data_buffer_.current_domain_start_time+dtMPC_*5};

    // for(int foot = 0; foot < 4; foot++){
    //     if(data_->data_buffer_.swing_states(foot)){
    //         if(data_->data_buffer_.first_swing[foot]){
    //             data_->data_buffer_.first_swing[foot] = false;
    //             data_->data_buffer_.leg_data_[foot].foot_init_position_ = data_->data_buffer_.leg_data_[foot].leg_end_p_ ;
    //         }

    //         std::vector<MatrixXd> toe(3, MatrixXd::Zero(3, 1));

    //         // x
    //         toe[0](0, 0) = data_->data_buffer_.leg_data_[foot].foot_init_position_(0);
    //         toe[1](0, 0) = data_->data_buffer_.leg_data_[foot].foot_init_position_(0) +
    //                        (data_->data_buffer_.leg_data_[foot].raibert_position_(0) -data_->data_buffer_.leg_data_[foot].foot_init_position_(0))/2;
    //         toe[2](0, 0) = data_->data_buffer_.leg_data_[foot].raibert_position_(0);
    //         // y
    //         toe[0](1, 0) = data_->data_buffer_.leg_data_[foot].foot_init_position_(1);
    //         toe[1](1, 0) = data_->data_buffer_.leg_data_[foot].foot_init_position_(1) +
    //                        (data_->data_buffer_.leg_data_[foot].raibert_position_(1) - data_->data_buffer_.leg_data_[foot].foot_init_position_(1))/2;
    //         toe[2](1, 0) = data_->data_buffer_.leg_data_[foot].raibert_position_(1);

    //         // z
    //         toe[0](2, 0) = data_->data_buffer_.leg_data_[foot].foot_init_position_(2);
    //         toe[1](2, 0) = data_->data_buffer_.leg_data_[foot].foot_init_position_(2) + 0.06;
    //         toe[2](2, 0) = 0.0;

    //         std::vector<MatrixXd> toe_dot(3, MatrixXd::Zero(3, 1));

    //         // x
    //         toe_dot[0](0, 0) = 0;
    //         toe_dot[1](0, 0) = (data_->data_buffer_.leg_data_[foot].raibert_position_(0) -
    //                             data_->data_buffer_.leg_data_[foot].foot_init_position_(0))/(dtMPC_*5);
    //         toe_dot[2](0, 0) = 0;
    //         // y
    //         toe_dot[0](1, 0) = 0;
    //         toe_dot[1](1, 0) = (data_->data_buffer_.leg_data_[foot].raibert_position_(1) -
    //                             data_->data_buffer_.leg_data_[foot].foot_init_position_(1))/(dtMPC_*5);
    //         toe_dot[2](1, 0) = 0;
    //         // z
    //         toe_dot[0](2, 0) = 0;
    //         toe_dot[1](2, 0) = 0;
    //         toe_dot[2](2, 0) = 0;

    //         drake::trajectories::PiecewisePolynomial<double> swing_foot_spline =
    //                 drake::trajectories::PiecewisePolynomial<double>::CubicHermite(T_way_point, toe, toe_dot);

    //         Vector3d current_velocity;
    //         current_velocity(0) = v_(3);
    //         current_velocity(1) = v_(4);
    //         current_velocity(2) = v_(5);
    //         data_->data_buffer_.leg_data_[foot].position_des_ = base_ori.toRotationMatrix().matrix().transpose()*(
    //                 swing_foot_spline.value(current_time_)-data_->data_buffer_.leg_data_[foot].leg_base_p_);
    //         data_->data_buffer_.leg_data_[foot].velocity_des_ = base_ori.toRotationMatrix().matrix().transpose()*(
    //                 swing_foot_spline.MakeDerivative(1)->value(current_time_) - current_velocity);
    //         //std::cout<<data_->data_buffer_.leg_data_[foot].raibert_position_(1)<<std::endl;
    //         //std::cout<<swing_foot_spline.value(current_time)(1)<<" "<<data_->data_buffer_.leg_data_[foot].leg_base_p_(1)<<" "<<data_->data_buffer_.leg_data_[foot].position_des_(1)<<std::endl;
    //         data_->data_buffer_.leg_data_[foot].kp_task_ = kp_swing_;
    //         data_->data_buffer_.leg_data_[foot].kd_task_ = kd_swing_;

    //         data_->data_buffer_.leg_data_[foot].kp_joint_ = zero_matrix_;
    //         data_->data_buffer_.leg_data_[foot].kd_joint_ = zero_matrix_;

    //         data_->data_buffer_.leg_data_[foot].force_ff_ = Vector3d::Zero();


    //     }else{
    //         data_->data_buffer_.first_swing[foot] = true;

    //         data_->data_buffer_.leg_data_[foot].kp_task_ = zero_matrix_;
    //         data_->data_buffer_.leg_data_[foot].kd_task_ = kd_swing_;

    //         data_->data_buffer_.leg_data_[foot].kp_joint_ = zero_matrix_;
    //         data_->data_buffer_.leg_data_[foot].kd_joint_ = 0.2*iden_matrix_;

    //         Eigen::Vector3d current_velocity(v_(3), v_(4), v_(5));

    //         data_->data_buffer_.leg_data_[foot].velocity_des_ = -current_velocity;

    //         // cal joint ff force
    //         Vector3d tmp_force(data_->data_buffer_.torque_first[foot*3], data_->data_buffer_.torque_first[foot*3+1], data_->data_buffer_.torque_first[foot*3+2]);
    //         data_->data_buffer_.leg_data_[foot].force_ff_ = base_ori.toRotationMatrix().matrix().inverse()*(-tmp_force);
    //     }
    // }

    // for(int foot = 0; foot < 4; foot++){
    //     Vector3d leg_torque(0, 0, 0);

    //     Vector3d task_force(0, 0, 0);

    //     // task space
    //     task_force += data_->data_buffer_.leg_data_[foot].kp_task_*(data_->data_buffer_.leg_data_[foot].position_des_-
    //                                                           data_->data_buffer_.leg_data_[foot].position_act_);


    //     task_force += data_->data_buffer_.leg_data_[foot].kd_task_*(data_->data_buffer_.leg_data_[foot].velocity_des_-
    //                                                           data_->data_buffer_.leg_data_[foot].velocity_act_);


    //     task_force += data_->data_buffer_.leg_data_[foot].force_ff_;

    //     //joint space

    //     leg_torque = data_->data_buffer_.leg_data_[foot].J_.transpose()*task_force;

    //     //leg_torque += data_->data_buffer_.leg_data_[foot].kd_joint_*(-data_->data_buffer_.leg_data_[foot].joint_velocity_);

    //     //std::cout<<leg_torque.transpose()<<std::endl;
    //     //std::cout<<leg_torque<<std::endl;
    //     u_sol(motor_no_.at(foot).at(0)) = leg_torque(0);
    //     u_sol(motor_no_.at(foot).at(1)) = leg_torque(1);
    //     u_sol(motor_no_.at(foot).at(2)) = leg_torque(2);
    // }

    // if(print_debug_info_) {
    //     std::cout<<u_sol.transpose()<<std::endl;
    //     std::cout<<"~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
    // }

    // if(is_stop_ == 0) {
    //     for (int i = 0; i < 12; ++i) {
    //         this->data_->kp_[i] = 0;
    //         this->data_->kd_[i] = 0.2;
    //         //this->data_->kd_[i] = 0.0;
    //         this->data_->q_des_[i] = 0;
    //         this->data_->dq_des_[i] = 0;
    //         this->data_->tau_[i] = u_sol(i);
    //     }
    // }
}


FSM_StateName FSM_State_CMPC::checkTransition() {
    this->next_state_name_ = this->state_name_;

    iter_++;

    // Get the next state
    return this->next_state_name_;
}

TransitionData FSM_State_CMPC::transition() {
    // Switch FSM control mode
    // Finish transition
    this->transition_data_.done = true;

    // Return the transition data to the FSM
    return this->transition_data_;
}

void FSM_State_CMPC::onExit() {
    // Nothing to clean up when exiting
}

void FSM_State_CMPC::update_plan_contact(double current_time) const
{
    if(data_->data_buffer_.current_fsm_state == 0){
        for (int i = 0; i < 10; ++i) {
            if((current_time+i*tick_time_*dt_) <= (data_->data_buffer_.current_domain_start_time+tick_time_*dt_*5)){
                data_->data_buffer_.contact_data[0+i*4] = 1;
                data_->data_buffer_.contact_data[1+i*4] = 0;
                data_->data_buffer_.contact_data[2+i*4] = 0;
                data_->data_buffer_.contact_data[3+i*4] = 1;
            }else{
                data_->data_buffer_.contact_data[0+i*4] = 0;
                data_->data_buffer_.contact_data[1+i*4] = 1;
                data_->data_buffer_.contact_data[2+i*4] = 1;
                data_->data_buffer_.contact_data[3+i*4] = 0;
            }
        }
    }else{
        for (int i = 0; i < 10; ++i) {
            if((current_time+i*tick_time_*dt_) <= (data_->data_buffer_.current_domain_start_time+tick_time_*dt_*5)){
                data_->data_buffer_.contact_data[0+i*4] = 0;
                data_->data_buffer_.contact_data[1+i*4] = 1;
                data_->data_buffer_.contact_data[2+i*4] = 1;
                data_->data_buffer_.contact_data[3+i*4] = 0;
            }else{
                data_->data_buffer_.contact_data[0+i*4] = 1;
                data_->data_buffer_.contact_data[1+i*4] = 0;
                data_->data_buffer_.contact_data[2+i*4] = 0;
                data_->data_buffer_.contact_data[3+i*4] = 1;
            }
        }
    }

    /*if(data_->data_buffer_.current_fsm_state == 0){
        for (int i = 0; i < 10; ++i) {
            if((current_time+i*tick_time_*dt_) <= (data_->data_buffer_.current_domain_start_time+tick_time_*dt_*5)){
                data_->data_buffer_.contact_data[0+i*4] = 1;
                data_->data_buffer_.contact_data[1+i*4] = 1;
                data_->data_buffer_.contact_data[2+i*4] = 1;
                data_->data_buffer_.contact_data[3+i*4] = 1;
            }else{
                data_->data_buffer_.contact_data[0+i*4] = 1;
                data_->data_buffer_.contact_data[1+i*4] = 1;
                data_->data_buffer_.contact_data[2+i*4] = 1;
                data_->data_buffer_.contact_data[3+i*4] = 1;
            }
        }
    }else{
        for (int i = 0; i < 10; ++i) {
            if((current_time+i*tick_time_*dt_) <= (data_->data_buffer_.current_domain_start_time+tick_time_*dt_*5)){
                data_->data_buffer_.contact_data[0+i*4] = 1;
                data_->data_buffer_.contact_data[1+i*4] = 1;
                data_->data_buffer_.contact_data[2+i*4] = 1;
                data_->data_buffer_.contact_data[3+i*4] = 1;
            }else{
                data_->data_buffer_.contact_data[0+i*4] = 1;
                data_->data_buffer_.contact_data[1+i*4] = 1;
                data_->data_buffer_.contact_data[2+i*4] = 1;
                data_->data_buffer_.contact_data[3+i*4] = 1;
            }
        }
    }*/

}