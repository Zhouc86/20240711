#include "rt/PeriodicTask.h"

#include "yaml-cpp/yaml.h"

#include <csignal>
#include <iostream>


#include "FSM_States/ControlFSM.h"
#include "FSM_States/ControlFSMData.h"
#include "utils/udp.h"

#include "utils/SriLcmClient.h"
#include <poll.h>

#include "utils/InEkfStateEstimator.h"
#include <stdio.h>

#include "control/can_bus_protocol.h"
#include "control/rt_ethercat.h"
#include "control/imu_reader.h"

bool is_killed = false;

void signal_callback_handler(int signum) {
    printf("Caught kill signal\n");
    is_killed = true;
    exit(signum);
}

#define FOOT_MEAN 0.11 * 6
#define FOOT_OTHERMEAN 0.152 * 6

#define DEBUG 1

volatile double glo_kp = 0;
volatile double glo_kd = 0;

uint64_t hs = 0;
int disable = 0;
int is_enable = 0;
struct MotorStatus head_motor_status = {};
union SendByte8
{
    uint64_t udata;
    uint8_t buffer[8];
};

union headByte8
{
    uint64_t udata;
    uint8_t buffer[8];
};

SendByte8 byte_8;
headByte8 headbyte_8;

std::shared_ptr<ImuReader> imu_reader = nullptr;
std::shared_ptr<ControlFSM> control_fsm = nullptr;

// for udp
std::atomic<bool> msg_receive_{false};


sri_in_t sri_in{};
sri_out_t sri_out{};

sri_out_t sri_out_copy{};
sri_out_t To_State{};


bool is_work = false;

int debug_print = 0;

// for state estimator
bool is_first_state_est = true;

int state_est_counter = 0;

// for lcm
std::string lcm_url("udpm://239.255.76.67:7667?ttl=1");
std::string lcm_cmd_ch("robot_in");
std::string lcm_data_ch("robot_out");

SriLcmClient sri_lcm(lcm_url, lcm_cmd_ch, lcm_data_ch);

std::shared_ptr<struct pollfd> fds;

std::shared_ptr<InEkfStateEstimator> inekf_est;

int empty_counter = 0;

void LoopImu()
{
    if(imu_reader->clearyaw)
    {
        double cosroll,sinroll,cospitch,sinpitch,cosyaw,sinyaw;

        cosroll = std::cos(imu_reader->pub_buff[6] * Deg2Rad * 0.5);
        sinroll = std::sin(imu_reader->pub_buff[6] * Deg2Rad * 0.5);

        cospitch = std::cos(imu_reader->pub_buff[7] * Deg2Rad * 0.5);
        sinpitch = std::sin(imu_reader->pub_buff[7] * Deg2Rad * 0.5);

        cosyaw = std::cos(imu_reader->pub_buff[8] * Deg2Rad * 0.5);
        sinyaw = std::sin(imu_reader->pub_buff[8] * Deg2Rad * 0.5);

        To_State.robot_state_data.base_orient[0] = (float_t)(sinroll * cospitch * cosyaw - cosroll * sinpitch * sinyaw);
        To_State.robot_state_data.base_orient[1] = (float_t)(cosroll * sinpitch * cosyaw + sinroll * cospitch * sinyaw);
        To_State.robot_state_data.base_orient[2] = (float_t)(cosroll * cospitch * sinyaw - sinroll * sinpitch * cosyaw);
        To_State.robot_state_data.base_orient[3] = (float_t)(cosroll * cospitch * cosyaw + sinroll * sinpitch * sinyaw);

        To_State.robot_state_data.base_acc[0] = imu_reader->pub_buff[0];
        To_State.robot_state_data.base_acc[1] = imu_reader->pub_buff[1];
        To_State.robot_state_data.base_acc[2] = imu_reader->pub_buff[2];

        To_State.robot_state_data.base_rot_vel[0] = imu_reader->pub_buff[3];
        To_State.robot_state_data.base_rot_vel[1] = imu_reader->pub_buff[4];
        To_State.robot_state_data.base_rot_vel[2] = imu_reader->pub_buff[5];

        // printf("%f  %f  %f  %f\r\n",To_State.robot_state_data.base_orient[0],To_State.robot_state_data.base_orient[1],
        // To_State.robot_state_data.base_orient[2],To_State.robot_state_data.base_orient[3]);
    }
}

void real_run()
{
    if(is_work) {
        if(debug_print < 1000000) {
            if(control_fsm->data_->is_state_estimator_init_) {
                // for inekf
                if(is_first_state_est) {
                    inekf_est->init();
                    is_first_state_est = false;
                }else {
                    Eigen::Vector3d acc_body(sri_out_copy.robot_state_data.base_acc[0],
                                             sri_out_copy.robot_state_data.base_acc[1],
                                             sri_out_copy.robot_state_data.base_acc[2]);
                    Eigen::Vector3d omega_body(sri_out_copy.robot_state_data.base_rot_vel[0],
                                             sri_out_copy.robot_state_data.base_rot_vel[1],
                                             sri_out_copy.robot_state_data.base_rot_vel[2]);

                    control_fsm->data_->imu_acc_ = acc_body;
                    control_fsm->data_->imu_gyr_ = omega_body;

                    int is_contact_counter = 0;
                    if(sri_out_copy.robot_state_data.lf_force[0] > 0) {
                        control_fsm->data_->contact_est_[0] = 1;
                        ++is_contact_counter;
                    }else {
                        control_fsm->data_->contact_est_[0] = 0;
                    }
                    if(sri_out_copy.robot_state_data.rf_force[0] > 0) {
                        control_fsm->data_->contact_est_[1] = 1;
                        ++is_contact_counter;
                    }else {
                        control_fsm->data_->contact_est_[1] = 0;
                    }

                    if(sri_out_copy.robot_state_data.lh_force[0] > 0) {
                        control_fsm->data_->contact_est_[2] = 1;
                        ++is_contact_counter;
                    }else {
                        control_fsm->data_->contact_est_[2] = 0;
                    }
                    if(sri_out_copy.robot_state_data.rh_force[0] > 0) {
                        control_fsm->data_->contact_est_[3] = 1;
                        ++is_contact_counter;
                    }else {
                        control_fsm->data_->contact_est_[3] = 0;
                    }
                    if(is_contact_counter == 0) {
                        empty_counter++;
                    }
                    inekf_est->run();
                }
            }

            // copy data to fsm
            control_fsm->data_->position_[0] = sri_out_copy.robot_state_data.base_orient[0];
            control_fsm->data_->position_[1] = sri_out_copy.robot_state_data.base_orient[1];
            control_fsm->data_->position_[2] = sri_out_copy.robot_state_data.base_orient[2];
            control_fsm->data_->position_[3] = sri_out_copy.robot_state_data.base_orient[3];

            control_fsm->data_->position_[4] = control_fsm->data_->inekf_est_pos_(0);
            control_fsm->data_->position_[5] = control_fsm->data_->inekf_est_pos_(1);
            control_fsm->data_->position_[6] = sri_out_copy.robot_state_data.base_xyz[2];

            control_fsm->data_->velocity_[0] = sri_out_copy.robot_state_data.base_rot_vel[0];
            control_fsm->data_->velocity_[1] = sri_out_copy.robot_state_data.base_rot_vel[1];
            control_fsm->data_->velocity_[2] = sri_out_copy.robot_state_data.base_rot_vel[2];

            control_fsm->data_->velocity_[3] = control_fsm->data_->inekf_est_lin_vel_(0);
            control_fsm->data_->velocity_[4] = control_fsm->data_->inekf_est_lin_vel_(1);
            control_fsm->data_->velocity_[5] = control_fsm->data_->inekf_est_lin_vel_(2);

            for (int i=0; i != 12; ++i) {
                control_fsm->data_->position_[i+7] = sri_out_copy.motor_info_data.pos[i];
                control_fsm->data_->velocity_[i+6] = sri_out_copy.motor_info_data.vel[i];
            }


            control_fsm->runFSM();

            // data to lcm
            for (int i=0; i != 12; ++i) {
                sri_lcm.lcm_cmd_.torque[i] = control_fsm->data_->tau_[i];
                sri_lcm.lcm_cmd_.pTarget[i] = control_fsm->data_->q_des_[i];
                sri_lcm.lcm_cmd_.dTarget[i] = control_fsm->data_->dq_des_[i];
                sri_lcm.lcm_cmd_.pGain[i] = control_fsm->data_->kp_[i];
                sri_lcm.lcm_cmd_.dGain[i] = control_fsm->data_->kd_[i];
            }
            sri_lcm.Publish();
        }
            ++debug_print;
            ++state_est_counter;

            if(state_est_counter % 1000 == 501) {
                // std::cout<<sri_out_copy.robot_state_data.base_xyz[0]<<" "<<sri_out_copy.robot_state_data.base_xyz[1]<<" "<<sri_out_copy.robot_state_data.base_xyz[2]<<std::endl;
                // std::cout<<control_fsm->data_->inekf_est_pos_(0)<<" "<<control_fsm->data_->inekf_est_pos_(1)<<" "<<control_fsm->data_->inekf_est_pos_(2)<<std::endl;
                // std::cout<<sri_out_copy.robot_state_data.base_vel[0]<<" "<<sri_out_copy.robot_state_data.base_vel[1]<<" "<<sri_out_copy.robot_state_data.base_vel[2]<<std::endl;
                // std::cout<<control_fsm->data_->inekf_est_lin_vel_.transpose()<<std::endl;
                // std::cout<<control_fsm->data_->inekf_est_rot_.transpose()<<std::endl;
                // std::cout<<std::endl;
            }
    }
}

void LoopJoint()
{
    rt_ethercat_set_command();
    rt_ethercat_run();
    rt_ethercat_get_data();

    LegBoard* head_leg = dynamic_cast<LegBoard *>(slave_dict[0].get());
    LegBoard* last_leg = dynamic_cast<LegBoard *>(slave_dict[1].get());
    uint8_t data_buffer[8];

    disable = 1;

    if(is_enable == 0 && disable == 1 )
    {
        int i = 0;
        while( i < 2 ) {
            head_leg->leg_cmd_.control_word = 1; // enable
            last_leg->leg_cmd_.control_word = 1; // enable
            ++hs;
            head_leg->leg_cmd_.hs = hs;
            last_leg->leg_cmd_.hs = hs;
            i++;
            printf("DONE-send enable -----------------------------\r\n");
        }
            is_enable = 1 ;
    }

    if(is_enable == 1)
    {
        //in joint
        for (int i=0; i != 12; ++i) {

            if(control_fsm->data_->is_state_estimator_init_){
                    // if((i == 0) || (i == 3) || (i == 6) || (i == 9))
                    // sri_out.motor_info_data.pos[i] = 0;
            }
            else
            {
                sri_out_copy.motor_info_data.pos[i] = 0;
                sri_out_copy.motor_info_data.vel[i] = 0;
                sri_out_copy.motor_info_data.torque[i] = 0;
            }

            if(!control_fsm->data_->mode_change)
                glo_kp = 15.0;
            else
                glo_kp = 10.0;
            glo_kd = 0.1;
        }

        //head_left_leg(leg_index:0,joint_index:0)---------------LH
        motor_pack_msg(data_buffer, -sri_out_copy.motor_info_data.pos[6], -sri_out_copy.motor_info_data.vel[6], glo_kp, 
        glo_kd, -sri_out_copy.motor_info_data.torque[6],1);
        
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        head_leg->leg_cmd_.motor_1= byte_8.udata;

        ++hs;
        head_leg->leg_cmd_.hs = hs;

        if(head_leg->leg_data_.motor_1)
        {
            headbyte_8.udata = head_leg->leg_data_.motor_1;

            motor_unpack_msg(headbyte_8.buffer,1,head_motor_status);
            To_State.motor_info_data.pos[6] = -head_motor_status.position;
            To_State.motor_info_data.vel[6] = -head_motor_status.velocity;
            To_State.motor_info_data.torque[6] = -(head_motor_status.torque * FOOT_MEAN);
        }

         //(leg_index:0,joint_index:1)
        motor_pack_msg(data_buffer, -sri_out_copy.motor_info_data.pos[7], -sri_out_copy.motor_info_data.vel[7], glo_kp, 
        glo_kd, -sri_out_copy.motor_info_data.torque[7],2);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        head_leg->leg_cmd_.motor_2= byte_8.udata;
        ++hs;
        head_leg->leg_cmd_.hs = hs;


        if(head_leg->leg_data_.motor_2)
        {
            headbyte_8.udata = head_leg->leg_data_.motor_2;
            motor_unpack_msg(headbyte_8.buffer,2,head_motor_status);
            To_State.motor_info_data.pos[7] = -head_motor_status.position;
            To_State.motor_info_data.vel[7] = -head_motor_status.velocity;
            To_State.motor_info_data.torque[7] = -(head_motor_status.torque * FOOT_MEAN);
        }


        //(leg_index:0,joint_index:2)
        motor_pack_msg(data_buffer, -sri_out_copy.motor_info_data.pos[8], -sri_out_copy.motor_info_data.vel[8], glo_kp, 
        glo_kd, -sri_out_copy.motor_info_data.torque[8],3);

        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        head_leg->leg_cmd_.motor_3 = byte_8.udata; 
        ++hs;
        head_leg->leg_cmd_.hs = hs;

        if(head_leg->leg_data_.motor_3)
        {
            headbyte_8.udata = head_leg->leg_data_.motor_3;
            motor_unpack_msg(headbyte_8.buffer,3,head_motor_status);
            To_State.motor_info_data.pos[8] = -head_motor_status.position;
            To_State.motor_info_data.vel[8] = -head_motor_status.velocity;
            To_State.motor_info_data.torque[8] = -(head_motor_status.torque * FOOT_OTHERMEAN);
            
        }



        //head_right_leg(leg_index:1,joint_index:0)---------------RH
        motor_pack_msg(data_buffer, -sri_out_copy.motor_info_data.pos[9], -sri_out_copy.motor_info_data.vel[9], glo_kp, 
        glo_kd, -sri_out_copy.motor_info_data.torque[9],4);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        head_leg->leg_cmd_.motor_5 = byte_8.udata;
        ++hs;
        head_leg->leg_cmd_.hs = hs;

        if(head_leg->leg_data_.motor_5)
        {
            headbyte_8.udata = head_leg->leg_data_.motor_5;
            //headbyte_8.buffer[0] = ((headbyte_8.udata & 0xF) + 4);
            motor_unpack_msg(headbyte_8.buffer,5,head_motor_status);
            To_State.motor_info_data.pos[9] = -head_motor_status.position;
            To_State.motor_info_data.vel[9] = -head_motor_status.velocity;
            To_State.motor_info_data.torque[9] = -(head_motor_status.torque * FOOT_MEAN);
        }



        //(leg_index:1,joint_index:1)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[10], sri_out_copy.motor_info_data.vel[10], glo_kp, 
        glo_kd, sri_out_copy.motor_info_data.torque[10],5);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        head_leg->leg_cmd_.motor_6 = byte_8.udata;
        ++hs;
        head_leg->leg_cmd_.hs = hs;

        if(head_leg->leg_data_.motor_6)
        {
            headbyte_8.udata = head_leg->leg_data_.motor_6;
            //headbyte_8.buffer[0] = ((headbyte_8.udata & 0xF) + 4);
            motor_unpack_msg(headbyte_8.buffer,6,head_motor_status);
            To_State.motor_info_data.pos[10] = head_motor_status.position;
            To_State.motor_info_data.vel[10] = head_motor_status.velocity;
            To_State.motor_info_data.torque[10] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:1,joint_index:2)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[11], sri_out_copy.motor_info_data.vel[11], glo_kp, 
        glo_kd, sri_out_copy.motor_info_data.torque[11],6);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        head_leg->leg_cmd_.motor_7 = byte_8.udata;
        ++hs;
        head_leg->leg_cmd_.hs = hs;

        if(head_leg->leg_data_.motor_7)
        {
            headbyte_8.udata = head_leg->leg_data_.motor_7;
            //headbyte_8.buffer[0] = ((headbyte_8.udata & 0xF) + 4);
            motor_unpack_msg(headbyte_8.buffer,7,head_motor_status);
            To_State.motor_info_data.pos[11] = head_motor_status.position;
            To_State.motor_info_data.vel[11] = head_motor_status.velocity;
            To_State.motor_info_data.torque[11] = head_motor_status.torque * FOOT_OTHERMEAN;

        }


        // (leg_index:2,joint_index:0)---------------LF
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[0], sri_out_copy.motor_info_data.vel[0], glo_kp, 
        glo_kd, sri_out_copy.motor_info_data.torque[0],7);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        last_leg->leg_cmd_.motor_1 = byte_8.udata;
        ++hs;
        last_leg->leg_cmd_.hs = hs;

        if(last_leg->leg_data_.motor_1)
        {
            headbyte_8.udata = last_leg->leg_data_.motor_1;
            motor_unpack_msg(headbyte_8.buffer,1,head_motor_status);
            To_State.motor_info_data.pos[0] = head_motor_status.position;
            To_State.motor_info_data.vel[0] = head_motor_status.velocity;
            To_State.motor_info_data.torque[0] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:2,joint_index:1)
        motor_pack_msg(data_buffer, -sri_out_copy.motor_info_data.pos[1], -sri_out_copy.motor_info_data.vel[1], glo_kp, 
        glo_kd, -sri_out_copy.motor_info_data.torque[1],8);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        last_leg->leg_cmd_.motor_2 = byte_8.udata;
        ++hs;
        last_leg->leg_cmd_.hs = hs;

        if(last_leg->leg_data_.motor_2)
        {
            headbyte_8.udata = last_leg->leg_data_.motor_2;
            motor_unpack_msg(headbyte_8.buffer,2,head_motor_status);
            To_State.motor_info_data.pos[1] = -head_motor_status.position;
            To_State.motor_info_data.vel[1] = -head_motor_status.velocity;
            To_State.motor_info_data.torque[1] = -(head_motor_status.torque * FOOT_MEAN);
        }


        //(leg_index:2,joint_index:2)
        motor_pack_msg(data_buffer, -sri_out_copy.motor_info_data.pos[2], -sri_out_copy.motor_info_data.vel[2], glo_kp, 
        glo_kd, -sri_out_copy.motor_info_data.torque[2],9);
        
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        last_leg->leg_cmd_.motor_3 = byte_8.udata;
        ++hs;
        last_leg->leg_cmd_.hs = hs;

        if(last_leg->leg_data_.motor_3)
        {
            headbyte_8.udata = last_leg->leg_data_.motor_3;
            motor_unpack_msg(headbyte_8.buffer,3,head_motor_status);
            To_State.motor_info_data.pos[2] = -head_motor_status.position;
            To_State.motor_info_data.vel[2] = -head_motor_status.velocity;
            To_State.motor_info_data.torque[2] = -(head_motor_status.torque * FOOT_OTHERMEAN);
        }


        //(leg_index:3,joint_index:0)---------------RF
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[3], sri_out_copy.motor_info_data.vel[3], glo_kp, 
        glo_kd, sri_out_copy.motor_info_data.torque[3],10);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        last_leg->leg_cmd_.motor_5 = byte_8.udata;
        ++hs;
        last_leg->leg_cmd_.hs = hs;

        if(last_leg->leg_data_.motor_5)
        {
            headbyte_8.udata = last_leg->leg_data_.motor_5;
            //headbyte_8.buffer[0] = ((headbyte_8.udata & 0xF) + 4);
            motor_unpack_msg(headbyte_8.buffer,5,head_motor_status);
            To_State.motor_info_data.pos[3] = head_motor_status.position;
            To_State.motor_info_data.vel[3] = head_motor_status.velocity;
            To_State.motor_info_data.torque[3] = head_motor_status.torque * FOOT_MEAN;
        }
    


        //(leg_index:3,joint_index:1)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[4], sri_out_copy.motor_info_data.vel[4], glo_kp, 
        glo_kd, sri_out_copy.motor_info_data.torque[4],11);
        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        last_leg->leg_cmd_.motor_6= byte_8.udata;
        ++hs;
        last_leg->leg_cmd_.hs = hs;

        if(last_leg->leg_data_.motor_6)
        {
            headbyte_8.udata = last_leg->leg_data_.motor_6;
            //headbyte_8.buffer[0] = ((headbyte_8.udata & 0xF) + 4);
            motor_unpack_msg(headbyte_8.buffer,6,head_motor_status);
            To_State.motor_info_data.pos[4] = head_motor_status.position;
            To_State.motor_info_data.vel[4] = head_motor_status.velocity;
            To_State.motor_info_data.torque[4] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:3,joint_index:2)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[5], sri_out_copy.motor_info_data.vel[5], glo_kp, 
        glo_kd, sri_out_copy.motor_info_data.torque[5],12);

        for (int i = 0; i < 8; ++i) {
            byte_8.buffer[i] = data_buffer[i];
        }
        last_leg->leg_cmd_.motor_7= byte_8.udata;
        ++hs;
        last_leg->leg_cmd_.hs = hs;

        if(last_leg->leg_data_.motor_7)
        {
            headbyte_8.udata = last_leg->leg_data_.motor_7;
            //headbyte_8.buffer[0] = ((headbyte_8.udata & 0xF) + 4);
            motor_unpack_msg(headbyte_8.buffer,7,head_motor_status);
            To_State.motor_info_data.pos[5] = head_motor_status.position;
            To_State.motor_info_data.vel[5] = head_motor_status.velocity;
            To_State.motor_info_data.torque[5] = head_motor_status.torque * FOOT_OTHERMEAN;
        }
    }
}

void run_main()
{
    // put data
    bool is_receive = msg_receive_.load(std::memory_order_acquire);
    if (is_receive) {
        // should be add mutux
        sri_out_copy = sri_out;
        is_work = true;
        msg_receive_.store(false, std::memory_order_release);
    }

    if(!DEBUG)
    {
        imu_reader->run();
        LoopImu();
        LoopJoint();
    }
    real_run();
}


int main()
{
    signal(SIGINT, signal_callback_handler);


    YAML::Node config = YAML::LoadFile("ethercat_config.yaml");
    auto net_card = config["net_card"].as<std::string>();

    // lcm setting
    fds = std::make_shared<struct pollfd>();
    fds->fd = sri_lcm.GetLcmFileno();
    fds->events = POLLIN;
    fds->revents = 0;

    std::shared_ptr<ControlFSMData> control_fsm_data = std::make_shared<ControlFSMData>();
    control_fsm = std::make_shared<ControlFSM>(control_fsm_data);
    int print_counter = 0;

    inekf_est = std::make_shared<InEkfStateEstimator>(control_fsm_data, 1);
    
    if(!DEBUG)
        imu_reader = std::make_shared<ImuReader>();

    PeriodicTaskManager task_manager;
    PeriodicFunction main_task(&task_manager, 0.001, "main", { run_main }, 99, 3);

    if(!DEBUG)
    {
        init_motor_config();
        rt_ethercat_config();
        rt_ethercat_init(net_card);
    }

    main_task.start();

    // std::this_thread::sleep_for(std::chrono::seconds(5));

    sri_lcm.Publish();
    while( !is_killed ) {

        int result = poll(  fds.get(), 1, -1);

        if (result < 0) {
            perror("poll failed\n");
        }
        if(fds->revents != 0){
            fds->revents = 0;
            sri_lcm.PollForCommandMessage();
            // lcm to sri_out
            for (int i=0; i != 3; ++i) {
                sri_out.robot_state_data.base_xyz[i] = sri_lcm.lcm_data_.base_xyz[i];
                sri_out.robot_state_data.base_vel[i] = sri_lcm.lcm_data_.base_vel[i];
                sri_out.robot_state_data.base_acc[i] = sri_lcm.lcm_data_.base_acc[i];
                sri_out.robot_state_data.base_rot_vel[i] = sri_lcm.lcm_data_.base_rot_vel[i];
                sri_out.robot_state_data.lf_force[i] = sri_lcm.lcm_data_.lf_force[i];
                sri_out.robot_state_data.rf_force[i] = sri_lcm.lcm_data_.rf_force[i];
                sri_out.robot_state_data.lh_force[i] = sri_lcm.lcm_data_.lh_force[i];
                sri_out.robot_state_data.rh_force[i] = sri_lcm.lcm_data_.rh_force[i];
            }

            for (int i=0; i != 4; ++i) {
                sri_out.robot_state_data.base_orient[i] = sri_lcm.lcm_data_.base_orient[i];
            }

            for (int i=0; i != 12; ++i) {
                sri_out.motor_info_data.pos[i] = sri_lcm.lcm_data_.pos[i];
                sri_out.motor_info_data.vel[i] = sri_lcm.lcm_data_.vel[i];
                sri_out.motor_info_data.torque[i] = sri_lcm.lcm_data_.torque[i];
            }

            msg_receive_.store(true, std::memory_order_release);
            ++print_counter;

            if(print_counter % 2000 == 500) {
                task_manager.printStatus();
            }
        }

    }

    return 0;
}
