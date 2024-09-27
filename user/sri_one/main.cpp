#include "rt/PeriodicTask.h"

#include "yaml-cpp/yaml.h"

#include <csignal>
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "FSM_States/ControlFSM.h"
#include "FSM_States/ControlFSMData.h"
#include "utils/udp.h"
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
#define FOOT_KP 1.0
#define FOOT_KD 0.1

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
int sock_;

struct sockaddr_in mujoco_addr = {};

sri_in_t sri_in{};
sri_out_t sri_out{};

sri_out_t sri_out_copy{};

uint8_t sri_in_buffer[512];
uint8_t sri_out_buffer[512];

bool is_work = false;

int debug_print = 0;

// for state estimator
bool is_first_state_est = true;

int state_est_counter = 0;

void LoopImu()
{
    if(imu_reader->clearyaw)
    {
        is_work = true;

        double cy = cos(imu_reader->pub_buff[8] * Deg2Rad * 0.5);
        double sy = sin(imu_reader->pub_buff[8] * Deg2Rad * 0.5);
        double cp = cos(imu_reader->pub_buff[7] * Deg2Rad * 0.5);
        double sp = sin(imu_reader->pub_buff[7] * Deg2Rad * 0.5);
        double cr = cos(imu_reader->pub_buff[6] * Deg2Rad * 0.5);
        double sr = sin(imu_reader->pub_buff[6] * Deg2Rad * 0.5);

        sri_out_copy.robot_state_data.base_orient[0] = (float_t)(cy * cp * sr - sy * sp * cr);
        sri_out_copy.robot_state_data.base_orient[1] = (float_t)(sy * cp * sr + cy * sp * cr);
        sri_out_copy.robot_state_data.base_orient[2] = (float_t)(sy * cp * cr - cy * sp * sr);
        sri_out_copy.robot_state_data.base_orient[3] = (float_t)(cy * cp * cr + sy * sp * sr);

        sri_out_copy.robot_state_data.base_vel[0] = imu_reader->pub_buff[0];
        sri_out_copy.robot_state_data.base_vel[1] = imu_reader->pub_buff[1];
        sri_out_copy.robot_state_data.base_vel[2] = imu_reader->pub_buff[2];

        sri_out_copy.robot_state_data.base_rot_vel[0] = imu_reader->pub_buff[3];
        sri_out_copy.robot_state_data.base_rot_vel[1] = imu_reader->pub_buff[4];
        sri_out_copy.robot_state_data.base_rot_vel[2] = imu_reader->pub_buff[5];

        // printf("%f  %f  %f\r\n",sri_out.robot_state_data.base_vel[0],sri_out.robot_state_data.base_vel[1],sri_out.robot_state_data.base_vel[2]);
    }
}

void LoopJoint()
{
    /*
        joint 0: pos:0.183633  vel:0.001055    au:5.885335
        joint 1: pos:-0.592527  vel:0.001040    au:4.705595
        joint 2: pos:1.263909  vel:0.000533    au:10.421665
        joint 3: pos:-0.181326  vel:-0.000970    au:-6.116137
        joint 4: pos:-0.597975  vel:0.001346    au:5.250141
        joint 5: pos:1.257067  vel:0.000559    au:11.105810
        joint 6: pos:0.138217  vel:-0.001272    au:10.428764
        joint 7: pos:-0.597335  vel:-0.001082    au:5.188145
        joint 8: pos:1.183701  vel:-0.000386    au:18.443128
        joint 9: pos:-0.135913  vel:0.001189    au:-10.659119
        joint 10: pos:-0.599402  vel:-0.001255    au:5.394905
        joint 11: pos:1.178995  vel:-0.000313    au:18.913642

    */
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
            double _posDes_ = sri_out_copy.motor_info_data.pos[i];
            double _velDes_ = sri_out_copy.motor_info_data.vel[i];
            double _ff_ = sri_out_copy.motor_info_data.torque[i];


            if((i == 2) || (i == 5) || (i == 8) || (i == 11))
            {
                if((i == 2) || (i == 8))
                {
                    sri_out_copy.motor_info_data.pos[i] = -_posDes_;
                    sri_out_copy.motor_info_data.vel[i] = -_velDes_;
                    sri_out_copy.motor_info_data.torque[i] = (-_ff_ / FOOT_OTHERMEAN);
                }
                else
                    sri_out_copy.motor_info_data.torque[i] = (_ff_ / FOOT_OTHERMEAN);
            }
            else
            {
                if((i == 1) || (i == 6) || (i == 7) || (i == 9))
                {
                    sri_out_copy.motor_info_data.pos[i] = -_posDes_;
                    sri_out_copy.motor_info_data.vel[i] = -_velDes_;
                    sri_out_copy.motor_info_data.torque[i] = (-_ff_ / FOOT_MEAN);
                }
                else
                    sri_out_copy.motor_info_data.torque[i] = (_ff_ / FOOT_MEAN);
            }
        }

        //head_left_leg(leg_index:0,joint_index:0)---------------LH
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[6], sri_out_copy.motor_info_data.vel[6], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[6],1);
        
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
            control_fsm->data_->q_des_[6] = head_motor_status.position;
            control_fsm->data_->dq_des_[6] = head_motor_status.velocity;
            control_fsm->data_->tau_[6] = head_motor_status.torque * FOOT_MEAN;
        }

         //(leg_index:0,joint_index:1)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[7], sri_out_copy.motor_info_data.vel[7], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[7],2);
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
            control_fsm->data_->q_des_[7] = head_motor_status.position;
            control_fsm->data_->dq_des_[7] = head_motor_status.velocity;
            control_fsm->data_->tau_[7] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:0,joint_index:2)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[8], sri_out_copy.motor_info_data.vel[8], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[8],3);

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
            control_fsm->data_->q_des_[8] = head_motor_status.position;
            control_fsm->data_->dq_des_[8] = head_motor_status.velocity;
            control_fsm->data_->tau_[8] = head_motor_status.torque * FOOT_OTHERMEAN;
            
        }



        //head_right_leg(leg_index:1,joint_index:0)---------------RH
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[9], sri_out_copy.motor_info_data.vel[9], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[9],4);
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
            control_fsm->data_->q_des_[9] = head_motor_status.position;
            control_fsm->data_->dq_des_[9] = head_motor_status.velocity;
            control_fsm->data_->tau_[9] = head_motor_status.torque * FOOT_MEAN;
        }



        //(leg_index:1,joint_index:1)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[10], sri_out_copy.motor_info_data.vel[10], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[10],5);
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
            control_fsm->data_->q_des_[10] = head_motor_status.position;
            control_fsm->data_->dq_des_[10] = head_motor_status.velocity;
            control_fsm->data_->tau_[10] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:1,joint_index:2)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[11], sri_out_copy.motor_info_data.vel[11], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[11],6);
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
            control_fsm->data_->q_des_[11] = head_motor_status.position;
            control_fsm->data_->dq_des_[11] = head_motor_status.velocity;
            control_fsm->data_->tau_[11] = head_motor_status.torque * FOOT_OTHERMEAN;

        }


        // (leg_index:2,joint_index:0)---------------LF
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[0], sri_out_copy.motor_info_data.vel[0], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[0],7);
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
            control_fsm->data_->q_des_[0] = head_motor_status.position;
            control_fsm->data_->dq_des_[0] = head_motor_status.velocity;
            control_fsm->data_->tau_[0] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:2,joint_index:1)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[1], sri_out_copy.motor_info_data.vel[1], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[1],8);
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
            control_fsm->data_->q_des_[1] = head_motor_status.position;
            control_fsm->data_->dq_des_[1] = head_motor_status.velocity;
            control_fsm->data_->tau_[1] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:2,joint_index:2)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[2], sri_out_copy.motor_info_data.vel[2], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[2],9);
        
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
            control_fsm->data_->q_des_[2] = head_motor_status.position;
            control_fsm->data_->dq_des_[2] = head_motor_status.velocity;
            control_fsm->data_->tau_[2] = head_motor_status.torque * FOOT_OTHERMEAN;
        }


        //(leg_index:3,joint_index:0)---------------RF
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[3], sri_out_copy.motor_info_data.vel[3], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[3],10);
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
            control_fsm->data_->q_des_[3] = head_motor_status.position;
            control_fsm->data_->dq_des_[3] = head_motor_status.velocity;
            control_fsm->data_->tau_[3] = head_motor_status.torque * FOOT_MEAN;
        }
    


        //(leg_index:3,joint_index:1)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[4], sri_out_copy.motor_info_data.vel[4], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[4],11);
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
            control_fsm->data_->q_des_[4] = head_motor_status.position;
            control_fsm->data_->dq_des_[4] = head_motor_status.velocity;
            control_fsm->data_->tau_[4] = head_motor_status.torque * FOOT_MEAN;
        }


        //(leg_index:3,joint_index:2)
        motor_pack_msg(data_buffer, sri_out_copy.motor_info_data.pos[5], sri_out_copy.motor_info_data.vel[5], FOOT_KP, 
        FOOT_KD, sri_out_copy.motor_info_data.torque[5],12);

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
            control_fsm->data_->q_des_[5] = head_motor_status.position;
            control_fsm->data_->dq_des_[5] = head_motor_status.velocity;
            control_fsm->data_->tau_[5] = head_motor_status.torque * FOOT_OTHERMEAN;
        }

        //out joint
        for (int i=0; i != 12; ++i) {
            double _posDes_ = control_fsm->data_->q_des_[i];
            double _velDes_ = control_fsm->data_->dq_des_[i];
            double _ff_ = control_fsm->data_->tau_[i];


            if((i == 1) || (i == 2) || (i == 6) || (i == 7) || (i == 8) || (i == 9))
            {
                control_fsm->data_->q_des_[i] = -_posDes_;
                control_fsm->data_->dq_des_[i] = -_velDes_;
                control_fsm->data_->tau_[i] = -_ff_;
            }
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
        // is_work = true;
        msg_receive_.store(false, std::memory_order_release);
    }

    imu_reader->run();
    LoopImu();

    if(is_work) {
        if(debug_print < 1000000) {

            
            // LoopJoint();
            // copy data to fsm
            control_fsm->data_->position_[0] = sri_out_copy.robot_state_data.base_orient[0];
            control_fsm->data_->position_[1] = sri_out_copy.robot_state_data.base_orient[1];
            control_fsm->data_->position_[2] = sri_out_copy.robot_state_data.base_orient[2];
            control_fsm->data_->position_[3] = sri_out_copy.robot_state_data.base_orient[3];

            control_fsm->data_->position_[4] = sri_out_copy.robot_state_data.base_xyz[0];
            control_fsm->data_->position_[5] = sri_out_copy.robot_state_data.base_xyz[1];
            control_fsm->data_->position_[6] = sri_out_copy.robot_state_data.base_xyz[2];

            // printf("base_xyz: x:%f  y:%f    z:%f\r\n",sri_out_copy.robot_state_data.base_xyz[0],
            //     sri_out_copy.robot_state_data.base_xyz[1],sri_out_copy.robot_state_data.base_xyz[2]);

            control_fsm->data_->velocity_[0] = sri_out_copy.robot_state_data.base_rot_vel[0];
            control_fsm->data_->velocity_[1] = sri_out_copy.robot_state_data.base_rot_vel[1];
            control_fsm->data_->velocity_[2] = sri_out_copy.robot_state_data.base_rot_vel[2];

            control_fsm->data_->velocity_[3] = sri_out_copy.robot_state_data.base_vel[0];
            control_fsm->data_->velocity_[4] = sri_out_copy.robot_state_data.base_vel[1];
            control_fsm->data_->velocity_[5] = sri_out_copy.robot_state_data.base_vel[2];

            for (int i=0; i != 12; ++i) {
                control_fsm->data_->position_[i+7] = sri_out_copy.motor_info_data.pos[i];
                control_fsm->data_->velocity_[i+6] = sri_out_copy.motor_info_data.vel[i];

                // printf("joint %d: pos:%f  vel:%f    au:%f\r\n",i,sri_out_copy.motor_info_data.pos[i],
                // sri_out_copy.motor_info_data.vel[i],sri_out_copy.motor_info_data.torque[i]);
            }

            if(control_fsm->data_->is_state_estimator_init_) {
                if(is_first_state_est) {
                    control_fsm->data_->pos_vel_estimator_->setup();
                    control_fsm->data_->pos_vel_estimator_->_xhat[0] = sri_out_copy.robot_state_data.base_xyz[0];
                    control_fsm->data_->pos_vel_estimator_->_xhat[1] = sri_out_copy.robot_state_data.base_xyz[1];
                    control_fsm->data_->pos_vel_estimator_->_xhat[2] = sri_out_copy.robot_state_data.base_xyz[2];
                    control_fsm->data_->pos_vel_estimator_->_xhat[3] = sri_out_copy.robot_state_data.base_vel[0];
                    control_fsm->data_->pos_vel_estimator_->_xhat[4] = sri_out_copy.robot_state_data.base_vel[1];
                    control_fsm->data_->pos_vel_estimator_->_xhat[5] = sri_out_copy.robot_state_data.base_vel[2];
                    is_first_state_est = false;
                }else {
                    // update some data
                    Eigen::Quaterniond quat(sri_out_copy.robot_state_data.base_orient[0],
                                             sri_out_copy.robot_state_data.base_orient[1],
                                             sri_out_copy.robot_state_data.base_orient[2],
                                             sri_out_copy.robot_state_data.base_orient[3]);

                    Eigen::Matrix3d R_body2World = quat.toRotationMatrix();
                    Eigen::Matrix3d R_world2body = R_body2World.inverse();
                    Eigen::Vector3d acc_body(sri_out_copy.robot_state_data.base_acc[0],
                                             sri_out_copy.robot_state_data.base_acc[1],
                                             sri_out_copy.robot_state_data.base_acc[2]);
                    Eigen::Vector3d omega_body(sri_out_copy.robot_state_data.base_rot_vel[0],
                                             sri_out_copy.robot_state_data.base_rot_vel[1],
                                             sri_out_copy.robot_state_data.base_rot_vel[2]);


                    control_fsm->data_->state_estimator_data_->result.rBody = R_world2body;
                    control_fsm->data_->state_estimator_data_->result.aWorld = R_body2World * acc_body;
                    control_fsm->data_->state_estimator_data_->result.omegaBody = omega_body;

                    // leg data processing
                    // 1. LF, 2. RF, 3. LH, 4. RH
                    control_fsm->data_->state_estimator_data_->leg_data[0].hip_location = Eigen::Vector3d(0.191, 0.05475, 0.058);
                    control_fsm->data_->state_estimator_data_->leg_data[0].p = control_fsm->data_->sri_kin_->LF_.block(0, 3, 3, 1);
                    control_fsm->data_->state_estimator_data_->leg_data[0].v = control_fsm->data_->sri_kin_->lf_jac_ * Eigen::Vector3d(sri_out_copy.motor_info_data.vel[0],
                                                                                                                             sri_out_copy.motor_info_data.vel[1],
                                                                                                                             sri_out_copy.motor_info_data.vel[2]);
                    control_fsm->data_->state_estimator_data_->leg_data[1].hip_location = Eigen::Vector3d(0.191, -0.05475, 0.058);
                    control_fsm->data_->state_estimator_data_->leg_data[1].p = control_fsm->data_->sri_kin_->RF_.block(0, 3, 3, 1);
                    control_fsm->data_->state_estimator_data_->leg_data[1].v = control_fsm->data_->sri_kin_->rf_jac_ * Eigen::Vector3d(sri_out_copy.motor_info_data.vel[3],
                                                                                                                             sri_out_copy.motor_info_data.vel[4],
                                                                                                                             sri_out_copy.motor_info_data.vel[5]);

                    control_fsm->data_->state_estimator_data_->leg_data[2].hip_location = Eigen::Vector3d(-0.191, 0.05475, 0.058);
                    control_fsm->data_->state_estimator_data_->leg_data[2].p = control_fsm->data_->sri_kin_->LH_.block(0, 3, 3, 1);
                    control_fsm->data_->state_estimator_data_->leg_data[2].v = control_fsm->data_->sri_kin_->lh_jac_ * Eigen::Vector3d(sri_out_copy.motor_info_data.vel[6],
                                                                                                                             sri_out_copy.motor_info_data.vel[7],
                                                                                                                             sri_out_copy.motor_info_data.vel[8]);

                    control_fsm->data_->state_estimator_data_->leg_data[3].hip_location = Eigen::Vector3d(-0.191, -0.05475, 0.058);
                    control_fsm->data_->state_estimator_data_->leg_data[3].p = control_fsm->data_->sri_kin_->RH_.block(0, 3, 3, 1);
                    control_fsm->data_->state_estimator_data_->leg_data[3].v = control_fsm->data_->sri_kin_->rh_jac_ * Eigen::Vector3d(sri_out_copy.motor_info_data.vel[9],
                                                                                                                             sri_out_copy.motor_info_data.vel[10],
                                                                                                                             sri_out_copy.motor_info_data.vel[11]);

                    // contact data
                    // 1. LF, 2. RF, 3. LH, 4. RH
                    Eigen::Vector3d contact_force_lf = -(control_fsm->data_->sri_kin_->lf_jac_.inverse().transpose())*Eigen::Vector3d(sri_out_copy.motor_info_data.torque[0],
                                                                                                                             sri_out_copy.motor_info_data.torque[1],
                                                                                                                             sri_out_copy.motor_info_data.torque[2]);
                    Eigen::Vector3d contact_force_lf_in_world = R_body2World * contact_force_lf;
                    if(contact_force_lf_in_world[2] > 20) {
                        control_fsm->data_->state_estimator_data_->result.contactEstimate(0) = 0.5;
                    }

                    Eigen::Vector3d contact_force_rf = -(control_fsm->data_->sri_kin_->rf_jac_.inverse().transpose())*Eigen::Vector3d(sri_out_copy.motor_info_data.torque[3],
                                                                                                                             sri_out_copy.motor_info_data.torque[4],
                                                                                                                             sri_out_copy.motor_info_data.torque[5]);
                    Eigen::Vector3d contact_force_rf_in_world = R_body2World * contact_force_rf;
                    if(contact_force_rf_in_world[2] > 20) {
                        control_fsm->data_->state_estimator_data_->result.contactEstimate(1) = 0.5;
                    }

                    Eigen::Vector3d contact_force_lh = -(control_fsm->data_->sri_kin_->lh_jac_.inverse().transpose())*Eigen::Vector3d(sri_out_copy.motor_info_data.torque[6],
                                                                                                                             sri_out_copy.motor_info_data.torque[7],
                                                                                                                             sri_out_copy.motor_info_data.torque[8]);
                    Eigen::Vector3d contact_force_lh_in_world = R_body2World * contact_force_lh;
                    if(contact_force_lh_in_world[2] > 20) {
                        control_fsm->data_->state_estimator_data_->result.contactEstimate(2) = 0.5;
                    }

                    Eigen::Vector3d contact_force_rh = -(control_fsm->data_->sri_kin_->rh_jac_.inverse().transpose())*Eigen::Vector3d(sri_out_copy.motor_info_data.torque[9],
                                                                                                                             sri_out_copy.motor_info_data.torque[10],
                                                                                                                             sri_out_copy.motor_info_data.torque[11]);
                    Eigen::Vector3d contact_force_rh_in_world = R_body2World * contact_force_rh;
                    if(contact_force_rh_in_world[2] > 20) {
                        control_fsm->data_->state_estimator_data_->result.contactEstimate(3) = 0.5;
                    }

                    control_fsm->data_->pos_vel_estimator_->setData(*(control_fsm->data_->state_estimator_data_));
                    control_fsm->data_->pos_vel_estimator_->run();

                    // std::cout<<control_fsm->data_->position_[4]<<" "<<control_fsm->data_->position_[5]<<" "<<control_fsm->data_->position_[6]<<std::endl;

                }
            }

            control_fsm->runFSM();

            // send to mujoco

            for (int i=0; i != 12 ; ++i) {
                sri_in.motor_cmd_data.torque[i] = control_fsm->data_->tau_[i];
                sri_in.motor_cmd_data.pTarget[i] = control_fsm->data_->q_des_[i];
                sri_in.motor_cmd_data.dTarget[i] = control_fsm->data_->dq_des_[i];
                sri_in.motor_cmd_data.pGain[i] = control_fsm->data_->kp_[i];
                sri_in.motor_cmd_data.dGain[i] = control_fsm->data_->kd_[i];
            }
            pack_pd_in_t(&sri_in, sri_in_buffer);
            send_packet(sock_, sri_in_buffer, sizeof(sri_in_t),
                (struct sockaddr *)&mujoco_addr, sizeof(mujoco_addr));
        }
        ++debug_print;
        ++state_est_counter;

        // if(state_est_counter % 1000 == 501) {
        //     std::cout<<control_fsm->data_->position_[4]<<" "<<control_fsm->data_->position_[5]<<" "<<control_fsm->data_->position_[6]<<std::endl;
        //     std::cout<<control_fsm->data_->pos_vel_estimator_->_stateEstimatorData.result.position.transpose()<<std::endl;

        // }
    }
}


int main()
{
    signal(SIGINT, signal_callback_handler);


    YAML::Node config = YAML::LoadFile("ethercat_config.yaml");
    auto net_card = config["net_card"].as<std::string>();

    // setting udp server
    char const *iface_addr_str = "0.0.0.0";
    char const *iface_port_str = "25002";

    sock_ = udp_init_host(iface_addr_str, iface_port_str);
    if (-1 == sock_) exit(EXIT_FAILURE);

    mujoco_addr.sin_family = AF_INET;
    mujoco_addr.sin_port = htons(25000);
    mujoco_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    struct sockaddr_storage src_addr = {};
    socklen_t addrlen;

    std::shared_ptr<ControlFSMData> control_fsm_data = std::make_shared<ControlFSMData>();
    control_fsm = std::make_shared<ControlFSM>(control_fsm_data);
    
    imu_reader = std::make_shared<ImuReader>();

    PeriodicTaskManager task_manager;
    PeriodicFunction main_task(&task_manager, 0.002, "main", { run_main }, 99, 3);

    main_task.start();

    // first send empty command
    send_packet(sock_, (uint8_t *)&sri_in, sizeof(sri_in_t), (struct sockaddr *)&mujoco_addr, sizeof(mujoco_addr));

    while( !is_killed ) {

        ssize_t nbytes = get_newest_packet_ros2(sock_, &sri_out_buffer,
                                               (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == STATE_OUT_T_PACKED_LEN){

            unpack_state_out_t(sri_out_buffer, &sri_out);

            /*std::cout<<sri_out.robot_state_data.base_xyz[0]<<" "<<sri_out.robot_state_data.base_xyz[1]<<" "<<sri_out.robot_state_data.base_xyz[2]<<std::endl;
            std::cout<<sri_out.robot_state_data.base_orient[0]<<" "<<sri_out.robot_state_data.base_orient[1]<<" "<<sri_out.robot_state_data.base_orient[2]<<" "<<sri_out.robot_state_data.base_orient[3]<<std::endl;
            std::cout<<sri_out.robot_state_data.base_vel[0]<<" "<<sri_out.robot_state_data.base_vel[1]<<" "<<sri_out.robot_state_data.base_vel[2]<<std::endl;
            std::cout<<sri_out.robot_state_data.base_rot_vel[0]<<" "<<sri_out.robot_state_data.base_rot_vel[1]<<" "<<sri_out.robot_state_data.base_rot_vel[2]<<std::endl;*/
            msg_receive_.store(true, std::memory_order_release);
        }
    }

    return 0;
}
