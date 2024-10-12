//
// Created by han on 22-6-29.
//

#ifndef BAER_ETHERCAT_CAN_BUS_PROTOCOL_H
#define BAER_ETHERCAT_CAN_BUS_PROTOCOL_H

#include <cstdint>

struct MotorStatus
{
    int id;
    double position;
    double velocity;
    double torque;
    //uint16_t motor_status_word;
    uint8_t motor_error_code;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint8_t empty;
};

struct MotorConfig
{
    double p_min;
    double p_max;
    double v_min;
    double v_max;
    double t_min;
    double t_max;
    double kp_min;
    double kp_max;
    double kd_min;
    double kd_max;
};

void motor_pack_msg(uint8_t* buffer_out, double position_des, double velocity_des,
                    double kp, double kd, double torque, int motor_no);
void motor_unpack_msg(uint8_t* buffer_in, int motor_no, MotorStatus& motor_status);
void speed_mode_pack_msg(uint8_t* buffer_out, double speed_des,int motor_no);
void init_motor_config();


#endif //BAER_ETHERCAT_CAN_BUS_PROTOCOL_H
