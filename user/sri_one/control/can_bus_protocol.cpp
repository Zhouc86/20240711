
#include "can_bus_protocol.h"

#include <memory>
#include <unordered_map>
#include <stdio.h>

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

std::unordered_map<int, MotorConfig> motor_config_dict;


uint16_t float_to_uint(double x, double x_min, double x_max, uint8_t bits)
{
    double span = x_max - x_min;
    double offset = x_min;

    return (uint16_t)((x - offset)*((double)((1 << bits) - 1)) / span);
}

double uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bit ///
    double span = x_max - x_min;
    double offset = x_min;
    return ((double)x_int)*span / ((double)((1 << bits) - 1)) + offset;
}

void motor_pack_msg(uint8_t* buffer_out, double position_des, double velocity_des,
                    double kp_des, double kd_des, double torque, int motor_no)
{
    uint16_t p, v, kp, kd, t;

    LIMIT_MIN_MAX(position_des, motor_config_dict[motor_no].p_min, motor_config_dict[motor_no].p_max);
    LIMIT_MIN_MAX(velocity_des, motor_config_dict[motor_no].v_min, motor_config_dict[motor_no].v_max);
    LIMIT_MIN_MAX(kp_des, motor_config_dict[motor_no].kp_min, motor_config_dict[motor_no].kp_max);
    LIMIT_MIN_MAX(kd_des, motor_config_dict[motor_no].kd_min, motor_config_dict[motor_no].kd_max);
    LIMIT_MIN_MAX(torque, motor_config_dict[motor_no].t_min, motor_config_dict[motor_no].t_max);

    p = float_to_uint(position_des, motor_config_dict[motor_no].p_min, motor_config_dict[motor_no].p_max, 16);
    v = float_to_uint(velocity_des, motor_config_dict[motor_no].v_min, motor_config_dict[motor_no].v_max, 12);
    kp = float_to_uint(kp_des, motor_config_dict[motor_no].kp_min, motor_config_dict[motor_no].kp_max, 12);
    kd = float_to_uint(kd_des, motor_config_dict[motor_no].kd_min, motor_config_dict[motor_no].kd_max, 12);
    t = float_to_uint(torque, motor_config_dict[motor_no].t_min, motor_config_dict[motor_no].t_max, 12);

    buffer_out[0] = p >> 8;
    buffer_out[1] = p & 0xFF;
    buffer_out[2] = v >> 4;
    buffer_out[3] = ((v & 0xF) << 4) | (kp >> 8);
    buffer_out[4] = kp & 0xFF;
    buffer_out[5] = kd >> 4;
    buffer_out[6] = ((kd & 0xF) << 4) | (t >> 8);
    buffer_out[7] = t & 0xff;
}

void speed_mode_pack_msg(uint8_t* buffer_out, double  speed_des,int motor_no) {


    buffer_out[0]= 0XFF;
    buffer_out[1]= 0XFF;
    buffer_out[2]= 0XFF;
    buffer_out[3]= 0XFF;
    buffer_out[4]= 0XFF;
    buffer_out[5]= 0XFF;


    if(speed_des<0){
        speed_des = -speed_des;
        buffer_out[6] = 0XE8;
        buffer_out[7] = uint8_t(speed_des);
    }else{
        buffer_out[6] = 0XE0;
        buffer_out[7] = uint8_t(speed_des);
    }
}

void motor_unpack_msg(uint8_t* buffer_in, int motor_no, MotorStatus& motor_status)
{
    int  p_int=0x00, v_int=0x00, t_int=0x00;
    uint8_t id = buffer_in[0] & 0xF;
    p_int = (buffer_in[1] << 8)| (0x00ff & buffer_in[2]);
    v_int = (buffer_in[3] << 4) | ((0x00ff & buffer_in[4]) >> 4);
    t_int = ((buffer_in[4] & 0xF) << 8) | (0x00ff&buffer_in[5]);
    p_int &= 0x0000ffff; //16bit
    v_int &= 0x00000fff; //12bit
    t_int &= 0x00000fff; //12bit

    motor_status.position = uint_to_float(p_int,motor_config_dict[motor_no].p_min,
                                 motor_config_dict[motor_no].p_max,16);
    motor_status.velocity = uint_to_float(v_int,motor_config_dict[motor_no].v_min,
                                 motor_config_dict[motor_no].v_max,12);
    motor_status.torque = uint_to_float(t_int,motor_config_dict[motor_no].t_min,
                               motor_config_dict[motor_no].t_max,12);

    // double torque_ = uint_to_float(t_int,motor_config_dict[motor_no].t_min,
    //                            motor_config_dict[motor_no].t_max,12);

    // motor_status.torque = torque_;
    motor_status.id = id;

    // printf("%d: position : %f,velocity : %f,torque :%f\r\n",motor_status.id,motor_status.position,
    //        motor_status.velocity,motor_status.torque);
}

void init_motor_config()
{
    MotorConfig a1{};
    a1.p_min = -12.5;
    a1.p_max = 12.5;
    a1.v_min = -65;
    a1.v_max = 65;
    a1.t_max = 60;
    a1.t_min = -60;
    a1.kp_min = 0.0;
    a1.kp_max = 500;
    a1.kd_max = 5;
    a1.kd_min = 0.0;

    motor_config_dict[1] = a1;
    motor_config_dict[2] = a1;
    motor_config_dict[3] = a1;
    motor_config_dict[4] = a1;
    motor_config_dict[5] = a1;
    motor_config_dict[6] = a1;

    motor_config_dict[7] = a1;
    motor_config_dict[8] = a1;
    motor_config_dict[9] = a1;
    motor_config_dict[10] = a1;
    motor_config_dict[11] = a1;
    motor_config_dict[12] = a1;
}
