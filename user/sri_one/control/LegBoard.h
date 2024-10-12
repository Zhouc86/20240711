//
// Created by han on 22-7-1.
//

#ifndef BAER_ETHERCAT_LEGBOARD_H
#define BAER_ETHERCAT_LEGBOARD_H

#include "EthercatSlaveBase.h"
#include "ethercat.h"

struct LegBoardData{
    uint64_t    hs;
    uint64_t    motor_1;
    uint64_t    motor_2;
    uint64_t    motor_3;
    uint64_t    motor_4;
    uint64_t    motor_5;
    uint64_t    motor_6;
    uint64_t    motor_7;
    uint64_t    motor_8;
    uint64_t    motor_9;
    uint64_t    motor_10;
    uint64_t    test_word_byte_8;
    uint32_t    can1_error_log;
    uint32_t    can2_error_log;
    uint32_t    test_word_byte_4;
    uint16_t    fsm;
    uint16_t    rec_error_can1;
    uint16_t    rec_error_can2;
    uint16_t    motor_status;
};

struct LegBoardCommand{
    uint64_t    hs;
    uint64_t    motor_1;
    uint64_t    motor_2;
    uint64_t    motor_3;
    uint64_t    motor_4;
    uint64_t    motor_5;
    uint64_t    motor_6;
    uint64_t    motor_7;
    uint64_t    motor_8;
    uint64_t    motor_9;
    uint64_t    motor_10;
    uint32_t    test_word;
    uint16_t    control_word;
    uint16_t    motor_enable;
};

class LegBoard: public EthercatSlaveBase{
public:
    LegBoard(const std::string& name, const uint32_t address);

    LegBoardData leg_data_;
    LegBoardCommand leg_cmd_;

    void updateRead() override;
    void updateWrite() override;

};


#endif //BAER_ETHERCAT_LEGBOARD_H
