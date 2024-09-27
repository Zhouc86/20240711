
#include "LegBoard.h"

#include <cstring>
#include "stdio.h"

LegBoard::LegBoard(const std::string& name, const uint32_t address):
EthercatSlaveBase(name, address){}

void LegBoard::updateRead()
{
    memcpy(&leg_data_, ec_slave[address_].inputs, sizeof(LegBoardData));
//    printf("%ld--%ld --%ld --%ld --%ld --%ld --%ld --%ld \r\n",leg_data_.hs,leg_data_.motor_1,leg_data_.motor_2,leg_data_.motor_3,
//            leg_data_.motor_4,leg_data_.motor_5,leg_data_.motor_6,leg_data_.motor_7);

}

void LegBoard::updateWrite()
{
    memcpy(ec_slave[address_].outputs, &leg_cmd_, sizeof(LegBoardCommand));

}