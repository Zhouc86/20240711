//
// Created by suj on 24-4-7.
//

#ifndef IMU_READER_H
#define IMU_READER_H
#include <iostream>
#include <vector>
#include <cstring>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <atomic>
#include <stdio.h>

#define Deg2Rad 0.017453292519943

class ImuReader
{
public:
    ImuReader();
    ~ImuReader();
    void run();
    std::vector<float> pub_buff; // 用于存储发送到的数据
    void CheckZero(int type);

    bool clearpos;
    std::atomic<bool> clearyaw;
    

private:
    bool checkSum(const std::vector<uint8_t>& list_data, uint16_t check_data);
    std::vector<int16_t> hex_to_int16(const std::vector<uint8_t>& raw_data);
    void printSensorData();
    void handleSerialData(uint8_t raw_data);
    void data_publish();

private:
    std::vector<uint8_t> buff; // 用于存储接收到的数据
    int key = 0; 
    std::vector<int16_t> angle_degree, magnetometer, acceleration, angularVelocity;
    bool pub_flag[2] = {true, true};
    std::string serial_port;
    uint32_t baud_rate;
    bool mag_enabled = false;
    bool open_flag = false;
    float acc_x,acc_y,acc_z,angV_x,angV_y,angV_z,angle_x,angle_y,angle_z;

    int fd;

};



#endif