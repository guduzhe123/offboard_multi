//
// Created by zhouhua on 2020/3/31.
//

#ifndef OFFBOARD_SERIALCONNECTION_HPP
#define OFFBOARD_SERIALCONNECTION_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>

using namespace std;

class MasterConnection {
public:
//    void main();
    MasterConnection();
    ~MasterConnection();

    void init();
    void initSerial(serial::Serial &slave1_port_USB, string port, double baund, serial::Timeout &time);
    bool openSerial(serial::Serial &serial, string port);
    bool readSerial(serial::Serial &serial_in, serial::Serial &serial_out,
                    bool serial_in_ready, bool serial_out_ready);

private:
    std::string slave1_port,slave2_port, slave3_port, telem_port;
    std::string master_port;
    double slave_baund, master_baund;

    //创建一个serial类
    serial::Serial slave1_serial_;
    serial::Serial slave2_serial_;
    serial::Serial slave3_serial_;
    serial::Serial telem_serial_;
    serial::Serial master_serial_;
    //创建timeout
    serial::Timeout serial_time = serial::Timeout::simpleTimeout(100000);

    bool master_ready = false;
    bool slave1_ready = false;
    bool slave2_ready = false;
    bool slave3_ready = false;
    bool telem_ready = false;

};

#endif //OFFBOARD_SERIALCONNECTION_HPP
