//
// Created by zhouhua on 2020/3/31.
//

#ifndef OFFBOARD_SERIALCONNECTION_HPP
#define OFFBOARD_SERIALCONNECTION_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "util.h"

using namespace std;

class SerialConnection {
public:
//    void main();
    SerialConnection();
    ~SerialConnection();
};

#endif //OFFBOARD_SERIALCONNECTION_HPP
