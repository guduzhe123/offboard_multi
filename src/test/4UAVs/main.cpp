//
// Created by zhouhua on 2020/5/8.
//

#include <ros/ros.h>
#include "util.h"
#include "test/4UAVs//uavs_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uavs");
    util_daemonize ();

    auto lead_node = uavs_control::getInstance();
    DataMan::getInstance()->OnInit(lead_node);
    PathCreator::geInstance()->onInit(lead_node);

    lead_node->OnInit();

    ros::Rate rate(20.0);

    while(ros::ok()){
        FlightManager::getInstance()->GetData();
        FlightManager::getInstance()->DoProgress();
//        avoidance::getInstance()->GetData();
//        avoidance::getInstance()->DoProgress();
        lead_node->getData();
        lead_node->doProgress();


        DataMan::getInstance()->PrintData();
        ros::spinOnce();
        rate.sleep();

    }
    return 0;

}