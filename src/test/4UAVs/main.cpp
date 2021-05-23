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
    ros::NodeHandle nh("~");
    bool follow_usv;
    nh.param("follow_usv", follow_usv, false);

    auto lead_node = uavs_control::getInstance();
    DataMan::getInstance()->OnInit(lead_node, false, false);

    PathCreator::geInstance()->onInit(lead_node, follow_usv);

    lead_node->OnInit(0);

    ros::Rate rate(50.0);

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