//
// Created by zhouhua on 2020/5/3.
//
#include <ros/ros.h>
#include "util.h"
#include "uav_ros_Manager.hpp"
#include "usv_ros_Manager.hpp"
#include "usv_lead_uav.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usv_lead_uav");
//    util_daemonize ();

    usv_lead_uav* lead_node;
    lead_node = usv_lead_uav::getInstance();
    lead_node->onInit();

    ros::Rate rate(50.0);
    while(ros::ok()){
        lead_node->getData();
        lead_node->doProgress();

        DataMan::getInstance()->PrintData();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}