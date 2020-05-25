//
// Created by zhouhua on 2020/5/8.
//

#include <ros/ros.h>
#include "util.h"
#include "test/4UAVs//uavs_control.hpp"
#include "test/4UAVs/uav1_ros_Manager.hpp"
#include "test/4UAVs/uav2_ros_Manager.hpp"
#include "test/4UAVs/uav3_ros_Manager.hpp"
#include "test/4UAVs/uav4_ros_Manager.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uavs");
    util_daemonize ();

    uavs_control* lead_node;
    lead_node = uavs_control::getInstance();
    lead_node->onInit();
//    avoidance::getInstance()->Oninit();

    ros::Rate rate(10.0);
    while(ros::ok()){
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