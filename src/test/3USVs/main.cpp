//
// Created by zhouhua on 2020/5/8.
//

#include <ros/ros.h>
#include "util.h"
#include "test/3USVs/usvs_control.hpp"
#include "test/3USVs/usv5_ros_Manager.hpp"
#include "test/3USVs/usv6_ros_Manager.hpp"
#include "test/3USVs/usv7_ros_Manager.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usvs");
    util_daemonize ();

    usvs_control* lead_node;
    lead_node = usvs_control::getInstance();
    lead_node->onInit();
//    avoidance::getInstance()->Oninit();

    ros::Rate rate(10.0);
    while(ros::ok()){
//        avoidance::getInstance()->GetData();
//        avoidance::getInstance()->DoProgress();
        lead_node->getData();   //achieve data
        lead_node->doProgress();    //progress

        DataMan::getInstance()->PrintData();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}