//
// Created by zhouhua on 2020/5/8.
//

#include <ros/ros.h>
#include "util.h"
#include "test/2uav2/uav1_ros_Manager.hpp"
#include "test/2uav2/uav2_ros_Manager.hpp"
#include "test/2uav2/uav_lead_uav.hpp"
#include "test/2uav2//avoidance.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_lead_uav");
    util_daemonize ();

    usv_lead_uav* lead_node;
    lead_node = usv_lead_uav::getInstance();
    lead_node->onInit();
    avoidance::getInstance()->Oninit();

    ros::Rate rate(10.0);
    while(ros::ok()){
        avoidance::getInstance()->GetData();
        avoidance::getInstance()->DoProgress();
        lead_node->getData();
        lead_node->doProgress();

        DataMan::getInstance()->PrintData();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}