//
// Created by zhouhua on 2020/5/8.
//

#include <ros/ros.h>
#include "util.h"
#include "test/3USVs/usvs_control.hpp"
#include "FlightManager.hpp"
#include "PathCreator.hpp"
#include "log/Chlog.hpp"

int main(int argc, char **argv)
{
    chlog::initChannel("motion_plan");
    chlog::setEnCout("motion_plan", true);

    chlog::initChannel("data");
    chlog::setEnCout("data", false);

    ros::init(argc, argv, "usvs");
//    util_daemonize ();
    ros::NodeHandle nh("~");
    bool is_motion_plan, is_usv, is_uav;
    nh.param("is_motion_plan", is_motion_plan, false);
    nh.param("is_usv", is_usv, false);
    nh.param("is_uav", is_uav, false);
    chlog::info("data", "~~~~ is motion plan = ", is_motion_plan, ", is_usv = ", is_usv, ", is_uav = ", is_uav);

    usvs_control* lead_node;
    lead_node = usvs_control::getInstance();
    lead_node->OnInit();
    DataMan::getInstance()->OnInit(lead_node, is_uav, is_usv);

    DataMan::getInstance()->OnInit(lead_node);

    ros::NodeHandle nh("~");
    bool is_motion_plan;
    nh.param("is_motion_plan", is_motion_plan, false);
    chlog::info("data", "~~~~ is motion plan = ", is_motion_plan);
//    is_motion_plan = true;
    PathCreator::geInstance()->onInit(lead_node, true);
    PathCreator::geInstance()->initMotionPlan();

    ros::Rate rate(50.0);
    while(ros::ok()){
        FlightManager::getInstance()->GetData();
        FlightManager::getInstance()->DoProgress();
        lead_node->getData();   //achieve data
        lead_node->doProgress();    //progress

        DataMan::getInstance()->PrintData();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}