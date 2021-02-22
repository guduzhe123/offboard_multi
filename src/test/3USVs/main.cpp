//
// Created by zhouhua on 2020/5/8.
//

#include <ros/ros.h>
#include "util.h"
#include "test/3USVs/usvs_control.hpp"
#include "test/3USVs/usv1_ros_Manager.hpp"
#include "test/3USVs/usv2_ros_Manager.hpp"
#include "test/3USVs/usv3_ros_Manager.hpp"
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

    usvs_control* lead_node;
    lead_node = usvs_control::getInstance();
    lead_node->OnInit();
    DataMan::getInstance()->OnInit(lead_node);

    ros::NodeHandle nh("~");
    bool is_motion_plan;
    nh.param("is_motion_plan", is_motion_plan, false);
    chlog::info("data", "~~~~ is motion plan = ", is_motion_plan);
    is_motion_plan = true;
    if (!is_motion_plan) {
        PathCreator::geInstance()->onInit(lead_node, true);
    } else {
        PathCreator::geInstance()->initMotionPlan();
    }

//    avoidance::getInstance()->Oninit();

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