//
// Created by zft on 20-4-7.
//



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include "Cinc.hpp"
#include "init.hpp"


mutiOffboard* mutiOffboard::l_pInst = NULL;

mutiOffboard::mutiOffboard() {};

void mutiOffboard::uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
   uav1.current_state = *msg;
   uav1.re_local_pos_pub.publish(*msg);
}

void mutiOffboard::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    uav1.current_local_pos = *msg;
}


void mutiOffboard::OnInit() {
    uav1.state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &mutiOffboard::uav1_state_cb, this);
    uav1.re_local_pos_pub = nh.advertise<mavros_msgs::State>
            ("uav1/mavros/state", 5);

    uav1.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 5);
    uav1.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    uav1.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    uav1.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &mutiOffboard::local_pos_cb,this);

    /*
    ros::Subscriber vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10, vrf_hud_cb);

    ros::Publisher gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 1000);
    ros::Publisher global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1000);
    ros::Publisher g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 100);
            */
}

mutiOffboard* mutiOffboard::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new mutiOffboard();
    }
    return l_pInst;
}
