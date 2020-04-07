//
// Created by zft on 20-4-7.
//

#ifndef OFFBOARD_INIT_HPP
#define OFFBOARD_INIT_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include "Cinc.hpp"

class mutiOffboard{
public:
    mutiOffboard();     //创建类的时候先执行构造函数；函数后面不用；
    ~mutiOffboard();
    void OnInit();         //初始化一些配置函数
//  static  mutiOffboard* getInstance();
    static  mutiOffboard* getInstance();
    TVehicleMsg uav1;
    ros::NodeHandle nh;

    /******包括很多接收函数的定义*******************/
    void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg);

    void re_uav1_state_cb(const mavros_msgs::State::ConstPtr& msg);

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    static mutiOffboard* l_pInst;
};

#endif //OFFBOARD_INIT_HPP
