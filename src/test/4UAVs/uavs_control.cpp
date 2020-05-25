//
// Created by zhouhua on 2020/5/24.
//

#include "test/4UAVs/uavs_control.hpp"

uavs_control* uavs_control::l_pInst = NULL;

uavs_control::uavs_control():
        uav_state_(TAKEOFF),
        uav_reached_(false),
        is_get_takeoff_pos_(false),
        is_avoidance_(false),
        command_(-1),
        danger_distance_(-1){

}

uavs_control* uavs_control::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new uavs_control();
    }
    return l_pInst;
}

void uavs_control::onInit() {
    ros::NodeHandle uav1_nh("uav1");
    uav1_control_.reset(new uav1_ros_Manager);
    uav1_control_->uavOnInit(uav1_nh);

    ros::NodeHandle uav2_nh("uav2");
    uav2_control_.reset(new uav2_ros_Manager);
    uav2_control_->usvOnInit(uav2_nh);

    ros::NodeHandle uav3_nh("uav3");
    uav3_control_.reset(new uav3_ros_Manager);
    uav3_control_->uavOnInit(uav3_nh);

    ros::NodeHandle uav4_nh("uav4");
    uav4_control_.reset(new uav4_ros_Manager);
    uav4_control_->uavOnInit(uav4_nh);

    ros::NodeHandle nh("~");
    int waypoint_num_;
    nh.param<double>("formation_distance", formation_distance_, 5);
    nh.param("waypoint_num", waypoint_num_, -1);
    nh.param("danger_distance", danger_distance_, 0.0);
    util_log("formation distance = %.2f, waypoint_num_ = %d", formation_distance_, waypoint_num_);
//    usvLocalPositionSp();
}

void uavs_control::getData() {

}

void uavs_control::doProgress() {

}

