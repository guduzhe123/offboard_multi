//
// Created by zhouhua on 2020/5/24.
//

#include "test/3USVs/usvs_control.hpp"

usvs_control* usvs_control::l_pInst = NULL;

usvs_control::usvs_control():
        usv_state_(TAKEOFF),
        usv_reached_(false),
        is_get_takeoff_pos_(false),
        is_avoidance_(false),
        command_(-1),
        danger_distance_(-1){

}

usvs_control* usvs_control::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new usvs_control();
    }
    return l_pInst;
}

void usvs_control::onInit() {
    ros::NodeHandle usv5_nh("usv5");
    usv5_control_.reset(new usv5_ros_Manager);
    usv5_control_->usvOnInit(usv5_nh);

    ros::NodeHandle usv6_nh("usv6");
    usv6_control_.reset(new usv6_ros_Manager);
    usv6_control_->usvOnInit(usv6_nh);

    ros::NodeHandle usv7_nh("usv7");
    usv7_control_.reset(new usv7_ros_Manager);
    usv7_control_->usvOnInit(usv7_nh);

    ros::NodeHandle nh("~");
    int waypoint_num_;
    nh.param<double>("formation_distance", formation_distance_, 5);
    nh.param("waypoint_num", waypoint_num_, -1);
    nh.param("danger_distance", danger_distance_, 0.0);
    util_log("formation distance = %.2f, waypoint_num_ = %d", formation_distance_, waypoint_num_);
//    usvLocalPositionSp();
}

void usvs_control::getData() {

}

void usvs_control::doProgress() {

}

