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
    ros::NodeHandle usv1_nh("usv1");
    usv1_control_.reset(new usv1_ros_Manager);
    usv1_control_->usvOnInit(usv1_nh);

    ros::NodeHandle usv2_nh("usv2");
    usv2_control_.reset(new usv1_ros_Manager);
    usv2_control_->usvOnInit(usv2_nh);

    ros::NodeHandle usv3_nh("usv3");
    usv3_control_.reset(new usv3_ros_Manager);
    usv3_control_->usvOnInit(usv3_nh);

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

