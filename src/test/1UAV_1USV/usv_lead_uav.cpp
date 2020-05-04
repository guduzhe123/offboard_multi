//
// Created by zhouhua on 2020/5/3.
//
#include "test/1UAV_1USV/usv_lead_uav.hpp"

usv_lead_uav* usv_lead_uav::l_pInst = NULL;

usv_lead_uav::usv_lead_uav():
        uav_state_(TAKEOFF),
        uav_reached_(false){

}

usv_lead_uav* usv_lead_uav::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new usv_lead_uav();
    }
    return l_pInst;
}

void usv_lead_uav::onInit() {
    ros::NodeHandle uav_nh("uav1");
    uav_control_.reset(new uav_ros_Manager);
    uav_control_->uavOnInit(uav_nh);

    ros::NodeHandle usv_nh("usv1");
    usv_control_.reset(new usv_ros_Manager);
    usv_control_->usvOnInit(usv_nh);

    usvLocalPositionSp();
}

void usv_lead_uav::getData() {
    multiVehicle = DataMan::getInstance()->GetData();
}

void usv_lead_uav::doProgress() {
    usvlocalControl();
    uavlocalControl();
}

void usv_lead_uav::usvLocalPositionSp() {
    way_point.pose.position.x = 30;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 15;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 5;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
}

void usv_lead_uav::usvlocalControl() {
    current_usv_local_pos_ = multiVehicle.usv1.current_local_pos;
    if (!usv_way_points.empty()) {
        way_point = usv_way_points.back();
        if (pos_reached(current_usv_local_pos_, usv_way_points.back())) {
            ROS_INFO("Finished one way point = (%.2f, %.2f, %.2f)",
                     usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                     usv_way_points.back().pose.position.z);
            usv_way_points.pop_back();

            if (!usv_way_points.empty()) {
                ROS_INFO("Goto next way point = (%.2f, %.2f, %.2f)",
                         usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                         usv_way_points.back().pose.position.z);
            } else {
                ROS_INFO("Finish all target points!");
            }
        }

        if (uav_reached_)  usv_control_->usvPosSp(way_point);
    }
}

void usv_lead_uav::uavlocalControl() {
    current_uav_local_pos_ = multiVehicle.uav1.current_local_pos;
    switch (uav_state_) {
        case TAKEOFF: {
            uav_way_point.pose.position.x = 0;
            uav_way_point.pose.position.y = 0;
            uav_way_point.pose.position.z = 15;
            uav_control_->uavPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FOLLOW;
                uav_reached_ = true;
            }
        }
            break;

        case FOLLOW : {
            uav_reached_ = false;
            uav_way_point.pose.position = multiVehicle.usv1.current_local_pos.pose.position;
            uav_way_point.pose.position.z = multiVehicle.uav1.current_local_pos.pose.position.z;

            uav_control_->uavPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FOLLOW;
                uav_reached_ = true;
            }
        }
            break;

        default:
            break;
    }
}

bool usv_lead_uav::pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < 2.0f;
}

