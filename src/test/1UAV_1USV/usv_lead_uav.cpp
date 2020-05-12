//
// Created by zhouhua on 2020/5/3.
//
#include "test/1UAV_1USV/usv_lead_uav.hpp"

usv_lead_uav* usv_lead_uav::l_pInst = NULL;

usv_lead_uav::usv_lead_uav():
        uav_state_(TAKEOFF),
        uav_reached_(false),
        is_get_takeoff_pos_(false){

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
    GetTakeoffPos(multiVehicle.usv1, multiVehicle.uav1, follow_slave_first_local_);
    current_uav_local_pos_ = multiVehicle.uav1.current_local_pos;
    util_log("uav state = %d", uav_state_);
    switch (uav_state_) {
        case TAKEOFF: {
            uav_way_point.pose.position.x = 0;
            uav_way_point.pose.position.y = 0;
            uav_way_point.pose.position.z = 15;
            uav_control_->uavPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FORMATION;
                uav_reached_ = true;
            }
        }
            break;

        case FORMATION: {
            // uav at head of the usv
            uav_way_point.pose.position.x = follow_slave_first_local_.x();
            uav_way_point.pose.position.y = follow_slave_first_local_.y();
            uav_control_->uavPosSp(uav_way_point);

            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FOLLOW;
                uav_reached_ = true;
            }
        }
            break;

        case FOLLOW : {
            uav_reached_ = false;
            uav_way_point.pose.position.x = multiVehicle.usv1.current_local_pos.pose.position.x + follow_slave_first_local_.x();
            uav_way_point.pose.position.y = multiVehicle.usv1.current_local_pos.pose.position.y + follow_slave_first_local_.y();
            uav_way_point.pose.position.z = multiVehicle.uav1.current_local_pos.pose.position.z;

            uav_control_->uavPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FOLLOW;
                uav_reached_ = true;
            }
        }
            break;

        case RETURN: {
            uav_way_point.pose.position.x = 0;
            uav_way_point.pose.position.y = 0;
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

void usv_lead_uav::GetTakeoffPos(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_first_local) {
    if (slave.current_state.armed && !is_get_takeoff_pos_) {
        master_start_gps_ = GlobalPosition{master.latitude, master.longtitude, 0};
        slave_takeoff_gps_ = GlobalPosition{slave.latitude, slave.longtitude, 0};
        util_log("master_start_gps_ = ( %.9f, %.9f)", master_start_gps_.latitude, master_start_gps_.longitude);
        util_log("slave_takeoff_gps_ = ( %.9f, %.9f)", slave_takeoff_gps_.latitude, slave_takeoff_gps_.longitude);

        Calculate::getInstance()->GetLocalPos(master_start_gps_, slave_takeoff_gps_, follow_slave_first_local);
        util_log("follow_slave_first_local = ( %.2f, %.2f, %.2f)", follow_slave_first_local.x(), follow_slave_first_local.y(),
                           follow_slave_first_local.z());

        is_get_takeoff_pos_ = true;
    }
}

