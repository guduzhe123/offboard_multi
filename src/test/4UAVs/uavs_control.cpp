//
// Created by zhouhua on 2020/5/24.
//

#include "test/4UAVs/uavs_control.hpp"

uavs_control::uavs_control():
        uav_state_(TAKEOFF),
        uav_reached_(false),
        is_get_takeoff_pos_(false),
        is_avoidance_(false),
        command_(-1),
        danger_distance_(-1){

}

uavs_control* uavs_control::getInstance() {
    static uavs_control* l_pInst = NULL;
    if (l_pInst == NULL) {
        l_pInst = new uavs_control();
    }
    return l_pInst;
}

void uavs_control::OnInit(const bool is_sim) {
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
    chlog::info("data","formation distance = %.2f, waypoint_num_ = %d", formation_distance_, waypoint_num_);

//    PathCreator::geInstance()->onInit(msgRos_);
//    usvLocalPositionSp();
}

void uavs_control::getData() {
    multiVehicle = DataMan::getInstance()->GetData();
}

void uavs_control::doProgress() {

}

void uavs_control::PublishDronePosControl(const multi_vehicle &multi_vehicles) {
    DroneControl uav1, uav2, uav3, uav4;
    uav1.target_pose = multi_vehicles.uav1.target_local_pos_sp;
    uav2.target_pose = multi_vehicles.uav2.target_local_pos_sp;
    uav3.target_pose = multi_vehicles.uav3.target_local_pos_sp;
    uav4.target_pose = multi_vehicles.uav4.target_local_pos_sp;

    // keep height different for safety
    uav2.target_pose.pose.position.z = multi_vehicles.uav2.target_local_pos_sp.pose.position.z + 2;
    uav3.target_pose.pose.position.z = multi_vehicles.uav3.target_local_pos_sp.pose.position.z + 4;
    uav4.target_pose.pose.position.z = multi_vehicles.uav4.target_local_pos_sp.pose.position.z + 6;

    if (multi_vehicles.uav1.droneControl.speed_ctrl) {
        chlog::info("data","ua1 send vel control = %.d", multi_vehicles.uav1.droneControl.speed_ctrl);
        uav1 = multi_vehicles.uav1.droneControl;
    }
    uav1_control_->uavPosSp(uav1);
    uav2_control_->uavPosSp(uav2);
    uav3_control_->uavPosSp(uav3);
    uav4_control_->uavPosSp(uav4);
}

void uavs_control::PublishBoatPosControl(const multi_vehicle &multi_vehicles) {

}

void uavs_control::SetUAVState(mavros_msgs::SetMode &m_mode) {
    uav1_control_->uavCallService(m_mode);
    uav2_control_->uavCallService(m_mode);
    uav3_control_->uavCallService(m_mode);
    uav4_control_->uavCallService(m_mode);
}

void uavs_control::SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) {

}

void uavs_control::PublishUUVPosControl(const multi_vehicle &multi_vehicles) {

}

void uavs_control::SetUSVAvoData(const bool usv1_usv2_crash, const bool usv1_usv3_crash, const bool usv2_usv3_crash) {

}


void uavs_control::PublishUSVPosControl(const multi_vehicle &multi_vehicles, int id) {

}
