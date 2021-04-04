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

void usvs_control::OnInit() {
    ros::NodeHandle usv1_nh("usv1");
    usv1_control_.reset(new usv1_ros_Manager);
    usv1_control_->usvOnInit(usv1_nh);

    ros::NodeHandle usv2_nh("usv2");
    usv2_control_.reset(new usv2_ros_Manager);
    usv2_control_->usvOnInit(usv2_nh);

    ros::NodeHandle usv3_nh("usv3");
    usv3_control_.reset(new usv3_ros_Manager);
    usv3_control_->usvOnInit(usv3_nh);

    ros::NodeHandle nh("~");
    int waypoint_num_;
    nh.param<double>("formation_distance", formation_distance_, 5);
    nh.param("waypoint_num", waypoint_num_, -1);
    nh.param("danger_distance", danger_distance_, 0.0);
    chlog::info("data","formation distance = ", formation_distance_, ", waypoint_num_ = ", waypoint_num_);
//    usvLocalPositionSp();
}

void usvs_control::getData() {

}

void usvs_control::doProgress() {

}

void usvs_control::PublishDronePosControl(const multi_vehicle &multi_vehicles) {

}

void usvs_control::PublishBoatPosControl(const multi_vehicle &multi_vehicles) {
    DroneControl usv2, usv3;
    usv2.target_pose = multi_vehicles.usv2.target_local_pos_sp;
    usv3.target_pose = multi_vehicles.usv3.target_local_pos_sp;

/*    if (multi_vehicles.usv1.droneControl.speed_ctrl) {
        chlog::info("data","usv1 send vel control = %.d", multi_vehicles.usv1.droneControl.speed_ctrl);
        usv1 = multi_vehicles.usv1.droneControl;
    }*/

    TVec3 target_pos{multi_vehicles.usv1.target_local_pos_sp.pose.position.x,
                     multi_vehicles.usv1.target_local_pos_sp.pose.position.y,
                     multi_vehicles.usv1.target_local_pos_sp.pose.position.z};
    float pre_cur_err = (target_pos - usv1_target_pre_).norm();
    if (pre_cur_err > 0.8) {
        OnInitMotionPlan(multi_vehicles);
        usv1_target_pre_ = target_pos;
    }

//    usv1_control_->usvPosSp(usv1);
    usv2_control_->usvPosSp(usv2);
    usv3_control_->usvPosSp(usv3);

}

void usvs_control::SetUSVAvoData(const bool usv1_crash, const bool usv2_crash, const bool usv3_crash) {
    usv1_control_->usvCrash(usv1_crash);
    usv2_control_->usvCrash(usv2_crash);
    usv3_control_->usvCrash(usv3_crash);
    chlog::info("data","usv1_crash = %d, usv2_crash = %d, usv3_crash = %d", usv1_crash, usv2_crash, usv3_crash);
}

void usvs_control::OnInitMotionPlan(const multi_vehicle &multi_vehicles) {
    TVec3 target_pos{multi_vehicles.usv1.target_local_pos_sp.pose.position.x,
                     multi_vehicles.usv1.target_local_pos_sp.pose.position.y,
                     multi_vehicles.usv1.target_local_pos_sp.pose.position.z};
    MP_Config mp_config;
    mp_config.is_track_point = true;
    mp_config.is_speed_mode = false;
    mp_config.control_mode = POSITION_WITHOUT_CUR;
    mp_config.is_enable = true;
    mp_config.max_vel = 1.5;
    mp_config.max_acc = 2.0;
    mp_config.mp_map = multi_vehicles.usv1.Imap;
    mp_config.end_pos = target_pos;
    ActionMotionPlan::getInstance()->initMP(mp_config);
    ActionMotionPlan::getInstance()->setEnable(true);
}

void usvs_control::SetUAVState(mavros_msgs::SetMode &m_mode) {

}

void usvs_control::SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) {

}

void usvs_control::PublishUUVPosControl(const multi_vehicle &multi_vehicles) {

}

void usvs_control::PublishUSV1PosControl(const multi_vehicle &multi_vehicles)  {
    DroneControl usv1;
    usv1.target_pose = multi_vehicles.usv1.target_local_pos_sp;
    usv1_control_->usvPosSp(usv1);
}

