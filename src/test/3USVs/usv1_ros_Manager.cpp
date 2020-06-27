//
// Created by cheng on 6/27/20.
//

#include "test/3USVs/usv1_ros_Manager.hpp"

usv1_ros_Manager::usv1_ros_Manager()  :
        arm_command_(0),
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_takeoff_(false) {

}

void usv1_ros_Manager::usv1OnInit(ros::NodeHandle &nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &usv1_ros_Manager::state_cb, this);
    vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,  &usv1_ros_Manager::vrf_hud_cb, this);
    local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &usv1_ros_Manager::local_pos_cb, this);
    mavlink_from_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10, &usv1_ros_Manager::mavlink_from_sb, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &usv1_ros_Manager::global_pos_cb, this);
    commander_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("mavros/debug_value/debug_vector", 10, &usv1_ros_Manager::debug_value_cb, this);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 1000);
    global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1000);
    g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &usv1_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.01), &usv1_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.01), &usv1_ros_Manager::publishDronePosControl, this);
}

void usv1_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    usv1_.current_state = *msg;
}

void usv1_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void usv1_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv1_.current_local_pos = *msg;
}

void usv1_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    usv1_.drone_id = current_mavlink.sysid;
    util_log("usv1 sys_id = %d", current_mavlink.sysid);
}

void usv1_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    usv1_.altitude = msg->altitude;
    usv1_.longtitude = msg->longitude;
    usv1_.latitude = msg->latitude;
}

void usv1_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("usv1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];
    arm_command_ = config;
}

void usv1_ros_Manager::commander_update(const ros::TimerEvent& e) {
    if (arm_command_ == VF_UAV_START) {
        util_log("usv1 begain to start!");
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (!current_state.armed && !is_arm_) {
            static int arm_i;
            while (arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    util_log("usv1 Vehicle armed");
                    is_arm_ = true;
                }
                arm_i_--;
            }
        }

        if (current_state.mode != "OFFBOARD" && !is_offboard_) {
            static int i;
            for (i = 10; ros::ok() && i > 0; --i) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    util_log("usv1 Offboard enabled");
                    is_offboard_ = true;
                }
            }
        }
    }
}

void usv1_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
    DataMan::getInstance()->SetDroneData(usv1_);
}

void usv1_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    local_pos_pub.publish(target_local_pos_sp_);
}

void usv1_ros_Manager::usv1PosSp(const geometry_msgs::PoseStamped& way_point) {
    target_local_pos_sp_ = way_point;
}