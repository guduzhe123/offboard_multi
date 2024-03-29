//
// Created by zhouhua on 2020/5/3.
//

#include "test/1UAV_1USV/uav_ros_Manager.hpp"

uav_ros_Manager::uav_ros_Manager() :
        arm_command_(0),
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_takeoff_(false),
        is_land_(false)
{

}

void uav_ros_Manager::uavOnInit(ros::NodeHandle &nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &uav_ros_Manager::state_cb, this);
    vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,  &uav_ros_Manager::vrf_hud_cb, this);
    local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &uav_ros_Manager::local_pos_cb, this);
    mavlink_from_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10, &uav_ros_Manager::mavlink_from_sb, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &uav_ros_Manager::global_pos_cb, this);
    commander_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("mavros/debug_value/debug_vector", 10, &uav_ros_Manager::debug_value_cb, this);

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

    exec_timer_ = nh.createTimer(ros::Duration(0.01), &uav_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.01), &uav_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.01), &uav_ros_Manager::publishDronePosControl, this);
}

void uav_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    uav_.current_state = *msg;
}

void uav_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void uav_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    uav_.current_local_pos = *msg;
}

void uav_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    uav_.drone_id = current_mavlink.sysid;
    util_log("sys_id = %d", current_mavlink.sysid);
}

void uav_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    uav_.altitude = msg->altitude;
    uav_.longtitude = msg->longitude;
    uav_.latitude = msg->latitude;
}

void uav_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];
    arm_command_ = config;
}

void uav_ros_Manager::commander_update(const ros::TimerEvent& e) {
    if (arm_command_ == VF_UAV_START) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        util_log("uav arm_i = %d, is_arm = %d", arm_i_, is_arm_);
        if (!current_state.armed && !is_arm_) {
            while(arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    util_log("uav Vehicle armed");
                    is_arm_ = true;
                }
                --arm_i_;
            }
        }

        mavros_msgs::SetMode takeoff_set_mode;
        takeoff_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        if (current_state.mode != "AUTO.TAKEOFF" && uav_.current_local_pos.pose.position.z < 0.5f && !is_takeoff_) {
            static int takeoff_i ;
            for (takeoff_i = 10; ros::ok() && takeoff_i > 0; --takeoff_i) {
                if (current_state.mode != "AUTO.TAKEOFF") {
                    if (set_mode_client.call(takeoff_set_mode)  &&
                        takeoff_set_mode.response.mode_sent) {
                        util_log("Takeoff enabled");
                        is_takeoff_ = true;
                    }
                }
            }
        } else {
            util_log("Already in the air!");
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        util_log("is_offboard = %d", is_offboard_);
        if (current_state.mode != "OFFBOARD" && !is_offboard_) {
            static int offboard_i;
            for (offboard_i = 10; ros::ok() && offboard_i > 0; --offboard_i) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    util_log("uav Offboard enabled");
                    is_offboard_ = true;
                }
            }
        }
    }

    if (arm_command_ == VF_UAV_RETURN) {
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.Return";
        if (current_state.mode != "AUTO.Land" && !is_land_) {
            static int land_i;
            for (land_i = 10; ros::ok() && land_i > 0; --land_i) {
                if (set_mode_client.call(land_set_mode) &&
                        land_set_mode.response.mode_sent) {
                    util_log("uav Return enabled");
                    is_land_ = true;
                }
            }
        }
    }
}

void uav_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
    DataMan::getInstance()->SetDroneData(uav_);
}

void uav_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    local_pos_pub.publish(target_local_pos_sp_);
}

void uav_ros_Manager::uavPosSp(const geometry_msgs::PoseStamped& way_point) {
    target_local_pos_sp_ = way_point;
}


