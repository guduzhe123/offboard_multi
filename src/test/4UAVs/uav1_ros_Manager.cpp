//
// Created by zhouhua on 2020/5/3.
//

#include "test/4UAVs/uav1_ros_Manager.hpp"

uav1_ros_Manager::uav1_ros_Manager() :
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_takeoff_(false),
        is_land_(false),
        is_speed_ctrl_(false)
{

}

void uav1_ros_Manager::uavOnInit(ros::NodeHandle &nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &uav1_ros_Manager::state_cb, this);
    vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,  &uav1_ros_Manager::vrf_hud_cb, this);
    local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &uav1_ros_Manager::local_pos_cb, this);
    mavlink_from_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10, &uav1_ros_Manager::mavlink_from_sb, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &uav1_ros_Manager::global_pos_cb, this);
    commander_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("mavros/debug_value/debug_vector", 10, &uav1_ros_Manager::debug_value_cb, this);
    way_point_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, &uav1_ros_Manager::wayPointCB, this);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 20);
    gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 100);
    global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 100);
    g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);
    dronePosPub = nh.advertise<offboard::DronePosUpdate>
            ("drone/PosUpDate", 100);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &uav1_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.01), &uav1_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.01), &uav1_ros_Manager::publishDronePosControl, this);
}

void uav1_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    uav_.current_state = *msg;
}

void uav1_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void uav1_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    uav_.current_local_pos = *msg;
    double yaw, roll, pitch;
    EulerAngles angles;

    yaw = Calculate::getInstance()->quaternion_get_yaw(uav_.current_local_pos.pose.orientation, angles);
    Calculate::getInstance()->quaternion_to_rpy(uav_.current_local_pos.pose.orientation, roll, pitch, yaw);

    dronepos_.m_heading = yaw * 180 / M_PI;
    if (dronepos_.m_heading  < 0) dronepos_.m_heading  += 360;

    dronepos_.m_x = uav_.current_local_pos.pose.position.x;
    dronepos_.m_y = uav_.current_local_pos.pose.position.y;
    dronepos_.m_z = uav_.current_local_pos.pose.position.z;
    dronepos_.m_roll = roll * 180 / M_PI;
    dronepos_.m_pitch = pitch * 180 / M_PI;
    dronePosPub.publish(dronepos_);
    uav_.yaw = dronepos_.m_heading;
    util_log("uav1 m_heading = %.2f", dronepos_.m_heading);
}

void uav1_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    uav_.drone_id = current_mavlink.sysid;
//    util_log("sys_id = %d", current_mavlink.sysid);
}

void uav1_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    uav_.altitude = msg->altitude;
    uav_.longtitude = msg->longitude;
    uav_.latitude = msg->latitude;
}

void uav1_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];

    DataMan::getInstance()->setCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void uav1_ros_Manager::commander_update(const ros::TimerEvent& e) {
    int command;
    DataMan::getInstance()->getCommand(command);
    if (command == VF_UAV_ALL_START /*|| command == SLAVESTART*/) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        util_log("uav arm_i = %d, is_arm = %d", arm_i_, is_arm_);
        if (!current_state.armed && !is_arm_) {
            while(arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    util_log("uav1 Vehicle armed");
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
                        util_log("uav1 Takeoff enabled");
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
                    util_log("uav1 Offboard enabled");
                    is_offboard_ = true;
                }
            }
        }
    }

    if (command == VF_UAV_ALL_STOP) {
        target_local_pos_sp_ = uav_.current_local_pos;
    }

    if (command == VF_UAV_ALL_LAND) {
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.Land";
        if (current_state.mode != "AUTO.Land" && !is_land_) {
            static int land_i;
            for (land_i = 10; ros::ok() && land_i > 0; --land_i) {
                if (set_mode_client.call(land_set_mode) &&
                        land_set_mode.response.mode_sent) {
                    util_log("uav1 Return enabled");
                    is_land_ = true;
                }
            }
        }
    }
}

void uav1_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
//    DataMan::getInstance()->SetDroneData(uav_);
    DataMan::getInstance()->SetDroneData(uav_);
}

void uav1_ros_Manager::uavPosSp(const DroneControl& droneControl) {
    target_local_pos_sp_ = droneControl.target_pose;
    is_speed_ctrl_ = droneControl.speed_ctrl;
    target_heading_ = droneControl.target_heading;
    vel_ctrl_sp_ = droneControl.g_vel_sp;
}

void uav1_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    if (is_speed_ctrl_) {
        drone_yaw_control();
        g_speed_control_pub.publish(vel_ctrl_sp_);
    } else {
//        target_local_pos_sp_.pose.orientation = uav_.current_local_pos.pose.orientation;
        /*TVec3 pos_sp, pos_cur, pos_offset;
        pos_sp = TVec3(target_local_pos_sp_.pose.position.x, target_local_pos_sp_.pose.position.y,
                target_local_pos_sp_.pose.position.z);
        pos_cur = TVec3(uav_.current_local_pos.pose.position.x, uav_.current_local_pos.pose.position.y,
                        uav_.current_local_pos.pose.position.z);
        Calculate::getInstance()->posToPosCtrl(pos_sp, pos_offset, pos_cur, m_speedLimit);

        target_local_pos_sp_.pose.orientation = uav_.current_local_pos.pose.orientation;
        target_local_pos_sp_.pose.position.x = pos_offset.x();
        target_local_pos_sp_.pose.position.y = pos_offset.y();
        target_local_pos_sp_.pose.position.z = pos_offset.z();*/

        target_local_pos_sp_.pose.orientation = uav_.current_local_pos.pose.orientation;
        local_pos_pub.publish(target_local_pos_sp_);
    }
}

void uav1_ros_Manager::drone_yaw_control() {
    float delta_heading = target_heading_ - dronepos_.m_heading;
    int g_yaw_rate_sign = delta_heading / fabs(delta_heading);
    float yaw_rate = g_yaw_rate_sign * 2;
    vel_ctrl_sp_.twist.angular.z = yaw_rate;
    util_log("yaw_rate = %.2f, target_heading_ = %.2f, drone_pos_.m_heading = %.2f", yaw_rate, target_heading_,
             dronepos_.m_heading);
}

void uav1_ros_Manager::uavCallService(mavros_msgs::SetMode &m_mode) {
    set_mode_client.call(m_mode);
}

void uav1_ros_Manager::wayPointCB(const mavros_msgs::WaypointList::ConstPtr &msg) {
    uav_.waypointList = *msg;
    util_log("uav1 mission waypoint size = %d", uav_.waypointList.waypoints.size());
}

