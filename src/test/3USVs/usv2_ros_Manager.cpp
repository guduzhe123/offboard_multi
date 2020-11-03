//
// Created by zhouhua on 2020/5/3.
//

#include <test/3USVs/usv2_ros_Manager.hpp>

usv2_ros_Manager::usv2_ros_Manager()  :
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_takeoff_(false),
        is_land_(false),
        home_pos_updated_(false)
{

}

void usv2_ros_Manager::usvOnInit(ros::NodeHandle &nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &usv2_ros_Manager::state_cb, this);
    vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10, &usv2_ros_Manager::vrf_hud_cb, this);
    local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20, &usv2_ros_Manager::local_pos_cb, this);
    mavlink_from_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10, &usv2_ros_Manager::mavlink_from_sb, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &usv2_ros_Manager::global_pos_cb, this);
    commander_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("mavros/debug_value/debug_vector", 10, &usv2_ros_Manager::debug_value_cb, this);
    way_point_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, &usv2_ros_Manager::wayPointCB, this);
    homePos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, &usv2_ros_Manager::homePositionCB, this);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 100);
    global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 100);
    g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);
    dronePosPub = nh.advertise<offboard::DronePosUpdate>
            ("drone/PosUpDate", 100);
    home_pos_pub = nh.advertise<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 100);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &usv2_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.01), &usv2_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.01), &usv2_ros_Manager::publishDronePosControl, this);
}

void usv2_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    usv_.current_state = *msg;
}

void usv2_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void usv2_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv_.current_local_pos = *msg;

    double yaw, roll, pitch;
    EulerAngles angles;

    yaw = Calculate::getInstance()->quaternion_get_yaw(usv_.current_local_pos.pose.orientation, angles);
    Calculate::getInstance()->quaternion_to_rpy(usv_.current_local_pos.pose.orientation, roll, pitch, yaw);

    dronepos_.m_heading = yaw * 180 / M_PI;
    if (dronepos_.m_heading  < 0) dronepos_.m_heading  += 360;

    dronepos_.m_x = usv_.current_local_pos.pose.position.x;
    dronepos_.m_y = usv_.current_local_pos.pose.position.y;
    dronepos_.m_z = usv_.current_local_pos.pose.position.z;
    dronepos_.m_roll = roll * 180 / M_PI;
    dronepos_.m_pitch = pitch * 180 / M_PI;
    dronePosPub.publish(dronepos_);
//    usv_.yaw = dronepos_.m_heading;
    usv_.yaw = current_vfr_hud.heading;
    util_log("usv1 heading = %.2f, usv_.yaw = %.2f", dronepos_.m_heading, usv_.yaw);
}

void usv2_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    usv_.drone_id = current_mavlink.sysid;
    util_log("usv sys_id = %d", current_mavlink.sysid);
}

void usv2_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    usv_.altitude = msg->altitude;
    usv_.longtitude = msg->longitude;
    usv_.latitude = msg->latitude;
}

void usv2_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("usv1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];
    DataMan::getInstance()->setCommand(config);
    PathCreator::geInstance()->CreateUSVFormationInit(config);
}

void usv2_ros_Manager::commander_update(const ros::TimerEvent& e) {
    int command;
    DataMan::getInstance()->getCommand(command);
    if (command == VF_USV_ALL_START /*|| command == MASTERSTART*/) {
        util_log("usv2 begain to start!");
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (!current_state.armed && !is_arm_) {
            static int arm_i;
            while (arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    util_log("usv2 Vehicle armed");
                    is_arm_ = true;
                    break;
                }
                arm_i_--;
            }
        }

        if (current_state.mode != "OFFBOARD" && !is_offboard_) {
            static int i;
            for (i = 10; ros::ok() && i > 0; --i) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    util_log("usv2 Offboard enabled");
                    is_offboard_ = true;
                }
            }
        }
    }

    if (command == VF_USV_ALL_STOP) {
        target_local_pos_sp_ = usv_.current_local_pos;
    }

    if (command == VF_SET_HOME) {
        mavros_msgs::HomePosition homepos_manual;
        if (!home_pos_updated_) {
            homepos_manual.geo.latitude = usv_.latitude;
            homepos_manual.geo.longitude = usv_.longtitude;
            home_pos_pub.publish(homepos_manual);
            util_log("usv2 homepos_manual.geo.latitude = %.6f", homepos_manual.geo.latitude);
//            home_pos_updated_ = true;
        }
    }
}

void usv2_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
//    DataMan::getInstance()->SetDroneData(usv_);
    DataMan::getInstance()->SetDroneData(usv_);
}

void usv2_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    util_log("usv2 is_speed_ctrl_ = %d", is_speed_ctrl_);
    if (is_speed_ctrl_) {
        g_speed_control_pub.publish(vel_ctrl_sp_);
    } else {
        local_pos_pub.publish(target_local_pos_sp_);
    }
}

void usv2_ros_Manager::usvPosSp(const DroneControl& droneControl) {
    target_local_pos_sp_ = droneControl.target_pose;
    is_speed_ctrl_ = droneControl.speed_ctrl;
    target_heading_ = droneControl.target_heading;
    vel_ctrl_sp_ = droneControl.g_vel_sp;
    yaw_rate_ = droneControl.yaw_rate;
}

void usv2_ros_Manager::wayPointCB(const mavros_msgs::WaypointList::ConstPtr &msg) {
    usv_.waypointList = *msg;
}

void usv2_ros_Manager::homePositionCB(const mavros_msgs::HomePosition::ConstPtr& msg){
    usv_.homePosition = *msg;
    util_log("usv2 home position lat = %.8f, lon = %.8f", msg->geo.latitude, msg->geo.longitude);
}

void usv2_ros_Manager::usvCallService(mavros_msgs::CommandBool &m_mode) {
    util_log("usv2 call for arm mode = %d", m_mode);
//    arming_client.call(m_mode);
}
