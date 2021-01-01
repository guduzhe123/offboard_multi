//
// Created by zhouhua on 2020/5/3.
//

#include "test/3USVs/usv3_ros_Manager.hpp"

usv3_ros_Manager::usv3_ros_Manager() :
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_takeoff_(false),
        is_land_(false),
        home_pos_updated_(false)
{

}

void usv3_ros_Manager::usvOnInit(ros::NodeHandle &nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &usv3_ros_Manager::state_cb, this);
    vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,  &usv3_ros_Manager::vrf_hud_cb, this);
    local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &usv3_ros_Manager::local_pos_cb, this);
    mavlink_from_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10, &usv3_ros_Manager::mavlink_from_sb, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &usv3_ros_Manager::global_pos_cb, this);
    commander_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("mavros/debug_value/debug_vector", 10, &usv3_ros_Manager::debug_value_cb, this);
    way_point_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, &usv3_ros_Manager::wayPointCB, this);
    homePos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, &usv3_ros_Manager::homePositionCB, this);
    usv1_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/usv1/mavros/global_position/global", 10, &usv3_ros_Manager::usv1_local_pos_cb, this);

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
    marker_target_pub_ = nh.advertise<visualization_msgs::Marker>("target_point", 100);
    heading_vec_ = nh.advertise<visualization_msgs::Marker>("heading_vis", 100); // mark
    marker_cur_pos_ = nh.advertise<visualization_msgs::Marker>("cur_pos", 100); // mark

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    exec_timer_ = nh.createTimer(ros::Duration(0.01), &usv3_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.01), &usv3_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.01), &usv3_ros_Manager::publishDronePosControl, this);
}

void usv3_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    uav_.current_state = *msg;
}

void usv3_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void usv3_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
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
//    uav_.yaw = dronepos_.m_heading;
    uav_.yaw = current_vfr_hud.heading;
    util_log("usv3 heading = %.2f, uav_.yaw = %d", dronepos_.m_heading, uav_.yaw);
}


void usv3_ros_Manager::usv1_local_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    usv1_current_local_pos_ = *msg;
    if (uav_.longtitude > 0.08) {
        GlobalPosition loc_usv1, loc_usv2;
        loc_usv1.longitude = usv1_current_local_pos_.longitude;
        loc_usv1.latitude = usv1_current_local_pos_.latitude;
        loc_usv2.longitude = uav_.longtitude;
        loc_usv2.latitude = uav_.latitude;
        TVec3 follow_leader_offset;
        Calculate::getInstance()->GetLocalPos(loc_usv1, loc_usv2, follow_leader_offset);
        util_log("usv3 follow usv1 local offset = (%.2f, %.2f, %.2f)", follow_leader_offset.x(), follow_leader_offset.y(),
                 follow_leader_offset.z());

        geometry_msgs::Point p;
        p.x = uav_.current_local_pos.pose.position.x - follow_leader_offset.x();
        p.y = uav_.current_local_pos.pose.position.y - follow_leader_offset.y();
        p.z = uav_.current_local_pos.pose.position.z;

        TVec3 dir(cos((current_vfr_hud.heading + 90) * M_PI / 180), sin((current_vfr_hud.heading + 90) * M_PI / 180),
                  0.0);
        TVec3 pos = TVec3{p.x, p.y, p.z};
        DrawTrajCommand(pos, 2 * dir, usv3_color_);
        poublisMarker(p, usv3_color_, marker_cur_pos_);
    }
}

void
usv3_ros_Manager::poublisMarker(const geometry_msgs::Point &p, const TVec4 &color, const ros::Publisher &publisher) {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "world";
    target_marker.header.stamp =ros::Time::now();
    target_marker.ns = "points_and_lines";
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.pose.orientation.w = 1.0;
    target_marker.id = 1;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    publisher.publish(target_marker);

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    target_marker.scale.x = 1;
    target_marker.scale.y = 1;
    target_marker.scale.z = 1;
    // Line strip is blue
    target_marker.color.r = color(0);
    target_marker.color.g = color(1);
    target_marker.color.b = color(2);
    target_marker.color.a = color(3);
    target_marker.pose.position.x = p.x;
    target_marker.pose.position.y = p.y;
    target_marker.pose.position.z = p.z;
    target_marker.points.push_back(p);

    publisher.publish(target_marker);
}

void usv3_ros_Manager::DrawTrajCommand(const TVec3 &pos, const TVec3 &vec, const TVec4 &color) {
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "world";
    mk_state.header.stamp = ros::Time::now();
    mk_state.id = 1;
    mk_state.type = visualization_msgs::Marker::ARROW;
    mk_state.action = visualization_msgs::Marker::ADD;

    mk_state.pose.orientation.w = 1.0;
    mk_state.scale.x = 0.2;
    mk_state.scale.y = 0.4;
    mk_state.scale.z = 1;

    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state.points.push_back(pt);

    pt.x = pos(0) + 2 *vec(0);
    pt.y = pos(1) + 2 *vec(1);
    pt.z = pos(2) + 2 *vec(2);
    mk_state.points.push_back(pt);

    mk_state.color.r = color(0);
    mk_state.color.g = color(1);
    mk_state.color.b = color(2);
    mk_state.color.a = color(3);

    heading_vec_.publish(mk_state);
}

void usv3_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    uav_.drone_id = current_mavlink.sysid;
//    util_log("sys_id = %d", current_mavlink.sysid);
}

void usv3_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    uav_.altitude = msg->altitude;
    uav_.longtitude = msg->longitude;
    uav_.latitude = msg->latitude;
}

void usv3_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];
    DataMan::getInstance()->setCommand(config);
    PathCreator::geInstance()->CreateUSVFormationInit(config);
}

void usv3_ros_Manager::commander_update(const ros::TimerEvent& e) {
    int command;
    DataMan::getInstance()->getCommand(command);
    if (command == VF_USV_ALL_START /*|| command == SLAVESTART*/) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        util_log("usv3 arm_i = %d, is_arm = %d", arm_i_, is_arm_);
        if (!current_state.armed && !is_arm_) {
            while(arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    util_log("usv3 Vehicle armed");
                    is_arm_ = true;
                    break;
                }
                --arm_i_;
            }
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        util_log("is_offboard = %d", is_offboard_);
        if (current_state.mode != "OFFBOARD" && !is_offboard_) {
            static int offboard_i;
            for (offboard_i = 10; ros::ok() && offboard_i > 0; --offboard_i) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    util_log("usv3 Offboard enabled");
                    is_offboard_ = true;
                }
            }
        }
    }

    if (command == VF_USV_ALL_STOP) {
        target_local_pos_sp_ = uav_.current_local_pos;
    }

    if (command == VF_SET_HOME) {
        mavros_msgs::HomePosition homepos_manual;
        if (!home_pos_updated_) {
            homepos_manual.geo.latitude = uav_.latitude;
            homepos_manual.geo.longitude = uav_.longtitude;
            home_pos_pub.publish(homepos_manual);
            util_log("usv3 homepos_manual.geo.latitude = %.6f", homepos_manual.geo.latitude);
//            home_pos_updated_ = true;
        }
    }
}

void usv3_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
//    DataMan::getInstance()->SetDroneData(uav_);
    DataMan::getInstance()->SetDroneData(uav_);
}

void usv3_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    util_log("usv3 is_speed_ctrl_ = %d", is_speed_ctrl_);
    if (is_speed_ctrl_) {
        g_speed_control_pub.publish(vel_ctrl_sp_);
    } else {
        local_pos_pub.publish(target_local_pos_sp_);

        geometry_msgs::Point p;
        p.x = target_local_pos_sp_.pose.position.x - follow_leader_offset.x();
        p.y = target_local_pos_sp_.pose.position.y - follow_leader_offset.y();
        p.z = target_local_pos_sp_.pose.position.z;
        util_log("draw usv1 target pos = %.2f, %.2f, %.2f", p.x, p.y, p.z);
        poublisMarker(p, usv3_color_, marker_target_pub_);
    }
}

void usv3_ros_Manager::usvPosSp(const DroneControl& droneControl) {
    target_local_pos_sp_ = droneControl.target_pose;
    is_speed_ctrl_ = droneControl.speed_ctrl;
    target_heading_ = droneControl.target_heading;
    vel_ctrl_sp_ = droneControl.g_vel_sp;
    yaw_rate_ = droneControl.yaw_rate;
}

void usv3_ros_Manager::wayPointCB(const mavros_msgs::WaypointList::ConstPtr &msg) {
    uav_.waypointList = *msg;
}

void usv3_ros_Manager::homePositionCB(const mavros_msgs::HomePosition::ConstPtr& msg){
    uav_.homePosition = *msg;
    util_log("usv3 home position lat = %.8f, lon = %.8f", msg->geo.latitude, msg->geo.longitude);
}
void usv3_ros_Manager::usvCallService(mavros_msgs::CommandBool &m_mode) {
    util_log("usv3 call for arm mode = %d", m_mode);
//    arming_client.call(m_mode);
}
