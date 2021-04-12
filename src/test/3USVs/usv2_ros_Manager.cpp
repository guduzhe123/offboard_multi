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
        home_pos_updated_(false),
        usv_crash_(false)
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
    usv1_pos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("/usv1/mavros/home_position/home", 10, &usv2_ros_Manager::usv1_home_pos_cb, this);
    usv1_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv1/mavros/local_position/pose", 20, &usv2_ros_Manager::usv1_local_pos_cb, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, &usv2_ros_Manager::imuCB, this);

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
    exec_timer_ = nh.createTimer(ros::Duration(0.05), &usv2_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.05), &usv2_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.05), &usv2_ros_Manager::publishDronePosControl, this);

    pcl_manager_.reset(new PCLROSMessageManager);
    pcl_manager_->OnInit(nh);
}

void usv2_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    usv_.current_state = *msg;
}

void usv2_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void usv2_ros_Manager::usv1_home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg) {
    usv1_home_pos_ = *msg;
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
}

void usv2_ros_Manager::usv1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    usv1_current_local_pos_ = *msg;
}

void usv2_ros_Manager::imuCB(const sensor_msgs::Imu::ConstPtr& msg) {
    double r, p, y;
    Calculate::getInstance()->quaternion_to_rpy(msg->orientation, r, p, y);
    static int i = 0;
    if (i++ % 10 == 0) {
        usv_.yaw = Calculate::getInstance()->dgrIn180s(static_cast<float>(Calculate::getInstance()->rad2deg(y) /*- 90*/));//??? Todo
        chlog::info("data","[USV2]: IMU usv2 heading = %.2f, usv_.yaw = %.2f", dronepos_.m_heading, usv_.yaw);
    }
    usv_.q.w() = msg->orientation.w;
    usv_.q.x() = msg->orientation.x;
    usv_.q.y() = msg->orientation.y;
    usv_.q.z() = msg->orientation.z;

    pnt_.x = usv_.current_local_pos.pose.position.x - follow_leader_offset.x();
    pnt_.y = usv_.current_local_pos.pose.position.y - follow_leader_offset.y();
    pnt_.z = usv_.current_local_pos.pose.position.z;

    TVec3 dir(cos(usv_.yaw * M_PI / 180), sin(usv_.yaw * M_PI / 180), 0.0);
//    TVec3 pos = TVec3{pnt_.x, pnt_.y, pnt_.z};
    TVec3 pos ;
    pos << pnt_.x, pnt_.y, pnt_.z;
    DrawTrajCommand(pos, 2 * dir, usv2_color_);
    poublisMarker(pnt_, usv2_color_, marker_cur_pos_);
}

void
usv2_ros_Manager::poublisMarker(const geometry_msgs::Point &p, const TVec4 &color, const ros::Publisher &publisher) {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "map";
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

void usv2_ros_Manager::DrawTrajCommand(const TVec3 &pos, const TVec3 &vec, const TVec4 &color) {
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "map";
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

void usv2_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    usv_.drone_id = current_mavlink.sysid;
//    chlog::info("data","[USV2]: usv2 sys_id = %d", current_mavlink.sysid);
}

void usv2_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    usv_.altitude = msg->altitude;
    usv_.longtitude = msg->longitude;
    usv_.latitude = msg->latitude;
}

void usv2_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    chlog::info("data","[USV2]: usv2 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];
    DataMan::getInstance()->setCommand(config);
    PathCreator::geInstance()->CreateUSVFormationInit(config);
}

void usv2_ros_Manager::commander_update(const ros::TimerEvent& e) {
    if (! current_state.connected) return;
    int command;
    DataMan::getInstance()->getCommand(command);
    if (command == VF_USV_ALL_START /*|| command == MASTERSTART*/) {
        chlog::info("data","[USV2]: usv2 begain to start!");
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (!current_state.armed && !is_arm_) {
            static int arm_i;
            while (arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    chlog::info("data","[USV2]: usv2 Vehicle armed");
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
                    chlog::info("data","[USV2]: usv2 Offboard enabled");
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
            chlog::info("data","[USV2]: usv2 homepos_manual.geo.latitude = %.6f", homepos_manual.geo.latitude);
//            home_pos_updated_ = true;
        }
    }
}

void usv2_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
//    DataMan::getInstance()->SetDroneData(usv_);
    DataMan::getInstance()->SetDroneData(usv_);
}

void usv2_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    geometry_msgs::Point p;
    p.x = target_local_pos_sp_.pose.position.x - follow_leader_offset.x();
    p.y = target_local_pos_sp_.pose.position.y - follow_leader_offset.y();
    p.z = target_local_pos_sp_.pose.position.z;
    poublisMarker(p, usv2_color_, marker_target_pub_);

    if (is_speed_ctrl_) {
        g_speed_control_pub.publish(vel_ctrl_sp_);
    } else {
/*        TVec3 target_pos(p.x, p.y, p.z);
        TVec3 cur_pos(pnt_.x, pnt_.y,pnt_.z);
        TVec3 target_vec = cur_pos - target_pos;
        TVec3 heading_vec = Calculate::getInstance()->toVec(0,0,(usv_.yaw - 90)* M_PI / 180);*/

        /*TODO to calculate if the target is in the front of the current usv.
         * If true publish the target position to let the usv run,
         * else publish the current position to let the usv stop.*/
        TVec3 target_pos(p.x, p.y, 0);
        TVec3 cur_pos(pnt_.x, pnt_.y,0);
        TVec3 target_vec = cur_pos - target_pos;
        TVec3 usv1_cur_pos = TVec3(usv1_current_local_pos_.pose.position.x, usv1_current_local_pos_.pose.position.y,
                                   0);
        TVec3 usv2_cur_usv1_cur = (cur_pos - usv1_cur_pos).normalized();
        TVec3 usv2_cur_usv2_target = target_vec.normalized();
        float ang = acos(usv2_cur_usv1_cur.dot(usv2_cur_usv2_target));
/*
        chlog::info("data","[USV2]: usv1_cur_pos = (%.2f, %.2f, %.2f), usv2 cur_pos = (%.2f, %.2f, %.2f), usv2 target_pos = (%.2f, %.2f, %.2f)", usv1_cur_pos.x(),
                 usv1_cur_pos.y(), usv1_cur_pos.z(), cur_pos.x(), cur_pos.y(), cur_pos.z(), target_pos.x(), target_pos.y(), target_pos.z());*/
        if (ang * 180 / M_PI < 90 && !usv_crash_) {
            local_pos_pub.publish(target_local_pos_sp_);
        } else {
            local_pos_pub.publish(usv_.current_local_pos);
//            chlog::info("data","[USV2]: usv2 disable the target, ang = %.2f, usv2 crash = %d", ang * 180 / M_PI, usv_crash_);
        }

    }
}

void usv2_ros_Manager::usvPosSp(const DroneControl& droneControl) {
    target_local_pos_sp_ = droneControl.target_pose;
    is_speed_ctrl_ = droneControl.speed_ctrl;
    target_heading_ = droneControl.target_heading;
    vel_ctrl_sp_ = droneControl.g_vel_sp;
    yaw_rate_ = droneControl.yaw_rate;
    TVec3 cur_target_err;
    cur_target_err.x() = target_local_pos_sp_.pose.position.x - usv_.current_local_pos.pose.position.x;
    cur_target_err.y() = target_local_pos_sp_.pose.position.y - usv_.current_local_pos.pose.position.y;
    cur_target_err.z() = target_local_pos_sp_.pose.position.z - usv_.current_local_pos.pose.position.z;
    float len = cur_target_err.norm();
    chlog::info("data","[USV2]: usv2 target and current local pos error = ", len);
}

void usv2_ros_Manager::wayPointCB(const mavros_msgs::WaypointList::ConstPtr &msg) {
    usv_.waypointList = *msg;
}

void usv2_ros_Manager::homePositionCB(const mavros_msgs::HomePosition::ConstPtr& msg){
    usv_.homePosition = *msg;
    chlog::info("data","[USV2]: usv2 home position lat = %.8f, lon = %.8f", msg->geo.latitude, msg->geo.longitude);
    if (usv1_home_pos_.geo.longitude > 0.08) {
        GlobalPosition loc_usv1, loc_usv2;
        loc_usv1.longitude = usv1_home_pos_.geo.longitude;
        loc_usv1.latitude = usv1_home_pos_.geo.latitude;
        loc_usv2.longitude = usv_.homePosition.geo.longitude;
        loc_usv2.latitude = usv_.homePosition.geo.latitude;
        Calculate::getInstance()->GetLocalPos(loc_usv1, loc_usv2, follow_leader_offset);
        chlog::info("data","[USV2]: usv3 follow usv1 local offset = (%.2f, %.2f, %.2f)", follow_leader_offset.x(), follow_leader_offset.y(),
                 follow_leader_offset.z());
    }
}

void usv2_ros_Manager::usvCallService(mavros_msgs::CommandBool &m_mode) {
//    arming_client.call(m_mode);
}

void usv2_ros_Manager::usvCrash(bool usv2_crash) {
    usv_crash_ = usv2_crash;
    if (usv_crash_)
    chlog::info("data","[USV2]: usv2 crash!");
};