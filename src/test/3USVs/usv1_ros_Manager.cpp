//
// Created by zhouhua on 2020/5/3.
//

#include "test/3USVs/usv1_ros_Manager.hpp"

usv1_ros_Manager::usv1_ros_Manager() :
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_offboard_called_(false),
        is_takeoff_(false),
        is_land_(false),
        home_pos_updated_(false),
        usv_crash_(false),
        yaw_cur_(0)
{

}

void usv1_ros_Manager::usvOnInit(ros::NodeHandle &nh) {
    chlog::info("data","~~~ init usv1");
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
    way_point_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, &usv1_ros_Manager::wayPointCB, this);
    way_point_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, &usv1_ros_Manager::wayPointReachedCB, this);
    homePos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, &usv1_ros_Manager::homePositionCB, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, &usv1_ros_Manager::imuCB, this);
    rviz_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal", 10, &usv1_ros_Manager::rvizUsv1GoalCB, this);

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
    exec_timer_ = nh.createTimer(ros::Duration(0.05), &usv1_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.05), &usv1_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.05), &usv1_ros_Manager::publishDronePosControl, this);

    pcl_manager_.reset(new PCLROSMessageManager);
    pcl_manager_->OnInit(nh);

    usv_.Imap.reset(new OctoMap);
    usv_.Imap->onInit();
    usv_.Imap->setSafeRaduis(10);
}

void usv1_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    usv_.current_state = *msg;
}

void usv1_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
    chlog::info("data","current_vfr_hud heading = %d, header seq = %d", current_vfr_hud.heading, current_vfr_hud.header.seq);
}

void usv1_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv_.current_local_pos = *msg;
    chlog::info("data", "usv1 current_local_pos = (%.2f, %.2f, %.2f)", usv_.current_local_pos.pose.position.x,
             usv_.current_local_pos.pose.position.y,
             usv_.current_local_pos.pose.position.z);

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
    chlog::info("data","vir_hub usv1 heading = %.2f, usv_.yaw = %d", dronepos_.m_heading, usv_.yaw);
    dronepos_.m_heading = usv_.yaw;
    usv_.roll = dronepos_.m_roll;
    usv_.pitch = dronepos_.m_pitch;
}

void usv1_ros_Manager::imuCB(const sensor_msgs::Imu::ConstPtr& msg) {
    double r, p, y;
    Calculate::getInstance()->quaternion_to_rpy(msg->orientation, r, p, y);
//    static int i = 0;
//    if (i++ % 10 == 0) {
    usv_.yaw = Calculate::getInstance()->dgrIn180s(static_cast<float>(Calculate::getInstance()->rad2deg(y) /*- 90*/));//??? Todo
    yaw_cur_ = usv_.yaw;
    chlog::info("data","IMU usv1 heading = %.2f, usv_.yaw = %.2f", dronepos_.m_heading, usv_.yaw);
//    }

    geometry_msgs::Point pnt;
    pnt.x = usv_.current_local_pos.pose.position.x;
    pnt.y = usv_.current_local_pos.pose.position.y;
    pnt.z = usv_.current_local_pos.pose.position.z;

    TVec3 dir(cos(usv_.yaw * M_PI / 180), sin(usv_.yaw * M_PI / 180), 0.0);
    TVec3 pos = TVec3{pnt.x, pnt.y, pnt.z};
    DrawTrajCommand(pos, 2 * dir, usv1_color_);
    poublisMarker(pnt, usv1_color_, marker_cur_pos_);
}

void usv1_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    usv_.drone_id = current_mavlink.sysid;
    chlog::info("data","sys_id = %d", current_mavlink.sysid);
}

void usv1_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    usv_.altitude = msg->altitude;
    usv_.longtitude = msg->longitude;
    usv_.latitude = msg->latitude;
}

void usv1_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    chlog::info("data", "usv5 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];

    DataMan::getInstance()->setCommand(config);
    PathCreator::geInstance()->CreateUSVFormationInit(config);
}

void usv1_ros_Manager::rvizUsv1GoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    TVec3 goal;
    goal.x() = msg->pose.position.x;
    goal.y() = msg->pose.position.y;
    goal.z() = msg->pose.position.z;
    chlog::info("data", "received rviz goal ", toStr(goal));

    MP_Config mp_config;
    mp_config.is_track_point = true;
    mp_config.is_speed_mode = false;
    mp_config.control_mode = POSITION_WITHOUT_CUR;
    mp_config.is_enable = true;
    mp_config.max_vel = 1.5;
    mp_config.max_acc = 2.0;
    mp_config.mp_map = usv_.Imap;
    mp_config.end_pos = goal;
    ActionMotionPlan::getInstance()->initMP(mp_config);
    ActionMotionPlan::getInstance()->setEnable(true);
}

void usv1_ros_Manager::commander_update(const ros::TimerEvent& e) {
    int command;
    DataMan::getInstance()->getCommand(command);
    if (command == VF_USV_ALL_START /*|| command == SLAVESTART*/) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        chlog::info("data", "usv1 arm_i = %d, is_arm = %d", arm_i_, is_arm_);
        if (!current_state.armed && !is_arm_) {
            while(arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    chlog::info("data", "usv1 Vehicle armed");
                    is_arm_ = true;
                    break;
                }
                --arm_i_;
            }
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        chlog::info("data","is_offboard = %d", is_offboard_);
        if (current_state.mode != "OFFBOARD" && !is_offboard_) {
            static int offboard_i;
            for (offboard_i = 10; ros::ok() && offboard_i > 0; --offboard_i) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    chlog::info("data", "usv1 Offboard enabled");
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
            chlog::info("data", "usv1 homepos_manual.geo.latitude = %.6f", homepos_manual.geo.latitude);
//            home_pos_updated_ = true;
        }
    }

}

void usv1_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
//    DataMan::getInstance()->SetDroneData(usv_);
    chlog::info("data","m_multi_vehicle_.usv1.yaw = ", usv_.yaw,
            ", dronepos_.m_heading =", dronepos_.m_heading,
            ", yaw_cur_ = ", yaw_cur_);
    DataMan::getInstance()->SetDroneData(usv_);
    pcl_manager_->setVehicleMessage(usv_);
    getOctomap();
}

void usv1_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    if (is_speed_ctrl_) {
        g_speed_control_pub.publish(vel_ctrl_sp_);
    } else {
        local_pos_pub.publish(target_local_pos_sp_);

        geometry_msgs::Point p;
        p.x = target_local_pos_sp_.pose.position.x;
        p.y = target_local_pos_sp_.pose.position.y;
        p.z = target_local_pos_sp_.pose.position.z;
        chlog::info("data","draw usv1 target pos = %.2f, %.2f, %.2f", p.x, p.y, p.z);
        poublisMarker(p, usv1_color_, marker_target_pub_);
    }
}

void
usv1_ros_Manager::poublisMarker(const geometry_msgs::Point &p, const TVec4 &color, const ros::Publisher &publisher) {
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

void usv1_ros_Manager::DrawTrajCommand(const TVec3 &pos, const TVec3 &vec, const TVec4 &color) {
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

void usv1_ros_Manager::usvPosSp(const DroneControl& droneControl) {
    target_local_pos_sp_ = droneControl.target_pose;
    is_speed_ctrl_ = droneControl.speed_ctrl;
    target_heading_ = droneControl.target_heading;
    vel_ctrl_sp_ = droneControl.g_vel_sp;
    TVec3 cur_target_err;
    cur_target_err.x() = target_local_pos_sp_.pose.position.x - usv_.current_local_pos.pose.position.x;
    cur_target_err.y() = target_local_pos_sp_.pose.position.y - usv_.current_local_pos.pose.position.y;
    cur_target_err.z() = target_local_pos_sp_.pose.position.z - usv_.current_local_pos.pose.position.z;
    float len = cur_target_err.norm();
    chlog::info("data", "usv1 target and current local pos error = ", len);
}

void usv1_ros_Manager::wayPointCB(const mavros_msgs::WaypointList::ConstPtr &msg) {
    usv_.waypointList = *msg;
    chlog::info("data", "usv1 waypoint size = ", usv_.waypointList.waypoints.size(),", current_seq =",  usv_.waypointList.current_seq);
}

void usv1_ros_Manager::wayPointReachedCB(const mavros_msgs::WaypointReached::ConstPtr &msg) {
    usv_.waypointReached = *msg;
    chlog::info("data", "usv1 wayPoint reached num = ", usv_.waypointReached.wp_seq);
}

void usv1_ros_Manager::homePositionCB(const mavros_msgs::HomePosition::ConstPtr& msg){
    usv_.homePosition = *msg;
    chlog::info("data","usv1 home position lat = ", to_string(msg->geo.latitude), ", , current_seq = ",
            to_string(msg->geo.longitude));
}

void usv1_ros_Manager::usvCallService(mavros_msgs::SetMode &m_mode) {

    if (current_state.mode != "OFFBOARD" && !is_offboard_called_) {
        static int offboard_j;
        for (offboard_j = 10; ros::ok() && offboard_j > 0; --offboard_j) {
            if (set_mode_client.call(m_mode) &&
                m_mode.response.mode_sent) {
                chlog::info("data", "usv1 called Offboard enabled");
                is_offboard_called_ = true;
            }
        }
    }

//    arming_client.call(m_mode);
}

void usv1_ros_Manager::usvCrash(bool usv1_crash) {
    usv_crash_ = usv1_crash;
};

void usv1_ros_Manager::getOctomap() {
    pcl_manager_->getOctomap(usv_.octomap);
    usv_.Imap->updateOctomap(usv_.octomap);
    TVec3 drone_pos = TVec3(usv_.current_local_pos.pose.position.x,
                            usv_.current_local_pos.pose.position.y,
                            usv_.current_local_pos.pose.position.z);
    if (!usv_.Imap->isStateValid(drone_pos)) {
        chlog::info("data","usv1 is in collision!");
    }
}


