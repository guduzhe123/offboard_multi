//
// Created by zhouhua on 2020/5/3.
//

#include "test/UUV_Control/uuv1_ros_Manager.hpp"

uuv1_ros_Manager::uuv1_ros_Manager() :
        arm_i_(5),
        is_arm_(false),
        is_offboard_(false),
        is_takeoff_(false),
        is_land_(false)
{

}

void uuv1_ros_Manager::usvOnInit(ros::NodeHandle &nh) {
    chlog::info("data","~~~ init usv4");
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &uuv1_ros_Manager::state_cb, this);
    vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,  &uuv1_ros_Manager::vrf_hud_cb, this);
    local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &uuv1_ros_Manager::local_pos_cb, this);
    mavlink_from_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10, &uuv1_ros_Manager::mavlink_from_sb, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &uuv1_ros_Manager::global_pos_cb, this);
    commander_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("mavros/debug_value/debug_vector", 10, &uuv1_ros_Manager::debug_value_cb, this);
    way_point_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, &uuv1_ros_Manager::wayPointCB, this);
    homePos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, &uuv1_ros_Manager::homePositionCB, this);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 100);
    global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 100);
    g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &uuv1_ros_Manager::drone_pos_update, this);
    commander_timer_ = nh.createTimer(ros::Duration(0.01), &uuv1_ros_Manager::commander_update, this);
    publish_timer_ = nh.createTimer(ros::Duration(0.01), &uuv1_ros_Manager::publishDronePosControl, this);
}

void uuv1_ros_Manager::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    usv_.current_state = *msg;
}

void uuv1_ros_Manager::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    current_vfr_hud = *msg;
}

void uuv1_ros_Manager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv_.current_local_pos = *msg;
    chlog::info("data","usv1 current_local_pos = ", usv_.current_local_pos.pose.position.x);
}

void uuv1_ros_Manager::mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg) {
    current_mavlink = *msg;
    usv_.drone_id = current_mavlink.sysid;
    chlog::info("data","sys_id = %d", current_mavlink.sysid);
}

void uuv1_ros_Manager::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    usv_.altitude = msg->altitude;
    usv_.longtitude = msg->longitude;
    usv_.latitude = msg->latitude;
}

void uuv1_ros_Manager::debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    chlog::info("data","usv5 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1],
             debugValue.data[2]);
    int config = (int) debugValue.data[0];

    DataMan::getInstance()->setCommand(config);
    PathCreator::geInstance()->CreateUSVFormationInit(config);
}

void uuv1_ros_Manager::commander_update(const ros::TimerEvent& e) {
    int command;
    DataMan::getInstance()->getCommand(command);
    if (command == VF_UUV_ALL_START /*|| command == SLAVESTART*/) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        chlog::info("data","uav arm_i = %d, is_arm = %d", arm_i_, is_arm_);
        if (!current_state.armed && !is_arm_) {
            while(arm_i_ > 0) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    chlog::info("data","uav Vehicle armed");
                    is_arm_ = true;
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
                    chlog::info("data","uav Offboard enabled");
                    is_offboard_ = true;
                }
            }
        }
    }

    if (command == VF_USV_ALL_STOP) {
        target_local_pos_sp_ = usv_.current_local_pos;
    }

}

void uuv1_ros_Manager::drone_pos_update(const ros::TimerEvent& e) {
//    DataMan::getInstance()->SetDroneData(usv_);
    DataMan::getInstance()->SetDroneData(usv_);
}

void uuv1_ros_Manager::publishDronePosControl(const ros::TimerEvent& e) {
    local_pos_pub.publish(target_local_pos_sp_);
}

void uuv1_ros_Manager::usvPosSp(const geometry_msgs::PoseStamped& way_point) {
    target_local_pos_sp_ = way_point;
}

void uuv1_ros_Manager::wayPointCB(const mavros_msgs::WaypointList::ConstPtr &msg) {
    usv_.waypointList = *msg;
}

void uuv1_ros_Manager::homePositionCB(const mavros_msgs::HomePosition::ConstPtr& msg){
    usv_.homePosition = *msg;
    chlog::info("data","usv1 home position lat = %.8f, lon = %.8f", msg->geo.latitude, msg->geo.longitude);
}
