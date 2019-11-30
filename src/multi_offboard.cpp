/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


//#include <mavros_msgs/>
#include "multi_offboard.hpp"

void MultiOffboard::uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}

void MultiOffboard::uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}

void MultiOffboard::uav3_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav3_current_state = *msg;
}

void MultiOffboard::uav4_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav4_current_state = *msg;
}

void MultiOffboard::uav5_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav5_current_state = *msg;
}

void MultiOffboard::uav6_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav6_current_state = *msg;
}

void MultiOffboard::uav7_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav7_current_state = *msg;
}


void MultiOffboard::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_vfr_hud = *msg;
//    curr_altitude = current_vfr_hud.altitude;
//    ROS_INFO("altitude = %d",  current_vfr_hud.altitude);
}

void MultiOffboard::uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_current_local_pos = *msg;
}

void MultiOffboard::uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav2_current_local_pos = *msg;
}

void MultiOffboard::uav3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav3_current_local_pos = *msg;
}

void MultiOffboard::uav4_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav4_current_local_pos = *msg;
}

void MultiOffboard::uav5_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav5_current_local_pos = *msg;
}

void MultiOffboard::uav6_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav6_current_local_pos = *msg;
}

void MultiOffboard::uav7_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav7_current_local_pos = *msg;
}

void MultiOffboard::uav2_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    uav2_current_local_pos_sp = *msg;
}

bool MultiOffboard::pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos, float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < err_allow;
}

void MultiOffboard::uav_add_way_points() {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    std::reverse(uav_way_points.begin(), uav_way_points.end());
}

void MultiOffboard::usv_add_way_points() {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -40;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
}

void MultiOffboard::usa_targte_local_pos() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;

    if (uav5_current_state.mode == "OFFBOARD") {
        switch (usv_state_) {
            case USA_INIT:
                usv_state_ = USA_WAYPOINT;
                usv_add_way_points();
                break;

            case USA_WAYPOINT:
                if (!usv_way_points.empty()) {
                    uav5_target_pose = usv_way_points.back();
                    if (pos_reached(uav5_current_local_pos, usv_way_points.back(), 3)) {
                        usv5_reached_ = true;
                        uav5_arming_client.call(arm_cmd);
                        ROS_INFO("usv5 disarm at one point");
                    }
                    if (pos_reached(uav6_current_local_pos, usv_way_points.back(), 3)) {
                        usv6_reached_ = true;
                        uav6_arming_client.call(arm_cmd);
                        ROS_INFO("usv6 disarm at one point");
                    }
                    if (pos_reached(uav7_current_local_pos, usv_way_points.back(), 3)) {
                        usv7_reached_ = true;
                        uav7_arming_client.call(arm_cmd);
                        ROS_INFO("usv7 disarm at one point");
                    }

                    if (usv5_reached_ && usv6_reached_ && usv7_reached_) {
                        ROS_INFO("Finished one way point = (%.2f, %.2f, %.2f)",
                                 usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                                 usv_way_points.back().pose.position.z);
                        usv_way_points.pop_back();

                        if (!usv_way_points.empty()) {
                            ROS_INFO("Goto next way point = (%.2f, %.2f, %.2f)",
                                     usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                                     usv_way_points.back().pose.position.z);
                        } else {
                            ROS_INFO("Finish all target points!");
                            uav_state_ = USA_DISARM;
                        }
                        usv5_reached_ = false;
                        usv6_reached_ = false;
                        usv7_reached_ = false;
                    }

                    if ((!usv5_reached_ || !usv6_reached_ || !usv7_reached_) && uav_state_ == USA_WAYPOINT){
                        arm_cmd.request.value = true;
                        uav5_arming_client.call(arm_cmd);
                        uav6_arming_client.call(arm_cmd);
                        uav7_arming_client.call(arm_cmd);
                    }

                } else {
                    uav_state_ = USA_DISARM;
                }
                break;

            case USA_DISARM:
                arm_cmd.request.value = false;
                uav5_arming_client.call(arm_cmd);
                uav6_arming_client.call(arm_cmd);
                uav7_arming_client.call(arm_cmd);
                ROS_INFO("Disarm all usv");
                break;
        }
    }

    uav6_target_pose.pose.position = uav5_target_pose.pose.position;
    uav7_target_pose.pose.position = uav5_target_pose.pose.position;

/*    uav6_target_pose.pose.orientation = uav5_current_local_pos.pose.orientation;
    uav7_target_pose.pose.orientation = uav5_current_local_pos.pose.orientation;*/


    uav5_local_pos_pub.publish(uav5_target_pose);
    uav6_local_pos_pub.publish(uav6_target_pose);
    uav7_local_pos_pub.publish(uav7_target_pose);
}

void MultiOffboard::uav_targte_local_pos() {
    if (uav2_current_state.mode == "OFFBOARD") {
        switch (uav_state_) {
            // takeoff
            case TAKEOFF:
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 3;

                if (pos_reached(uav2_current_local_pos, target_pos_, 0.8)){
                    uav_state_ = WAYPOINT;
                    uav_add_way_points();
                    ROS_INFO("Finish takeoff");
                }
                break;

            case WAYPOINT:
                if (!uav_way_points.empty()) {
                    target_pos_ = uav_way_points.back();
                    if (pos_reached(uav2_current_local_pos,target_pos_, 0.8)) {
                        uav_way_points.pop_back();
                        ROS_INFO("Finished one way point = (%.2f, %.2f, %.2f)",
                                target_pos_.pose.position.x, target_pos_.pose.position.y, target_pos_.pose.position.z);
                    }
                } else {
                    uav_state_ = LAND;
                }

                break;

            case LAND:
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 0;
                is_offboard = true;
                is_armed = true;

                if (pos_reached(uav2_current_local_pos,target_pos_, 0.8)) {
                    ROS_INFO("reached the land");
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";

                    if ((uav1_current_state.mode != "AUTO.LAND" || uav3_current_state.mode != "AUTO.LAND" ||
                         uav4_current_state.mode != "AUTO.LAND") ) {
                        if( uav1_set_mode_client.call(land_set_mode) &&
                            land_set_mode.response.mode_sent){
                            uav2_set_mode_client.call(land_set_mode);
                            uav3_set_mode_client.call(land_set_mode);
                            uav4_set_mode_client.call(land_set_mode);
                            // init
                            ROS_INFO("Land enabled");
                        }
                    }
                }

                break;
        }

        // uav2 is the head drone.
        uav2_target_pose = target_pos_;
    } else {
        uav2_target_pose.pose.position = uav2_current_local_pos_sp.position;
        mavros_msgs::SetMode land_set_mode;
//        mavros_msgs::SetMode return_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";

        if ((uav1_current_state.mode != "AUTO.LAND" || uav3_current_state.mode != "AUTO.LAND" ||
             uav4_current_state.mode != "AUTO.LAND") && uav2_current_state.mode == "AUTO.LAND" && is_armed) {
            if( uav1_set_mode_client.call(land_set_mode) &&
                land_set_mode.response.mode_sent){
                uav2_set_mode_client.call(land_set_mode);
                uav3_set_mode_client.call(land_set_mode);
                uav4_set_mode_client.call(land_set_mode);
                ROS_INFO("Land enabled out");
            }
        }
    }

/*    ROS_INFO("target pos = %.2f, %.2f, %.2f", target_pos_.pose.position.x,
             target_pos_.pose.position.y, target_pos_.pose.position.z);*/
    uav1_target_pose.pose.position = uav2_target_pose.pose.position;
    uav3_target_pose.pose.position = uav2_target_pose.pose.position;
    uav4_target_pose.pose.position = uav2_target_pose.pose.position;

    uav1_local_pos_pub.publish(uav1_target_pose);
    uav2_local_pos_pub.publish(uav2_target_pose);
    uav3_local_pos_pub.publish(uav3_target_pose);
    uav4_local_pos_pub.publish(uav4_target_pose);
}


void MultiOffboard::Oninit() {
    uav1_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, &MultiOffboard::uav1_state_cb, this);
    uav2_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, &MultiOffboard::uav2_state_cb, this);
    uav3_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, &MultiOffboard::uav3_state_cb, this);
    uav4_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav4/mavros/state", 10, &MultiOffboard::uav4_state_cb, this);
    uav5_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav5/mavros/state", 10, &MultiOffboard::uav5_state_cb, this);
    uav6_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav6/mavros/state", 10, &MultiOffboard::uav6_state_cb, this);
    uav7_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav7/mavros/state", 10, &MultiOffboard::uav7_state_cb, this);

    uav1_vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("uav1/mavros/vfr_hud", 10, &MultiOffboard::vrf_hud_cb, this);
    uav1_gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/raw/fix", 1000);
    uav1_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/global", 1000);
    uav1_g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav1/mavros/setpoint_velocity/cmd_vel", 100);


    uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 5);
    uav1_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, &MultiOffboard::uav1_local_pos_cb, this);

    uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");
    uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");
    uav2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 5);
    uav2_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, &MultiOffboard::uav2_local_pos_cb, this);
    uav2_local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav2/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav2_local_pos_sp_cb, this);

    uav3_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav3/mavros/set_mode");
    uav3_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav3/mavros/cmd/arming");
    uav3_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav3/mavros/setpoint_position/local", 5);
    uav3_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav3/mavros/local_position/pose", 10, &MultiOffboard::uav3_local_pos_cb, this);

    uav4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav4/mavros/set_mode");
    uav4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav4/mavros/cmd/arming");
    uav4_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav4/mavros/setpoint_position/local", 5);
    uav4_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav4/mavros/local_position/pose", 10, &MultiOffboard::uav4_local_pos_cb, this);

    uav5_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav5/mavros/set_mode");
    uav5_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav5/mavros/cmd/arming");
    uav5_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav5/mavros/setpoint_position/local", 5);
    uav5_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav5/mavros/local_position/pose", 10, &MultiOffboard::uav5_local_pos_cb, this);

    uav6_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav6/mavros/set_mode");
    uav6_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav6/mavros/cmd/arming");
    uav6_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav6/mavros/setpoint_position/local", 5);
    uav6_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav6/mavros/local_position/pose", 10, &MultiOffboard::uav6_local_pos_cb, this);

    uav7_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav7/mavros/set_mode");
    uav7_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav7/mavros/cmd/arming");
    uav7_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav7/mavros/setpoint_position/local", 5);
    uav7_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav7/mavros/local_position/pose", 10, &MultiOffboard::uav7_local_pos_cb, this);

    uav_state_ = TAKEOFF;
    is_offboard = false;
    is_armed = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    MultiOffboard m_multi;

    m_multi.Oninit();

    ros::Rate rate(10.0);

    while(ros::ok() /*&& !m_multi.uav1_current_state.connected */&& !m_multi.uav5_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("is uav5 connected = %d", m_multi.uav5_current_state.connected);
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int count = 0;

    while(ros::ok()){
        if( (m_multi.uav1_current_state.mode != "OFFBOARD" || m_multi.uav3_current_state.mode != "OFFBOARD" ||
                m_multi.uav4_current_state.mode != "OFFBOARD" ) &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !m_multi.is_offboard){
            if( m_multi.uav1_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                m_multi.uav2_set_mode_client.call(offb_set_mode);
                m_multi.uav3_set_mode_client.call(offb_set_mode);
                m_multi.uav4_set_mode_client.call(offb_set_mode);

                m_multi.uav5_set_mode_client.call(offb_set_mode);
                m_multi.uav6_set_mode_client.call(offb_set_mode);
                m_multi.uav7_set_mode_client.call(offb_set_mode);
//                ROS_INFO("UAS vehicle calling offboard");

                if (m_multi.uav5_current_state.mode == "OFFBOARD") {
                    ROS_INFO("USA Vehicle offboard");
                }
                m_multi.is_offboard = true;
                ROS_INFO("UAV Offboard enabled");

            }

            last_request = ros::Time::now();
        } else {
            if( ! m_multi.uav1_current_state.armed /*&&
                (ros::Time::now() - last_request > ros::Duration(5.0)) */&& !m_multi.is_armed){
                if( m_multi.uav1_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    m_multi.uav2_arming_client.call(arm_cmd);
                    m_multi.uav3_arming_client.call(arm_cmd);
                    m_multi.uav4_arming_client.call(arm_cmd);
                    m_multi.uav5_arming_client.call(arm_cmd);
                    m_multi.uav6_arming_client.call(arm_cmd);
                    m_multi.uav7_arming_client.call(arm_cmd);
                    m_multi.is_armed = true;
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if ((m_multi.uav5_current_state.mode != "OFFBOARD" || m_multi.uav6_current_state.mode != "OFFBOARD" ||
            m_multi.uav7_current_state.mode != "OFFBOARD"))  {
            if (m_multi.uav5_set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                m_multi.uav6_set_mode_client.call(offb_set_mode);
                m_multi.uav7_set_mode_client.call(offb_set_mode);
//                last_request = ros::Time::now();
                ROS_INFO("UAS vehicle calling offboard");

                if (m_multi.uav5_current_state.mode == "OFFBOARD") {
                    ROS_INFO("USA Vehicle offboard");
                }
            }
        }

/*        if( ! m_multi.uav5_current_state.armed && !m_multi.usv_armed
            *//*(ros::Time::now() - last_request > ros::Duration(5.0))*//*){
            if( m_multi.uav5_arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                m_multi.uav6_arming_client.call(arm_cmd);
                m_multi.uav7_arming_client.call(arm_cmd);
                m_multi.usv_armed = true;
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }*/

        m_multi.uav_targte_local_pos();
        m_multi.usa_targte_local_pos();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
