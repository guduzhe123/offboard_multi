/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


//#include <mavros_msgs/>
#include "multi_offboard.hpp"

MultiOffboard* MultiOffboard::l_pInst = NULL;

MultiOffboard::MultiOffboard() :
        m_drone_uav1_{},
        m_drone_uav2_{},
        m_drone_uav3_{},
        m_drone_uav4_{},
        m_drone_uav5_{},
        m_drone_uav6_{},
        m_drone_uav7_{},
        drone_uav1_{},
        drone_uav2_{},
        drone_uav3_{},
        drone_uav4_{},
        drone_uav5_{},
        drone_uav6_{},
        drone_uav7_{},
        drone_uav_leader_{},
        drone_usv_leader_{},
        is_offboard(false),
        is_armed(false),
        usv_armed(false),
        curr_altitude(0),
        uav_state_(TAKEOFF),
        usv_state_(USA_INIT),
        leader_uav_id_(UAV1),
        leader_usv_id_(USV1),
        usv5_reached_(false),
        usv6_reached_(false),
        usv7_reached_(false)
        {

}
void MultiOffboard::uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav1_.current_state = *msg;
    m_drone_uav1_.current_state = *msg;
    m_drone_uav1_.drone_id = UAV1;
}

void MultiOffboard::uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav2_.current_state = *msg;
    m_drone_uav2_.current_state = *msg;
    m_drone_uav2_.drone_id = UAV2;
}

void MultiOffboard::uav3_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav3_.current_state = *msg;
    m_drone_uav3_.current_state = *msg;
    m_drone_uav3_.drone_id = UAV3;
}

void MultiOffboard::uav4_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav4_.current_state = *msg;
    m_drone_uav4_.current_state = *msg;
    m_drone_uav4_.drone_id = UAV4;
}

void MultiOffboard::uav5_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav5_.current_state = *msg;
    m_drone_uav5_.current_state = *msg;
    m_drone_uav5_.drone_id = USV1;
}

void MultiOffboard::uav6_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav6_.current_state = *msg;
    m_drone_uav6_.current_state = *msg;
    m_drone_uav6_.drone_id = USV2;
}

void MultiOffboard::uav7_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav7_.current_state = *msg;
    m_drone_uav7_.current_state = *msg;
    m_drone_uav7_.drone_id = USV3;
}


void MultiOffboard::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_vfr_hud = *msg;
}

void MultiOffboard::drone_pos_update() {
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav1_);
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav2_);
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav3_);
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav4_);
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav5_);
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav6_);
    FlightManager::getInstance()->DoPosUpdate(m_drone_uav7_);
    FlightManager::getInstance()->ChooseUAVLeader(leader_uav_id_);
    FlightManager::getInstance()->ChooseUSVLeader(leader_usv_id_);
    update_leader_vehicle();
    Avoidance::getInstance()->get_uav_avo_output(uav_avoidance_);
}

void MultiOffboard::update_leader_vehicle() {
    if (leader_uav_id_ == UAV1) {
        drone_uav_leader_ = drone_uav1_;

    } else if (leader_uav_id_ == UAV2) {
        drone_uav_leader_ = drone_uav2_;

    } else if (leader_uav_id_ == UAV3) {
        drone_uav_leader_ = drone_uav3_;

    } else {
        drone_uav_leader_ = drone_uav4_;
    }

    if (leader_usv_id_ == USV1) {
        drone_usv_leader_ = drone_uav5_;

    } else if (leader_usv_id_ == USV2) {
        drone_usv_leader_ = drone_uav6_;

    } else {
        drone_usv_leader_ = drone_uav7_;
    }
}

void MultiOffboard::uav1_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav1_.altitude = msg->altitude;
    m_drone_uav1_.longtitude = msg->longitude;
    m_drone_uav1_.latitude = msg->latitude;
    m_drone_uav1_.drone_id = UAV1;
}
void MultiOffboard::uav2_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav2_.altitude = msg->altitude;
    m_drone_uav2_.longtitude = msg->longitude;
    m_drone_uav2_.latitude = msg->latitude;
    m_drone_uav2_.drone_id = UAV2;
}
void MultiOffboard::uav3_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav3_.altitude = msg->altitude;
    m_drone_uav3_.longtitude = msg->longitude;
    m_drone_uav3_.latitude = msg->latitude;
    m_drone_uav3_.drone_id = UAV3;
}
void MultiOffboard::uav4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav4_.altitude = msg->altitude;
    m_drone_uav4_.longtitude = msg->longitude;
    m_drone_uav4_.latitude = msg->latitude;
    m_drone_uav4_.drone_id = UAV4;
}
void MultiOffboard::uav5_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav5_.altitude = msg->altitude;
    m_drone_uav5_.longtitude = msg->longitude;
    m_drone_uav5_.latitude = msg->latitude;
    m_drone_uav5_.drone_id = USV1;
}
void MultiOffboard::uav6_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav6_.altitude = msg->altitude;
    m_drone_uav6_.longtitude = msg->longitude;
    m_drone_uav6_.latitude = msg->latitude;
    m_drone_uav6_.drone_id = USV2;
}

void MultiOffboard::uav7_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav7_.altitude = msg->altitude;
    m_drone_uav7_.longtitude = msg->longitude;
    m_drone_uav7_.latitude = msg->latitude;
    m_drone_uav7_.drone_id = USV3;
}

void MultiOffboard::uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav1_.current_local_pos = *msg;

    m_drone_uav1_.current_local_pos = *msg;
    m_drone_uav1_.drone_id = UAV1;
}

void MultiOffboard::uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav2_.current_local_pos = *msg;

    m_drone_uav2_.current_local_pos = *msg;
    m_drone_uav2_.drone_id = UAV2;
}

void MultiOffboard::uav3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav3_.current_local_pos = *msg;

    m_drone_uav3_.current_local_pos = *msg;
    m_drone_uav3_.drone_id = UAV3;
}

void MultiOffboard::uav4_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav4_.current_local_pos = *msg;

    m_drone_uav4_.current_local_pos = *msg;
    m_drone_uav4_.drone_id = UAV4;
}

void MultiOffboard::uav5_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav5_.current_local_pos = *msg;

    m_drone_uav5_.current_local_pos = *msg;
    m_drone_uav5_.drone_id = USV1;
}

void MultiOffboard::uav6_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav6_.current_local_pos = *msg;

    m_drone_uav6_.current_local_pos = *msg;
    m_drone_uav6_.drone_id = USV2;
}

void MultiOffboard::uav7_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav7_.current_local_pos = *msg;

    m_drone_uav7_.current_local_pos = *msg;
    m_drone_uav7_.drone_id = USV3;
}

void MultiOffboard::uav2_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    uav2_current_local_pos_sp = *msg;
}

void MultiOffboard::uav1_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

void MultiOffboard::uav2_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav2 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

void MultiOffboard::uav3_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav3 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

void MultiOffboard::uav4_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav4 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

void MultiOffboard::uav5_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav5 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

void MultiOffboard::uav6_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav6 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

void MultiOffboard::uav7_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav7 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    debug_value_ = config;
    FlightManager::getInstance()->OnInit(config);
}

bool MultiOffboard::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos,
                                float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < err_allow;
}

void MultiOffboard::uav_global_pos_sp() {
    if (leader_uav_id_ == UAV1) {
        FlightManager::getInstance()->GetFormationOutput(m_drone_uav2_.target_local_pos_sp, m_drone_uav3_.target_local_pos_sp,
                                                         m_drone_uav4_.target_local_pos_sp, is_uav_formation_);

        FlightManager::getInstance()->FlightManager::getInstance()->OnCheckFormationArrived();
        if (is_uav_formation_) {
            m_drone_uav1_.current_local_pos.pose.position.z += uav_avoidance_[0].local_target_pos_avo.z();
            m_drone_uav2_.target_local_pos_sp.pose.position.z += uav_avoidance_[1].local_target_pos_avo.z();
            m_drone_uav3_.target_local_pos_sp.pose.position.z += uav_avoidance_[2].local_target_pos_avo.z();
            m_drone_uav4_.target_local_pos_sp.pose.position.z += uav_avoidance_[3].local_target_pos_avo.z();

            drone_uav1_.local_pos_pub.publish(m_drone_uav1_.current_local_pos);
            drone_uav2_.local_pos_pub.publish(m_drone_uav2_.target_local_pos_sp);
            drone_uav3_.local_pos_pub.publish(m_drone_uav3_.target_local_pos_sp);
            drone_uav4_.local_pos_pub.publish(m_drone_uav4_.target_local_pos_sp);
            util_log("m_drone_uav2_ target local pos.x = %.2f, y = %.2f, z = %.2f",
                    m_drone_uav2_.target_local_pos_sp.pose.position.x,
                    m_drone_uav2_.target_local_pos_sp.pose.position.y,
                    m_drone_uav2_.target_local_pos_sp.pose.position.z);

            util_log("m_drone_uav3_ target local pos.x = %.2f, y = %.2f, z = %.2f",
                     m_drone_uav3_.target_local_pos_sp.pose.position.x,
                     m_drone_uav3_.target_local_pos_sp.pose.position.y,
                     m_drone_uav3_.target_local_pos_sp.pose.position.z);

            util_log("m_drone_uav4_ target local pos.x = %.2f, y = %.2f, z = %.2f",
                     m_drone_uav4_.target_local_pos_sp.pose.position.x,
                     m_drone_uav4_.target_local_pos_sp.pose.position.y,
                     m_drone_uav4_.target_local_pos_sp.pose.position.z);
        }
    }

}

void MultiOffboard::usv_global_pos_sp() {

}

void MultiOffboard::usv_targte_local_pos() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;

    usv_global_pos_sp();
    if (drone_usv_leader_.current_state.mode == "OFFBOARD") {

        switch (usv_state_) {
            case USA_INIT:
                usv_state_ = USA_WAYPOINT;
                PathCreator::geInstance()->usv_add_way_points(usv_way_points);
                break;

            case USA_WAYPOINT:
                if (!usv_way_points.empty()) {
                    drone_usv_leader_.target_pose = usv_way_points.back();
                    if (pos_reached(drone_uav5_.current_local_pos, usv_way_points.back(), 3)) {
                        usv5_reached_ = true;
                        drone_uav5_.arming_client.call(arm_cmd);
                        util_log("usv5 disarm at one point");
                    }
                    if (pos_reached(drone_uav6_.current_local_pos, usv_way_points.back(), 3)) {
                        usv6_reached_ = true;
                        drone_uav6_.arming_client.call(arm_cmd);
                        util_log("usv6 disarm at one point");
                    }
                    if (pos_reached(drone_uav7_.current_local_pos, usv_way_points.back(), 3)) {
                        usv7_reached_ = true;
                        drone_uav7_.arming_client.call(arm_cmd);
                        util_log("usv7 disarm at one point");
                    }

                    if (usv5_reached_ && usv6_reached_ && usv7_reached_) {
                        util_log("Finished one way point = (%.2f, %.2f, %.2f)",
                                 usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                                 usv_way_points.back().pose.position.z);
                        usv_way_points.pop_back();

                        if (!usv_way_points.empty()) {
                            util_log("Goto next way point = (%.2f, %.2f, %.2f)",
                                     usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                                     usv_way_points.back().pose.position.z);
                        } else {
                            util_log("Finish all target points!");
                            usv_state_ = USA_DISARM;
                        }
                        usv5_reached_ = false;
                        usv6_reached_ = false;
                        usv7_reached_ = false;
                    }

                    if ((!usv5_reached_ || !usv6_reached_ || !usv7_reached_) && usv_state_ == USA_WAYPOINT){
                        arm_cmd.request.value = true;
                        drone_uav5_.arming_client.call(arm_cmd);
                        drone_uav6_.arming_client.call(arm_cmd);
                        drone_uav7_.arming_client.call(arm_cmd);
                    }

                } else {
                    usv_state_ = USA_DISARM;
                }
                break;

            case USA_DISARM:
                arm_cmd.request.value = false;
                drone_uav5_.arming_client.call(arm_cmd);
                drone_uav6_.arming_client.call(arm_cmd);
                drone_uav7_.arming_client.call(arm_cmd);
                util_log("Disarm all usv");
                uav_state_ = LAND;
                break;
        }
    }

    if (uav_state_ == FALLOW_USV ) {
        drone_uav5_.local_pos_pub.publish(drone_usv_leader_.target_pose);
        drone_uav6_.local_pos_pub.publish(drone_usv_leader_.target_pose);
        drone_uav7_.local_pos_pub.publish(drone_usv_leader_.target_pose);
/*        util_log("1111 usv leader target pos x = %.2f, y = %.2f, z = %.2f", drone_usv_leader_.target_pose.pose.position.x,
                 drone_usv_leader_.target_pose.pose.position.y, drone_usv_leader_.target_pose.pose.position.z);*/
    }
}

void MultiOffboard::uav_target_local_pos() {
    uav_global_pos_sp();
    FlightManager::getInstance()->GetKeepFormation(follow_uav1_keep_, follow_uav2_keep_, follow_uav3_keep_,
                                                   follow_uav4_keep_);
    util_log("follow_uav1_keep_ = %.2f, %.2f", follow_uav1_keep_(0), follow_uav1_keep_(1));
    util_log("follow_uav2_keep_ = %.2f, %.2f", follow_uav2_keep_(0), follow_uav2_keep_(1));
    util_log("follow_uav3_keep_ = %.2f, %.2f", follow_uav3_keep_(0), follow_uav3_keep_(1));
    util_log("follow_uav4_keep_ = %.2f, %.2f", follow_uav4_keep_(0), follow_uav4_keep_(1));
    if (drone_uav_leader_.current_state.mode == "OFFBOARD" && drone_uav_leader_.current_state.armed) {
        switch (uav_state_) {
            // takeoff
            case TAKEOFF:
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 15;

                if (pos_reached(drone_uav_leader_.current_local_pos, target_pos_, 0.8)){

                    uav_state_ = FALLOW_USV;
                    PathCreator::geInstance()->uav_add_way_points(uav_way_points);
                    util_log("Finish takeoff");
                }
                break;

            case WAYPOINT:
                if (!uav_way_points.empty()) {
                    target_pos_ = uav_way_points.back();
                    if (pos_reached(drone_uav_leader_.current_local_pos, target_pos_, 0.8)) {
                        uav_way_points.pop_back();
                        util_log("Finished one way point = (%.2f, %.2f, %.2f)",
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

                if (pos_reached(drone_uav_leader_.current_local_pos,target_pos_, 0.8)) {
                    util_log("reached the land");
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";

                    if ((drone_uav1_.current_state.mode != "AUTO.LAND" || drone_uav3_.current_state.mode != "AUTO.LAND" ||
                         drone_uav4_.current_state.mode != "AUTO.LAND") ) {
                        if( drone_uav1_.set_mode_client.call(land_set_mode) &&
                            land_set_mode.response.mode_sent){
                            drone_uav2_.set_mode_client.call(land_set_mode);
                            drone_uav3_.set_mode_client.call(land_set_mode);
                            drone_uav4_.set_mode_client.call(land_set_mode);
                            util_log("Land enabled");
                        }
                    }
                }

                break;

            case FALLOW_USV:
                if (drone_usv_leader_.current_state.armed) {
                    target_pos_.pose.position.x = drone_usv_leader_.target_pose.pose.position.x;
                    target_pos_.pose.position.y = drone_usv_leader_.target_pose.pose.position.y;
/*                    util_log("usv leader target pos x = %.2f, y = %.2f, z = %.2f", drone_usv_leader_.target_pose.pose.position.x,
                             drone_usv_leader_.target_pose.pose.position.y, drone_usv_leader_.target_pose.pose.position.z);*/
                }
                break;

            default:
                break;
        }

        // uav2 is the head drone.
        drone_uav_leader_.target_pose = target_pos_;
    } else {
        drone_uav_leader_.target_pose.pose.position = uav2_current_local_pos_sp.position;
        mavros_msgs::SetMode land_set_mode;
//        mavros_msgs::SetMode return_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";

        if ((drone_uav1_.current_state.mode != "AUTO.LAND" || drone_uav3_.current_state.mode != "AUTO.LAND" ||
             drone_uav4_.current_state.mode != "AUTO.LAND" || drone_uav2_.current_state.mode == "AUTO.LAND") &&
                drone_uav_leader_.current_state.mode == "AUTO.LAND" && is_armed) {
            if( drone_uav_leader_.set_mode_client.call(land_set_mode) &&
                land_set_mode.response.mode_sent){
                drone_uav1_.set_mode_client.call(land_set_mode);
                drone_uav2_.set_mode_client.call(land_set_mode);
                drone_uav3_.set_mode_client.call(land_set_mode);
                drone_uav4_.set_mode_client.call(land_set_mode);
                util_log("Land enabled out");
            }
        }
    }

    drone_uav1_.target_pose.pose.position.x = drone_uav_leader_.target_pose.pose.position.x + follow_uav1_keep_(0);
    drone_uav1_.target_pose.pose.position.y = drone_uav_leader_.target_pose.pose.position.y + follow_uav1_keep_(1);
    drone_uav1_.target_pose.pose.position.z = drone_uav_leader_.target_pose.pose.position.z;

    drone_uav2_.target_pose.pose.position.x = drone_uav_leader_.target_pose.pose.position.x + follow_uav2_keep_(0);
    drone_uav2_.target_pose.pose.position.y = drone_uav_leader_.target_pose.pose.position.y + follow_uav2_keep_(1);
    drone_uav2_.target_pose.pose.position.z = drone_uav_leader_.target_pose.pose.position.z;

    drone_uav3_.target_pose.pose.position.x = drone_uav_leader_.target_pose.pose.position.x + follow_uav3_keep_(0);
    drone_uav3_.target_pose.pose.position.y = drone_uav_leader_.target_pose.pose.position.y + follow_uav3_keep_(1);
    drone_uav3_.target_pose.pose.position.z = drone_uav_leader_.target_pose.pose.position.z;

    drone_uav4_.target_pose.pose.position.x = drone_uav_leader_.target_pose.pose.position.x + follow_uav4_keep_(0);
    drone_uav4_.target_pose.pose.position.y = drone_uav_leader_.target_pose.pose.position.y + follow_uav4_keep_(1);
    drone_uav4_.target_pose.pose.position.z = drone_uav_leader_.target_pose.pose.position.z;

    if (!uav_avoidance_.empty()) {
        drone_uav1_.target_pose.pose.position.z += uav_avoidance_[0].local_target_pos_avo.z();
        drone_uav2_.target_pose.pose.position.z += uav_avoidance_[1].local_target_pos_avo.z();
        drone_uav3_.target_pose.pose.position.z += uav_avoidance_[2].local_target_pos_avo.z();
        drone_uav4_.target_pose.pose.position.z += uav_avoidance_[3].local_target_pos_avo.z();
        util_log("uav avoidance = drone_uav1_ position.z = %.2f", drone_uav1_.target_pose.pose.position.z);
        util_log("uav avoidance = drone_uav2_ position.z = %.2f", drone_uav2_.target_pose.pose.position.z);
        util_log("uav avoidance = drone_uav3_ position.z = %.2f", drone_uav3_.target_pose.pose.position.z);
        util_log("uav avoidance = drone_uav4_ position.z = %.2f", drone_uav4_.target_pose.pose.position.z);
    }

    if ( !is_uav_formation_) {
        drone_uav1_.local_pos_pub.publish(drone_uav1_.target_pose);
        drone_uav2_.local_pos_pub.publish(drone_uav2_.target_pose);
        drone_uav3_.local_pos_pub.publish(drone_uav3_.target_pose);
        drone_uav4_.local_pos_pub.publish(drone_uav4_.target_pose);
    }
}


void MultiOffboard::Oninit() {
    drone_uav1_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, &MultiOffboard::uav1_state_cb, this);
    drone_uav2_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, &MultiOffboard::uav2_state_cb, this);
    drone_uav3_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, &MultiOffboard::uav3_state_cb, this);
    drone_uav4_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav4/mavros/state", 10, &MultiOffboard::uav4_state_cb, this);
    drone_uav5_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav5/mavros/state", 10, &MultiOffboard::uav5_state_cb, this);
    drone_uav6_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav6/mavros/state", 10, &MultiOffboard::uav6_state_cb, this);
    drone_uav7_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav7/mavros/state", 10, &MultiOffboard::uav7_state_cb, this);
    uav1_vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("uav1/mavros/vfr_hud", 10, &MultiOffboard::vrf_hud_cb, this);
    uav1_g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav1/mavros/setpoint_velocity/cmd_vel", 100);


    drone_uav1_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    drone_uav1_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    drone_uav1_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 5);
    drone_uav1_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, &MultiOffboard::uav1_local_pos_cb, this);
    drone_uav1_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/global", 10, &MultiOffboard::uav1_global_pos_cb, this);
    drone_uav1_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav1/mavros/setpoint_raw/target_global", 10);
    drone_uav1_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav1/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav1_debug_value_cb, this);

    drone_uav2_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");
    drone_uav2_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");
    drone_uav2_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 5);
    drone_uav2_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, &MultiOffboard::uav2_local_pos_cb, this);
    uav2_local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav2/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav2_local_pos_sp_cb, this);
    drone_uav2_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav2/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav2_debug_value_cb, this);
    drone_uav2_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav2/mavros/global_position/global", 10, &MultiOffboard::uav2_global_pos_cb, this);
    drone_uav2_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav2/mavros/setpoint_raw/target_global", 10);

    drone_uav3_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav3/mavros/set_mode");
    drone_uav3_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav3/mavros/cmd/arming");
    drone_uav3_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav3/mavros/setpoint_position/local", 5);
    drone_uav3_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav3/mavros/local_position/pose", 10, &MultiOffboard::uav3_local_pos_cb, this);
    drone_uav3_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav3/mavros/global_position/global", 10, &MultiOffboard::uav3_global_pos_cb, this);
    drone_uav3_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav3/mavros/setpoint_raw/target_global", 10);
    drone_uav3_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav3/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav3_debug_value_cb, this);

    drone_uav4_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav4/mavros/set_mode");
    drone_uav4_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav4/mavros/cmd/arming");
    drone_uav4_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav4/mavros/setpoint_position/local", 5);
    drone_uav4_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav4/mavros/local_position/pose", 10, &MultiOffboard::uav4_local_pos_cb, this);
    drone_uav4_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav4/mavros/global_position/global", 10, &MultiOffboard::uav4_global_pos_cb, this);
    drone_uav4_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav4/mavros/setpoint_raw/target_global", 10);
    drone_uav4_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav4/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav4_debug_value_cb, this);
    drone_uav4_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav4/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav4_debug_value_cb, this);

    drone_uav5_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav5/mavros/set_mode");
    drone_uav5_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav5/mavros/cmd/arming");
    drone_uav5_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav5/mavros/setpoint_position/local", 5);
    drone_uav5_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav5/mavros/local_position/pose", 10, &MultiOffboard::uav5_local_pos_cb, this);
    drone_uav5_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav5/mavros/global_position/global", 10, &MultiOffboard::uav5_global_pos_cb, this);
    drone_uav5_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav5/mavros/setpoint_raw/target_global", 10);
    drone_uav5_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav5/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav5_debug_value_cb, this);

    drone_uav6_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav6/mavros/set_mode");
    drone_uav6_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav6/mavros/cmd/arming");
    drone_uav6_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav6/mavros/setpoint_position/local", 5);
    drone_uav6_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav6/mavros/local_position/pose", 10, &MultiOffboard::uav6_local_pos_cb, this);
    drone_uav6_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav6/mavros/global_position/global", 10, &MultiOffboard::uav6_global_pos_cb, this);
    drone_uav6_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav6/mavros/setpoint_raw/target_global", 10);
    drone_uav6_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav6/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav6_debug_value_cb, this);

    drone_uav7_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav7/mavros/set_mode");
    drone_uav7_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav7/mavros/cmd/arming");
    drone_uav7_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav7/mavros/setpoint_position/local", 5);
    drone_uav7_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav7/mavros/local_position/pose", 10, &MultiOffboard::uav7_local_pos_cb, this);
    drone_uav7_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav7/mavros/global_position/global", 10, &MultiOffboard::uav7_global_pos_cb, this);
    drone_uav7_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav7/mavros/setpoint_raw/target_global", 10);
    drone_uav7_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav7/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav7_debug_value_cb, this);

    uav_state_ = TAKEOFF;
    is_offboard = false;
    is_armed = false;
}

MultiOffboard* MultiOffboard::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new MultiOffboard();
    }
    return l_pInst;
}
