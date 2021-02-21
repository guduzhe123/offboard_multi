//
// Created by zhouhua on 2020/1/26.
//

#include "VehicleControl/MultDroneControl.hpp"
MultiDroneControl* MultiDroneControl:: l_lint = NULL;

MultiDroneControl::MultiDroneControl() :
                    uav_state_(INIT),
                    is_uav_follow_(false),
                    state_changed_(false){

}

MultiDroneControl* MultiDroneControl::getInstance() {
    if (l_lint == NULL) {
        l_lint = new MultiDroneControl();
    }
    return l_lint;
}

void MultiDroneControl::onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) {
    chlog::info("data","drone control start! size of uav way points = ", way_points.size());
    uav_way_points_init_ = way_points;
    is_uav_follow_ = is_uav_follow;
    chlog::info("data","is uav state = ", is_uav_follow_);
}

void MultiDroneControl::DoProgress() {
    chlog::info("data","drone is formation = ", is_formation_);
    if (is_formation_) {
        return;
    }

    drone_uav_leader_.droneControl.speed_ctrl = false;
//    uav_global_pos_sp();
    if (drone_uav_leader_.drone_id != 0) {
        chlog::info("data","uav state = ", uav_state_);
        switch (uav_state_) {
            case INIT: {
                chlog::info("data","drone_uav_leader_.drone_id = ", drone_uav_leader_.drone_id, ", , uav1.waypointList.waypoints size = ",
                         m_multi_vehicle_.uav1.waypointList.waypoints.size());
                if (!m_multi_vehicle_.uav1.waypointList.waypoints.empty()) {
                    for (auto &i : m_multi_vehicle_.leader_uav.waypointList.waypoints) {
                        GlobalPosition takeoff, waypnt;
                        geometry_msgs::PoseStamped target_init;
                        TVec3 target_local;
                        takeoff.longitude = m_multi_vehicle_.leader_uav.homePosition.geo.longitude;
                        takeoff.latitude = m_multi_vehicle_.leader_uav.homePosition.geo.latitude;
                        waypnt.longitude = i.y_long;
                        waypnt.latitude = i.x_lat;
                        Calculate::getInstance()->GetLocalPos(takeoff, waypnt, target_local);
                        target_init.pose.position.x = -target_local.x();
                        target_init.pose.position.y = -target_local.y();
                        target_init.pose.position.z = K_uav_height;
                        uav_way_points_.push_back(target_init);
                        chlog::info("data","target_local = " + toStr(target_local));
                    }
                    std::reverse(uav_way_points_.begin(), uav_way_points_.end());
                    chlog::info("data","drone mission waypoint size = ", uav_way_points_.size());
                } else {
                    for (auto &i : uav_way_points_init_) {
                        geometry_msgs::PoseStamped target_body;
                        Calculate::getInstance()->bodyFrame2LocalFrame(i, target_body,
                                                                       (float) (m_multi_vehicle_.uav1.yaw * M_PI /
                                                                                180.0f));
                        uav_way_points_.push_back(target_body);
                        chlog::info("data","drone local size = ", uav_way_points_.size());
                    }
                }
                uav_state_ = TAKEOFF;
                break;
            }
                // takeoff
            case TAKEOFF: {
                // get usv and uav takeoff location different
                /*
                Calculate::getInstance()->getTakeoffPos(m_multi_vehicle_.usv1, m_multi_vehicle_.uav1,
                                                        follow_slave_first_local_);*/
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = K_uav_height;

                bool uav1_reached = pos_reached(drone_uav_leader_, target_pos_, K_pos_target_arrived_len_);
                bool uav2_reached = pos_reached(m_multi_vehicle_.uav2, m_multi_vehicle_.uav2.droneControl.target_pose, K_pos_target_arrived_len_);
                bool uav3_reached = pos_reached(m_multi_vehicle_.uav3, m_multi_vehicle_.uav3.droneControl.target_pose, K_pos_target_arrived_len_);
                bool uav4_reached = pos_reached(m_multi_vehicle_.uav4, m_multi_vehicle_.uav4.droneControl.target_pose, K_pos_target_arrived_len_);
                chlog::info("data","uav1 reached = ", uav1_reached, ", 2 = ", uav2_reached, ", 3 = ", uav3_reached, ", 4 = ", 
                         uav4_reached);
                if (pos_reached(drone_uav_leader_, target_pos_, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav2, m_multi_vehicle_.uav2.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav3, m_multi_vehicle_.uav3.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav4, m_multi_vehicle_.uav4.droneControl.target_pose, K_pos_target_arrived_len_)) {
                    if (m_multi_vehicle_.uuv1.drone_id != 0) {
                        chlog::info("data","uuv1 drone id = ", m_multi_vehicle_.uuv1.drone_id);
                        is_uav_follow_ = true;
                        Calculate::getInstance()->getTakeoffPos(m_multi_vehicle_.uuv1, m_multi_vehicle_.uav1,
                                                                follow_slave_first_local_);
                    }

                    if (is_uav_follow_) {
                        uav_state_ = FORMATION;
                    } else {
                        uav_state_ = WAYPOINT;
                    }
                    chlog::info("data","finish takeoff uav state = ", uav_state_);
                }
                break;
            }

            case WAYPOINT:
                if (!uav_way_points_.empty()) {
                    target_pos_ = uav_way_points_.back();

                    if (pos_reached(drone_uav_leader_, target_pos_, K_pos_target_arrived_len_) &&
                        pos_reached(m_multi_vehicle_.uav2, m_multi_vehicle_.uav2.droneControl.target_pose, K_pos_target_arrived_len_) &&
                        pos_reached(m_multi_vehicle_.uav3, m_multi_vehicle_.uav3.droneControl.target_pose, K_pos_target_arrived_len_) &&
                        pos_reached(m_multi_vehicle_.uav4, m_multi_vehicle_.uav4.droneControl.target_pose, K_pos_target_arrived_len_) ) {
                        uav_way_points_.pop_back();
                        chlog::info("data","Finished one way point = (",
                                 target_pos_.pose.position.x, ", ", target_pos_.pose.position.y, ", ", target_pos_.pose.position.z, ")");
                    }
                } else {
                    uav_state_ = LAND;
                }

                break;

            case LAND:
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 0;

                if (pos_reached(drone_uav_leader_, target_pos_, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav2, m_multi_vehicle_.uav2.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav3, m_multi_vehicle_.uav3.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav4, m_multi_vehicle_.uav4.droneControl.target_pose, K_pos_target_arrived_len_)  ) {

                    chlog::info("data","reached the land");
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    DataMan::getInstance()->SetUAVState(land_set_mode);

                }

                break;

            case FORMATION: {
                target_pos_.pose.position.x = follow_slave_first_local_.x();
                target_pos_.pose.position.y = follow_slave_first_local_.y();

                if (pos_reached(m_multi_vehicle_.leader_uav, target_pos_, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav2, m_multi_vehicle_.uav2.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav3, m_multi_vehicle_.uav3.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav4, m_multi_vehicle_.uav4.droneControl.target_pose, K_pos_target_arrived_len_)  ) {
                    uav_state_ = FALLOW_UUV;
                    state_changed_ = false;
                    leader_reset_home_ = m_multi_vehicle_.uuv1.current_local_pos;
                }
                break;
            }

            case FALLOW_USV: {
                m_multi_vehicle_.leader_usv.current_state.armed = true; // TODO for test.
                if (m_multi_vehicle_.leader_usv.current_state.armed) {
                    target_pos_.pose.position.x = m_multi_vehicle_.leader_usv.current_local_pos.pose.position.x -
                                                  leader_reset_home_.pose.position.x + follow_slave_first_local_.x();
                    target_pos_.pose.position.y = m_multi_vehicle_.leader_usv.current_local_pos.pose.position.y -
                                                  leader_reset_home_.pose.position.y + follow_slave_first_local_.y();

                    chlog::info("data","usv leader target pos (",
                             m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.x, ", ",
                                m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.y,", ",
                                m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.z, ")");
                }
/*                if (m_multi_vehicle_.leader_usv.movement_state == USV_DISARM) {
                    uav_state_ = LAND;
                }*/
                break;
            }

            case FALLOW_UUV: {
                if (m_multi_vehicle_.uuv1.drone_id != 0) {
                    target_pos_.pose.position.x = m_multi_vehicle_.uuv1.current_local_pos.pose.position.x -
                                                  leader_reset_home_.pose.position.x + follow_slave_first_local_.x();
                    target_pos_.pose.position.y = m_multi_vehicle_.uuv1.current_local_pos.pose.position.y -
                                                  leader_reset_home_.pose.position.y + follow_slave_first_local_.y();

                    chlog::info("data","uuv1 current pos (",
                             m_multi_vehicle_.uuv1.current_local_pos.pose.position.x, ", ", 
                             m_multi_vehicle_.uuv1.current_local_pos.pose.position.y,", ",
                                m_multi_vehicle_.uuv1.current_local_pos.pose.position.z, ")");
                }
                break;
            }

            case CIRCLE_INIT: {
                TCircleConfig circle_config;
                circle_config.target_heading = m_multi_vehicle_.uav1.yaw;
                circle_config.m_radius = 10;
                circle_config.m_circle_pos = TVec3(m_multi_vehicle_.uav1.current_local_pos.pose.position.x,
                                                   m_multi_vehicle_.uav1.current_local_pos.pose.position.y +
                                                   circle_config.m_radius,
                                                   m_multi_vehicle_.uav1.current_local_pos.pose.position.z);
                circle_config.m_speed = 1.0;

                ActionCircle::getInstance()->onInit(circle_config);
                uav_state_ = CIRCLE;
                chlog::info("data","Circle init!!!!");
                state_changed_ = true;
                break;
            }

            case CIRCLE: {
                TVec3 uav1_pos;
                uav1_pos.x() = m_multi_vehicle_.uav1.current_local_pos.pose.position.x;
                uav1_pos.y() = m_multi_vehicle_.uav1.current_local_pos.pose.position.y;
                uav1_pos.z() = m_multi_vehicle_.uav1.current_local_pos.pose.position.z;

                ActionCircle::getInstance()->doProgress(uav1_pos, m_multi_vehicle_.uav1.yaw);
                TCircleOutput circle_output;
                ActionCircle::getInstance()->GetOutput(circle_output);
                drone_uav_leader_.droneControl.speed_ctrl = true;
                drone_uav_leader_.droneControl.g_vel_sp.twist.linear.x = circle_output.v_out.x();
                drone_uav_leader_.droneControl.g_vel_sp.twist.linear.y = circle_output.v_out.y();
                drone_uav_leader_.droneControl.g_vel_sp.twist.linear.z = circle_output.v_out.z();
                drone_uav_leader_.droneControl.target_heading = circle_output.m_target_heading;
                drone_uav_leader_.droneControl.yaw_rate = circle_output.m_yaw_rate;

                chlog::info("data","circle result: ", toStr(circle_output.v_out));

                break;
            }

            case RETURN: {
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                chlog::info("data","ALL UAV return home!");
                if (pos_reached(m_multi_vehicle_.leader_uav, target_pos_, K_pos_target_arrived_len_)&&
                    pos_reached(m_multi_vehicle_.uav2, m_multi_vehicle_.uav2.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav3, m_multi_vehicle_.uav3.droneControl.target_pose, K_pos_target_arrived_len_) &&
                    pos_reached(m_multi_vehicle_.uav4, m_multi_vehicle_.uav4.droneControl.target_pose, K_pos_target_arrived_len_)  ) {
                    uav_state_ = LAND;
                    state_changed_ = true;
                }
                break;
            }

            default:
                break;
        }

        // uav2 is the head drone.
        drone_uav_leader_.target_local_pos_sp = target_pos_;

        setVehicleCtrlData();
    } else {
        drone_uav_leader_.target_local_pos_sp.pose.position = m_multi_vehicle_.leader_uav.current_local_pos.pose.position;
        droneManualControl();
    }

}

void MultiDroneControl::chooseLeader() {
    if (m_multi_vehicle_.uav1.current_state.connected /*&&
        m_multi_vehicle_.uav1.current_state.mode == "OFFBOARD"*/) {
        m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav1;
    } else {
        if (m_multi_vehicle_.uav2.current_state.connected /*&&
            m_multi_vehicle_.uav2.current_state.mode == "OFFBOARD"*/) {
            m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav2;
        } else {
            if (m_multi_vehicle_.uav3.current_state.connected  /*&&
                m_multi_vehicle_.uav3.current_state.mode == "OFFBOARD"*/) {
                m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav3;
            } else {
                if (m_multi_vehicle_.uav4.current_state.connected/*&&
                    m_multi_vehicle_.uav4.current_state.mode == "OFFBOARD"*/) {
                    m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav4;
                }
            }
        }
    }
    drone_uav_leader_ = m_multi_vehicle_.leader_uav;
    m_multi_vehicle_.leader_uav.movement_state = uav_state_;
    m_multi_vehicle_.leader_uav.is_formation = is_formation_;
    DataMan::getInstance()->SetUAVLeader(m_multi_vehicle_.leader_uav);
}

void MultiDroneControl::getData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    is_formation_ = m_multi_vehicle_.leader_uav.is_formation;
    config_ = m_multi_vehicle_.user_command;
    if (config_ == VF_UAV_FALLOW_USV && !state_changed_) {
        uav_state_ = FORMATION;
    }
    if (config_ == VF_UAV_CIRCLE && !state_changed_) {
        uav_state_ = CIRCLE_INIT;
    }

    chlog::info("data","uav user command config_ = ", config_,",  state_changed_ = ",  state_changed_);
    if (config_ == VF_UAV_RETURN && !state_changed_) {
        uav_state_ = RETURN;
    }

}

geometry_msgs::PoseStamped MultiDroneControl::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
    chlog::info("data","formation_target (", formation_target(0), ", ", formation_target(1), ")");
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiDroneControl::setVehicleCtrlData() {
    chlog::info("data","uav set vehicle control !!!!!");
    m_multi_vehicle_.uav1.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav1.follower_keep_pos);
    m_multi_vehicle_.uav2.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav2.follower_keep_pos);
    m_multi_vehicle_.uav3.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav3.follower_keep_pos);
    m_multi_vehicle_.uav4.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav4.follower_keep_pos);
    m_multi_vehicle_.leader_uav.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;

    chlog::info("data","!!!!!!drone control output = (", m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.x, ", ",
             m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.y, ", ",
             m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.z, ")");

    if (drone_uav_leader_.droneControl.speed_ctrl) {
        m_multi_vehicle_.uav1.droneControl = drone_uav_leader_.droneControl;
    }
    DataMan::getInstance()->SetDroneControlData(m_multi_vehicle_);
}

bool MultiDroneControl::pos_reached(M_Drone &current_drone, geometry_msgs::PoseStamped &target_pos,
                                    float err_allow) {
    if (current_drone.drone_id == 0 && !current_drone.current_state.armed) {
        return true;
    } else {
        if (current_drone.current_state.mode != "OFFBOARD")
            return false;
    }

    chlog::info("data","drone id = ", current_drone.drone_id);
    chlog::info("data","current_drone = (", current_drone.current_local_pos.pose.position.x,
            ", ", current_drone.current_local_pos.pose.position.y, ", "
            ,current_drone.current_local_pos.pose.position.z, ")");
    chlog::info("data","target_drone = (", target_pos.pose.position.x, ", ", target_pos.pose.position.y
            ,target_pos.pose.position.z, ")");
    float err_px = current_drone.current_local_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_drone.current_local_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_drone.current_local_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < err_allow;
}

void MultiDroneControl::droneManualControl() {
    m_multi_vehicle_.uav1.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;
    m_multi_vehicle_.uav2.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;
    m_multi_vehicle_.uav3.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;
    m_multi_vehicle_.uav4.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;

    chlog::info("data","!!!!!!uav manual control output = (", drone_uav_leader_.target_local_pos_sp.pose.position.x, ", ",
             drone_uav_leader_.target_local_pos_sp.pose.position.y, ", ",
             drone_uav_leader_.target_local_pos_sp.pose.position.z, ")");

    DataMan::getInstance()->SetDroneControlData(m_multi_vehicle_);
}



