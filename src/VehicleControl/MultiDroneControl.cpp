//
// Created by zhouhua on 2020/1/26.
//

#include "VehicleControl/MultDroneControl.hpp"
MultiDroneControl* MultiDroneControl:: l_lint = NULL;

MultiDroneControl::MultiDroneControl() :
                    uav_state_(TAKEOFF),
                    is_uav_follow_(false){

}

MultiDroneControl* MultiDroneControl::getInstance() {
    if (l_lint == NULL) {
        l_lint = new MultiDroneControl();
    }
    return l_lint;
}

void MultiDroneControl::onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) {
    util_log("drone control start! size of uav way points = %d", way_points.size());
    uav_way_points_ = way_points;
    is_uav_follow_ = is_uav_follow;
    util_log("is uav state = %d", is_uav_follow_);
}

void MultiDroneControl::DoProgress() {
    util_log("drone is formation = %d", is_formation_);
    if (is_formation_) {
        return;
    }
//    uav_global_pos_sp();
    if (drone_uav_leader_.current_state.mode == "OFFBOARD"  /*&& drone_uav_leader_.current_state.armed*/) {
        util_log("uav state = %d", uav_state_);
        switch (uav_state_) {
            // takeoff
            case TAKEOFF:
                // get usv and uav takeoff location different
                Calculate::getInstance()->getTakeoffPos(m_multi_vehicle_.usv1, m_multi_vehicle_.uav1,
                                                        follow_slave_first_local_);
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = K_multi_usv_formation_distance;

                if (pos_reached(drone_uav_leader_.current_local_pos, target_pos_, 0.8)
/*                    pos_reached(m_multi_vehicle_.uav2.current_local_pos, target_pos_, 5) &&
                    pos_reached(m_multi_vehicle_.uav3.current_local_pos, target_pos_, 5) &&
                    pos_reached(m_multi_vehicle_.uav4.current_local_pos, target_pos_, 5)*/
                    ){
                    if (is_uav_follow_) {
                        uav_state_ = FALLOW_USV;
                    } else {
                        uav_state_ = WAYPOINT;
                    }
                    util_log("finish takeoff uav state = %d", uav_state_);
                }
                break;

            case WAYPOINT:
                if (!uav_way_points_.empty()) {
                    target_pos_ = uav_way_points_.back();
                    if (pos_reached(drone_uav_leader_.current_local_pos, target_pos_, 0.8) /*&&
                        pos_reached(m_multi_vehicle_.uav2.current_local_pos, target_pos_, 5) &&
                        pos_reached(m_multi_vehicle_.uav3.current_local_pos, target_pos_, 5) &&
                        pos_reached(m_multi_vehicle_.uav4.current_local_pos, target_pos_, 5) */) {
                        uav_way_points_.pop_back();
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

                if (pos_reached(drone_uav_leader_.current_local_pos,target_pos_, 0.8)/* &&
                    pos_reached(m_multi_vehicle_.uav2.current_local_pos, target_pos_, 5) &&
                    pos_reached(m_multi_vehicle_.uav3.current_local_pos, target_pos_, 5) &&
                    pos_reached(m_multi_vehicle_.uav4.current_local_pos, target_pos_, 5) */) {

                    util_log("reached the land");
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    DataMan::getInstance()->SetUAVState(land_set_mode);

                }

                break;

            case FORMATION: {
                target_pos_.pose.position.x = follow_slave_first_local_.x();
                target_pos_.pose.position.y = follow_slave_first_local_.y();

                if (pos_reached(m_multi_vehicle_.leader_uav.current_local_pos, target_pos_, 0.8)) {
                    uav_state_ = FALLOW_USV;
                }
                break;
            }

            case FALLOW_USV:
                m_multi_vehicle_.leader_usv.current_state.armed = true; // TODO for test.
                if (m_multi_vehicle_.leader_usv.current_state.armed) {
                    target_pos_.pose.position.x = m_multi_vehicle_.leader_usv.current_local_pos.pose.position.x +
                            follow_slave_first_local_.x();
                    target_pos_.pose.position.y = m_multi_vehicle_.leader_usv.current_local_pos.pose.position.y +
                            follow_slave_first_local_.y();
                    util_log("usv leader target pos x = %.2f, y = %.2f, z = %.2f", m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.x,
                             m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.y, m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.z);
                }
/*                if (m_multi_vehicle_.leader_usv.movement_state == USA_DISARM) {
                    uav_state_ = LAND;
                }*/
                break;

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
    if (m_multi_vehicle_.uav1.current_state.connected &&
        m_multi_vehicle_.uav1.current_state.armed /*&&
        m_multi_vehicle_.uav1.current_state.mode == "OFFBOARD"*/) {
        m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav1;
    } else {
        if (m_multi_vehicle_.uav2.current_state.connected &&
            m_multi_vehicle_.uav2.current_state.armed /*&&
            m_multi_vehicle_.uav2.current_state.mode == "OFFBOARD"*/) {
            m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav2;
        } else {
            if (m_multi_vehicle_.uav3.current_state.connected &&
                m_multi_vehicle_.uav3.current_state.armed /*&&
                m_multi_vehicle_.uav3.current_state.mode == "OFFBOARD"*/) {
                m_multi_vehicle_.leader_uav = m_multi_vehicle_.uav3;
            } else {
                if (m_multi_vehicle_.uav4.current_state.connected &&
                    m_multi_vehicle_.uav4.current_state.armed /*&&
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
    if (config_ == VF_UAV_FALLOW_USV) uav_state_ = FORMATION;

}

geometry_msgs::PoseStamped MultiDroneControl::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
    util_log("formation_target (%.2f, %.2f, %.2f)", formation_target(0), formation_target(1));
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiDroneControl::setVehicleCtrlData() {
    util_log("set vehicle control !!!!!");
    m_multi_vehicle_.uav1.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav1.follower_keep_pos);
    m_multi_vehicle_.uav2.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav2.follower_keep_pos);
    m_multi_vehicle_.uav3.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav3.follower_keep_pos);
    m_multi_vehicle_.uav4.target_local_pos_sp = CalculateTargetPos(drone_uav_leader_.target_local_pos_sp, m_multi_vehicle_.uav4.follower_keep_pos);
    m_multi_vehicle_.leader_uav.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;

    util_log("!!!!!!drone control output = (%.2f, %.2f, %.2f)", m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.x,
             m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.y, m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.z);

    DataMan::getInstance()->SetDroneControlData(m_multi_vehicle_);
}

bool MultiDroneControl::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos,
                                float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < err_allow;
}

void MultiDroneControl::droneManualControl() {
    m_multi_vehicle_.uav1.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;
    m_multi_vehicle_.uav2.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;
    m_multi_vehicle_.uav3.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;
    m_multi_vehicle_.uav4.target_local_pos_sp = drone_uav_leader_.target_local_pos_sp;

    util_log("!!!!!!manual control output = (%.2f, %.2f, %.2f)", drone_uav_leader_.target_local_pos_sp.pose.position.x,
             drone_uav_leader_.target_local_pos_sp.pose.position.y, drone_uav_leader_.target_local_pos_sp.pose.position.z);

    DataMan::getInstance()->SetDroneControlData(m_multi_vehicle_);
}



