//
// Created by zhouhua on 2020/1/26.
//

#include "VehicleControl/MultBoatControl.hpp"
MultiBoatControl* MultiBoatControl:: l_lint = NULL;

MultiBoatControl::MultiBoatControl() :
        usv_state_(USA_INIT),
        is_formation_(false){

}

MultiBoatControl* MultiBoatControl::getInstance() {
    if (l_lint == NULL) {
        l_lint = new MultiBoatControl();
    }
    return l_lint;
}

void MultiBoatControl::onInit(vector<geometry_msgs::PoseStamped> way_points) {
    util_log("boat control start! sizeof usv waypoints = %d", way_points.size());
    usv_way_points_ = way_points;

}

void MultiBoatControl::getData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    is_formation_ = m_multi_vehicle_.leader_usv.is_formation;
}

void MultiBoatControl::DoProgress() {
    if (is_formation_) {
        return;
    }
    util_log("leader uav movement_state = %d", m_multi_vehicle_.leader_uav.movement_state);
    if (m_multi_vehicle_.leader_usv.current_state.mode == "OFFBOARD" && m_multi_vehicle_.leader_uav.movement_state == FALLOW_USV) {
        mavros_msgs::CommandBool arm_cmd;
        switch (usv_state_) {
            case USA_INIT:
                util_log("leader usv armed = %d", m_multi_vehicle_.leader_usv.current_state.armed);
                if (!m_multi_vehicle_.leader_usv.current_state.armed) {
                    arm_cmd.request.value = true;
                    DataMan::getInstance()->SetUSVState(arm_cmd, 0);
                    setVehicleCtrlData(); // keep leader usv offboard
                    return;
                }
                usv_state_ = USA_WAYPOINT;
                break;

            case USA_WAYPOINT:
                if (!usv_way_points_.empty()) {
                    m_multi_vehicle_.leader_usv.target_local_pos_sp = usv_way_points_.back();
                    if (pos_reached(m_multi_vehicle_.usv1.current_local_pos, usv_way_points_.back(), usv_position_allow_reached_)) {
                        usv5_reached_ = true;
                        arm_cmd.request.value = false;
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv1.drone_id);
                        util_log("usv5 disarm at one point");
                    }
                    if (pos_reached(m_multi_vehicle_.usv2.current_local_pos, usv_way_points_.back(), usv_position_allow_reached_)) {
                        usv6_reached_ = true;
                        arm_cmd.request.value = false;
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv2.drone_id);
                        util_log("usv6 disarm at one point");
                    }
                    if (pos_reached(m_multi_vehicle_.usv3.current_local_pos, usv_way_points_.back(), usv_position_allow_reached_)) {
                        usv7_reached_ = true;
                        arm_cmd.request.value = false;
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv3.drone_id);
                        util_log("usv7 disarm at one point");
                    }

                    if (usv5_reached_ /*&& usv6_reached_ && usv7_reached_*/) {
                        util_log("Finished one way point = (%.2f, %.2f, %.2f)",
                                 usv_way_points_.back().pose.position.x, usv_way_points_.back().pose.position.y,
                                 usv_way_points_.back().pose.position.z);
                        usv_way_points_.pop_back();

                        if (!usv_way_points_.empty()) {
                            util_log("Goto next way point = (%.2f, %.2f, %.2f)",
                                     usv_way_points_.back().pose.position.x, usv_way_points_.back().pose.position.y,
                                     usv_way_points_.back().pose.position.z);
                        } else {
                            util_log("Finish all target points!");
                            usv_state_ = USA_DISARM;
                        }
                        usv5_reached_ = false;
                        usv6_reached_ = false;
                        usv7_reached_ = false;
                    }

                    // arm all if at least one of them cannot reach at the target.
                    if ((!usv5_reached_ || !usv6_reached_ || !usv7_reached_) && usv_state_ == USA_WAYPOINT){
                        arm_cmd.request.value = true;
                        DataMan::getInstance()->SetUSVState(arm_cmd, 0);
                    }

                } else {
                    usv_state_ = USA_DISARM;
                }
                break;

            case USA_DISARM:
                // disarm all.
                arm_cmd.request.value = false;
                DataMan::getInstance()->SetUSVState(arm_cmd, 0);
                util_log("Disarm all usv");
                break;
        }
    }
    setVehicleCtrlData();

}

void MultiBoatControl::chooseLeader() {
    if (m_multi_vehicle_.usv1.current_state.connected &&
        /*m_multi_vehicle_.usv1.current_state.armed &&*/
        m_multi_vehicle_.usv1.current_state.mode == "OFFBOARD") {
        m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv1;
    } else {
        if (m_multi_vehicle_.usv2.current_state.connected &&
           /* m_multi_vehicle_.usv2.current_state.armed &&*/
            m_multi_vehicle_.usv2.current_state.mode == "OFFBOARD") {
            m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv2;
        } else {
            if (m_multi_vehicle_.usv3.current_state.connected &&
                /*m_multi_vehicle_.usv3.current_state.armed &&*/
                m_multi_vehicle_.usv3.current_state.mode == "OFFBOARD") {
                m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv3;
            }
        }
    }
    m_multi_vehicle_.leader_usv.movement_state = usv_state_;
    m_multi_vehicle_.leader_uav.is_formation = is_formation_;
    DataMan::getInstance()->SetUSVLeader(m_multi_vehicle_.leader_usv);
}

geometry_msgs::PoseStamped MultiBoatControl::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
    util_log("formation_target (%.2f, %.2f, %.2f)", formation_target(0), formation_target(1));
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiBoatControl::setVehicleCtrlData() {
    m_multi_vehicle_.usv1.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.target_local_pos_sp, m_multi_vehicle_.usv1.follower_keep_pos);
    m_multi_vehicle_.usv2.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.target_local_pos_sp, m_multi_vehicle_.usv2.follower_keep_pos);
    m_multi_vehicle_.usv3.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.target_local_pos_sp, m_multi_vehicle_.usv3.follower_keep_pos);
    m_multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;

    DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);
}

bool MultiBoatControl::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos,
                                float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py /*+ err_pz * err_pz*/) < err_allow;
}
