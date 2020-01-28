//
// Created by zhouhua on 2020/1/26.
//

#include "VehicleControl/MultBoatControl.hpp"

MultiBoatControl::MultiBoatControl() :
        usv_state_(USA_INIT){

}
void MultiBoatControl::onInit(vector<geometry_msgs::PoseStamped> way_points) {
    usv_way_points_ = way_points;
}

void MultiBoatControl::getData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
}

void MultiBoatControl::DoProgress() {
    if (m_multi_vehicle_.leader_usv.current_state.mode == "OFFBOARD") {
        mavros_msgs::CommandBool arm_cmd;
        switch (usv_state_) {
            case USA_INIT:
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

                    if (usv5_reached_ && usv6_reached_ && usv7_reached_) {
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

                    if ((!usv5_reached_ || !usv6_reached_ || !usv7_reached_) && usv_state_ == USA_WAYPOINT){
                        arm_cmd.request.value = true;
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv1.drone_id);
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv2.drone_id);
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv3.drone_id);
                    }

                } else {
                    usv_state_ = USA_DISARM;
                }
                break;

            case USA_DISARM:
                arm_cmd.request.value = false;
                DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv1.drone_id);
                DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv2.drone_id);
                DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv3.drone_id);
                util_log("Disarm all usv");
                mavros_msgs::SetMode uav_state;
                uav_state.request.custom_mode = "AUTO.LAND";
                DataMan::getInstance()->SetUAVState(uav_state);
                break;
        }
    }
    setVehicleCtrlData();

}

void MultiBoatControl::chooseLeader() {
    if (m_multi_vehicle_.usv1.current_state.connected &&
        m_multi_vehicle_.usv1.current_state.armed &&
        m_multi_vehicle_.usv1.current_state.mode == "OFFBOARD") {
        m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv1;
    } else {
        if (m_multi_vehicle_.usv2.current_state.connected &&
            m_multi_vehicle_.usv2.current_state.armed &&
            m_multi_vehicle_.usv2.current_state.mode == "OFFBOARD") {
            m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv2;
        } else {
            if (m_multi_vehicle_.usv3.current_state.connected &&
                m_multi_vehicle_.usv3.current_state.armed &&
                m_multi_vehicle_.usv3.current_state.mode == "OFFBOARD") {
                m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv3;
            }
        }
    }
    DataMan::getInstance()->SetUSVLeader(m_multi_vehicle_.leader_usv);
}

void MultiBoatControl::setVehicleCtrlData() {
    m_multi_vehicle_.usv1.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
    m_multi_vehicle_.usv1.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
    m_multi_vehicle_.usv1.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
    DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);
}

bool MultiBoatControl::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos,
                                float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < err_allow;
}
