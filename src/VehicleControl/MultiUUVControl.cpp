//
// Created by zhouhua on 2020/10/23.
//

#include "VehicleControl/MultUUVControl.hpp"

MultiUUVControl* MultiUUVControl:: l_lint = NULL;

MultiUUVControl::MultiUUVControl() :
        usv_state_(UUV_INIT),
        is_formation_(false){

}

MultiUUVControl* MultiUUVControl::getInstance() {
    if (l_lint == NULL) {
        l_lint = new MultiUUVControl();
    }
    return l_lint;
}

void MultiUUVControl::onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) {
    util_log("boat control start! sizeof usv waypoints = %d", way_points.size());
    uuv_way_points_init_ = way_points;

}

void MultiUUVControl::getData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    is_formation_ = m_multi_vehicle_.leader_uuv.is_formation;
}

void MultiUUVControl::DoProgress() {
/*    if (is_formation_) {
        return;
    }*/
    if (m_multi_vehicle_.leader_uuv.current_state.mode == "OFFBOARD" /*&& m_multi_vehicle_.leader_uav.movement_state == FALLOW_USV*/) {
        mavros_msgs::CommandBool arm_cmd;
        switch (usv_state_) {
            case UUV_INIT:
                util_log("leader uuv armed = %d", m_multi_vehicle_.leader_uuv.current_state.armed);
                if (!m_multi_vehicle_.leader_uuv.current_state.armed) {
                    arm_cmd.request.value = true;
                    DataMan::getInstance()->SetUSVState(arm_cmd, 0);
                    setVehicleCtrlData(); // keep leader usv offboard
                    init_yaw_ = (float)m_multi_vehicle_.usv1.yaw * M_PI / 180.0f;
                    return;
                }

                if (!m_multi_vehicle_.leader_uuv.waypointList.waypoints.empty()) {
                    for (auto &i : m_multi_vehicle_.leader_uuv.waypointList.waypoints) {
                        GlobalPosition takeoff, waypnt;
                        geometry_msgs::PoseStamped target_init;
                        TVec3 target_local;
                        takeoff.longitude = m_multi_vehicle_.leader_uuv.longtitude;
                        takeoff.latitude = m_multi_vehicle_.leader_uuv.latitude;
                        waypnt.longitude = i.y_long;
                        waypnt.latitude = i.x_lat;
                        Calculate::getInstance()->GetLocalPos(takeoff, waypnt, target_local);
                        target_init.pose.position.x = -target_local.x();
                        target_init.pose.position.y = -target_local.y();
                        target_init.pose.position.z = 0;
                        uuv_way_points_.push_back(target_init);
                        util_log("target_local = (%.2f, %.2f)", target_local.x(), target_local.y());
                    }
                    std::reverse(uuv_way_points_.begin(), uuv_way_points_.end());
                    util_log("uuv mission waypoint size = %d", uuv_way_points_.size());
                } else {
                    for (auto & i : uuv_way_points_init_) {
                        geometry_msgs::PoseStamped target_body;
                        Calculate::getInstance()->bodyFrame2LocalFrame(i, target_body,
                                                                       (float)(m_multi_vehicle_.uav1.yaw * M_PI / 180.0f));
                        uuv_way_points_.push_back(target_body);
                        util_log("drone local size = %d", uuv_way_points_.size());
                    }
                }
                usv_state_ = UUV_WAYPOINT;
                break;

            case UUV_WAYPOINT:
                if (!uuv_way_points_.empty()) {
                    m_multi_vehicle_.leader_uuv.target_local_pos_sp = uuv_way_points_.back();
                    target_pos_ = uuv_way_points_.back();
//                    Calculate::getInstance()->bodyFrame2LocalFrame(body_pos_, target_pos_,init_yaw_);

                    if (pos_reached(m_multi_vehicle_.uuv1.current_local_pos, target_pos_, usv_position_allow_reached_)) {
                        arm_cmd.request.value = false;
                        uuv1_reached_ = true;
                        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv1.drone_id);
                        util_log("uuv1 disarm at one point");
                    }

                    if (uuv1_reached_ ) {
                        util_log("Finished one way point = (%.2f, %.2f, %.2f)",
                                 uuv_way_points_.back().pose.position.x, uuv_way_points_.back().pose.position.y,
                                 uuv_way_points_.back().pose.position.z);
                        uuv_way_points_.pop_back();

                        if (!uuv_way_points_.empty()) {
                            util_log("Goto next way point = (%.2f, %.2f, %.2f)",
                                     uuv_way_points_.back().pose.position.x, uuv_way_points_.back().pose.position.y,
                                     uuv_way_points_.back().pose.position.z);
                        } else {
                            util_log("Finish all target points!");
                            usv_state_ = UUV_DISARM;
                        }
                    }


                } else {
                    usv_state_ = UUV_DISARM;
                }
                break;

            case UUV_DISARM:
                // disarm all.
                arm_cmd.request.value = false;
                DataMan::getInstance()->SetUSVState(arm_cmd, 0);
                util_log("Disarm all usv");
                break;

            case UUV_FORMATION:

                break;
        }
        setVehicleCtrlData();
    } else {
        m_multi_vehicle_.leader_uuv.target_local_pos_sp.pose.position =
                m_multi_vehicle_.leader_uuv.current_local_pos.pose.position;
        UUVManualControl();
    }
}

bool MultiUUVControl::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos,
                                   float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;

    return sqrt(err_px * err_px + err_py * err_py) < err_allow;
}

void MultiUUVControl::setVehicleCtrlData() {
    m_multi_vehicle_.leader_uuv.target_local_pos_sp = m_multi_vehicle_.leader_uuv.target_local_pos_sp;
    m_multi_vehicle_.uuv1.target_local_pos_sp = m_multi_vehicle_.leader_uuv.target_local_pos_sp;

    DataMan::getInstance()->SetUUVControlData(m_multi_vehicle_);
}

void MultiUUVControl::UUVManualControl() {

}

void MultiUUVControl::chooseLeader() {
    if (m_multi_vehicle_.uuv1.current_state.connected &&
        m_multi_vehicle_.uuv1.current_state.armed ) {
        m_multi_vehicle_.leader_uav = m_multi_vehicle_.uuv1;
    }
    DataMan::getInstance()->SetUUVLeader(m_multi_vehicle_.leader_uav);
}
