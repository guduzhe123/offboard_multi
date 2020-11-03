 //
// Created by zhouhua on 19-12-1.
//

#include <geometry_msgs/PoseStamped.h>
#include <DataMan.hpp>
#include <Calculate.hpp>
#include "Multi_UAV_formation.hpp"

MultiUAVFormation* MultiUAVFormation::multi_formation = NULL;

MultiUAVFormation::MultiUAVFormation() :
        is_formation_(false),
        is_get_takeoff_pos_(false){

}

void MultiUAVFormation::Oninit(const int config) {
    util_log("formation config = %d", config );
    config_ = config;
    leader_curr_pos_ = TVec3(m_multi_vehicle_.uav1.current_local_pos.pose.position.x, m_multi_vehicle_.uav1.current_local_pos.pose.position.y,
                            m_multi_vehicle_.uav1.current_local_pos.pose.position.z);
    switch (config) {
        case VF_UAV_SQUARE: {
            is_formation_ = true;
            util_log("Formation call! Square!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(-K_multi_formation_distance, 0 , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(0, -K_multi_formation_distance , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);

            calcFollowUAVPos();
            util_log("uav2 target local pos x= %.2f, y = %.2f", follow_uav1_.x(), follow_uav1_.y());
            util_log("uav3 target local pos x= %.2f, y = %.2f", follow_uav2_.x(), follow_uav2_.y());
            util_log("uav4 target local pos x= %.2f, y = %.2f", follow_uav3_.x(), follow_uav3_.y());
        }
            break;

        case VF_UAV_TRIANGLE: {
            is_formation_ = true;
            util_log("Formation call! Triangle!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(0, -K_multi_formation_distance, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(K_multi_formation_distance, -K_multi_formation_distance , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);
            calcFollowUAVPos();

        }
            break;


        case VF_UAV_LINE_HORIZONTAL : {
            is_formation_ = true;
            util_log("Formation call! Line horizontal!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(0, -K_multi_formation_distance , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(0, -2 * K_multi_formation_distance, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(0, -3 * K_multi_formation_distance , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);
            calcFollowUAVPos();

        }
            break;

        case VF_UAV_LINE_VERTICAL : {
            is_formation_ = true;
            util_log("Formation call! Line Vertical!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(K_multi_formation_distance, 0 , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(2* K_multi_formation_distance, 0, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(3 * K_multi_formation_distance, 0 , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);
            calcFollowUAVPos();

        }
            break;

        case VF_UAV_RETURN: {
            is_formation_ = true;
            util_log("Formation call! ALL Return!");
            leader_drone_ = m_multi_vehicle_.uav1;
            break;
        }

        default:
            break;

    }
}

MultiUAVFormation* MultiUAVFormation::getInstance() {
    if (multi_formation == NULL) {
        multi_formation = new MultiUAVFormation();
    }
    return multi_formation;
}


void MultiUAVFormation::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
}

// follow uav number small to large. follow_ua1 and follow_uav1_local_target must be the same drone.
void
MultiUAVFormation::calcFollowUAVPos() {
    // 目标相对位置-当前相对位置+当前在该飞机坐标系下的绝对位置
    follow_uav1_.x() = m_multi_vehicle_.uav1.current_local_pos.pose.position.x + Drone_uav2_.x() + follow_uav1_first_local_.x();
    follow_uav1_.y() = m_multi_vehicle_.uav1.current_local_pos.pose.position.y + Drone_uav2_.y() + follow_uav1_first_local_.y();
    follow_uav1_.z() = m_multi_vehicle_.leader_uav.current_local_pos.pose.position.z;
    follow_uav1_keep_local_ = TVec3 (Drone_uav2_.x() + follow_uav1_first_local_.x(), Drone_uav2_.y() + follow_uav1_first_local_.y(), 0);

    follow_uav2_.x() = m_multi_vehicle_.uav1.current_local_pos.pose.position.x + Drone_uav3_.x() + follow_uav2_first_local_.x();
    follow_uav2_.y() = m_multi_vehicle_.uav1.current_local_pos.pose.position.y + Drone_uav3_.y() + follow_uav2_first_local_.y();
    follow_uav2_.z() = m_multi_vehicle_.leader_uav.current_local_pos.pose.position.z;
    follow_uav2_keep_local_ = TVec3 (Drone_uav3_.x() + follow_uav2_first_local_.x(), Drone_uav3_.y() + follow_uav2_first_local_.y(), 0);

    follow_uav3_.x() = m_multi_vehicle_.uav1.current_local_pos.pose.position.x + Drone_uav4_.x() + follow_uav3_first_local_.x();
    follow_uav3_.y() = m_multi_vehicle_.uav1.current_local_pos.pose.position.y + Drone_uav4_.y() + follow_uav3_first_local_.y();
    follow_uav3_.z() = m_multi_vehicle_.leader_uav.current_local_pos.pose.position.z;
    follow_uav3_keep_local_ = TVec3 (Drone_uav4_.x() + follow_uav3_first_local_.x(), Drone_uav4_.y() + follow_uav3_first_local_.y(), 0);
}

void MultiUAVFormation::OnCheckFormationArrived() {
    if (pos_reached(m_multi_vehicle_.uav2.current_local_pos, follow_uav1_) &&
        pos_reached(m_multi_vehicle_.uav3.current_local_pos, follow_uav2_) &&
        pos_reached(m_multi_vehicle_.uav4.current_local_pos, follow_uav3_) && config_ != VF_UAV_RETURN) {
        is_formation_ = false;
    }
}

bool
MultiUAVFormation::pos_reached(geometry_msgs::PoseStamped &current_pos, TVec3 &follow_uav_target) {
    float err_px = current_pos.pose.position.x - follow_uav_target.x();
    float err_py = current_pos.pose.position.y - follow_uav_target.y();
    float err_pz = current_pos.pose.position.z - follow_uav_target.z();

    return sqrt(err_px * err_px + err_py * err_py /*+ err_pz * err_pz*/) < K_err_allow;
}

void MultiUAVFormation::GetTakeoffPos() {
    if (m_multi_vehicle_.leader_uav.current_state.armed ) {

        uav1_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.uav1.homePosition.geo.latitude,
                                               m_multi_vehicle_.uav1.homePosition.geo.longitude,0};
        uav2_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.uav2.homePosition.geo.latitude,
                                               m_multi_vehicle_.uav2.homePosition.geo.longitude,0};
        uav3_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.uav3.homePosition.geo.latitude,
                                               m_multi_vehicle_.uav3.homePosition.geo.longitude,0};
        uav4_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.uav4.homePosition.geo.latitude,
                                               m_multi_vehicle_.uav4.homePosition.geo.longitude,0};
        util_log("uav1_takeoff_gps_pos_ = ( %.9f, %.9f)", uav1_takeoff_gps_pos_.latitude, uav1_takeoff_gps_pos_.longitude);
        util_log("uav2_takeoff_gps_pos_ = ( %.9f, %.9f)", uav2_takeoff_gps_pos_.latitude, uav2_takeoff_gps_pos_.longitude);
        util_log("uav3_takeoff_gps_pos_ = ( %.9f, %.9f)", uav3_takeoff_gps_pos_.latitude, uav3_takeoff_gps_pos_.longitude);
        util_log("uav4_takeoff_gps_pos_ = ( %.9f, %.9f)", uav4_takeoff_gps_pos_.latitude, uav4_takeoff_gps_pos_.longitude);

        Calculate::getInstance()->GetLocalPos(uav1_takeoff_gps_pos_, uav2_takeoff_gps_pos_, follow_uav1_first_local_);
        Calculate::getInstance()->GetLocalPos(uav1_takeoff_gps_pos_, uav3_takeoff_gps_pos_, follow_uav2_first_local_);
        Calculate::getInstance()->GetLocalPos(uav1_takeoff_gps_pos_, uav4_takeoff_gps_pos_, follow_uav3_first_local_);

    }
}

void MultiUAVFormation::DoProgress() {
    GetTakeoffPos();
    OnCheckFormationArrived();
    SetFunctionOutPut();
}

geometry_msgs::PoseStamped MultiUAVFormation::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, Eigen::Matrix<float, 3, 1> formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
    util_log("formation_target (%.2f, %.2f, %.2f)", formation_target(0), formation_target(1), formation_target(2));
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiUAVFormation::SetFunctionOutPut() {
    leader_uav_id_ = m_multi_vehicle_.leader_uav.drone_id;
    DataMan::getInstance()->SetUAVFormationData(is_formation_, leader_uav_id_, follow_uav1_,
                                                follow_uav2_, follow_uav3_);
    DataMan::getInstance()->SetUAVFormationKeepData(TVec3(0, 0, 0), follow_uav1_keep_local_,
                                                    follow_uav2_keep_local_, follow_uav3_keep_local_);

    if (is_formation_) {
        util_log("formation drone output!!!!!");
        if (config_ != VF_UAV_RETURN) {
            geometry_msgs::PoseStamped leader_curr{};
            leader_curr.pose.position.z = m_multi_vehicle_.leader_uav.current_local_pos.pose.position.z;

            m_multi_vehicle_.uav1.target_local_pos_sp = CalculateTargetPos(leader_curr, leader_curr_pos_);
            m_multi_vehicle_.uav2.target_local_pos_sp = CalculateTargetPos(leader_curr, follow_uav1_);
            m_multi_vehicle_.uav3.target_local_pos_sp = CalculateTargetPos(leader_curr, follow_uav2_);
            m_multi_vehicle_.uav4.target_local_pos_sp = CalculateTargetPos(leader_curr, follow_uav3_);
            m_multi_vehicle_.leader_uav.target_local_pos_sp = m_multi_vehicle_.uav1.target_local_pos_sp;
        } else {
            m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.x = 0;
            m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.y = 0;
            m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.z = m_multi_vehicle_.uav1.current_local_pos.pose.position.z;

            m_multi_vehicle_.uav2.target_local_pos_sp = m_multi_vehicle_.uav1.target_local_pos_sp;
            m_multi_vehicle_.uav3.target_local_pos_sp = m_multi_vehicle_.uav1.target_local_pos_sp;
            m_multi_vehicle_.uav4.target_local_pos_sp = m_multi_vehicle_.uav1.target_local_pos_sp;

            m_multi_vehicle_.uav2.target_local_pos_sp.pose.position.z = K_multi_usv_formation_distance;
            m_multi_vehicle_.uav3.target_local_pos_sp.pose.position.z = K_multi_usv_formation_distance;
            m_multi_vehicle_.uav4.target_local_pos_sp.pose.position.z = K_multi_usv_formation_distance;

        }
        DataMan::getInstance()->SetDroneControlData(m_multi_vehicle_);
    }
}