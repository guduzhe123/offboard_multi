//
// Created by zhouhua on 19-12-1.
//

#include <geometry_msgs/PoseStamped.h>
#include <DataMan.hpp>
#include <Calculate.hpp>
#include "Multi_formation.hpp"

MultiFormation* MultiFormation::multi_formation = NULL;

MultiFormation::MultiFormation() :
        is_formation_(false),
        uav_formation_time_(0){

}

void MultiFormation::Oninit(const int config_) {
    uav_formation_time_++;
    util_log("formation config = %d", config_ );
    switch (config_) {
        case VF_SQUARE: {
            is_formation_ = true;
            util_log("Formation call! Square!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(-K_multi_formation_distance, 0 , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(0, -K_multi_formation_distance , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);

            calcFollowUAVPos(m_multi_vehicle_.uav2, m_multi_vehicle_.uav3, m_multi_vehicle_.uav4, Drone_uav2_,
                             Drone_uav3_, Drone_uav4_);

            util_log("uav2 target local pos x= %.2f, y = %.2f", follow_uav1_.x(), follow_uav1_.y());
            util_log("uav3 target local pos x= %.2f, y = %.2f", follow_uav2_.x(), follow_uav2_.y());
            util_log("uav4 target local pos x= %.2f, y = %.2f", follow_uav3_.x(), follow_uav3_.y());
        }
            break;

        case VF_TRIANGLE: {
            is_formation_ = true;
            util_log("Formation call! Triangle!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(0, -K_multi_formation_distance, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(K_multi_formation_distance, -K_multi_formation_distance , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);
            calcFollowUAVPos(m_multi_vehicle_.uav2, m_multi_vehicle_.uav3, m_multi_vehicle_.uav4, Drone_uav2_,
                             Drone_uav3_, Drone_uav4_);

        }
            break;


        case VF_LINE_HORIZONTAL : {
            is_formation_ = true;
            util_log("Formation call! Line horizontal!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(0, -K_multi_formation_distance , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(0, -2 * K_multi_formation_distance, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(0, -3 * K_multi_formation_distance , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);
            calcFollowUAVPos(m_multi_vehicle_.uav2, m_multi_vehicle_.uav3, m_multi_vehicle_.uav4, Drone_uav2_,
                             Drone_uav3_, Drone_uav4_);

        }
            break;

        case VF_LINE_VERTICAL : {
            is_formation_ = true;
            util_log("Formation call! Line Vertical!");
            leader_drone_ = m_multi_vehicle_.uav1;
            Drone_uav2_ = TVec3(K_multi_formation_distance, 0 , m_multi_vehicle_.uav2.current_local_pos.pose.position.z);
            Drone_uav3_ = TVec3(2* K_multi_formation_distance, 0, m_multi_vehicle_.uav3.current_local_pos.pose.position.z);
            Drone_uav4_ = TVec3(3 * K_multi_formation_distance, 0 , m_multi_vehicle_.uav4.current_local_pos.pose.position.z);
            calcFollowUAVPos(m_multi_vehicle_.uav2, m_multi_vehicle_.uav3, m_multi_vehicle_.uav4, Drone_uav2_,
                             Drone_uav3_, Drone_uav4_);

        }
            break;

        default:
            break;

    }
}

MultiFormation* MultiFormation::getInstance() {
    if (multi_formation == NULL) {
        multi_formation = new MultiFormation();
    }
    return multi_formation;
}


void MultiFormation::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
}

// follow uav number small to large. follow_ua1 and follow_uav1_local_target must be the same drone.
void
MultiFormation::calcFollowUAVPos(const M_Drone &follow_uav1, const M_Drone &follow_uav2, const M_Drone &follow_uav3,
                                 TVec3 &follow_uav1_local_target, TVec3 &follow_uav2_local_target,
                                 TVec3 &follow_uav3_local_target) {
    GlobalPosition target_gps_pos;
    target_gps_pos.latitude = leader_drone_.latitude;
    target_gps_pos.longitude = leader_drone_.longtitude;

    GlobalPosition follow_uav1_global_cur_, follow_uav2_global_cur_, follow_uav3_global_cur_;

    follow_uav1_global_cur_.longitude = follow_uav1.longtitude;
    follow_uav1_global_cur_.latitude = follow_uav1.latitude;

    follow_uav2_global_cur_.longitude = follow_uav2.longtitude;
    follow_uav2_global_cur_.latitude = follow_uav2.latitude;

    follow_uav3_global_cur_.longitude = follow_uav3.longtitude;
    follow_uav3_global_cur_.latitude = follow_uav3.latitude;

    // get distance between follow uav to leader.
    Calculate::getInstance()->GetLocalPos(follow_uav1_global_cur_, target_gps_pos, follow_uav1_to_leader);
    Calculate::getInstance()->GetLocalPos(follow_uav2_global_cur_, target_gps_pos, follow_uav2_to_leader);
    Calculate::getInstance()->GetLocalPos(follow_uav3_global_cur_, target_gps_pos, follow_uav3_to_leader);

    // get the first time data;
    if (uav_formation_time_ == 1) {
        follow_uav1_to_leader_first = follow_uav1_to_leader;
        follow_uav2_to_leader_first = follow_uav2_to_leader;
        follow_uav3_to_leader_first = follow_uav3_to_leader;
    }

    // 目标相对位置-当前相对位置+当前在该飞机坐标系下的绝对位置
    follow_uav1_.x() = follow_uav1_local_target(0) - follow_uav1_to_leader.pose.position.x + follow_uav1.current_local_pos.pose.position.x;
    follow_uav1_.y() = follow_uav1_local_target(1) - follow_uav1_to_leader.pose.position.y + follow_uav1.current_local_pos.pose.position.y;
    follow_uav1_.z() = leader_drone_.current_local_pos.pose.position.z;

    follow_uav2_.x() = follow_uav2_local_target(0) - follow_uav2_to_leader.pose.position.x  + follow_uav2.current_local_pos.pose.position.x;
    follow_uav2_.y() = follow_uav2_local_target(1) - follow_uav2_to_leader.pose.position.y  + follow_uav2.current_local_pos.pose.position.y;
    follow_uav2_.z() = leader_drone_.current_local_pos.pose.position.z;

    follow_uav3_.x() = follow_uav3_local_target(0) - follow_uav3_to_leader.pose.position.x + follow_uav3.current_local_pos.pose.position.x;
    follow_uav3_.y() = follow_uav3_local_target(1) - follow_uav3_to_leader.pose.position.y + follow_uav3.current_local_pos.pose.position.y;
    follow_uav3_.z() = leader_drone_.current_local_pos.pose.position.z;

}

void MultiFormation::OnCheckFormationArrived() {
    if (pos_reached(m_multi_vehicle_.uav2.current_local_pos, follow_uav1_) &&
        pos_reached(m_multi_vehicle_.uav3.current_local_pos, follow_uav2_) &&
        pos_reached(m_multi_vehicle_.uav4.current_local_pos, follow_uav3_)) {
        is_formation_ = false;
    }
}

bool
MultiFormation::pos_reached(geometry_msgs::PoseStamped &current_pos, TVec3 &follow_uav_target) {
    float err_px = current_pos.pose.position.x - follow_uav_target.x();
    float err_py = current_pos.pose.position.y - follow_uav_target.y();
    float err_pz = current_pos.pose.position.z - follow_uav_target.z();

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < K_err_allow;
}

void MultiFormation::DoProgress() {
    OnCheckFormationArrived();
    SetFunctionOutPut();
}
geometry_msgs::PoseStamped MultiFormation::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, Eigen::Matrix<float, 3, 1> formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
    util_log("formation_target (%.2f, %.2f, %.2f)", formation_target(0), formation_target(1));
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiFormation::SetFunctionOutPut() {
    leader_uav_id_ = m_multi_vehicle_.leader_uav.drone_id;
    DataMan::getInstance()->SetFormationData(is_formation_, leader_uav_id_, follow_uav1_,
            follow_uav2_, follow_uav3_);

    if (is_formation_) {
        util_log("formation drone output!!!!!");
        geometry_msgs::PoseStamped leader_curr{};
        leader_curr.pose.position.z = m_multi_vehicle_.leader_uav.current_local_pos.pose.position.z;

        m_multi_vehicle_.uav1.target_local_pos_sp = CalculateTargetPos(leader_curr , TVec3(0,0,0)) ;
        m_multi_vehicle_.uav2.target_local_pos_sp = CalculateTargetPos(leader_curr , follow_uav1_) ;
        m_multi_vehicle_.uav3.target_local_pos_sp = CalculateTargetPos(leader_curr , follow_uav2_) ;
        m_multi_vehicle_.uav4.target_local_pos_sp = CalculateTargetPos(leader_curr , follow_uav3_) ;
        m_multi_vehicle_.leader_uav.target_local_pos_sp = m_multi_vehicle_.uav1.target_local_pos_sp;
        DataMan::getInstance()->SetDroneControlData(m_multi_vehicle_);
    }
}