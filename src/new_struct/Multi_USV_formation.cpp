//
// Created by zhouhua on 19-12-1.
//

#include <geometry_msgs/PoseStamped.h>
#include <DataMan.hpp>
#include <Calculate.hpp>
#include "Multi_USV_formation.hpp"

MultiUSVFormation* MultiUSVFormation::multi_formation = NULL;

MultiUSVFormation::MultiUSVFormation() :
        config_(0),
        is_formation_(false),
        is_get_takeoff_pos_(false),
        usv1_reached_(false),
        usv2_reached_(false),
        usv3_reached_(false){

}

void MultiUSVFormation::Oninit(const int config) {
    util_log("usv formation config = %d", config );
    config_ = config;
    leader_curr_pos_ = TVec3(m_multi_vehicle_.usv1.current_local_pos.pose.position.x, m_multi_vehicle_.usv1.current_local_pos.pose.position.y,
                            m_multi_vehicle_.usv1.current_local_pos.pose.position.z);
/*    if (!m_multi_vehicle_.usv1.waypointList.waypoints.empty()) {

    }*/

    switch (config) {
        case VF_USV_TRIANGLE: {
            is_formation_ = true;
            util_log("usv Formation call! Triangle!");
            leader_drone_ = m_multi_vehicle_.usv1;
            Drone_usv2_ = TVec3(-K_multi_usv_formation_distance, -K_multi_usv_formation_distance , m_multi_vehicle_.usv2.current_local_pos.pose.position.z);
            Drone_usv3_ = TVec3(K_multi_usv_formation_distance, -K_multi_usv_formation_distance , m_multi_vehicle_.usv3.current_local_pos.pose.position.z);

//            changeToLocalTarget();
            calcFollowUSVPos();
        }
            break;

        case VF_USV_INVERSION_TRIANGLE: {
            is_formation_ = true;
            util_log("usv Formation call! INVERSION Triangle!");
            leader_drone_ = m_multi_vehicle_.usv1;
            Drone_usv2_ = TVec3(-K_multi_usv_formation_distance, K_multi_usv_formation_distance , m_multi_vehicle_.usv2.current_local_pos.pose.position.z);
            Drone_usv3_ = TVec3(K_multi_usv_formation_distance, K_multi_usv_formation_distance , m_multi_vehicle_.usv3.current_local_pos.pose.position.z);

            changeToLocalTarget();
            calcFollowUSVPos();
        }
            break;


        case VF_USV_LINE_HORIZONTAL : {
            is_formation_ = true;
            util_log("usv Formation call! Line horizontal!");
            leader_drone_ = m_multi_vehicle_.usv1;
            Drone_usv2_ = TVec3(0, -K_multi_usv_formation_distance , m_multi_vehicle_.usv2.current_local_pos.pose.position.z);
            Drone_usv3_ = TVec3(0, -2 * K_multi_usv_formation_distance, m_multi_vehicle_.usv3.current_local_pos.pose.position.z);
//            calcFollowUSVPos();
            changeToLocalTarget();
            calcFollowUSVPos();
        }
            break;

        case VF_USV_LINE_VERTICAL : {
            is_formation_ = true;
            util_log("usv Formation call! Line Vertical!");
            leader_drone_ = m_multi_vehicle_.usv1;
            Drone_usv2_ = TVec3(K_multi_usv_formation_distance, 0 , m_multi_vehicle_.usv2.current_local_pos.pose.position.z);
            Drone_usv3_ = TVec3(2* K_multi_usv_formation_distance, 0, m_multi_vehicle_.usv3.current_local_pos.pose.position.z);
//            calcFollowUSVPos();
            changeToLocalTarget();
            calcFollowUSVPos();
        }
            break;

        case VF_USV_ALL_RETURN: {
            is_formation_ = true;
            util_log("usv Formation call! All USVs Return!");
            changeToLocalTarget();
            calcFollowUSVPos();

        }
            break;

        default:
            break;

    }

}

void MultiUSVFormation::changeToLocalTarget() {
    geometry_msgs::PoseStamped body_usv2, body_usv3, local_usv2, local_usv3;
    body_usv2.pose.position.x = Drone_usv2_.x();
    body_usv2.pose.position.y = Drone_usv2_.y();
    body_usv2.pose.position.z = Drone_usv2_.z();

    body_usv3.pose.position.x = Drone_usv3_.x();
    body_usv3.pose.position.y = Drone_usv3_.y();
    body_usv3.pose.position.z = Drone_usv3_.z();

    Calculate::getInstance()->bodyFrame2LocalFrame(body_usv2, local_usv2, m_multi_vehicle_.usv1.yaw * M_PI / 180.0);
    Calculate::getInstance()->bodyFrame2LocalFrame(body_usv3, local_usv3, m_multi_vehicle_.usv1.yaw * M_PI / 180.0);

    Drone_usv2_.x() = local_usv2.pose.position.x;
    Drone_usv2_.y() = local_usv2.pose.position.y;
    Drone_usv2_.z() = local_usv2.pose.position.z;

    Drone_usv3_.x() = local_usv3.pose.position.x;
    Drone_usv3_.y() = local_usv3.pose.position.y;
    Drone_usv3_.z() = local_usv3.pose.position.z;
}

MultiUSVFormation* MultiUSVFormation::getInstance() {
    if (multi_formation == NULL) {
        multi_formation = new MultiUSVFormation();
    }
    return multi_formation;
}


void MultiUSVFormation::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
}

// follow usv number small to large. follow_us1 and follow_usv1_local_target must be the same drone.
void
MultiUSVFormation::calcFollowUSVPos() {
    // 目标相对位置-当前相对位置+当前在该飞机坐标系下的绝对位置
    follow_usv1_.x() = m_multi_vehicle_.usv1.current_local_pos.pose.position.x + Drone_usv2_.x() + follow_usv1_first_local_.x();
    follow_usv1_.y() = m_multi_vehicle_.usv1.current_local_pos.pose.position.y + Drone_usv2_.y() + follow_usv1_first_local_.y();
    follow_usv1_.z() = leader_drone_.current_local_pos.pose.position.z;
    follow_usv1_keep_local_ = TVec3 (Drone_usv2_.x() + follow_usv1_first_local_.x(), Drone_usv2_.y() + follow_usv1_first_local_.y(), 0);

    follow_usv2_.x() = m_multi_vehicle_.usv1.current_local_pos.pose.position.x + Drone_usv3_.x() + follow_usv2_first_local_.x();
    follow_usv2_.y() = m_multi_vehicle_.usv1.current_local_pos.pose.position.y + Drone_usv3_.y() + follow_usv2_first_local_.y();
    follow_usv2_.z() = leader_drone_.current_local_pos.pose.position.z;
    follow_usv2_keep_local_ = TVec3 (Drone_usv3_.x() + follow_usv2_first_local_.x(), Drone_usv3_.y() + follow_usv2_first_local_.y(), 0);
    util_log("m_multi_vehicle_.usv2.current_local_pos.pose.position.x = %.2f, y = %.2f",
            m_multi_vehicle_.usv2.current_local_pos.pose.position.x,
             m_multi_vehicle_.usv2.current_local_pos.pose.position.y);

}

void MultiUSVFormation::OnCheckFormationArrived() {
    mavros_msgs::CommandBool arm_cmd;
    if (is_formation_) {
        usv1_reached_ = true;
        usv2_reached_ = true;
        usv3_reached_ = true;

        arm_cmd.request.value = false;
//        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv1.drone_id);
    }

    if (pos_reached(m_multi_vehicle_.usv2.current_local_pos, m_multi_vehicle_.usv2.target_local_pos_sp, usv_position_allow_reached_)) {
        usv2_reached_ = true;
/*        arm_cmd.request.value = false;
        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv2.drone_id);
        util_log("usv6 disarm at one point");*/
        util_log("usv6 disarm at one point");
    }
    if (pos_reached(m_multi_vehicle_.usv3.current_local_pos, m_multi_vehicle_.usv2.target_local_pos_sp, usv_position_allow_reached_)) {
        usv3_reached_ = true;
/*        arm_cmd.request.value = false;
        DataMan::getInstance()->SetUSVState(arm_cmd, m_multi_vehicle_.usv3.drone_id);
        util_log("usv7 disarm at one point");*/
        util_log("usv7 disarm at one point");
    }

    util_log("is_formation_ = %d, usv1_reached_ = %d, usv2_reached_ = %d, usv3_reached_ = %d", is_formation_, usv1_reached_, usv2_reached_, usv3_reached_);
    if (usv1_reached_ && usv2_reached_ && usv3_reached_ && config_ != VF_USV_ALL_RETURN) {
        is_formation_ = false;

        usv1_reached_ = false;
        usv2_reached_ = false;
        usv3_reached_ = false;
    }
}

bool
MultiUSVFormation::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped  &follow_usv_target, float err_allow) {
    float err_px = current_pos.pose.position.x - follow_usv_target.pose.position.x;
    float err_py = current_pos.pose.position.y - follow_usv_target.pose.position.y;

    return sqrt(err_px * err_px + err_py * err_py ) < err_allow;
}

void MultiUSVFormation::GetTakeoffPos() {
    if (m_multi_vehicle_.usv1.drone_id != 0 && m_multi_vehicle_.usv1.homePosition.geo.latitude != 0) {

        usv1_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.usv1.homePosition.geo.latitude,
                                               m_multi_vehicle_.usv1.homePosition.geo.longitude,0};
        usv2_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.usv2.homePosition.geo.latitude,
                                               m_multi_vehicle_.usv2.homePosition.geo.longitude,0};
        usv3_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.usv3.homePosition.geo.latitude,
                                               m_multi_vehicle_.usv3.homePosition.geo.longitude,0};
        util_log("usv1_takeoff_gps_pos_ = ( %.9f, %.9f)", usv1_takeoff_gps_pos_.latitude, usv1_takeoff_gps_pos_.longitude);
        util_log("usv2_takeoff_gps_pos_ = ( %.9f, %.9f)", usv2_takeoff_gps_pos_.latitude, usv2_takeoff_gps_pos_.longitude);
        util_log("usv3_takeoff_gps_pos_ = ( %.9f, %.9f)", usv3_takeoff_gps_pos_.latitude, usv3_takeoff_gps_pos_.longitude);

        Calculate::getInstance()->GetLocalPos(usv1_takeoff_gps_pos_, usv2_takeoff_gps_pos_, follow_usv1_first_local_);
        Calculate::getInstance()->GetLocalPos(usv1_takeoff_gps_pos_, usv3_takeoff_gps_pos_, follow_usv2_first_local_);
        util_log("follow_usv1_first_local_ x = %.2f, y = %.2f", follow_usv1_first_local_.x(), follow_usv1_first_local_.y());

    }
}

void MultiUSVFormation::DoProgress() {
    GetTakeoffPos();
/*    if (!config_ && !is_formation_) {
        changeToLocalTarget();
        calcFollowUSVPos();
    }*/

    OnCheckFormationArrived();
    SetFunctionOutPut();
}

geometry_msgs::PoseStamped MultiUSVFormation::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, Eigen::Matrix<float, 3, 1> formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
    util_log("usv formation_target (%.2f, %.2f, %.2f)", formation_target(0), formation_target(1), formation_target(2));
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiUSVFormation::SetFunctionOutPut() {
    leader_usv_id_ = m_multi_vehicle_.leader_usv.drone_id;
    m_multi_vehicle_.usv1.follower_to_leader_pos = TVec3{0, 0, 0};
    m_multi_vehicle_.usv2.follower_to_leader_pos = follow_usv1_;
    m_multi_vehicle_.usv3.follower_to_leader_pos = follow_usv2_;

    m_multi_vehicle_.usv1.follower_keep_pos = TVec3(0, 0, 0);
    m_multi_vehicle_.usv2.follower_keep_pos = follow_usv1_keep_local_;
    m_multi_vehicle_.usv3.follower_keep_pos = follow_usv2_keep_local_;
    DataMan::getInstance()->SetUSVFormationData(m_multi_vehicle_, is_formation_);

    if (is_formation_) {
        util_log("formation boat output!!!!!");
        if (config_ != VF_USV_ALL_RETURN) {
            geometry_msgs::PoseStamped leader_curr{};
            leader_curr.pose.position.z = m_multi_vehicle_.leader_usv.current_local_pos.pose.position.z;

            m_multi_vehicle_.usv1.target_local_pos_sp = CalculateTargetPos(leader_curr, leader_curr_pos_);
            m_multi_vehicle_.usv2.target_local_pos_sp = CalculateTargetPos(leader_curr, follow_usv1_);
            m_multi_vehicle_.usv3.target_local_pos_sp = CalculateTargetPos(leader_curr, follow_usv2_);
            m_multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicle_.usv1.target_local_pos_sp;
        } else {
            m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.x = 0;
            m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.y = 0;
            m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.z = 0;

            m_multi_vehicle_.usv2.target_local_pos_sp = m_multi_vehicle_.usv1.target_local_pos_sp;
            m_multi_vehicle_.usv3.target_local_pos_sp = m_multi_vehicle_.usv1.target_local_pos_sp;
        }
        DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);
    }
}