//
// Created by zhouhua on 19-9-22.
//

#include "FlightManager.hpp"

FlightManager* FlightManager::l_pInst = NULL;

FlightManager::FlightManager():
                leader_drone_{},
                uav_formation_time_(0),
                is_formation_(false)
                {

}

void FlightManager::DoPosUpdate() {
    multi_vehicle_ = DataMan::getInstance()->GetData();
    Avoidance::getInstance()->DoPosUpdate();
}

FlightManager* FlightManager::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new FlightManager();
    }
    return l_pInst;
}
// follow uav number small to large. follow_ua1 and follow_uav1_local_target must be the same drone.
void
FlightManager::calcFollowUAVPos(const M_Drone &follow_uav1, const M_Drone &follow_uav2, const M_Drone &follow_uav3,
                                TVec3 &follow_uav1_local_target, TVec3 &follow_uav2_local_target,
                                TVec3 &follow_uav3_local_target) {
    GlobalPosition target_gps_pos;
    target_gps_pos.latitude = leader_drone_.latitude;
    target_gps_pos.longitude = leader_drone_.longtitude;
    if (K_Param_local_global) {
        GlobalPosition follow_uav1_global_cur_, follow_uav2_global_cur_, follow_uav3_global_cur_;

        follow_uav1_global_cur_.longitude = follow_uav1.longtitude;
        follow_uav1_global_cur_.latitude = follow_uav1.latitude;

        follow_uav2_global_cur_.longitude = follow_uav2.longtitude;
        follow_uav2_global_cur_.latitude = follow_uav2.latitude;

        follow_uav3_global_cur_.longitude = follow_uav3.longtitude;
        follow_uav3_global_cur_.latitude = follow_uav3.latitude;

/*        MultiFormation::getInstance()->GetLocalPos(target_gps_pos, follow_uav1_global_cur_, follow_uav1_to_leader);
        MultiFormation::getInstance()->GetLocalPos(target_gps_pos, follow_uav2_global_cur_, follow_uav2_to_leader);
        MultiFormation::getInstance()->GetLocalPos(target_gps_pos, follow_uav3_global_cur_, follow_uav3_to_leader);*/

        // get distance between follow uav to leader.
        MultiFormation::getInstance()->GetLocalPos(follow_uav1_global_cur_, target_gps_pos, follow_uav1_to_leader);
        MultiFormation::getInstance()->GetLocalPos(follow_uav2_global_cur_, target_gps_pos, follow_uav2_to_leader);
        MultiFormation::getInstance()->GetLocalPos(follow_uav3_global_cur_, target_gps_pos, follow_uav3_to_leader);

        // get the first time data;
        if (uav_formation_time_ == 1) {
            follow_uav1_to_leader_first = follow_uav1_to_leader;
            follow_uav2_to_leader_first = follow_uav2_to_leader;
            follow_uav3_to_leader_first = follow_uav3_to_leader;
        }

        // follow_uav_to_leader_ need to calculate the local target at the frame of the followers.
/*        follow_uav1_to_leader_(0) = follow_uav1_local_target(0) - follow_uav1_to_leader_first.pose.position.x;
        follow_uav1_to_leader_(1) = follow_uav1_local_target(1) - follow_uav1_to_leader_first.pose.position.y;*/

        follow_uav1_.pose.position.x = follow_uav1_local_target(0) - follow_uav1_to_leader.pose.position.x + follow_uav1.current_local_pos.pose.position.x;
        follow_uav1_.pose.position.y = follow_uav1_local_target(1) - follow_uav1_to_leader.pose.position.y + follow_uav1.current_local_pos.pose.position.y;
        follow_uav1_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;
        follow_uav1_to_leader_(0) = follow_uav1_.pose.position.x;
        follow_uav1_to_leader_(1) = follow_uav1_.pose.position.y;

        follow_uav2_.pose.position.x = follow_uav2_local_target(0) - follow_uav2_to_leader.pose.position.x  + follow_uav2.current_local_pos.pose.position.x;
        follow_uav2_.pose.position.y = follow_uav2_local_target(1) - follow_uav2_to_leader.pose.position.y  + follow_uav2.current_local_pos.pose.position.y;
        follow_uav2_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;
        follow_uav2_to_leader_(0) = follow_uav2_.pose.position.x;
        follow_uav2_to_leader_(1) = follow_uav2_.pose.position.y;

        follow_uav3_.pose.position.x = follow_uav3_local_target(0) - follow_uav3_to_leader.pose.position.x + follow_uav3.current_local_pos.pose.position.x;
        follow_uav3_.pose.position.y = follow_uav3_local_target(1) - follow_uav3_to_leader.pose.position.y + follow_uav3.current_local_pos.pose.position.y;
        follow_uav3_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;
        follow_uav3_to_leader_(0) = follow_uav3_.pose.position.x;
        follow_uav3_to_leader_(1) = follow_uav3_.pose.position.y;
    } else {
        GlobalPosition uav2_global_sp_, uav3_global_sp_, uav4_global_sp_;
        MultiFormation::getInstance()->GetGlobalPos(target_gps_pos, uav2_global_sp_, follow_uav1_local_target);
        MultiFormation::getInstance()->GetGlobalPos(target_gps_pos, uav3_global_sp_, follow_uav2_local_target);
        MultiFormation::getInstance()->GetGlobalPos(target_gps_pos, uav4_global_sp_, follow_uav3_local_target);
        GlobalPosition uav_current;
        uav_current.latitude =  follow_uav1.latitude;
        uav_current.longitude =  follow_uav1.longtitude;

        MultiFormation::getInstance()->GetLocalPos(uav2_global_sp_, uav_current, follow_uav1_);
        follow_uav1_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;

        uav_current.latitude =  follow_uav2.latitude;
        uav_current.longitude =  follow_uav2.longtitude;
        MultiFormation::getInstance()->GetLocalPos(uav3_global_sp_, uav_current, follow_uav2_);
        follow_uav2_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;

        uav_current.latitude =  follow_uav3.latitude;
        uav_current.longitude = follow_uav3.longtitude;
        MultiFormation::getInstance()->GetLocalPos(uav4_global_sp_, uav_current, follow_uav3_);
        follow_uav3_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;
    }
}

void FlightManager::OnInit(const int config) {
    is_formation_ = true;
    uav_formation_time_++;
    switch (config) {
        case VF_SQUARE: {
            util_log("Formation call! Square!");
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(-K_multi_formation_distance, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);

                calcFollowUAVPos(multi_vehicle_.uav2, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav2_,
                                 Drone_uav3_, Drone_uav4_);
                follow_uav1_keep_ = {0,0};
                follow_uav2_keep_ = follow_uav1_to_leader_;
                follow_uav3_keep_ = follow_uav2_to_leader_;
                follow_uav4_keep_ = follow_uav3_to_leader_;

                util_log("uav2 target local pos x= %.2f, y = %.2f", follow_uav1_.pose.position.x, follow_uav1_.pose.position.y);
                util_log("uav3 target local pos x= %.2f, y = %.2f", follow_uav2_.pose.position.x, follow_uav2_.pose.position.y);
                util_log("uav4 target local pos x= %.2f, y = %.2f", follow_uav3_.pose.position.x, follow_uav3_.pose.position.y);
                util_log("follow uav2 keep pos x = %.2f, y = %.2f", follow_uav2_keep_(0), follow_uav2_keep_(1) );
                util_log("follow uav3 keep pos x = %.2f, y = %.2f", follow_uav3_keep_(0), follow_uav3_keep_(1) );
                util_log("follow uav4 keep pos x = %.2f, y = %.2f", follow_uav4_keep_(0), follow_uav4_keep_(1) );
            }

            if (leader_uav_id_ == UAV2) {
                leader_drone_ = multi_vehicle_.uav2;
                Drone_uav1_ = TVec3(0, -K_multi_formation_distance , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(-K_multi_formation_distance, 0 , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, multi_vehicle_.uav4.current_local_pos.pose.position.z);

                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav1_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(-K_multi_formation_distance, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav1_ = TVec3(0, -K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(-K_multi_formation_distance, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }

        }
            break;

        case VF_TRIANGLE: {
            util_log("Formation call! Triangle!");
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(0, -K_multi_formation_distance, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav2, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav2_,
                                 Drone_uav3_, Drone_uav4_);
                follow_uav1_keep_ = {0,0};
                follow_uav2_keep_ = follow_uav1_to_leader_;
                follow_uav3_keep_ = follow_uav2_to_leader_;
                follow_uav4_keep_ = follow_uav3_to_leader_;
                util_log("uav2 target local pos x= %.2f, y = %.2f", follow_uav1_.pose.position.x, follow_uav1_.pose.position.y);
                util_log("uav3 target local pos x= %.2f, y = %.2f", follow_uav2_.pose.position.x, follow_uav2_.pose.position.y);
                util_log("uav4 target local pos x= %.2f, y = %.2f", follow_uav3_.pose.position.x, follow_uav3_.pose.position.y);
                util_log("follow uav2 keep pos x = %.2f, y = %.2f", follow_uav2_keep_(0), follow_uav2_keep_(1) );
                util_log("follow uav3 keep pos x = %.2f, y = %.2f", follow_uav3_keep_(0), follow_uav3_keep_(1) );
                util_log("follow uav4 keep pos x = %.2f, y = %.2f", follow_uav4_keep_(0), follow_uav4_keep_(1) );
            }

            if (leader_uav_id_ == UAV2) {
                leader_drone_ = multi_vehicle_.uav2;
                Drone_uav3_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -K_multi_formation_distance, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav2_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(0, -K_multi_formation_distance, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav3_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance, multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(0, -K_multi_formation_distance, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(-K_multi_formation_distance, -K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }
        }
            break;


        case VF_LINE_HORIZONTAL : {
            util_log("Formation call! Line horizontal!");
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(0, -K_multi_formation_distance , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(0, -2 * K_multi_formation_distance, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -3 * K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav2, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav2_,
                                 Drone_uav3_, Drone_uav4_);
                follow_uav1_keep_ = {0,0};
                follow_uav2_keep_ = follow_uav1_to_leader_;
                follow_uav3_keep_ = follow_uav2_to_leader_;
                follow_uav4_keep_ = follow_uav3_to_leader_;
            }

            if (leader_uav_id_ == UAV2) {
                leader_drone_ = multi_vehicle_.uav2;
                Drone_uav3_ = TVec3(0, -K_multi_formation_distance , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -2 * K_multi_formation_distance, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(0, -3 * K_multi_formation_distance , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav2_ = TVec3(-0, -K_multi_formation_distance , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(-0, -2 * K_multi_formation_distance, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -3 * K_multi_formation_distance, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav3_ = TVec3(-0, -K_multi_formation_distance , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(-0, -2 * K_multi_formation_distance, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(0, -3 * K_multi_formation_distance , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }
        }
            break;

        case VF_LINE_VERTICAL : {
            util_log("Formation call! Line Vertical!");
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(K_multi_formation_distance, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(2* K_multi_formation_distance, 0, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(3 * K_multi_formation_distance, 0 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav2, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav2_,
                                 Drone_uav3_, Drone_uav4_);
                follow_uav1_keep_ = {0,0};
                follow_uav2_keep_ = follow_uav1_to_leader_;
                follow_uav3_keep_ = follow_uav2_to_leader_;
                follow_uav4_keep_ = follow_uav3_to_leader_;
            }

            if (leader_uav_id_ == UAV2) {
                leader_drone_ = multi_vehicle_.uav2;
                Drone_uav3_ = TVec3(K_multi_formation_distance, 0 , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(2 * K_multi_formation_distance, 0, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(3 * K_multi_formation_distance, 0 , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav2_ = TVec3(K_multi_formation_distance, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(2 * K_multi_formation_distance, 0, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(3 * K_multi_formation_distance, 0 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav3_ = TVec3(K_multi_formation_distance, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(2 * K_multi_formation_distance, 0, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(3 * K_multi_formation_distance, 0 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }
        }
            break;

        default:
            break;

    }
}


void FlightManager::GetFormationOutput(geometry_msgs::PoseStamped &follow_uav_num1,
                                       geometry_msgs::PoseStamped &follow_uav_num2,
                                       geometry_msgs::PoseStamped &follow_uav_num3, bool &is_formation) {
    follow_uav_num1 = follow_uav1_;
    follow_uav_num2 = follow_uav2_;
    follow_uav_num3 = follow_uav3_;
    is_formation = is_formation_;
    follow_uav1 = TVec3(follow_uav1_.pose.position.x, follow_uav1_.pose.position.y, follow_uav1_.pose.position.z);
    follow_uav2 = TVec3(follow_uav2_.pose.position.x, follow_uav2_.pose.position.y, follow_uav2_.pose.position.z);
    follow_uav3 = TVec3(follow_uav3_.pose.position.x, follow_uav3_.pose.position.y, follow_uav3_.pose.position.z);
    DataMan::getInstance()->SetFormationData(leader_uav_id_, follow_uav1, follow_uav2, follow_uav3);
}

void FlightManager::GetKeepFormation(TVec2 &follow_uav1_keep, TVec2 &follow_uav2_keep, TVec2 &follow_uav3_keep,
                                     TVec2 &follow_uav4_keep) {
    follow_uav1_keep = follow_uav1_keep_;
    follow_uav2_keep = follow_uav2_keep_;
    follow_uav3_keep = follow_uav3_keep_;
    follow_uav4_keep = follow_uav4_keep_;
}


void FlightManager::ChooseUAVLeader(int &leader_uav_id) {
    if (multi_vehicle_.uav1.current_state.connected &&
        multi_vehicle_.uav1.current_state.armed &&
        multi_vehicle_.uav1.current_state.mode == "OFFBOARD") {
        leader_uav_id = UAV1;
    } else {
        if (multi_vehicle_.uav2.current_state.connected &&
            multi_vehicle_.uav2.current_state.armed &&
            multi_vehicle_.uav2.current_state.mode == "OFFBOARD") {
            leader_uav_id = UAV2;
        } else {
            if (multi_vehicle_.uav3.current_state.connected &&
                multi_vehicle_.uav3.current_state.armed &&
                multi_vehicle_.uav3.current_state.mode == "OFFBOARD") {
                leader_uav_id = UAV3;
            } else {
                if (multi_vehicle_.uav4.current_state.connected &&
                    multi_vehicle_.uav4.current_state.armed &&
                    multi_vehicle_.uav4.current_state.mode == "OFFBOARD") {
                    leader_uav_id = UAV4;
                }
            }
        }
    }
    leader_uav_id_ =leader_uav_id;
}

void FlightManager::ChooseUSVLeader(int &leader_usv_id) {
    if (multi_vehicle_.usv1.current_state.connected &&
        multi_vehicle_.usv1.current_state.armed &&
        multi_vehicle_.usv1.current_state.mode == "OFFBOARD") {
        leader_usv_id = USV1;
    } else {
        if (multi_vehicle_.usv2.current_state.connected &&
            multi_vehicle_.usv2.current_state.armed &&
            multi_vehicle_.usv2.current_state.mode == "OFFBOARD") {
            leader_usv_id = USV2;
        } else {
            if (multi_vehicle_.usv3.current_state.connected &&
                multi_vehicle_.usv3.current_state.armed &&
                multi_vehicle_.usv3.current_state.mode == "OFFBOARD") {
                leader_usv_id = USV3;
            }
        }
    }
    leader_usv_id_ = leader_usv_id;
}

void FlightManager::OnCheckFormationArrived() {
    switch (leader_uav_id_){
        case UAV1: {
            if (pos_reached(multi_vehicle_.uav2.current_local_pos, follow_uav1_) &&
                pos_reached(multi_vehicle_.uav3.current_local_pos, follow_uav2_) &&
                pos_reached(multi_vehicle_.uav4.current_local_pos, follow_uav3_)) {
                is_formation_ = false;
            }
            break;
        }
        case UAV2: {
            if (pos_reached(multi_vehicle_.uav1.current_local_pos, follow_uav1_) &&
                pos_reached(multi_vehicle_.uav3.current_local_pos, follow_uav2_) &&
                pos_reached(multi_vehicle_.uav4.current_local_pos, follow_uav3_)) {
                is_formation_ = false;
            }
            break;
        }
        case UAV3: {
            if (pos_reached(multi_vehicle_.uav1.current_local_pos, follow_uav1_) &&
                pos_reached(multi_vehicle_.uav2.current_local_pos, follow_uav2_) &&
                pos_reached(multi_vehicle_.uav4.current_local_pos, follow_uav3_)) {
                is_formation_ = false;
            }
            break;
        }
        case UAV4: {
            if (pos_reached(multi_vehicle_.uav1.current_local_pos, follow_uav1_) &&
                pos_reached(multi_vehicle_.uav2.current_local_pos, follow_uav2_) &&
                pos_reached(multi_vehicle_.uav3.current_local_pos, follow_uav3_)) {
                is_formation_ = false;
            }
            break;
        }
    }
}

bool
FlightManager::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &follow_uav_target) {
    float err_px = current_pos.pose.position.x - follow_uav_target.pose.position.x;
    float err_py = current_pos.pose.position.y - follow_uav_target.pose.position.y;
    float err_pz = current_pos.pose.position.z - follow_uav_target.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < K_err_allow;
}


