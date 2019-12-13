//
// Created by zhouhua on 19-9-22.
//

#include "FlightManager.hpp"

FlightManager* FlightManager::l_pInst = NULL;

FlightManager::FlightManager():
                leader_drone_{},
                uav_formation_time_(0),
                is_formation_(false),
                follow_uav1_keep_{},
                follow_uav2_keep_{},
                follow_uav3_keep_{},
                follow_uav4_keep_{}
                {

}

void FlightManager::DoPosUpdate(const M_Drone &mDrone) {
//    util_log("drone_id = %d", drone_id);
    switch (mDrone.drone_id) {
        case UAV1:{
            multi_vehicle_.uav1 = mDrone;
/*            util_log("uav1 drone local pos x = %.2f, y = %.2f, z = %.2f", mDrone.current_local_pos.pose.position.x,
                     mDrone.current_local_pos.pose.position.y, mDrone.current_local_pos.pose.position.z);
            util_log("uav1 drone gps lat = %.9f, longt = %.9f", mDrone.latitude, mDrone.longtitude);*/
        }
            break;
        case UAV2: {
            multi_vehicle_.uav2 = mDrone;
        }
            break;
        case UAV3: {
            multi_vehicle_.uav3 = mDrone;
        }
            break;
        case UAV4: {
            multi_vehicle_.uav4 = mDrone;
        }
            break;
        case USV1: {
            multi_vehicle_.usv1 = mDrone;
        }
            break;
        case USV2: {
            multi_vehicle_.usv2 = mDrone;
        }
            break;
        case USV3: {
            multi_vehicle_.usv3 = mDrone;
        }
            break;
        case UUV1: {
            multi_vehicle_.uuv1 = mDrone;
        }
            break;
        case UUV2: {
            multi_vehicle_.uuv2 = mDrone;
        }
            break;
        case UUV3: {
            multi_vehicle_.uuv3 = mDrone;
        }
            break;
        default:
            break;
    }
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

        MultiFormation::getInstance()->GetLocalPos(follow_uav1_global_cur_, target_gps_pos, follow_uav1_to_leader);
        MultiFormation::getInstance()->GetLocalPos(follow_uav2_global_cur_, target_gps_pos, follow_uav2_to_leader);
        MultiFormation::getInstance()->GetLocalPos(follow_uav3_global_cur_, target_gps_pos, follow_uav3_to_leader);

        // get the first time data;
        if (uav_formation_time_ == 1) {
            follow_uav1_to_leader_first = follow_uav1_to_leader;
            follow_uav2_to_leader_first = follow_uav2_to_leader;
            follow_uav3_to_leader_first = follow_uav3_to_leader;
        }

        follow_uav1_to_leader_(0) = follow_uav1_local_target(0) - follow_uav1_to_leader_first.pose.position.x;
        follow_uav1_to_leader_(1) = follow_uav1_local_target(1) - follow_uav1_to_leader_first.pose.position.y;
        follow_uav1_.pose.position.x = follow_uav1_local_target(0) - follow_uav1_to_leader.pose.position.x + follow_uav1.current_local_pos.pose.position.x;
        follow_uav1_.pose.position.y = follow_uav1_local_target(1) - follow_uav1_to_leader.pose.position.y + follow_uav1.current_local_pos.pose.position.y;
        follow_uav1_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;

        follow_uav2_to_leader_(0) = follow_uav2_local_target(0) - follow_uav2_to_leader_first.pose.position.x;
        follow_uav2_to_leader_(1) = follow_uav2_local_target(1) - follow_uav2_to_leader_first.pose.position.y;
        follow_uav2_.pose.position.x = follow_uav2_local_target(0) - follow_uav2_to_leader.pose.position.x  + follow_uav2.current_local_pos.pose.position.x;
        follow_uav2_.pose.position.y = follow_uav2_local_target(1) - follow_uav2_to_leader.pose.position.y  + follow_uav2.current_local_pos.pose.position.y;
        follow_uav2_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;

        follow_uav3_to_leader_(0) = follow_uav3_local_target(0) - follow_uav3_to_leader_first.pose.position.x;
        follow_uav3_to_leader_(1) = follow_uav3_local_target(1) - follow_uav3_to_leader_first.pose.position.y;
        follow_uav3_.pose.position.x = follow_uav3_local_target(0) - follow_uav3_to_leader.pose.position.x + follow_uav3.current_local_pos.pose.position.x;
        follow_uav3_.pose.position.y = follow_uav3_local_target(1) - follow_uav3_to_leader.pose.position.y + follow_uav3.current_local_pos.pose.position.y;
        follow_uav3_.pose.position.z = leader_drone_.current_local_pos.pose.position.z;
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
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(-5, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(-5, -5, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -5 , multi_vehicle_.uav4.current_local_pos.pose.position.z);

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
                Drone_uav1_ = TVec3(0, -5 , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(-5, 0 , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(-5, -5, multi_vehicle_.uav4.current_local_pos.pose.position.z);

                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav1_ = TVec3(-5, -5, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(-5, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -5 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav1_ = TVec3(0, -5 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(-5, -5, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(-5, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }

        }
            break;

        case VF_TRIANGLE: {
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(-5, -5 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(0, -5, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(5, -5 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
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
                Drone_uav3_ = TVec3(-5, -5 , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -5, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(-5, -5 , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav2_ = TVec3(-5, -5 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(0, -5, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(-5, -5 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav3_ = TVec3(-5, -5, multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(0, -5, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(-5, -5 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }
        }
            break;


        case VF_LINE_HORIZONTAL : {
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(0, -5 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(0, -10, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -15 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav2, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav2_,
                                 Drone_uav3_, Drone_uav4_);
                follow_uav1_keep_ = {0,0};
                follow_uav2_keep_ = follow_uav1_to_leader_;
                follow_uav3_keep_ = follow_uav2_to_leader_;
                follow_uav4_keep_ = follow_uav3_to_leader_;
            }

            if (leader_uav_id_ == UAV2) {
                leader_drone_ = multi_vehicle_.uav2;
                Drone_uav3_ = TVec3(0, -5 , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -10, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(0, -15 , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav2_ = TVec3(-0, -5 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(-0, -10, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(0, -15 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav3_ = TVec3(-0, -5 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(-0, -10, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(0, -15 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav3, Drone_uav1_,
                                 Drone_uav2_, Drone_uav3_);
            }
        }
            break;

        case VF_LINE_VERTICAL : {
            if (leader_uav_id_ == UAV1) {
                leader_drone_ = multi_vehicle_.uav1;
                Drone_uav2_ = TVec3(5, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav3_ = TVec3(10, 0, multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(15, 0 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav2, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav2_,
                                 Drone_uav3_, Drone_uav4_);
                follow_uav1_keep_ = {0,0};
                follow_uav2_keep_ = follow_uav1_to_leader_;
                follow_uav3_keep_ = follow_uav2_to_leader_;
                follow_uav4_keep_ = follow_uav3_to_leader_;
            }

            if (leader_uav_id_ == UAV2) {
                leader_drone_ = multi_vehicle_.uav2;
                Drone_uav3_ = TVec3(5, 0 , multi_vehicle_.uav3.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(10, 0, multi_vehicle_.uav4.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(15, 0 , multi_vehicle_.uav1.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav3, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav3_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV3) {
                leader_drone_ = multi_vehicle_.uav3;
                Drone_uav2_ = TVec3(5, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(10, 0, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav4_ = TVec3(15, 0 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
                calcFollowUAVPos(multi_vehicle_.uav1, multi_vehicle_.uav2, multi_vehicle_.uav4, Drone_uav1_,
                                 Drone_uav2_, Drone_uav4_);
            }

            if (leader_uav_id_ == UAV4) {
                leader_drone_ = multi_vehicle_.uav4;
                Drone_uav3_ = TVec3(5, 0 , multi_vehicle_.uav2.current_local_pos.pose.position.z);
                Drone_uav2_ = TVec3(10, 0, multi_vehicle_.uav1.current_local_pos.pose.position.z);
                Drone_uav1_ = TVec3(15, 0 , multi_vehicle_.uav4.current_local_pos.pose.position.z);
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


