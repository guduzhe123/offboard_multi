//
// Created by zhouhua on 19-12-1.
//

#include "USV_Avoidance.hpp"

USV_Avoidance* USV_Avoidance::l_lptr = NULL;

USV_Avoidance::USV_Avoidance() :
        is_run_avoidance_(false),
        config_(0),
        is_usv1_usv2_crash_(false),
        is_usv1_usv3_crash_(false),
        is_usv2_usv3_crash_(false),
        usv1_crash_(false),
        usv2_crash_(false),
        usv3_crash_(false){

}

void USV_Avoidance::Oninit(const int config) {
    util_log("usv avoidance init!!!!!");
}

USV_Avoidance* USV_Avoidance::getInstance() {
    if (l_lptr == NULL) {
        l_lptr = new USV_Avoidance();
    }
    return l_lptr;
}


void USV_Avoidance::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();

}

void USV_Avoidance::DoProgress() {
        // TODO need to use map.
        float avo_uav1, avo_uav2, avo_uav3;

        checkDistance(m_multi_vehicle_.usv1, m_multi_vehicle_.usv2);
        checkDistance(m_multi_vehicle_.usv1, m_multi_vehicle_.usv3);
        checkDistance(m_multi_vehicle_.usv2, m_multi_vehicle_.usv3);

        util_log("is_usv1_usv2_crash_ = %d, is_usv1_usv3_crash_ = %d, is_usv2_usv3_crash_ = %d",
                 is_usv1_usv2_crash_, is_usv1_usv3_crash_, is_usv2_usv3_crash_);
        if (is_usv1_usv2_crash_ && is_usv1_usv3_crash_ && is_usv2_usv3_crash_) {
            usv1_crash_ = false;
            usv2_crash_ = true;
            usv3_crash_ = true;
        } else if (is_usv1_usv2_crash_){
            usv1_crash_ = false;
            usv2_crash_ = true;
        } else if (is_usv1_usv3_crash_) {
            usv1_crash_ = false;
            usv3_crash_ = true;
        } else if (is_usv2_usv3_crash_) {
            TVec3 usv2_cur(m_multi_vehicle_.usv2.current_local_pos.pose.position.x,
                           m_multi_vehicle_.usv2.current_local_pos.pose.position.y, 0);
            TVec3 usv2_target(m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.x,
                              m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.y, 0);
            float dist_usv2 = (usv2_target - usv2_cur).norm();
            TVec3 usv3_cur(m_multi_vehicle_.usv3.current_local_pos.pose.position.x,
                           m_multi_vehicle_.usv3.current_local_pos.pose.position.y, 0);
            TVec3 usv3_target(m_multi_vehicle_.usv3.target_local_pos_sp.pose.position.x,
                              m_multi_vehicle_.usv3.target_local_pos_sp.pose.position.y, 0);
            float dist_usv3 = (usv3_target - usv3_cur).norm();

            if (dist_usv2 > dist_usv3) {
                usv2_crash_ = false;
                usv3_crash_ = true;
            } else {
                usv2_crash_ = true;
                usv3_crash_ = false;
            }

        }

    SetFunctionOutPut();
}

void USV_Avoidance::checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2) {
    if (fabs(vehicle1.latitude) < 0.1 || fabs(vehicle2.latitude) < 0.1) {
        return;
    }

    float  dist;
    Getvehicledistance(vehicle1, vehicle2, dist);

    if (dist < K_usv_avodiance_dist ) {
        // seems to be velocity.
        if (vehicle1.drone_id == USV1) {
            if ( vehicle2.drone_id == USV2)  is_usv1_usv2_crash_ = true;
            if ( vehicle2.drone_id == USV3)  is_usv1_usv3_crash_ = true;
        }
        if (vehicle1.drone_id == USV2) {
            if ( vehicle2.drone_id == UAV1)  is_usv1_usv2_crash_ = true;
            if ( vehicle2.drone_id == USV3)  is_usv2_usv3_crash_ = true;
        }
        if (vehicle1.drone_id == USV3) {
            if ( vehicle2.drone_id == UAV1)  is_usv1_usv3_crash_ = true;
            if ( vehicle2.drone_id == USV2)  is_usv2_usv3_crash_ = true;
        }
        util_log("vehlcie %d and %d distance < 2!!! distance = %.2f", vehicle1.drone_id, vehicle2.drone_id, dist);
    } else {
        if (vehicle1.drone_id == USV1) {
            if ( vehicle2.drone_id == USV2)  is_usv1_usv2_crash_ = false;
            if ( vehicle2.drone_id == USV3)  is_usv1_usv3_crash_ = false;
        }
        if (vehicle1.drone_id == USV2) {
            if ( vehicle2.drone_id == UAV1)  is_usv1_usv2_crash_ = false;
            if ( vehicle2.drone_id == USV3)  is_usv2_usv3_crash_ = false;
        }
        if (vehicle1.drone_id == USV3) {
            if ( vehicle2.drone_id == UAV1)  is_usv1_usv3_crash_ = false;
            if ( vehicle2.drone_id == USV2)  is_usv2_usv3_crash_ = false;
        }
    }

    util_log("2222  is_usv1_usv2_crash_ = %d, is_usv1_usv3_crash_ = %d, is_usv2_usv3_crash_ = %d",
             is_usv1_usv2_crash_, is_usv1_usv3_crash_, is_usv2_usv3_crash_);
}

void USV_Avoidance::Getvehicledistance(const M_Drone &vehicle1, const M_Drone &vehicle2, float &distance) {
    GlobalPosition loc1,loc2;
    TVec3 two_uav_local_pos;
    loc1.latitude = vehicle1.latitude;
    loc1.longitude = vehicle1.longtitude;
    loc2.latitude = vehicle2.latitude;
    loc2.longitude = vehicle2.longtitude;

    Calculate::getInstance()->GetLocalPos(loc1, loc2, two_uav_local_pos);

    float err_px = two_uav_local_pos.x();
    float err_py = two_uav_local_pos.y();
    float err_pz = 0;
    util_log("vehicle %d %d between length = %.2f, %.2f, %.2f, length = %.2f", vehicle1.drone_id, vehicle2.drone_id,
            err_px, err_py, err_pz, sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz));

    distance = sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz);
    if (fabs(distance) > 1000) {
        util_log("error!");
    }
}

void USV_Avoidance::SetFunctionOutPut() {
    DataMan::getInstance()->SetUSVAvoData(usv1_crash_, usv2_crash_, usv3_crash_);
}
