//
// Created by zhouhua on 19-12-1.
//

#include "test/2uav2/avoidance.hpp"

avoidance* avoidance::l_lptr = NULL;

avoidance::avoidance() :
        is_run_avoidance_(false),
        height_avoidance_uav1_{},
        height_avoidance_uav2_{},
        config_(0){

}

void avoidance::Oninit(const int config) {

}

avoidance* avoidance::getInstance() {
    if (l_lptr == NULL) {
        l_lptr = new avoidance();
    }
    return l_lptr;
}

void avoidance::GetData() {
    multi_vehicle m_multi_vehicle;
    m_multi_vehicle = dataMan::getInstance()->GetData();

    multi_vehicle_.uav1_vec.push_back(m_multi_vehicle.uav1);
    multi_vehicle_.uav2_vec.push_back(m_multi_vehicle.uav2);
}

void avoidance::DoProgress() {
    if (!multi_vehicle_.uav1_vec.empty() && !multi_vehicle_.uav2_vec.empty()) {
        // TODO need to use map.
        float avo_uav1, avo_uav2;

        checkDistance(multi_vehicle_.uav1_vec.back(), multi_vehicle_.uav2_vec.back(), avo_uav1, avo_uav2);
        height_avoidance_uav1_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav2_.local_target_pos_avo.z() += avo_uav2;
        if (fabs(avo_uav1) > 0.01 && fabs(avo_uav2) > 0.01)
            util_log("distance between and uav1 and uav2 < 2m, auv1 move = %.2f, auv2 move = %.2f", avo_uav1, avo_uav2);


        if (fabs(height_avoidance_uav1_.local_target_pos_avo.z()) > K_max_avodiance_pos_) {
            height_avoidance_uav1_.local_target_pos_avo.z() = K_max_avodiance_pos_ *
                                                              fabs(height_avoidance_uav1_.local_target_pos_avo.z()) / height_avoidance_uav1_.local_target_pos_avo.z();
        }
        if (fabs(height_avoidance_uav2_.local_target_pos_avo.z()) > K_max_avodiance_pos_) {
            height_avoidance_uav2_.local_target_pos_avo.z() = K_max_avodiance_pos_ *
                                                              fabs(height_avoidance_uav2_.local_target_pos_avo.z()) / height_avoidance_uav2_.local_target_pos_avo.z();
        }


        is_run_avoidance_ = true;

        checkVerticalDistance(multi_vehicle_);
        checkHorizontalDistance(multi_vehicle_);
        util_log(
                "height_avoidance_uav1_ = %.2f, height_avoidance_uav2_ = %.2f",
                height_avoidance_uav1_.local_target_pos_avo.z(), height_avoidance_uav2_.local_target_pos_avo.z());

    }
    SetFunctionOutPut();

}

void avoidance::checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2,
                              float &m_drone_avoidance1, float &m_drone_avoidance2) {

    if (fabs(vehicle1.latitude) < 0.1 || fabs(vehicle2.latitude) < 0.1) {
        return;
    }

    float  dist, distance_h;
    Getvehicledistance(vehicle1, vehicle2, distance_h, dist);
    distance_h_12_ = distance_h;

    if (dist < 2 ) {
        util_log("vehlcie %d and %d distance < 2!!! distance = %.2f", vehicle1.drone_id, vehicle2.drone_id, dist);
        // seems to be velocity.
        if (vehicle1.current_local_pos.pose.position.z > vehicle2.current_local_pos.pose.position.z) {
            m_drone_avoidance1 = 3 / dist;
            m_drone_avoidance2 = -3 / dist;
        } else {
            m_drone_avoidance1 = -3 / dist;
            m_drone_avoidance2 = 3 / dist;
        }
    } else {
        m_drone_avoidance1 = 0;
        m_drone_avoidance2 = 0;
    }

}

bool avoidance::checkHorizontalArrive(const M_Drone &vehicle) {
    float err_x, err_y;
    err_x = vehicle.target_local_pos_sp.pose.position.x - vehicle.current_local_pos.pose.position.x;
    err_y = vehicle.target_local_pos_sp.pose.position.y - vehicle.current_local_pos.pose.position.y;
    util_log("vehicle%d arrived arrived at horizontal target! target_local_pos_sp.z = %.2f, current_local_pos = %.2f", vehicle.drone_id,
             vehicle.target_local_pos_sp.pose.position.z, vehicle.current_local_pos.pose.position.z);

    float err_xy = sqrt(err_x * err_x + err_y * err_y);
    if (err_xy < 0.8) {
/*        util_log("vehicle%d arrived arrived at horizontal target! target_local_pos_sp.z = %.2f, current_local_pos = %.2f", vehicle.drone_id,
                 vehicle.target_local_pos_sp.pose.position.z, vehicle.current_local_pos.pose.position.z);*/
        return true;
    }
    return false;
}

void avoidance::checkVerticalDistance(const multi_vehicle_vec &vehicles) {
    float err_z12;
    err_z12 = fabs(multi_vehicle_.uav1_vec.back().current_local_pos.pose.position.z - multi_vehicle_.uav2_vec.back().current_local_pos.pose.position.z);

    if (err_z12 > K_avodiance_safe_pos_) {
        height_avoidance_uav2_.local_target_pos_avo.z() = 0;
        height_avoidance_uav1_.local_target_pos_avo.z() = 0;
    }
}

void avoidance::checkHorizontalDistance(const multi_vehicle_vec &vehicles) {
    if (distance_h_12_ > K_avodiance_safe_pos_) {
        height_avoidance_uav1_.local_target_pos_avo.z() = 0;
        height_avoidance_uav2_.local_target_pos_avo.z() = 0;
    }
}

void avoidance::Getvehicledistance(const M_Drone &vehicle1, const M_Drone &vehicle2, float &distance_h, float &distance) {
    GlobalPosition loc1,loc2;
    TVec3 two_uav_local_pos;
    loc1.latitude = vehicle1.latitude;
    loc1.longitude = vehicle1.longtitude;
    loc2.latitude = vehicle2.latitude;
    loc2.longitude = vehicle2.longtitude;

    Calculate::getInstance()->GetLocalPos(loc1, loc2, two_uav_local_pos);

    float err_px = two_uav_local_pos.x();
    float err_py = two_uav_local_pos.y();
    float err_pz = vehicle1.current_local_pos.pose.position.z - vehicle2.current_local_pos.pose.position.z;
    util_log("vehicle %d %d between length = %.2f, %.2f, %.2f, length = %.2f", vehicle1.drone_id, vehicle2.drone_id,
             err_px, err_py, err_pz, sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz));
//    util_log("distance = %.2f", sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz));

    distance = sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz);
    distance_h = sqrt(err_px * err_px + err_py * err_py);
    if (fabs(distance) > 1000) {
        util_log("error!");
    }
}

void avoidance::SetFunctionOutPut() {
    dataMan::getInstance()->SetAvoidanceData(height_avoidance_uav1_, height_avoidance_uav2_);
    DataMan::getInstance()->SetAvoidanceData(height_avoidance_uav1_, height_avoidance_uav2_, height_avoidance_uav1_ , height_avoidance_uav1_);
}
