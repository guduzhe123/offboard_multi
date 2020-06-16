//
// Created by zhouhua on 19-12-1.
//

#include "test/3USVs/avoidance.hpp"

avoidance* avoidance::l_lptr = NULL;

avoidance::avoidance() :
        is_run_avoidance_(false),
        height_avoidance_usv1_{},
        height_avoidance_usv2_{},
        height_avoidance_usv3_{},
        config_(0),
        formation_distance_(5){

}

void avoidance::Oninit() {
    ros::NodeHandle nh("~");
    nh.param<double >("formation_distance", formation_distance_, 5);
}

avoidance* avoidance::getInstance() {
    if (l_lptr == NULL) {
        l_lptr = new avoidance();
    }
    return l_lptr;
}

void avoidance::GetData() {
    m_multi_vehicle_ = dataMan::getInstance()->GetData();
}

void avoidance::DoProgress() {
    // TODO need to use map.
    float avo_usv1, avo_usv2, avo_usv3;

    checkDistance(m_multi_vehicle_.usv1, m_multi_vehicle_.usv2, avo_usv1, avo_usv2);
    height_avoidance_usv1_.local_target_pos_avo.z() = avo_usv1;
    height_avoidance_usv2_.local_target_pos_avo.z() = avo_usv2;
    height_avoidance_usv2_.local_target_pos_avo.z() = avo_usv2; //heightVar = distance

    //write log
    if (fabs(avo_usv1) > 0.01 && fabs(avo_usv2) > 0.01)
        util_log("distance between and usv1 and usv2 < 2m, asv5 move = %.2f, asv6 move = %.2f", avo_usv1, avo_usv2);

    if (fabs(avo_usv1) > 0.01 && fabs(avo_usv3) > 0.01)
        util_log("distance between and usv1 and usv2 < 2m, asv5 move = %.2f, asv7 move = %.2f", avo_usv1, avo_usv3);

    if (fabs(avo_usv1) > 0.01 && fabs(avo_usv2) > 0.01)
        util_log("distance between and usv2 and usv7 < 2m, asv6 move = %.2f, asv7 move = %.2f", avo_usv2, avo_usv3);


//Based distance to do process
    if (fabs(height_avoidance_usv1_.local_target_pos_avo.z()) > K_max_avodiance_pos_) {
        height_avoidance_usv1_.local_target_pos_avo.z() = K_max_avodiance_pos_ *
                                                          fabs(height_avoidance_usv1_.local_target_pos_avo.z()) / height_avoidance_usv1_.local_target_pos_avo.z();
    }
    if (fabs(height_avoidance_usv2_.local_target_pos_avo.z()) > K_max_avodiance_pos_) {
        height_avoidance_usv2_.local_target_pos_avo.z() = K_max_avodiance_pos_ *
                                                          fabs(height_avoidance_usv2_.local_target_pos_avo.z()) / height_avoidance_usv2_.local_target_pos_avo.z();
    }
    if (fabs(height_avoidance_usv3_.local_target_pos_avo.z()) > K_max_avodiance_pos_) {
        height_avoidance_usv2_.local_target_pos_avo.z() = K_max_avodiance_pos_ *
                                                          fabs(height_avoidance_usv3_.local_target_pos_avo.z()) / height_avoidance_usv3_.local_target_pos_avo.z();
    }


    is_run_avoidance_ = true;

    checkVerticalDistance(m_multi_vehicle_);
    checkHorizontalDistance(m_multi_vehicle_);
    util_log(
            "height_avoidance_usv1_ = %.2f, height_avoidance_usv2_ = %.2f, height_avoidance_usv2_ = %.2f",
            height_avoidance_usv1_.local_target_pos_avo.z(),
            height_avoidance_usv2_.local_target_pos_avo.z(),
            height_avoidance_usv3_.local_target_pos_avo.z());

    SetFunctionOutPut();

}

void avoidance::checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2, 
                              float &m_drone_avoidance1, float &m_drone_avoidance2) {

    if (fabs(vehicle1.latitude) < 0.1 || fabs(vehicle2.latitude /*|| fabs(vehicle3.latitude) < 0.1*/) {
        return;
    }

    float  dist, distance_h;

    Getvehicledistance(vehicle1, vehicle2, distance_h, dist);
    Getvehicledistance(vehicle2, vehicle3, distance_h, dist);
    Getvehicledistance(vehicle1, vehicle3, distance_h, dist);

    distance_h_12_ = distance_h;

    if (dist < formation_distance_ / 2 ) {
        util_log("vehlcie %d and %d distance < %.2f!!! distance = %.2f", formation_distance_ / 2 , vehicle1.drone_id, vehicle2.drone_id, dist);
        util_log("usv1 target z = %.2f, usv2 target z = %.2f", vehicle1.target_local_pos_sp.pose.position.z, vehicle2.target_local_pos_sp.pose.position.z );
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

void avoidance::checkVerticalDistance(const multi_vehicle &vehicles) {
    float err_z12;
    err_z12 = fabs(m_multi_vehicle_.usv1.current_local_pos.pose.position.z - m_multi_vehicle_.usv2.current_local_pos.pose.position.z);

    if (err_z12 >  formation_distance_ / 2) {
        height_avoidance_usv2_.local_target_pos_avo.z() = 0;
        height_avoidance_usv1_.local_target_pos_avo.z() = 0;
    }
}

void avoidance::checkHorizontalDistance(const multi_vehicle &vehicles) {
    if (distance_h_12_ >  formation_distance_ / 2) {
        height_avoidance_usv1_.local_target_pos_avo.z() = 0;
        height_avoidance_usv2_.local_target_pos_avo.z() = 0;
    }
}

void avoidance::Getvehicledistance(const M_Drone &vehicle1, const M_Drone &vehicle2, float &distance_h, float &distance) {
    GlobalPosition loc1,loc2;
    TVec3 two_usv_local_pos;
    loc1.latitude = vehicle1.latitude;
    loc1.longitude = vehicle1.longtitude;
    loc2.latitude = vehicle2.latitude;
    loc2.longitude = vehicle2.longtitude;

    Calculate::getInstance()->GetLocalPos(loc1, loc2, two_usv_local_pos);

    float err_px = two_usv_local_pos.x();
    float err_py = two_usv_local_pos.y();
//    float err_pz = vehicle1.current_local_pos.pose.position.z - vehicle2.current_local_pos.pose.position.z;
//    util_log("vehicle %d %d between length = %.2f, %.2f, %.2f, length = %.2f", vehicle1.drone_id, vehicle2.drone_id,
//             err_px, err_py, err_pz, sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz));

    util_log("vehicle %d %d between length = %.2f, %.2f, %.2f, length = %.2f", vehicle1.drone_id, vehicle2.drone_id,
             err_px, err_py, err_pz, sqrt(err_px * err_px + err_py * err_py));

//    util_log("distance = %.2f", sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz));
    util_log("distance = %.2f", sqrt(err_px * err_px + err_py * err_py));

//    distance = sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz);
    distance = sqrt(err_px * err_px + err_py * err_py);
//    distance_h = sqrt(err_px * err_px + err_py * err_py);
    distance_h=0;
    if (fabs(distance) > 1000) {
        util_log("error!");
    }
}

void avoidance::SetFunctionOutPut() {
    dataMan::getInstance()->SetAvoidanceData(height_avoidance_usv1_, height_avoidance_usv2_);
    DataMan::getInstance()->SetAvoidanceData(height_avoidance_usv1_, height_avoidance_usv2_, height_avoidance_usv1_ , height_avoidance_usv1_);
}

void avoidance::checkCollision(bool &is_collision, bool &is_avodiance, double danger_distance) {
    float  dist, distance_h;
    Getvehicledistance(m_multi_vehicle_.usv1, m_multi_vehicle_.usv2, distance_h, dist);
    is_collision = dist < danger_distance;
    is_avodiance = dist <= formation_distance_ / 2;
}