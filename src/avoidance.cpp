//
// Created by zhouhua on 19-12-1.
//

#include "avoidance.hpp"

Avoidance* Avoidance::l_lptr = NULL;

Avoidance::Avoidance() :
        is_run_avoidance_(false),
        height_avoidance_uav1_{},
        height_avoidance_uav2_{},
        height_avoidance_uav3_{},
        height_avoidance_uav4_{}{

}

void Avoidance::Oninit() {

}

Avoidance* Avoidance::getInstance() {
    if (l_lptr == NULL) {
        l_lptr = new Avoidance();
    }
    return l_lptr;
}

void Avoidance::usv_avoidance() {

}

void Avoidance::uav_avoidance(const M_Drone &mDrone) {
    util_log("avo drone id = %d", mDrone.drone_id);
    switch (mDrone.drone_id) {
        case UAV1: {
            multi_vehicle_.uav1_vec.push_back(mDrone);
/*            util_log("uav1 drone local pos x = %.2f, y = %.2f, z = %.2f", mDrone.current_local_pos.pose.position.x,
                     mDrone.current_local_pos.pose.position.y, mDrone.current_local_pos.pose.position.z);
            util_log("uav1 drone gps lat = %.9f, longt = %.9f", mDrone.latitude, mDrone.longtitude);*/
        }
            break;
        case UAV2: {
            multi_vehicle_.uav2_vec.push_back(mDrone);
        }
            break;
        case UAV3: {
            multi_vehicle_.uav3_vec.push_back(mDrone);
        }
            break;
        case UAV4: {
            multi_vehicle_.uav4_vec.push_back(mDrone);
        }
            break;
        default:
            break;
    }

    if (!multi_vehicle_.uav1_vec.empty() && !multi_vehicle_.uav2_vec.empty() && !multi_vehicle_.uav3_vec.empty() &&
        !multi_vehicle_.uav4_vec.empty()) {
        // TODO need to use map.
        float avo_uav1, avo_uav2, avo_uav3, avo_uav4;
/*        height_avoidance_uav1_= {};
        height_avoidance_uav2_= {};
        height_avoidance_uav3_= {};
        height_avoidance_uav4_= {};*/

        checkDistance(multi_vehicle_.uav1_vec.back(), multi_vehicle_.uav2_vec.back(), avo_uav1, avo_uav2);
        height_avoidance_uav1_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav2_.local_target_pos_avo.z() += avo_uav2;

        checkDistance(multi_vehicle_.uav1_vec.back(), multi_vehicle_.uav3_vec.back(), avo_uav1, avo_uav3);
        height_avoidance_uav1_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav3_.local_target_pos_avo.z() += avo_uav3;

        checkDistance(multi_vehicle_.uav1_vec.back(), multi_vehicle_.uav4_vec.back(), avo_uav1, avo_uav4);
        height_avoidance_uav1_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav4_.local_target_pos_avo.z() += avo_uav4;

        checkDistance(multi_vehicle_.uav2_vec.back(), multi_vehicle_.uav3_vec.back(), avo_uav2, avo_uav3);
        height_avoidance_uav2_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav3_.local_target_pos_avo.z() += avo_uav4;

        checkDistance(multi_vehicle_.uav2_vec.back(), multi_vehicle_.uav4_vec.back(), avo_uav2, avo_uav4);
        height_avoidance_uav2_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav4_.local_target_pos_avo.z() += avo_uav4;

        checkDistance(multi_vehicle_.uav3_vec.back(), multi_vehicle_.uav4_vec.back(), avo_uav3, avo_uav4);
        height_avoidance_uav3_.local_target_pos_avo.z() += avo_uav1;
        height_avoidance_uav4_.local_target_pos_avo.z() += avo_uav4;

        is_run_avoidance_ = true;
        util_log(
                "height_avoidance_uav1_ = %.2f, height_avoidance_uav2_ = %.2f, height_avoidance_uav3_ = %.2f, height_avoidance_uav4_ = %.2f",
                height_avoidance_uav1_.local_target_pos_avo.z(), height_avoidance_uav2_.local_target_pos_avo.z(),
                height_avoidance_uav3_.local_target_pos_avo.z(), height_avoidance_uav4_.local_target_pos_avo.z());

    }

}

void Avoidance::checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2,
                              float &m_drone_avoidance1, float &m_drone_avoidance2) {

    GlobalPosition loc1,loc2;
    geometry_msgs::PoseStamped two_uav_local_pos;
    loc1.latitude = vehicle1.latitude;
    loc1.longitude = vehicle1.longtitude;
    loc2.latitude = vehicle2.latitude;
    loc2.longitude = vehicle2.longtitude;

    MultiFormation::getInstance()->GetLocalPos(loc1, loc2, two_uav_local_pos);

    float err_px = two_uav_local_pos.pose.position.x;
    float err_py = two_uav_local_pos.pose.position.y;
    float err_pz = vehicle1.current_local_pos.pose.position.z - vehicle2.current_local_pos.pose.position.z;
    util_log("vehicle 1 2 between length = %.2f, %.2f, %.2f", err_px, err_py, err_pz);
    util_log("distance = %.2f", sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz));

    if (sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < 2 ) {
        util_log("distance < 2!!!");
        // seems to be velocity.
        if (vehicle1.current_local_pos.pose.position.z > vehicle2.current_local_pos.pose.position.z) {
            m_drone_avoidance1 = 0.5;
            m_drone_avoidance2 = -0.5;
        } else {
            m_drone_avoidance1 = -0.5;
            m_drone_avoidance2 = 0.5;
        }
    } else {
        m_drone_avoidance1 = 0;
        m_drone_avoidance2 = 0;
    }

}

void Avoidance::get_uav_avo_output(vector<M_Drone_Avoidace> &m_drone_avoidance) {
    m_drone_avoidance.clear();
    m_drone_avoidance.push_back(height_avoidance_uav1_);
    m_drone_avoidance.push_back(height_avoidance_uav2_);
    m_drone_avoidance.push_back(height_avoidance_uav3_);
    m_drone_avoidance.push_back(height_avoidance_uav4_);

    util_log(
            "get output height_avoidance_uav1_ = %.2f, height_avoidance_uav2_ = %.2f, height_avoidance_uav3_ = %.2f, height_avoidance_uav4_ = %.2f",
            height_avoidance_uav1_.local_target_pos_avo.z(), height_avoidance_uav2_.local_target_pos_avo.z(),
            height_avoidance_uav3_.local_target_pos_avo.z(), height_avoidance_uav4_.local_target_pos_avo.z());
            util_log("get output height_avoidance_uav1_ = %.2f, height_avoidance_uav2_ = %.2f, height_avoidance_uav3_ = %.2f,"
                     " height_avoidance_uav4_ = %.2f",
                     m_drone_avoidance[0].local_target_pos_avo.z(), m_drone_avoidance[1].local_target_pos_avo.z(),
                     m_drone_avoidance[2].local_target_pos_avo.z(), m_drone_avoidance[3].local_target_pos_avo.z());

}

void Avoidance::checkHorizontalDistance(const M_Drone &vehicle1, const M_Drone &vehicle2) {

}
