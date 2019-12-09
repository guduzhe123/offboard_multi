//
// Created by zhouhua on 19-9-22.
//

#include "FlightManager.hpp"

FlightManager* FlightManager::l_pInst = NULL;

FlightManager::FlightManager():
                m_drone_{}
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

void FlightManager::OnInit(const int config) {
    switch (config) {
        case VF_SQUARE_SMALL: {

        }
            break;

        case VF_TRIANGLE_SMALL: {

        }
            break;

        case VF_SQUARE_LARGE: {

        }
            break;

        case VF_TRIANGLE_LARGE: {

        }
            break;

        default:
            break;

    }
}

void FlightManager::ChooseUAVLeader() {

}

void FlightManager::ChooseUSVLeader() {

}



