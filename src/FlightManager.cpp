//
// Created by zhouhua on 19-9-22.
//

#include "FlightManager.hpp"

FlightManager* FlightManager::l_pInst = NULL;

FlightManager::FlightManager() {

}

void FlightManager::DoPosUpdate(const M_Drone &mDrone, const int drone_id) {
//    util_log("drone_id = %d", drone_id);
    switch (drone_id) {
        case UAV1:{
            multi_vehicle_.uav1 = mDrone;
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

