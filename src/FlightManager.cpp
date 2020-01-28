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
//    Avoidance::getInstance()->DoPosUpdate();
}

FlightManager* FlightManager::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new FlightManager();
    }
    return l_pInst;
}

void FlightManager::OnInitConfig(IMsgRosManager *msg_manager) {
    msg_manager_ = msg_manager;

}




