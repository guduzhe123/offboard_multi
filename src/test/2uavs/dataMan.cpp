//
// Created by zhouhua on 2020/5/13.
//

#include "test/2uav2/dataMan.hpp"

dataMan* dataMan::l_singleton = NULL;
dataMan* dataMan::getInstance() {
    if (l_singleton == NULL) {
        l_singleton = new dataMan();
    }
    return l_singleton;
}

dataMan::dataMan() :
        command_(-1){

}

void dataMan::SetDroneData(const M_Drone &mDrone) {
    {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        switch (mDrone.drone_id) {
            case UAV1: {
                multi_vehicle_.uav1 = mDrone;
            }
                break;
            case UAV2: {
                multi_vehicle_.uav2 = mDrone;
            }
                break;
            default:
                break;
        }
    }
}

void
dataMan::SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);

        multi_vehicle_.uav1.avoidance_pos = uav1.local_target_pos_avo;
        multi_vehicle_.uav2.avoidance_pos = uav2.local_target_pos_avo;
    }
}

void dataMan::SetDroneControlData(const multi_vehicle &m_multi_vehicles) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.uav1.target_local_pos_sp = m_multi_vehicles.uav1.target_local_pos_sp;
        multi_vehicle_.uav2.target_local_pos_sp = m_multi_vehicles.uav2.target_local_pos_sp;
    }

}

void dataMan::setCommand(int command) {
    command_ = command;
}

multi_vehicle &dataMan::GetData() {
    return multi_vehicle_;
}

void dataMan::getCommand(int& command) {
    command = command_;
}


