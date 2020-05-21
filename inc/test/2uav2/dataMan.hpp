//
// Created by zhouhua on 2020/5/13.
//

#ifndef OFFBOARD_dATAMAN_HPP
#define OFFBOARD_dATAMAN_HPP

#include "Cinc.hpp"
class dataMan {
public:
    dataMan();
    ~dataMan() = default;
    void SetDroneData(const M_Drone &mDrone);
    void SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2);
    multi_vehicle &GetData();
    static dataMan* getInstance();

private:
    multi_vehicle multi_vehicle_;
    boost::mutex m_mutex;
    static dataMan* l_singleton;
};

#endif //OFFBOARD_dATAMAN_HPP
