//
// Created by zhouhua on 2020/5/13.
//

#ifndef OFFBOARD_dATAMAN_HPP
#define OFFBOARD_dATAMAN_HPP

#include "Cinc.hpp"
enum UAVCOMMAND {
    UAVS_START,
    ALLSTOP,
    SLAVEBACKX,
    SLAVEFORWARDX,
    SLAVEBACKY,
    SLAVEFORWARDY,
    MASTERSTART,
    SLAVESTART,
    ALLRETURN,
    ALLLAND
};

class dataMan {
public:
    dataMan();
    ~dataMan() = default;
    void SetDroneData(const M_Drone &mDrone);
    void SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2);
    void SetDroneControlData(const multi_vehicle &m_multi_vehicles);
    multi_vehicle &GetData();
    static dataMan* getInstance();
    void setCommand(int command);
    void getCommand(int& command);

private:
    multi_vehicle multi_vehicle_;
    boost::mutex m_mutex;
    static dataMan* l_singleton;
    int command_;
};

#endif //OFFBOARD_dATAMAN_HPP
