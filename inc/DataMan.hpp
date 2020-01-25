//
// Created by zhouhua on 2020/1/23.
//

#ifndef OFFBOARD_DATAMAN_HPP
#define OFFBOARD_DATAMAN_HPP

#include "Cinc.hpp"
class DataMan {
public:
    static DataMan* getInstance();

    void SetDroneData(const M_Drone &mDrone);

    void PrinrDorneFlightDate();

    void PrintBoatData();

    void PrintDroneTargetPosData();

    void PrintBoatTargetPosData();

    void PrintAvoidanceData();

    void PrintDroneFormationData();

    void PrintData();

    void
    SetFormationData(int leader_uav_id_, const TVec3& follow_uav1, const TVec3& follow_uav2, const TVec3& follow_uav3);

    void SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2, const M_Drone_Avoidace& uav3, const M_Drone_Avoidace& uav4);



    multi_vehicle &GetData();

private:
    static DataMan* l_singleton;

    multi_vehicle multi_vehicle_;
    vector<M_Drone> multi_drone_;
    boost::mutex m_mutex;

    int leader_uav_ = 0;
};


#endif //OFFBOARD_DATAMAN_HPP
