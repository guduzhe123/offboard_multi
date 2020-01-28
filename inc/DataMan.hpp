//
// Created by zhouhua on 2020/1/23.
//

#ifndef OFFBOARD_DATAMAN_HPP
#define OFFBOARD_DATAMAN_HPP

#include "Cinc.hpp"
class DataMan {
public:
    static DataMan* getInstance();

    void OnInit();

    void SetDroneData(const M_Drone &mDrone);

    void PrinrDorneFlightDate();

    void PrintBoatData();

    void PrintDroneTargetPosData();

    void PrintBoatTargetPosData();

    void PrintAvoidanceData();

    void PrintDroneFormationData();

    void PrintDroneFormationKeep();

    void PrintData();

    void
    SetFormationData(int leader_uav_id_, const TVec3& follow_uav1, const TVec3& follow_uav2, const TVec3& follow_uav3);

    void
    SetFormationKeepData(const TVec3& follow_uav1, const TVec3& follow_uav2, const TVec3& follow_uav3, const TVec3& follow_uav4);


    void SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2, const M_Drone_Avoidace& uav3, const M_Drone_Avoidace& uav4);

    void SetDroneControlData(const multi_vehicle &m_multi_vehicles);

    void SetBoatControlData(const multi_vehicle &m_multi_vehicles);

    void SetUAVState(mavros_msgs::SetMode &m_mode);

    void SetUSVState(mavros_msgs::CommandBool &arm_command, const int usv_id);

    void SetUAVLeader(M_Drone &leader_uav);

    void SetUSVLeader(M_Drone &leader_usv);

    multi_vehicle &GetData();

private:
    static DataMan* l_singleton;

    multi_vehicle multi_vehicle_;
    vector<M_Drone> multi_drone_;
    boost::mutex m_mutex;

    int leader_uav_ = 0;
};


#endif //OFFBOARD_DATAMAN_HPP
