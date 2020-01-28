//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_FLIGHTMANAGER_HPP
#define OFFBOARD_FLIGHTMANAGER_HPP

#include "Cinc.hpp"

#include "Multi_formation.hpp"
#include "Avoidance.hpp"
#include "IMsgRosManager.hpp"

class FlightManager {
public:

    FlightManager();
    ~FlightManager() = default;

    void OnInitConfig(IMsgRosManager* msg_manager);

    void DoPosUpdate();

    static FlightManager* getInstance();

    void OnCheckFormationArrived();

private:

    M_Drone leader_drone_;

    u_int leader_uav_id_;
    int leader_usv_id_;
    int uav_formation_time_;

    bool is_formation_;

    multi_vehicle multi_vehicle_;
    static FlightManager* l_pInst;


    IMsgRosManager* msg_manager_;
};

#endif //OFFBOARD_FLIGHTMANAGER_HPP
