//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_FLIGHTMANAGER_HPP
#define OFFBOARD_FLIGHTMANAGER_HPP

#include "Cinc.hpp"
#include "Multi_formation.hpp"
#include <geometry_msgs/PoseStamped.h>
//#include "multi_offboard.hpp"

enum {
    UAV1 = 1,
    UAV2,
    UAV3,
    UAV4,
    USV1,
    USV2,
    USV3,
    UUV1,
    UUV2,
    UUV3
};

enum vehicle_formation {
    VF_SQUARE_SMALL,
    VF_TRIANGLE_SMALL,
    VF_SQUARE_LARGE,
    VF_TRIANGLE_LARGE
};

class FlightManager {
public:
    struct M_Drone {
        TVec3 velocity;
        float pitch ;
        float roll ;
        float yaw ;
        int drone_id;
        double latitude;
        double longtitude;
        double altitude;
        geometry_msgs::PoseStamped current_local_pos;
    };

    struct multi_vehicle{
        M_Drone uav1;
        M_Drone uav2;
        M_Drone uav3;
        M_Drone uav4;
        M_Drone usv1;
        M_Drone usv2;
        M_Drone usv3;
        M_Drone uuv1;
        M_Drone uuv2;
        M_Drone uuv3;
    };
    FlightManager();
    ~FlightManager() = default;

    void OnInit(const int config);

    void DoPosUpdate(const M_Drone &mDrone);

    void ChooseUAVLeader();

    void ChooseUSVLeader();

    static FlightManager* getInstance();

private:

    M_Drone m_drone_;

    multi_vehicle multi_vehicle_;
    static FlightManager* l_pInst;
};

#endif //OFFBOARD_FLIGHTMANAGER_HPP
