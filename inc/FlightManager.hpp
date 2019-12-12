//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_FLIGHTMANAGER_HPP
#define OFFBOARD_FLIGHTMANAGER_HPP

#include "Cinc.hpp"
#include "Multi_formation.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
//#include "multi_offboard.hpp"

static const bool K_Param_local_global = true;
static const float K_err_allow = 0.8;

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
    VF_SQUARE,
    VF_TRIANGLE,
    VF_LINE_HORIZONTAL,
    VF_LINE_VERTICAL
};

class FlightManager {
public:
    struct M_Drone {
        int drone_id;
        TVec3 velocity;
        float pitch ;
        float roll ;
        float yaw ;
        double latitude;
        double longtitude;
        double altitude;
        geometry_msgs::PoseStamped current_local_pos;
        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped target_local_pos_sp;
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

    void ChooseUAVLeader(int &leader_uav_id);

    void ChooseUSVLeader(int &leader_usv_id);

    void
    GetFormationOutput(geometry_msgs::PoseStamped &follow_uav_num1,
                       geometry_msgs::PoseStamped &follow_uav_num2,
                       geometry_msgs::PoseStamped &follow_uav_num3, bool &is_formation);

    static FlightManager* getInstance();

    void OnCheckFormationArrived();

private:

    void calcFollowUAVPos(const M_Drone &follow_uav1, const M_Drone &follow_uav2, const M_Drone &follow_uav3,
                          TVec3 &follow_uav1_local_target, TVec3 &follow_uav2_local_target,
                          TVec3 &follow_uav3_local_target);


    bool pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &follow_uav_target);

    M_Drone leader_drone_;

    int leader_uav_id_;
    int leader_usv_id_;

    bool is_formation_;

    multi_vehicle multi_vehicle_;
    static FlightManager* l_pInst;

    TVec3 Drone_uav1_;
    TVec3 Drone_uav2_;
    TVec3 Drone_uav3_;
    TVec3 Drone_uav4_;


    geometry_msgs::PoseStamped follow_uav1_;
    geometry_msgs::PoseStamped follow_uav2_;
    geometry_msgs::PoseStamped follow_uav3_;

    geometry_msgs::PoseStamped follow_uav1_to_leader, follow_uav2_to_leader, follow_uav3_to_leader;
};

#endif //OFFBOARD_FLIGHTMANAGER_HPP
