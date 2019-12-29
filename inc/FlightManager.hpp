//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_FLIGHTMANAGER_HPP
#define OFFBOARD_FLIGHTMANAGER_HPP

#include "Cinc.hpp"

#include "Multi_formation.hpp"
#include "avoidance.hpp"

static const bool K_Param_local_global = true;
static const float K_err_allow = 0.8;


enum vehicle_formation {
    VF_SQUARE,
    VF_TRIANGLE,
    VF_LINE_HORIZONTAL,
    VF_LINE_VERTICAL
};

class FlightManager {
public:

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

    void GetKeepFormation(TVec2 &follow_uav1_keep, TVec2 &follow_uav2_keep, TVec2 &follow_uav3_keep,
                          TVec2 &follow_uav4_keep);

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
    int uav_formation_time_;

    bool is_formation_;

    multi_vehicle multi_vehicle_;
    static FlightManager* l_pInst;

    TVec3 Drone_uav1_;
    TVec3 Drone_uav2_;
    TVec3 Drone_uav3_;
    TVec3 Drone_uav4_;

    TVec2 follow_uav1_keep_ = {0,0};
    TVec2 follow_uav2_keep_ = {0,0};
    TVec2 follow_uav3_keep_ = {0,0};
    TVec2 follow_uav4_keep_ = {0,0};

    TVec2 follow_uav1_to_leader_, follow_uav2_to_leader_, follow_uav3_to_leader_;

    geometry_msgs::PoseStamped follow_uav1_;
    geometry_msgs::PoseStamped follow_uav2_;
    geometry_msgs::PoseStamped follow_uav3_;

    geometry_msgs::PoseStamped follow_uav1_to_leader, follow_uav2_to_leader, follow_uav3_to_leader;
    geometry_msgs::PoseStamped follow_uav1_to_leader_first, follow_uav2_to_leader_first, follow_uav3_to_leader_first;

    vector<M_Drone_Avoidace> drone_avoidance_{};
};

#endif //OFFBOARD_FLIGHTMANAGER_HPP
