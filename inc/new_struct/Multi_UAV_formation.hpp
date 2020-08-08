//
// Created by zhouhua on 19-12-1.
//

#ifndef OFFBOARD_MULTI_UAV_FORMATION_HPP
#define OFFBOARD_MULTI_UAV_FORMATION_HPP

#include "Cinc.hpp"
#include "IControlFunction.hpp"

class MultiUAVFormation : public IControlFunction{
public:
    MultiUAVFormation();

    ~MultiUAVFormation() {};

    void Oninit(const int config) override ;

    void GetData() override ;

    void DoProgress() override ;

    void SetFunctionOutPut() override ;

    static MultiUAVFormation* getInstance();

private:

    void calcFollowUAVPos();

    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, Eigen::Matrix<float, 3, 1> formation_target);

    bool pos_reached(geometry_msgs::PoseStamped &current_pos, TVec3 &follow_uav_target);

    void OnCheckFormationArrived();

    void GetTakeoffPos();

    static MultiUAVFormation* multi_formation;

    multi_vehicle m_multi_vehicle_;

    int config_;
    bool is_formation_;
    bool is_get_takeoff_pos_;
    int leader_uav_id_;
    M_Drone leader_drone_;

    TVec3 leader_curr_pos_;
    TVec3 Drone_uav1_, Drone_uav2_, Drone_uav3_, Drone_uav4_;
    TVec3 follow_uav1_;
    TVec3 follow_uav2_;
    TVec3 follow_uav3_;

    TVec3 follow_uav1_to_leader, follow_uav2_to_leader, follow_uav3_to_leader;
    TVec3 follow_uav1_first_local_, follow_uav2_first_local_, follow_uav3_first_local_;
    TVec3 follow_uav1_keep_local_, follow_uav2_keep_local_, follow_uav3_keep_local_;

    GlobalPosition uav1_takeoff_gps_pos_, uav2_takeoff_gps_pos_, uav3_takeoff_gps_pos_, uav4_takeoff_gps_pos_;


};

class MultiUAVFormationFactory : public IFunctionFactory {
public:
    ~MultiUAVFormationFactory() {};

    IControlFunction* FunctionCreator()  {
        return MultiUAVFormation::getInstance();
    }
};
#endif //OFFBOARD_MULTI_UAV_FORMATION_HPP
