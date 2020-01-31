//
// Created by zhouhua on 19-12-1.
//

#ifndef OFFBOARD_MULTI_FORMATION_HPP
#define OFFBOARD_MULTI_FORMATION_HPP

#include "Cinc.hpp"
#include "IControlFunction.hpp"

class MultiFormation : public IControlFunction{
public:
    MultiFormation();

    ~MultiFormation() {};

    void Oninit(const float config);

    void GetData() override ;

    void DoProgress() override ;

    void SetFunctionOutPut() override ;

    static MultiFormation* getInstance();

private:
    void calcFollowUAVPos(const M_Drone &follow_uav1, const M_Drone &follow_uav2, const M_Drone &follow_uav3,
                          TVec3 &follow_uav1_local_target, TVec3 &follow_uav2_local_target,
                          TVec3 &follow_uav3_local_target);


    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, Eigen::Matrix<float, 3, 1> formation_target);

    bool pos_reached(geometry_msgs::PoseStamped &current_pos, TVec3 &follow_uav_target);

    void OnCheckFormationArrived();

    static MultiFormation* multi_formation;

    multi_vehicle m_multi_vehicle_;

    bool is_formation_;
    int uav_formation_time_;
    int leader_uav_id_;
    int config_;
    M_Drone leader_drone_;

    TVec3 Drone_uav1_;
    TVec3 Drone_uav2_;
    TVec3 Drone_uav3_;
    TVec3 Drone_uav4_;

    TVec3 follow_uav1_keep_ = {0,0,0};
    TVec3 follow_uav2_keep_ = {0,0,0};
    TVec3 follow_uav3_keep_ = {0,0,0};
    TVec3 follow_uav4_keep_ = {0,0,0};

    TVec2 follow_uav1_to_leader_, follow_uav2_to_leader_, follow_uav3_to_leader_;
    TVec3 follow_uav1, follow_uav2, follow_uav3;

    TVec3 follow_uav1_;
    TVec3 follow_uav2_;
    TVec3 follow_uav3_;

    geometry_msgs::PoseStamped follow_uav1_to_leader, follow_uav2_to_leader, follow_uav3_to_leader;
    geometry_msgs::PoseStamped follow_uav1_to_leader_first, follow_uav2_to_leader_first, follow_uav3_to_leader_first;

    vector<M_Drone_Avoidace> drone_avoidance_{};
};

class MultiFormationFactory : public IFunctionFactory {
public:
    ~MultiFormationFactory() {};

    IControlFunction* FunctionCreator()  {
        return new MultiFormation();
    }
};
#endif //OFFBOARD_MULTI_FORMATION_HPP
