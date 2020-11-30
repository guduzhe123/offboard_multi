//
// Created by zhouhua on 19-12-1.
//

#ifndef OFFBOARD_MULTI_USV_FORMATION_HPP
#define OFFBOARD_MULTI_USV_FORMATION_HPP

#include "Cinc.hpp"
#include "IControlFunction.hpp"

class MultiUSVFormation : public IControlFunction{
public:
    MultiUSVFormation();

    ~MultiUSVFormation() {};

    void Oninit(const int config) override ;

    void GetData() override ;

    void DoProgress() override ;

    void SetFunctionOutPut() override ;

    static MultiUSVFormation* getInstance();

private:

    void calcFollowUSVPos();

    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, Eigen::Matrix<float, 3, 1> formation_target);

    bool pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &follow_usv_target, float err_allow);

    void OnCheckFormationArrived();

    void GetTakeoffPos();

    void changeToLocalTarget();

    static MultiUSVFormation* multi_formation;

    multi_vehicle m_multi_vehicle_;

    bool is_formation_;
    bool is_get_takeoff_pos_;
    bool usv1_reached_;
    bool usv2_reached_;
    bool usv3_reached_;
    int leader_usv_id_;
    int config_;
    M_Drone leader_drone_;

    TVec3 leader_curr_pos_, leader_cur_target_;
    TVec3 Drone_usv1_, Drone_usv2_, Drone_usv3_, Drone_usv4_;
    TVec3 follow_usv1_;
    TVec3 follow_usv2_;
    TVec3 follow_usv3_;

    TVec3 follow_usv1_to_leader, follow_usv2_to_leader, follow_usv3_to_leader;
    TVec3 follow_usv1_first_local_, follow_usv2_first_local_, follow_usv3_first_local_;
    TVec3 follow_usv1_keep_local_, follow_usv2_keep_local_, follow_usv3_keep_local_;

    GlobalPosition usv1_takeoff_gps_pos_, usv2_takeoff_gps_pos_, usv3_takeoff_gps_pos_, usv4_takeoff_gps_pos_;


};

class MultiUSVFormationFactory : public IFunctionFactory {
public:
    ~MultiUSVFormationFactory() {};

    IControlFunction* FunctionCreator()  {
        return MultiUSVFormation::getInstance();
    }
};
#endif //OFFBOARD_MULTI_usv_FORMATION_HPP
