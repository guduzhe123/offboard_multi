//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_MULTBOATCONTROL_HPP
#define OFFBOARD_MULTBOATCONTROL_HPP

#include "Cinc.hpp"
#include "IVehicleControl.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"
#include "ActionCircle.hpp"
#include "PathCreator.hpp"

class MultiBoatControl : public IVehicleControl {
public:
    MultiBoatControl();

    ~MultiBoatControl() {};

    void onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) override ;

    void DoProgress() override ;

    void chooseLeader() override ;

    void getData() override ;

    void setVehicleCtrlData() override ;

    static MultiBoatControl* getInstance();

private:
    bool pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos, float err_allow);

    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target);

    void changeToLocalTarget();
    void GetTakeoffPos();
    void calcFollowUUVPos();
    void SetFunctionOutPut();

    void USVManualControl();
    static MultiBoatControl* l_lint;

    multi_vehicle m_multi_vehicle_;
    vector<geometry_msgs::PoseStamped> usv_way_points_;
    vector<geometry_msgs::PoseStamped> uav_way_points_init_;
    int usv_state_;
    int config_;
    int formation_config_;
    int usv_waypoints_size_init_;

    bool usv1_reached_;
    bool usv2_reached_;
    bool usv3_reached_;
    bool is_formation_;
    bool update_takeoff_;
    bool state_changed_;
    float way_bear_;
    TVec3 follow_usv1_, follow_usv2_, follow_usv3_;

    GlobalPosition usv1_takeoff_gps_pos_, usv2_takeoff_gps_pos_, usv3_takeoff_gps_pos_, uuv1_takeoff_gps_pos_;
    TVec3 follow_usv1_to_uuv1_, follow_usv2_to_uuv1_, follow_usv3_to_uuv1_;
    TVec3 target_usv1_, target_usv2_, target_usv3_;
    TVec3 follow_usv1_keep_local_, follow_usv2_keep_local_, follow_usv3_keep_local_;
    geometry_msgs::PoseStamped target_pos_;
    geometry_msgs::PoseStamped body_pos_;
    float init_yaw_;

    geometry_msgs::PoseStamped target_usv2_init_;
    geometry_msgs::PoseStamped target_usv3_init_;

};

class MultiBoatControlFactory : public IVehicleControlFactory {
public:
    ~MultiBoatControlFactory() {};

    IVehicleControl* VehicleControlCreator() {
        util_log("boat factory~");
        return  MultiBoatControl::getInstance();
    }
};

#endif //OFFBOARD_MULTDRONECONTROL_HPP
