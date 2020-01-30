//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_MULTDRONECONTROL_HPP
#define OFFBOARD_MULTDRONECONTROL_HPP

#include "Cinc.hpp"
#include "IVehicleControl.hpp"
#include "DataMan.hpp"

class MultiDroneControl : public IVehicleControl {
public:
    MultiDroneControl();

    ~MultiDroneControl() {};

    void onInit(vector<geometry_msgs::PoseStamped> way_points) override ;

    void DoProgress() override;

    void chooseLeader() override;

    void getData() override ;

    void setVehicleCtrlData() override ;

    static MultiDroneControl* getInstance();

private:
    bool pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos, float err_allow);

    void droneManualControl();

    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target);

    static MultiDroneControl* l_lint;

    int uav_state_;
    multi_vehicle m_multi_vehicle_;
    M_Drone drone_uav_leader_;
    geometry_msgs::PoseStamped target_pos_;
    vector<geometry_msgs::PoseStamped> uav_way_points_;

    TVec3 follow_uav1_keep_ = {0,0,0};
    TVec3 follow_uav2_keep_ = {0,0,0};
    TVec3 follow_uav3_keep_ = {0,0,0};
    TVec3 follow_uav4_keep_ = {0,0,0};
};

class MultiDroneControlFactory : public IVehicleControlFactory {
public:
    ~MultiDroneControlFactory() {};

    IVehicleControl* VehicleControlCreator() override {
        util_log("vehicle factory~");
        return MultiDroneControl::getInstance();
    }
};

#endif //OFFBOARD_MULTDRONECONTROL_HPP
