//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_MULTDRONECONTROL_HPP
#define OFFBOARD_MULTDRONECONTROL_HPP

#include "Cinc.hpp"
#include "IVehicleControl.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"
#include "ActionCircle.hpp"

static const float K_pos_target_arrived_len_ = 0.8;

class MultiDroneControl : public IVehicleControl {
public:
    MultiDroneControl();

    ~MultiDroneControl() {};

    void onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) override ;

    void DoProgress() override;

    void chooseLeader() override;

    void getData() override ;

    void setVehicleCtrlData() override ;

    static MultiDroneControl* getInstance();

private:
    bool pos_reached(M_Drone &current_drone, geometry_msgs::PoseStamped &target_pos,
                     float err_allow);

    void droneManualControl();

    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target);

    static MultiDroneControl* l_lint;

    int config_;
    int uav_state_;
    bool is_formation_;
    multi_vehicle m_multi_vehicle_;
    M_Drone drone_uav_leader_;
    geometry_msgs::PoseStamped target_pos_;
    geometry_msgs::PoseStamped body_pos_;
    vector<geometry_msgs::PoseStamped> uav_way_points_;
    vector<geometry_msgs::PoseStamped> uav_way_points_init_;
    vector<geometry_msgs::PoseStamped> uav_way_points_body_;
    TVec3 follow_slave_first_local_;

    bool is_uav_follow_;
    bool state_changed_;

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
