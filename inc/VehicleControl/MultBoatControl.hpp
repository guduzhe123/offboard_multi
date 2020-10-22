//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_MULTBOATCONTROL_HPP
#define OFFBOARD_MULTBOATCONTROL_HPP

#include "Cinc.hpp"
#include "IVehicleControl.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"

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

    void USVManualControl();
    static MultiBoatControl* l_lint;

    multi_vehicle m_multi_vehicle_;
    vector<geometry_msgs::PoseStamped> usv_way_points_;
    vector<geometry_msgs::PoseStamped> uav_way_points_init_;
    int usv_state_;

    bool usv1_reached_;
    bool usv2_reached_;
    bool usv3_reached_;
    bool is_formation_;

    geometry_msgs::PoseStamped target_pos_;
    geometry_msgs::PoseStamped body_pos_;
    float init_yaw_;
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
