//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_MULTUUVCONTROL_HPP
#define OFFBOARD_MULTUUVCONTROL_HPP

#include "Cinc.hpp"
#include "IVehicleControl.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"

class MultiUUVControl : public IVehicleControl {
public:
    MultiUUVControl();

    ~MultiUUVControl() {};

    void onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) override ;

    void DoProgress() override ;

    void chooseLeader() override ;

    void getData() override ;

    void setVehicleCtrlData() override ;

    static MultiUUVControl* getInstance();

private:
    bool pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos, float err_allow);

    geometry_msgs::PoseStamped CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target);

    void UUVManualControl();
    static MultiUUVControl* l_lint;

    multi_vehicle m_multi_vehicle_;
    vector<geometry_msgs::PoseStamped> uuv_way_points_;
    vector<geometry_msgs::PoseStamped> uuv_way_points_init_;
    int usv_state_;

    bool uuv1_reached_;
    bool is_formation_;

    geometry_msgs::PoseStamped target_pos_;
    geometry_msgs::PoseStamped body_pos_;
    float init_yaw_;
};

class MultiUUVControlFactory : public IVehicleControlFactory {
public:
    ~MultiUUVControlFactory() {};

    IVehicleControl* VehicleControlCreator() {
        util_log("UUV factory~");
        return  MultiUUVControl::getInstance();
    }
};

#endif //OFFBOARD_MULTDRONECONTROL_HPP
