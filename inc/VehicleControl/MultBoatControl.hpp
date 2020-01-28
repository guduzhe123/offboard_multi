//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_MULTBOATCONTROL_HPP
#define OFFBOARD_MULTBOATCONTROL_HPP

#include "Cinc.hpp"
#include "IVehicleControl.hpp"
#include "DataMan.hpp"
static const float usv_position_allow_reached_ = 3;

class MultiBoatControl : public IVehicleControl {
public:
    MultiBoatControl();

    ~MultiBoatControl();

    void onInit(vector<geometry_msgs::PoseStamped> way_points) override ;

    void DoProgress() override ;

    void chooseLeader() override ;

    void getData() override ;

    void setVehicleCtrlData() override ;

private:
    bool pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos, float err_allow);


    multi_vehicle m_multi_vehicle_;
    vector<geometry_msgs::PoseStamped> usv_way_points_;
    int usv_state_;

    bool usv5_reached_;
    bool usv6_reached_;
    bool usv7_reached_;
};

#endif //OFFBOARD_MULTDRONECONTROL_HPP
