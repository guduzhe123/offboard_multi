//
// Created by zhouhua on 2020/1/28.
//

#ifndef OFFBOARD_CALCULATE_HPP
#define OFFBOARD_CALCULATE_HPP

#include "Cinc.hpp"

class Calculate {
public:
    void GetLocalPos(const GlobalPosition &loc1, const GlobalPosition &loc2,
                     geometry_msgs::PoseStamped &follow_uav_local_pos);

    void GetGlobalPos(const GlobalPosition &loc1, GlobalPosition &loc2, TVec3 &local_target_pos);

    void setEachLoclation();

    double deg2rad(double deg);

    double rad2deg(double rad);

    void getMeterScaleHere(double &meterPerLatUnit, double &meterPerLongtUnit, const GlobalPosition &center_pos);

    double calcDist(const GlobalPosition &loc1, const GlobalPosition &loc2);

    static Calculate* getInstance();

private:

    static Calculate* l_pInst;
};
#endif //OFFBOARD_CALCULATE_HPP
