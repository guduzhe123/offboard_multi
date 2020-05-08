//
// Created by zhouhua on 2020/1/28.
//

#ifndef OFFBOARD_CALCULATE_HPP
#define OFFBOARD_CALCULATE_HPP

#include "Cinc.hpp"
#define M_DEG_TO_RAD 		0.017453292519943295
#define M_RAD_TO_DEG 		57.295779513082323
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/
class Calculate {
public:
    void GetLocalPos(const GlobalPosition &loc1, const GlobalPosition &loc2,
                     TVec3 &follow_uav_local_pos);

    void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next,
                                     float &v_n, float &v_e);

    void GetGlobalPos(const GlobalPosition &loc1, GlobalPosition &loc2, TVec2 &local_target_pos);

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
