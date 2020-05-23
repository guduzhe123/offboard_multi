//
// Created by zhouhua on 2020/1/28.
//

#include "Calculate.hpp"
Calculate* Calculate::l_pInst = NULL;

void Calculate::GetLocalPos(const GlobalPosition &loc1, const GlobalPosition &loc2,
                            TVec3 &follow_uav_local_pos) {
/*    GlobalPosition center_pos;
    double meterPerLat, meterPerLongt;
    center_pos.latitude = 0;
    center_pos.longitude = 0;
    getMeterScaleHere(meterPerLat, meterPerLongt, center_pos);

    double dLongt = loc1.longitude - loc2.longitude;
    double dLat = loc1.latitude - loc2.latitude;

    float  k = 2.0f/3.0f; // TODO PX4 x axis data is large 1.5
    follow_uav_local_pos.x() = k * dLongt * meterPerLongt;
    follow_uav_local_pos.y() = dLat * meterPerLat;*/

    float x, y;
    get_vector_to_next_waypoint_fast(loc1.latitude, loc1.longitude, loc2.latitude, loc2.longitude, x, y);

    follow_uav_local_pos.x() = -y;
    follow_uav_local_pos.y() = -x;
    util_log("distance orig x= %.2f, distance orig y= %.2f, x new = %.2f, y new = %.2f",
            follow_uav_local_pos.x(), follow_uav_local_pos.y(), x, y);
}

void Calculate::get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next,
                                               float &v_n, float &v_e)
{
    double lat_now_rad = lat_now * M_DEG_TO_RAD;
    double lon_now_rad = lon_now * M_DEG_TO_RAD;
    double lat_next_rad = lat_next * M_DEG_TO_RAD;
    double lon_next_rad = lon_next * M_DEG_TO_RAD;

    double d_lat = lat_next_rad - lat_now_rad;
    double d_lon = lon_next_rad - lon_now_rad;

    /* conscious mix of double and float trig function to maximize speed and efficiency */
    v_n = CONSTANTS_RADIUS_OF_EARTH * d_lat;
    v_e = CONSTANTS_RADIUS_OF_EARTH * d_lon * cos(lat_now_rad);
}

void Calculate::GetGlobalPos(const GlobalPosition &loc1, GlobalPosition &loc2, TVec2 &local_target_pos) {
    GlobalPosition center_pos;
    double meterPerLat, meterPerLongt;
    double x,y;
    x = local_target_pos(0);
    y = local_target_pos(1);

    center_pos.latitude = 0;
    center_pos.longitude = 0;
    getMeterScaleHere(meterPerLat, meterPerLongt, center_pos);

    double dLongt, dLat;
    float  k = 2.0f/3.0f; // TODO PX4 x axis data is large 1.5
    dLat = y / meterPerLat;
    dLongt = x /(meterPerLongt * k); // ?

    loc2.longitude = loc1.longitude + dLongt;
    loc2.latitude = loc1.latitude + dLat;
}

double Calculate::calcDist(const GlobalPosition &loc1, const GlobalPosition &loc2)
{
    double R = 6371; // Radius of the earth in km
    double dLat = deg2rad(loc2.latitude - loc1.latitude);  // deg2rad below
    double dLon = deg2rad(loc2.longitude - loc1.longitude);
    double a =
            sin(dLat / 2) * sin(dLat / 2) +
            cos(deg2rad(loc1.latitude)) * cos(deg2rad(loc2.longitude)) *
            sin(dLon / 2) * sin(dLon / 2)
    ;
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = R * c; // Distance in km
    return d*1000.0;	// from km to meters
}

double Calculate::deg2rad(double deg) {
    return deg * (M_PI / 180.0);
}

double Calculate::rad2deg(double rad) {
    return rad * (180.0 / M_PI);
}

void Calculate::getMeterScaleHere(double &meterPerLatUnit, double &meterPerLongtUnit,
                                       const GlobalPosition &center_pos)
{
    {
        GlobalPosition loc0 = center_pos;
        GlobalPosition loc1 = center_pos;

        loc0.latitude -= 0.5;
        loc1.latitude += 0.5;
        meterPerLatUnit = fabs(calcDist(loc0, loc1));

    }

    {
        GlobalPosition loc0 = center_pos;
        GlobalPosition loc1 = center_pos;

        loc0.longitude -= 0.5;
        loc1.longitude += 0.5;
        meterPerLongtUnit = fabs(calcDist(loc0, loc1));

    }

}

Calculate* Calculate::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new Calculate();
    }
    return l_pInst;
}