//
// Created by zhouhua on 19-12-1.
//

#ifndef OFFBOARD_MULTI_FORMATION_HPP
#define OFFBOARD_MULTI_FORMATION_HPP

#include "Cinc.hpp"

typedef struct GlobalPosition
{
    double latitude;  /*!< unit: rad */
    double longitude; /*!< unit: rad */
    double altitude;  /*!< WGS 84 reference ellipsoid */
    double height;    /*!< relative height to the ground */
    uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} GlobalPosition;      // pack(1)


class MultiFormation {
public:
    MultiFormation();

    ~MultiFormation() = default;

    void Oninit(const float config);

    void GetLocalPos(const GlobalPosition &loc1, const GlobalPosition &loc2,
                     geometry_msgs::PoseStamped &follow_uav_local_pos);

    void GetGlobalPos(const GlobalPosition &loc1, GlobalPosition &loc2, TVec3 &local_target_pos);

    static MultiFormation* getInstance();

private:

    static MultiFormation* multi_formation;


    void setEachLoclation();

    double deg2rad(double deg);

    double rad2deg(double rad);

    void getMeterScaleHere(double &meterPerLatUnit, double &meterPerLongtUnit, const GlobalPosition &center_pos);

    double calcDist(const GlobalPosition &loc1, const GlobalPosition &loc2);

};
#endif //OFFBOARD_MULTI_FORMATION_HPP
