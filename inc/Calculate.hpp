//
// Created by zhouhua on 2020/1/28.
//

#ifndef OFFBOARD_CALCULATE_HPP
#define OFFBOARD_CALCULATE_HPP

#include "Cinc.hpp"
#include <tf/transform_broadcaster.h>
#define M_DEG_TO_RAD 		0.017453292519943295
#define M_RAD_TO_DEG 		57.295779513082323
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/
const float m_speedLimit = 2.0;
/**
 * @brief Orientation transform options when applying rotations to data
 */
enum class StaticTF {
    NED_TO_ENU,		//!< will change orientation from being expressed WRT NED frame to WRT ENU frame
    ENU_TO_NED,		//!< change from expressed WRT ENU frame to WRT NED frame
    AIRCRAFT_TO_BASELINK,	//!< change from expressed WRT aircraft frame to WRT to baselink frame
    BASELINK_TO_AIRCRAFT,	//!< change from expressed WRT baselnk to WRT aircraft
    ECEF_TO_ENU,		//!< change from expressed WRT ECEF frame to WRT ENU frame
    ENU_TO_ECEF		//!< change from expressed WRT ENU frame to WRT ECEF frame
};

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

    float dgrIn180s(float d);

    void getMeterScaleHere(double &meterPerLatUnit, double &meterPerLongtUnit, const GlobalPosition &center_pos);

    double calcDist(const GlobalPosition &loc1, const GlobalPosition &loc2);

    void quaternion_to_rpy(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw);
    Eigen::Quaterniond quaternion_from_rpy(const double roll, const double pitch, const double yaw);
    Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);
    Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform);
    double quaternion_get_yaw(const geometry_msgs::Quaternion &orientation, EulerAngles& angles);
    void posToPosCtrl(TVec3 &target_point, TVec3 &target_after_judge, TVec3 &drone_cur_pos, float speed_limit);
    void getTakeoffPos(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_first_local);
    void circleCenter(M_Drone &uav1, M_Drone &uav2, TVec3 &target_pos);
    static Calculate* getInstance();

private:
    bool is_get_takeoff_pos_ = false;
    static Calculate* l_pInst;
};
#endif //OFFBOARD_CALCULATE_HPP
