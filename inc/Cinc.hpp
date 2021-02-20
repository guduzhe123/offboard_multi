//
// Created by zhouhua on 19-12-5.
//

#ifndef OFFBOARD_CINC_HPP
#define OFFBOARD_CINC_HPP

#include "math.h"
#include <string.h>
#include <Eigen/Core>
#include <vector>       // std::vector
#include <algorithm>    // std::reverse
#include <Eigen/Dense>
#include "util.h"
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/DebugValue.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointReached.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/Imu.h>
#include "pid.hpp"
#include "offboard/DronePosUpdate.h"
#include <visualization_msgs/Marker.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

#include "PCL/IMap.hpp"
#include "log/Chlog.hpp"
using namespace std;
using namespace chlog;

typedef Eigen::Vector2f TVec2;
typedef Eigen::Vector3f TVec3;
typedef Eigen::Vector4f TVec4;
typedef Eigen::Matrix3f TMat3;
typedef Eigen::Matrix4f TMat4;
typedef Eigen::Quaternionf TQuat;

static const bool K_Param_local_global = true;
static const float K_err_allow = 0.5;
static const float K_multi_formation_distance = 6;
static const float K_multi_usv_formation_distance = 5;
static const float K_uav_height = 15;
static const float usv_position_allow_reached_ = 3;

typedef struct GlobalPosition
{
    double latitude;  /*!< unit: rad */
    double longitude; /*!< unit: rad */
    uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} GlobalPosition;      // pack(1)

struct DroneControl {
    bool speed_ctrl = false;
    float target_heading = 0;
    float yaw_rate = 0;
    geometry_msgs::PoseStamped target_pose{};
    geometry_msgs::TwistStamped g_vel_sp{};
};

typedef struct EulerAngles {
    double roll;
    double pitch;
    double yaw;
} EulerAngles;


struct M_Drone {
    int drone_id;
    int movement_state;
    bool is_formation;
    float pitch ;
    float roll ;
    float yaw ;
    double latitude;
    double longtitude;
    double altitude;
    TQuat q;
    geometry_msgs::PoseStamped current_local_pos;
    mavros_msgs::State current_state;
    TVec3 follower_to_leader_pos;
    TVec3 follower_keep_pos;
    TVec3 avoidance_pos;
    TVec3 velocity;
    geometry_msgs::PoseStamped target_local_pos_sp;
    mavros_msgs::PositionTarget current_local_pos_sp;
    DroneControl droneControl;
    mavros_msgs::WaypointList  waypointList;
    mavros_msgs::WaypointReached waypointReached;
    mavros_msgs::HomePosition homePosition;
    octomap_msgs::Octomap octomap;
    shared_ptr<IMap> Imap;
};

struct multi_vehicle{
    M_Drone uav1;
    M_Drone uav2;
    M_Drone uav3;
    M_Drone uav4;
    M_Drone usv1;
    M_Drone usv2;
    M_Drone usv3;
    M_Drone uuv1;
    M_Drone uuv2;
    M_Drone uuv3;
    M_Drone leader_uav;
    M_Drone leader_usv;
    M_Drone leader_uuv;
    int user_command;
};

struct multi_vehicle_vec {
    vector<M_Drone> uav1_vec;
    vector<M_Drone> uav2_vec;
    vector<M_Drone> uav3_vec;
    vector<M_Drone> uav4_vec;
    vector<M_Drone> uav5_vec;
    vector<M_Drone> uav6_vec;
    vector<M_Drone> uav7_vec;
};

struct M_Drone_Avoidace{
    int drone_id;
    int avo_mode;
    TVec3 local_target_pos_avo;
    TVec3 local_target_vel_avo;
};

enum {
    UAV1 = 1,
    UAV2,
    UAV3,
    UAV4,
    USV1,
    USV2,
    USV3,
    UUV1,
    UUV2,
    UUV3
};

enum  {
    INIT,
    TAKEOFF,
    ADJUSTHEADING,
    WAYPOINT,
    LAND,
    FALLOW_USV,
    FALLOW_UUV,
    FORMATION,
    CIRCLE_INIT,
    CIRCLE,
    RETURN
};

enum {
    USV_INIT,
    USV_WAYPOINT,
    USV_DISARM,
    USV_FORMATION,
    USV_CIRCLE_INIT,
    USV_CIRCLE,
    USV_FOLLOW_UUV,
    USV_FOLLOW_UUV_FORMATION,
    USV_FOLLOW_UUV_FORMATION_INIT

};

enum {
    UUV_INIT,
    UUV_WAYPOINT,
    UUV_DISARM,
    UUV_FORMATION
};

enum COMMAND_TYPE {
    VF_UAV_SQUARE = 1,
    VF_UAV_TRIANGLE = 2,
    VF_UAV_LINE_HORIZONTAL = 3,
    VF_UAV_LINE_VERTICAL = 4,
    VF_USV_TRIANGLE = 5,
    VF_USV_LINE_HORIZONTAL = 6,
    VF_USV_LINE_VERTICAL = 7,
    VF_UAV_START = 8,
    VF_UAV_RETURN = 9,
    VF_UAV_ALL_START = 10,
    VF_UAV_ALL_STOP = 11,
    VF_UAV_ALL_LAND = 12,
    VF_USV_ALL_START = 13,
    VF_USV_ALL_STOP = 14,
    VF_USV_ALL_RETURN = 15,
    VF_UAV_FALLOW_USV = 16,
    VF_USV_FALLOW_UAV = 17,
    VF_UAV_CIRCLE = 18,
    VF_USV_CIRCLE = 19,
    VF_UUV_ALL_START = 20,
    VF_SET_HOME = 21,
    VF_USV_INVERSION_TRIANGLE = 22,
};


struct TVehicleMsg {
    int drone_id;
    int debug_received_id;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local_pos;
    geometry_msgs::PoseStamped target_pose;
    sensor_msgs::NavSatFix current_global_pos;

    ros::Subscriber state_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber multi_formation_sub;
    ros::Subscriber global_pos_sub;
    ros::Subscriber local_pos_sp_sub;

    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    ros::Publisher local_pos_pub;
    ros::Publisher global_pos_pub;
};

enum FDATA_TYPE {
    FDATA_DRONE = 0,
    FDATA_DRONE_TARGTE,
    FDATA_AVOIDANCE,
    FDATA_BOAT,
    FDATA_BOAT_TARGET,
    FDATA_MANUAL_COMMAND
};

struct TCircleConfig{
    bool is_clockWise = true;
    TVec3 m_target_pos = TVec3(NAN, NAN, NAN); ///< target position to move at
    TVec3 m_circle_pos = TVec3(NAN, NAN, NAN); ///< circle point
    TVec3 m_start_pos = TVec3(NAN, NAN, NAN); ///< start point
    float m_speed = NAN;  ///< total speed
    float m_radius = NAN; ///< circle radius;
    float target_heading = NAN; //< target head
};

/// This struct defines output of circle tracking
///
struct TCircleOutput {
    TVec3 v_out = TVec3(NAN, NAN, NAN);
    float m_target_heading = NAN;
    float m_gimbal_pitch = NAN;
    bool m_drone_circle_finished;
    float m_speed = NAN;  ///< total speed
    float m_yaw_rate = NAN;
    vector<TVec3> circle_target;
    vector<geometry_msgs::PoseStamped> usv_way_points_;
};

/// This struct defines status of drone during circle point tracking
///
struct TCircleStatus{
    TVec3 m_curPos = TVec3(NAN, NAN, NAN); ///< current drone position
    float m_radius = NAN; ///< the radius of circle
    float m_heading = NAN;///< current heading of drone
    TVec3 m_cross_v = TVec3(NAN, NAN, NAN); ///< cross
};

enum MotionPlanState {
    TRACKING,
    CIRCLE_MP_MP,
    POINTTOPOINT,
};

struct MP_Config{
    TVec3 start_pos;
    TVec3 end_pos;
    float exp_vel;
    float max_vel;
    float max_acc;
    float safe_zone_r; // the flight corrider radius.
    float m_drone_heading;
    float target_heading;
    float turn_rate;
    bool is_enable = false;
    int control_mode;
    MotionPlanState mp_plan_state;
    double safe_dist;
    double replan_thresh;
    double plan_horizon; // fitting line planning length
    TVec3 m_toward_point;
    bool is_track_point = false;
    bool is_speed_mode = false;
    bool is_gazebo_sim = false;
    int m_blade_id;
    int m_path_id;
    Sp<IMap> mp_map;
};
#endif //OFFBOARD_CINC_HPP
