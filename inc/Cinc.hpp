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
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
using namespace std;

typedef Eigen::Vector2f TVec2;
typedef Eigen::Vector3f TVec3;
typedef Eigen::Vector4f TVec4;
typedef Eigen::Matrix3f TMat3;
typedef Eigen::Matrix4f TMat4;
typedef Eigen::Quaternionf TQuat;

static const bool K_Param_local_global = true;
static const float K_err_allow = 0.5;
static const float K_multi_formation_distance = 5;
static const float K_multi_usv_formation_distance = 15;
static const float usv_position_allow_reached_ = 3;

typedef struct GlobalPosition
{
    double latitude;  /*!< unit: rad */
    double longitude; /*!< unit: rad */
    uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} GlobalPosition;      // pack(1)

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
    geometry_msgs::PoseStamped current_local_pos;
    mavros_msgs::State current_state;
    TVec3 follower_to_leader_pos;
    TVec3 follower_keep_pos;
    TVec3 avoidance_pos;
    TVec3 velocity;
    geometry_msgs::PoseStamped target_local_pos_sp;
    mavros_msgs::PositionTarget current_local_pos_sp;
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
    TAKEOFF,
    WAYPOINT,
    LAND,
    FALLOW_USV
};

enum {
    USA_INIT,
    USA_WAYPOINT,
    USA_DISARM
};

enum COMMAND_TYPE {
    VF_UAV_SQUARE = 1,
    VF_UAV_TRIANGLE,
    VF_UAV_LINE_HORIZONTAL,
    VF_UAV_LINE_VERTICAL,
    VF_USV_TRIANGLE,
    VF_USV_LINE_HORIZONTAL,
    VF_USV_LINE_VERTICAL,
    VF_UAV_START,
    VF_UAV_RETURN
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

#endif //OFFBOARD_CINC_HPP
