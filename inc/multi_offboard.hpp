//
// Created by zhouhua on 19-9-15.
//

#ifndef OFFBOARD_MULTI_OFFBOARD_HPP
#define OFFBOARD_MULTI_OFFBOARD_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/PositionTarget.h>
#include "math.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <Eigen/Core>
#include <vector>       // std::vector
#include <algorithm>    // std::reverse

using namespace std;
using namespace Eigen;
class MultiOffboard {
public:
    enum  {
        TAKEOFF,
        WAYPOINT,
        LAND
    };

    enum {
        USA_INIT,
        USA_WAYPOINT,
        USA_DISARM
    };

    ~MultiOffboard() {};
    void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);

    void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav3_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav4_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav5_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav6_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav7_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav4_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav5_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav6_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav7_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void uav2_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);

    bool pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos, float err_allow);
    void Oninit();
    void uav_targte_local_pos();
    void usa_targte_local_pos();
    void uav_add_way_points();
    void usv_add_way_points();

    mavros_msgs::State uav1_current_state;
    mavros_msgs::State uav2_current_state;
    mavros_msgs::State uav3_current_state;
    mavros_msgs::State uav4_current_state;
    mavros_msgs::State uav5_current_state;
    mavros_msgs::State uav6_current_state;
    mavros_msgs::State uav7_current_state;
    mavros_msgs:: VFR_HUD current_vfr_hud;
    mavros_msgs::PositionTarget uav2_current_local_pos_sp;
    geometry_msgs::PoseStamped uav1_current_local_pos;
    geometry_msgs::PoseStamped uav2_current_local_pos;
    geometry_msgs::PoseStamped uav3_current_local_pos;
    geometry_msgs::PoseStamped uav4_current_local_pos;
    geometry_msgs::PoseStamped uav5_current_local_pos;
    geometry_msgs::PoseStamped uav6_current_local_pos;
    geometry_msgs::PoseStamped uav7_current_local_pos;

    geometry_msgs::PoseStamped uav1_target_pose;
    geometry_msgs::PoseStamped uav2_target_pose;
    geometry_msgs::PoseStamped uav3_target_pose;
    geometry_msgs::PoseStamped uav4_target_pose;
    geometry_msgs::PoseStamped uav5_target_pose;
    geometry_msgs::PoseStamped uav6_target_pose;
    geometry_msgs::PoseStamped uav7_target_pose;

    ros::NodeHandle nh;
    ros::Subscriber uav1_local_position_sub;
    ros::ServiceClient uav1_set_mode_client;
    ros::ServiceClient uav1_arming_client;
    ros::Publisher uav1_local_pos_pub;
    ros::Subscriber uav1_state_sub;
    ros::Subscriber uav2_state_sub;
    ros::Subscriber uav3_state_sub;
    ros::Subscriber uav4_state_sub;
    ros::Subscriber uav5_state_sub;
    ros::Subscriber uav6_state_sub;
    ros::Subscriber uav7_state_sub;
    ros::Subscriber uav1_vfr_hud_sub;
    ros::Publisher uav1_gps_global_pos_pub;
    ros::Publisher uav1_global_pos_pub;
    ros::Publisher uav1_g_speed_control_pub;

    ros::Subscriber uav2_local_position_sub;
    ros::ServiceClient uav2_set_mode_client;
    ros::ServiceClient uav2_arming_client;
    ros::Publisher uav2_local_pos_pub;
    ros::Subscriber uav2_local_pos_sp_sub;

    ros::Subscriber uav3_local_position_sub;
    ros::ServiceClient uav3_set_mode_client;
    ros::ServiceClient uav3_arming_client;
    ros::Publisher uav3_local_pos_pub;

    ros::Subscriber uav4_local_position_sub;
    ros::ServiceClient uav4_set_mode_client;
    ros::ServiceClient uav4_arming_client;
    ros::Publisher uav4_local_pos_pub;

    ros::Subscriber uav5_local_position_sub;
    ros::ServiceClient uav5_set_mode_client;
    ros::ServiceClient uav5_arming_client;
    ros::Publisher uav5_local_pos_pub;

    ros::Subscriber uav6_local_position_sub;
    ros::ServiceClient uav6_set_mode_client;
    ros::ServiceClient uav6_arming_client;
    ros::Publisher uav6_local_pos_pub;

    ros::Subscriber uav7_local_position_sub;
    ros::ServiceClient uav7_set_mode_client;
    ros::ServiceClient uav7_arming_client;
    ros::Publisher uav7_local_pos_pub;

    bool is_offboard = false;
    bool is_armed = false;
    bool usv_armed = false;
    ros::Time last_request_;
private:

    float curr_altitude;
    int uav_state_ = TAKEOFF;
    int usv_state_ = USA_INIT;
    geometry_msgs::PoseStamped target_pos_;
    vector<geometry_msgs::PoseStamped> uav_way_points;
    vector<geometry_msgs::PoseStamped> usv_way_points;

    bool usv5_reached_ = false;
    bool usv6_reached_ = false;
    bool usv7_reached_ = false;
};

#endif //OFFBOARD_MULTI_OFFBOARD_HPP
