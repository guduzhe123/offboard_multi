//
// Created by zhouhua on 2020/5/3.
//

#ifndef OFFBOARD_UAV2_ROS_MANAGER_HPP
#define OFFBOARD_UAV2_ROS_MANAGER_HPP

#include "Cinc.hpp"
#include "dataMan.hpp"
#include "DataMan.hpp"
#include "PathCreator.hpp"
#include "offboard/DronePosUpdate.h"

class uav2_ros_Manager {
public:
    uav2_ros_Manager();
    ~uav2_ros_Manager() = default;
    void usvOnInit(ros::NodeHandle &nh);
    //void usvPosSp(const geometry_msgs::PoseStamped& way_point);
    void uavPosSp(const DroneControl& droneControl);
    void uavCallService(mavros_msgs::SetMode &m_mode);
    typedef shared_ptr<uav2_ros_Manager> Ptr;

private:
    void drone_pos_update(const ros::TimerEvent& e);
    void commander_update(const ros::TimerEvent& e);
    void publishDronePosControl(const ros::TimerEvent& e);

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg);
    void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void drone_yaw_control();
    void wayPointCB(const mavros_msgs::WaypointList::ConstPtr& msg);

    ros::Subscriber state_sub, vfr_hud_sub, local_position_sub, mavlink_from_sub, global_pos_sub, commander_sub, way_point_sub;
    ros::Publisher local_pos_pub, gps_global_pos_pub, global_pos_pub, g_speed_control_pub, dronePosPub;
    ros::ServiceClient arming_client, set_mode_client;
    ros::Timer exec_timer_, commander_timer_, publish_timer_;

    M_Drone uav_;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local_pos;
    mavros_msgs:: VFR_HUD current_vfr_hud;
    mavros_msgs::Mavlink current_mavlink;
    geometry_msgs::PoseStamped target_local_pos_sp_;
    geometry_msgs::TwistStamped vel_ctrl_sp_;
    offboard::DronePosUpdate dronepos_;

    int arm_i_;
    bool is_arm_;
    bool is_offboard_;
    bool is_takeoff_;
    bool is_land_;
    bool is_speed_ctrl_;

    float target_heading_;
};
#endif //OFFBOARD_UAV2_ROS_MANAGER_HPP
