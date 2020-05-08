//
// Created by zhouhua on 2020/5/3.
//

#ifndef OFFBOARD_USV_ROS_MANAGER_HPP
#define OFFBOARD_USV_ROS_MANAGER_HPP

#include "Cinc.hpp"
#include "DataMan.hpp"

class usv_ros_Manager {
public:
    usv_ros_Manager();
    ~usv_ros_Manager() = default;
    void usvOnInit(ros::NodeHandle &nh);
    void usvPosSp(const geometry_msgs::PoseStamped& way_point);
    typedef shared_ptr<usv_ros_Manager> Ptr;

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

    ros::Subscriber state_sub, vfr_hud_sub, local_position_sub, mavlink_from_sub, global_pos_sub, commander_sub;
    ros::Publisher local_pos_pub, gps_global_pos_pub, global_pos_pub, g_speed_control_pub;
    ros::ServiceClient arming_client, set_mode_client;
    ros::Timer exec_timer_, commander_timer_, publish_timer_;

    M_Drone usv_;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local_pos;
    mavros_msgs:: VFR_HUD current_vfr_hud;
    mavros_msgs::Mavlink current_mavlink;
    geometry_msgs::PoseStamped target_local_pos_sp_;

    int arm_command_;
    int arm_i_;
    bool is_arm_;
    bool is_offboard_;
    bool is_takeoff_;

};
#endif //OFFBOARD_USV_ROS_MANAGER_HPP
