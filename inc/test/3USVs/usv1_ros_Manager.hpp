//
// Created by zhouhua on 2020/5/3.
//

#ifndef OFFBOARD_USV1_ROS_MANAGER_HPP
#define OFFBOARD_USV1_ROS_MANAGER_HPP

#include "Cinc.hpp"
#include "dataMan.hpp"
#include "DataMan.hpp"
#include "PathCreator.hpp"

class usv1_ros_Manager {
public:
    usv1_ros_Manager();
    ~usv1_ros_Manager() = default;

    void usvOnInit(ros::NodeHandle &nh);
    void usvPosSp(const DroneControl& droneControl);
    void usvCallService(mavros_msgs::SetMode &m_mode);
    void usvCrash(bool usv1_carsh);
    typedef shared_ptr<usv1_ros_Manager> Ptr;

private:
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mavlink_from_sb(const mavros_msgs::Mavlink::ConstPtr& msg);
    void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void wayPointCB(const mavros_msgs::WaypointList::ConstPtr& msg);
    void wayPointReachedCB(const mavros_msgs::WaypointReached::ConstPtr& msg);
    void homePositionCB(const mavros_msgs::HomePosition::ConstPtr& msg);
    void poublisMarker(const geometry_msgs::Point &p, const TVec4 &color, const ros::Publisher &publisher);
    void DrawTrajCommand(const TVec3 &pos, const TVec3 &vec, const TVec4 &color);
    void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
    
    void drone_pos_update(const ros::TimerEvent& e);
    void commander_update(const ros::TimerEvent& e);
    void publishDronePosControl(const ros::TimerEvent& e);

    ros::Subscriber state_sub, vfr_hud_sub, local_position_sub, mavlink_from_sub, global_pos_sub, commander_sub, way_point_sub, homePos_sub,
            way_point_reached_sub, imu_sub;
    ros::Publisher local_pos_pub, gps_global_pos_pub, global_pos_pub, g_speed_control_pub, dronePosPub,
                    home_pos_pub, marker_target_pub_, heading_vec_, marker_cur_pos_;
    ros::ServiceClient arming_client, set_mode_client;
    ros::Timer exec_timer_, commander_timer_, publish_timer_;

    M_Drone usv_;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local_pos;
    mavros_msgs:: VFR_HUD current_vfr_hud;
    mavros_msgs::Mavlink current_mavlink;
    geometry_msgs::PoseStamped target_local_pos_sp_;
    offboard::DronePosUpdate dronepos_;
    geometry_msgs::TwistStamped vel_ctrl_sp_;


    int arm_i_;
    bool is_arm_;
    bool is_offboard_;
    bool is_offboard_called_;
    bool is_takeoff_;
    bool is_land_;
    bool is_speed_ctrl_;
    bool home_pos_updated_;
    bool usv_crash_;

    float target_heading_;
    float yaw_cur_;

    TVec4 usv1_color_ = TVec4{1, 0.1, 1, 1};
};


#endif //OFFBOARD_UAV1_ROS_MANAGER_HPP
