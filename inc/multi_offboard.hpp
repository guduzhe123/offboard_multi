//
// Created by zhouhua on 19-9-15.
//

#ifndef OFFBOARD_MULTI_OFFBOARD_HPP
#define OFFBOARD_MULTI_OFFBOARD_HPP

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/DebugValue.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include "Multi_UAV_formation.hpp"
#include "Cinc.hpp"
#include "PathCreator.hpp"
#include "DataMan.hpp"
#include "IMsgRosManager.hpp"

//static const float usv_position_allow_reached_ = 3;
using namespace std;
using namespace Eigen;

class MultiOffboard : public IMsgRosManager{
public:
    ~MultiOffboard() {};

    MultiOffboard();

    void OnInit(const bool is_sim) override ;

    void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);

    void uav1_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void uav2_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void uav3_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void uav4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void uav5_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void uav6_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void uav7_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);

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

    void uav1_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void uav2_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void uav3_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void uav4_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void uav5_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void uav6_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void uav7_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);

    void uav1_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void uav2_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void uav3_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void uav4_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void uav5_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void uav6_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
    void uav7_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg);

    void drone_pos_update();
    void PublishDronePosControl(const multi_vehicle &multi_vehicles) override;
    void PublishBoatPosControl(const multi_vehicle &multi_vehicles) override;
    void PublishUUVPosControl(const multi_vehicle &multi_vehicles) override;
    void PublishUSVPosControl(const multi_vehicle &multi_vehicles, int id) override ;
    void SetUAVState(mavros_msgs::SetMode &m_mode) override ;
    void SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) override;
    void SetUSVAvoData(const bool usv1_usv2_crash, const bool usv1_usv3_crash, const bool usv2_usv3_crash) override ;

    static  MultiOffboard* getInstance();

    mavros_msgs:: VFR_HUD current_vfr_hud;
    mavros_msgs::PositionTarget uav2_current_local_pos_sp;


    ros::NodeHandle nh;
    ros::Subscriber uav1_vfr_hud_sub;
    ros::Subscriber uav1_global_pos_sub;
    ros::Publisher uav1_g_speed_control_pub;

    ros::Subscriber uav2_multi_formation_sub;
    ros::Subscriber uav2_global_pos_sub;


    TVehicleMsg drone_uav1_;
    TVehicleMsg drone_uav2_;
    TVehicleMsg drone_uav3_;
    TVehicleMsg drone_uav4_;
    TVehicleMsg drone_uav5_;
    TVehicleMsg drone_uav6_;
    TVehicleMsg drone_uav7_;
    TVehicleMsg drone_uav_leader_;
    TVehicleMsg drone_usv_leader_;

    bool is_offboard = false;
    bool is_armed = false;
    bool usv_armed = false;
    ros::Time last_request_;

    int arm_command_{};

private:
    static MultiOffboard* l_pInst;

    float curr_altitude;
    int uav_state_ ;
    int usv_state_ ;
    int leader_uav_id_;
    int leader_usv_id_;

    geometry_msgs::PoseStamped target_pos_;
    vector<geometry_msgs::PoseStamped> uav_way_points;
    vector<geometry_msgs::PoseStamped> usv_way_points;

    M_Drone m_drone_uav1_;
    M_Drone m_drone_uav2_;
    M_Drone m_drone_uav3_;
    M_Drone m_drone_uav4_;
    M_Drone m_drone_uav5_;
    M_Drone m_drone_uav6_;
    M_Drone m_drone_uav7_;
};

#endif //OFFBOARD_MULTI_OFFBOARD_HPP
