//
// Created by zhouhua on 2020/5/24.
//

#ifndef OFFBOARD_UAVS_CONTROL_HPP
#define OFFBOARD_UAVS_CONTROL_HPP

#include "Cinc.hpp"
#include "uav1_ros_Manager.hpp"
#include "uav2_ros_Manager.hpp"
#include "uav3_ros_Manager.hpp"
#include "uav4_ros_Manager.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"
#include "PathCreator.hpp"

class uavs_control : public IMsgRosManager{
public:
    uavs_control();
    ~uavs_control() = default;
    void getData();
    void doProgress();
    static  uavs_control* getInstance();

    void OnInit() override ;
    void PublishDronePosControl(const multi_vehicle &multi_vehicles) override ;
    void PublishBoatPosControl(const multi_vehicle &multi_vehicles) override ;
    void SetUAVState(mavros_msgs::SetMode &m_mode) override ;
    void SetUSVState(mavros_msgs::CommandBool &arm_command, int usv_id) override ;

private:
    uav1_ros_Manager::Ptr uav1_control_;
    uav2_ros_Manager::Ptr uav2_control_;
    uav3_ros_Manager::Ptr uav3_control_;
    uav4_ros_Manager::Ptr uav4_control_;

    multi_vehicle multiVehicle;
    int uav_state_;
    bool uav_reached_;
    bool is_get_takeoff_pos_;
    bool is_avoidance_;
    double formation_distance_;
    int command_ ;
    double danger_distance_;
    bool follow_usv_;
};


#endif //OFFBOARD_UAVS_CONTROL_HPP
