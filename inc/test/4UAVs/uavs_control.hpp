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
#include "msgRos.hpp"
#include "PathCreator.hpp"

class uavs_control {
public:
    uavs_control();
    ~uavs_control() = default;
    void onInit();
    void getData();
    void doProgress();
    static  uavs_control* getInstance();

private:
    uav1_ros_Manager::Ptr uav1_control_;
    uav2_ros_Manager::Ptr uav2_control_;
    uav3_ros_Manager::Ptr uav3_control_;
    uav4_ros_Manager::Ptr uav4_control_;
    MsgRos::Ptr msgRos_;


    static uavs_control* l_pInst;
    int uav_state_;
    bool uav_reached_;
    bool is_get_takeoff_pos_;
    bool is_avoidance_;
    double formation_distance_;
    int command_ ;
    double danger_distance_;
};


#endif //OFFBOARD_UAVS_CONTROL_HPP
