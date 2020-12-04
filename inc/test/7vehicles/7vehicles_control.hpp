//
// Created by zhouhua on 2020/8/9.
//

#ifndef OFFBOARD_7VEHICLES_CONTROL_HPP
#define OFFBOARD_7VEHICLES_CONTROL_HPP
#include "Cinc.hpp"
#include "test/4UAVs/uav1_ros_Manager.hpp"
#include "test/4UAVs/uav2_ros_Manager.hpp"
#include "test/4UAVs/uav3_ros_Manager.hpp"
#include "test/4UAVs/uav4_ros_Manager.hpp"
#include "test/3USVs/usv1_ros_Manager.hpp"
#include "test/3USVs/usv2_ros_Manager.hpp"
#include "test/3USVs/usv3_ros_Manager.hpp"
#include "test/UUV_Control/uuv1_ros_Manager.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"
#include "PathCreator.hpp"

class seven_vehicle_control : public IMsgRosManager {
public:
    seven_vehicle_control();
    ~seven_vehicle_control() = default;
    void getData();
    void doProgress();
    static  seven_vehicle_control* getInstance();

    void OnInit() override ;
    void PublishDronePosControl(const multi_vehicle &multi_vehicles) override ;
    void PublishBoatPosControl(const multi_vehicle &multi_vehicles) override ;
    void PublishUUVPosControl(const multi_vehicle &multi_vehicles) override;
    void SetUAVState(mavros_msgs::SetMode &m_mode) override ;
    void SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) override ;

private:
    uav1_ros_Manager::Ptr uav1_control_;
    uav2_ros_Manager::Ptr uav2_control_;
    uav3_ros_Manager::Ptr uav3_control_;
    uav4_ros_Manager::Ptr uav4_control_;
    usv1_ros_Manager::Ptr usv1_control_;
    usv2_ros_Manager::Ptr usv2_control_;
    usv3_ros_Manager::Ptr usv3_control_;
    uuv1_ros_Manager::Ptr uuv1_control_;

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

#endif //OFFBOARD_7VEHICLES_CONTROL_HPP
