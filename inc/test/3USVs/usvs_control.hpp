//
// Created by zhouhua on 2020/5/24.
//

#ifndef OFFBOARD_USVS_CONTROL_HPP
#define OFFBOARD_USVS_CONTROL_HPP

#include "Cinc.hpp"
#include "usv1_ros_Manager.hpp"
#include "usv2_ros_Manager.hpp"
#include "usv3_ros_Manager.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"

class usvs_control : public IMsgRosManager{
public:
    usvs_control();
    ~usvs_control() = default;
    static  usvs_control* getInstance();
    void OnInit() override ;
    void PublishDronePosControl(const multi_vehicle &multi_vehicles) override ;
    void PublishBoatPosControl(const multi_vehicle &multi_vehicles) override ;
    void PublishUUVPosControl(const multi_vehicle &multi_vehicles) override;
    void SetUAVState(mavros_msgs::SetMode &m_mode) override ;
    void SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) override ;

    void getData();
    void doProgress();

private:
    usv1_ros_Manager::Ptr usv1_control_;
    usv2_ros_Manager::Ptr usv2_control_;
    usv3_ros_Manager::Ptr usv3_control_;

    static usvs_control* l_pInst;
    int usv_state_;
    bool usv_reached_;
    bool is_get_takeoff_pos_;
    bool is_avoidance_;
    double formation_distance_;
    int command_ ;
    double danger_distance_;
};


#endif //OFFBOARD_UAVS_CONTROL_HPP
