//
// Created by zhouhua on 2020/1/3.
//

#ifndef OFFBOARD_IMSGROSMANAGER_HPP
#define OFFBOARD_IMSGROSMANAGER_HPP

#include "Cinc.hpp"

class IMsgRosManager {
public:
    virtual  ~IMsgRosManager() {};

    virtual void OnInit() = 0;

    virtual void PublishDronePosControl(const multi_vehicle &multi_vehicles) = 0 ;

    virtual void PublishBoatPosControl(const multi_vehicle &multi_vehicles) = 0;

    virtual void PublishUUVPosControl(const multi_vehicle &multi_vehicles) = 0;

    virtual void SetUAVState(mavros_msgs::SetMode &m_mode) = 0;

    virtual void SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) = 0;

    virtual void SetUSVAvoData(const bool usv1_usv2_crash, const bool usv1_usv3_crash, const bool usv2_usv3_crash) = 0;

    virtual void PublishUSVPosControl(const multi_vehicle &multi_vehicles, int id) = 0;
};
#endif //OFFBOARD_IMSGROSMANAGER_HPP
