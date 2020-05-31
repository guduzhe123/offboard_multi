//
// Created by zhouhua on 2020/5/31.
//

#ifndef OFFBOARD_MSGROS_HPP
#define OFFBOARD_MSGROS_HPP

#include "IMsgRosManager.hpp"
class MsgRos : public IMsgRosManager {
public:
    void OnInit() override ;

    void PublishDronePosControl(const multi_vehicle &multi_vehicles) override ;

    void PublishBoatPosControl(const multi_vehicle &multi_vehicles) override ;

    void SetUAVState(mavros_msgs::SetMode &m_mode) override ;

    void SetUSVState(mavros_msgs::CommandBool &arm_command, int usv_id) override ;

    typedef shared_ptr<MsgRos> Ptr;
};
#endif //OFFBOARD_MSGROS_HPP
