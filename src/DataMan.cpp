//
// Created by zhouhua on 2020/1/23.
//

#include "DataMan.hpp"

DataMan* DataMan::l_singleton = NULL;

DataMan* DataMan::getInstance() {
    if (l_singleton == NULL) {
        l_singleton = new DataMan();
    }
    return l_singleton;
}

// initialize config parameters.
void DataMan::OnInit(IMsgRosManager *msg_ros) {
    msg_config_ = msg_ros;
}

void DataMan::SetDroneData(const M_Drone &mDrone) {
    {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        switch (mDrone.drone_id) {
            case UAV1: {
                multi_vehicle_.uav1 = mDrone;
                multi_vehicle_.leader_uav.waypointList = multi_vehicle_.uav1.waypointList;
            }
                break;
            case UAV2: {
                multi_vehicle_.uav2 = mDrone;
            }
                break;
            case UAV3: {
                multi_vehicle_.uav3 = mDrone;
//                multi_vehicle_.leader_uav.waypointList = multi_vehicle_.uav3.waypointList;
            }
                break;
            case UAV4: {
                multi_vehicle_.uav4 = mDrone;
//                multi_vehicle_.leader_uav.waypointList = multi_vehicle_.uav4.waypointList;
            }
                break;
            case USV1: {
                multi_vehicle_.usv1 = mDrone;
                multi_vehicle_.leader_usv.waypointList = multi_vehicle_.usv1.waypointList;
            }
                break;
            case USV2: {
                multi_vehicle_.usv2 = mDrone;
                multi_vehicle_.leader_usv.waypointList = multi_vehicle_.usv2.waypointList;
            }
                break;
            case USV3: {
                multi_vehicle_.usv3 = mDrone;
                multi_vehicle_.leader_usv.waypointList = multi_vehicle_.usv3.waypointList;
            }
                break;
            case UUV1: {
                multi_vehicle_.uuv1 = mDrone;
                multi_vehicle_.leader_uuv.waypointList = multi_vehicle_.uuv1.waypointList;
            }
                break;
            case UUV2: {
                multi_vehicle_.uuv2 = mDrone;
            }
                break;
            case UUV3: {
                multi_vehicle_.uuv3 = mDrone;
            }
                break;
            default:
                break;
        }
    }
}

void
DataMan::SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2,
        const M_Drone_Avoidace& uav3, const M_Drone_Avoidace& uav4) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);

        multi_vehicle_.uav1.avoidance_pos = uav1.local_target_pos_avo;
        multi_vehicle_.uav2.avoidance_pos = uav2.local_target_pos_avo;
        multi_vehicle_.uav3.avoidance_pos = uav3.local_target_pos_avo;
        multi_vehicle_.uav4.avoidance_pos = uav4.local_target_pos_avo;
    }
    if (callback_) {
        callback_->OnFlightDataUpdate(FDATA_AVOIDANCE);
    }
}

void
DataMan::SetUAVFormationData(bool is_formation, int leader_uav_id_, const TVec3 &follow_uav1, const TVec3 &follow_uav2,
                             const TVec3 &follow_uav3) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.leader_uav.is_formation = is_formation;
        leader_uav_ = leader_uav_id_;
        switch (leader_uav_id_) {
            case UAV1: {
                multi_vehicle_.uav1.follower_to_leader_pos = {0, 0, 0};
                multi_vehicle_.uav2.follower_to_leader_pos = follow_uav1;
                multi_vehicle_.uav3.follower_to_leader_pos = follow_uav2;
                multi_vehicle_.uav4.follower_to_leader_pos = follow_uav3;
                break;
            }
            case UAV2: {
                multi_vehicle_.uav1.follower_to_leader_pos = follow_uav1;
                multi_vehicle_.uav2.follower_to_leader_pos = {0, 0, 0};
                multi_vehicle_.uav3.follower_to_leader_pos = follow_uav2;
                multi_vehicle_.uav4.follower_to_leader_pos = follow_uav3;
                break;
            }
            case UAV3: {
                multi_vehicle_.uav1.follower_to_leader_pos = follow_uav1;
                multi_vehicle_.uav2.follower_to_leader_pos = follow_uav2;
                multi_vehicle_.uav3.follower_to_leader_pos = {0, 0, 0};
                multi_vehicle_.uav4.follower_to_leader_pos = follow_uav3;
                break;
            }
            case UAV4: {
                multi_vehicle_.uav1.follower_to_leader_pos = follow_uav1;
                multi_vehicle_.uav2.follower_to_leader_pos = follow_uav2;
                multi_vehicle_.uav3.follower_to_leader_pos = follow_uav3;
                multi_vehicle_.uav4.follower_to_leader_pos = {0, 0, 0};
                break;
            }
            default:
                break;
        }
    }

}

void DataMan::SetUAVFormationKeepData(const TVec3 &follow_uav1, const TVec3 &follow_uav2, const TVec3 &follow_uav3,
                                      const TVec3 &follow_uav4) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.uav1.follower_keep_pos = follow_uav1;
        multi_vehicle_.uav2.follower_keep_pos = follow_uav2;
        multi_vehicle_.uav3.follower_keep_pos = follow_uav3;
        multi_vehicle_.uav4.follower_keep_pos = follow_uav4;
    }
}

void DataMan::SetUSVFormationData(const multi_vehicle &m_multi_vehicles, bool is_formation) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.usv1.follower_to_leader_pos = m_multi_vehicles.usv1.follower_to_leader_pos;
        multi_vehicle_.usv2.follower_to_leader_pos = m_multi_vehicles.usv2.follower_to_leader_pos;
        multi_vehicle_.usv3.follower_to_leader_pos = m_multi_vehicles.usv3.follower_to_leader_pos;


        multi_vehicle_.usv1.follower_keep_pos = m_multi_vehicles.usv1.follower_keep_pos;
        multi_vehicle_.usv2.follower_keep_pos = m_multi_vehicles.usv2.follower_keep_pos;
        multi_vehicle_.usv3.follower_keep_pos = m_multi_vehicles.usv3.follower_keep_pos;
        multi_vehicle_.leader_usv.is_formation = is_formation;
        util_log("data man is lead usv in formation %d", is_formation);
    }
}


void DataMan::SetUAVState(mavros_msgs::SetMode &m_mode) {
    msg_config_->SetUAVState(m_mode);
}

void DataMan::SetUSVState(mavros_msgs::CommandBool &arm_command, int usv_id) {
    msg_config_->SetUSVState(arm_command, usv_id);
}

void DataMan::SetDroneControlData(const multi_vehicle &m_multi_vehicles) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.uav1.target_local_pos_sp = m_multi_vehicles.uav1.target_local_pos_sp;
        multi_vehicle_.uav2.target_local_pos_sp = m_multi_vehicles.uav2.target_local_pos_sp;
        multi_vehicle_.uav3.target_local_pos_sp = m_multi_vehicles.uav3.target_local_pos_sp;
        multi_vehicle_.uav4.target_local_pos_sp = m_multi_vehicles.uav4.target_local_pos_sp;
        multi_vehicle_.leader_uav.target_local_pos_sp = m_multi_vehicles.leader_uav.target_local_pos_sp;

        multi_vehicle_.uav1.droneControl = m_multi_vehicles.uav1.droneControl;
    }

    if(callback_) {
        callback_->OnFlightDataUpdate(FDATA_DRONE_TARGTE);
    }
}

void DataMan::PublishDroneControlData(const multi_vehicle &m_multi_vehicles) {
    {
//        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.uav1.target_local_pos_sp = m_multi_vehicles.uav1.target_local_pos_sp;
        multi_vehicle_.uav2.target_local_pos_sp = m_multi_vehicles.uav2.target_local_pos_sp;
        multi_vehicle_.uav3.target_local_pos_sp = m_multi_vehicles.uav3.target_local_pos_sp;
        multi_vehicle_.uav4.target_local_pos_sp = m_multi_vehicles.uav4.target_local_pos_sp;
        multi_vehicle_.leader_uav.target_local_pos_sp = m_multi_vehicles.leader_uav.target_local_pos_sp;
        multi_vehicle_.uav1.droneControl = m_multi_vehicles.uav1.droneControl;
    }
    msg_config_-> PublishDronePosControl(m_multi_vehicles);
}

void DataMan::SetBoatControlData(const multi_vehicle &m_multi_vehicles) {
    {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.usv1.target_local_pos_sp = m_multi_vehicles.usv1.target_local_pos_sp;
        multi_vehicle_.usv2.target_local_pos_sp = m_multi_vehicles.usv2.target_local_pos_sp;
        multi_vehicle_.usv3.target_local_pos_sp = m_multi_vehicles.usv3.target_local_pos_sp;
        multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicles.leader_usv.target_local_pos_sp;
        multi_vehicle_.usv1.droneControl = m_multi_vehicles.usv1.droneControl;
    }
    msg_config_->PublishBoatPosControl(m_multi_vehicles);
}

void DataMan::SetUUVControlData(const multi_vehicle &m_multi_vehicles) {
    {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.uuv1.target_local_pos_sp = m_multi_vehicles.uuv1.target_local_pos_sp;
        multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicles.leader_usv.target_local_pos_sp;
    }
    msg_config_->PublishUUVPosControl(m_multi_vehicles);
}

void DataMan::SetUAVLeader(M_Drone &leader_uav) {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    multi_vehicle_.leader_uav = leader_uav;
}

void DataMan::SetUSVLeader(M_Drone &leader_usv) {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    multi_vehicle_.leader_usv = leader_usv;

}

void DataMan::SetUUVLeader(M_Drone &leader_uuv) {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    multi_vehicle_.leader_uuv = leader_uuv;

}

void DataMan::SetUserCommand(const int value) {
//    boost::unique_lock<boost::mutex> lock(m_mutex);
//    {
        multi_vehicle_.user_command = value;
//    }

/*    if(callback_) {
        callback_->OnFlightDataUpdate(FDATA_MANUAL_COMMAND);
    }*/
}

void DataMan::PrinrDorneFlightDate() {
    util_log("uav1 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav1.current_local_pos.pose.position.x,
             multi_vehicle_.uav1.current_local_pos.pose.position.y, multi_vehicle_.uav1.current_local_pos.pose.position.z);
    util_log("uav2 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav2.current_local_pos.pose.position.x,
             multi_vehicle_.uav2.current_local_pos.pose.position.y, multi_vehicle_.uav2.current_local_pos.pose.position.z);
    util_log("uav3 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav3.current_local_pos.pose.position.x,
             multi_vehicle_.uav3.current_local_pos.pose.position.y, multi_vehicle_.uav3.current_local_pos.pose.position.z);
    util_log("uav4 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav4.current_local_pos.pose.position.x,
             multi_vehicle_.uav4.current_local_pos.pose.position.y, multi_vehicle_.uav4.current_local_pos.pose.position.z);
    util_log("leader uav current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.leader_uav.current_local_pos.pose.position.x,
             multi_vehicle_.leader_uav.current_local_pos.pose.position.y, multi_vehicle_.leader_uav.current_local_pos.pose.position.z);

}

void DataMan::PrintBoatData() {
    util_log("usv1 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.usv1.current_local_pos.pose.position.x,
             multi_vehicle_.usv1.current_local_pos.pose.position.y, multi_vehicle_.usv1.current_local_pos.pose.position.z);
    util_log("usv2 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.usv2.current_local_pos.pose.position.x,
             multi_vehicle_.usv2.current_local_pos.pose.position.y, multi_vehicle_.usv2.current_local_pos.pose.position.z);
    util_log("usv3 current pos = (%.2f, %.2f, %.2f)", multi_vehicle_.usv3.current_local_pos.pose.position.x,
             multi_vehicle_.usv3.current_local_pos.pose.position.y, multi_vehicle_.usv3.current_local_pos.pose.position.z);
}

void DataMan::PrintDroneTargetPosData() {
    util_log("uav1 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav1.target_local_pos_sp.pose.position.x,
             multi_vehicle_.uav1.target_local_pos_sp.pose.position.y, multi_vehicle_.uav1.target_local_pos_sp.pose.position.z);
    util_log("uav2 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav2.target_local_pos_sp.pose.position.x,
             multi_vehicle_.uav2.target_local_pos_sp.pose.position.y, multi_vehicle_.uav2.target_local_pos_sp.pose.position.z);
    util_log("uav3 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav3.target_local_pos_sp.pose.position.x,
             multi_vehicle_.uav3.target_local_pos_sp.pose.position.y, multi_vehicle_.uav3.target_local_pos_sp.pose.position.z);
    util_log("uav4 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.uav4.target_local_pos_sp.pose.position.x,
             multi_vehicle_.uav4.target_local_pos_sp.pose.position.y, multi_vehicle_.uav4.target_local_pos_sp.pose.position.z);
}

void DataMan::PrintBoatTargetPosData() {
    util_log("usv1 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.usv1.target_local_pos_sp.pose.position.x,
             multi_vehicle_.usv1.target_local_pos_sp.pose.position.y, multi_vehicle_.usv1.target_local_pos_sp.pose.position.z);
    util_log("usv2 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.usv2.target_local_pos_sp.pose.position.x,
             multi_vehicle_.usv2.target_local_pos_sp.pose.position.y, multi_vehicle_.usv2.target_local_pos_sp.pose.position.z);
    util_log("usv3 target local pos = (%.2f, %.2f, %.2f)", multi_vehicle_.usv3.target_local_pos_sp.pose.position.x,
             multi_vehicle_.usv3.target_local_pos_sp.pose.position.y, multi_vehicle_.usv3.target_local_pos_sp.pose.position.z);
}

void DataMan::PrintAvoidanceData() {
    util_log("get output height_avoidance_uav1_ = %.2f, height_avoidance_uav2_ = %.2f, "
             "height_avoidance_uav3_ = %.2f, height_avoidance_uav4_ = %.2f",
             multi_vehicle_.uav1.avoidance_pos.z(), multi_vehicle_.uav2.avoidance_pos.z(),
             multi_vehicle_.uav3.avoidance_pos.z(), multi_vehicle_.uav4.avoidance_pos.z());
}

void DataMan::PrintDroneFormationData() {
    util_log("is uav formation = %d, uav formation leader = %d", multi_vehicle_.leader_uav.is_formation, leader_uav_);
    util_log("uav1 follow to leader = (%.2f, %.2f, %.2f) ",multi_vehicle_.uav1.follower_to_leader_pos(0),
             multi_vehicle_.uav1.follower_to_leader_pos(1), multi_vehicle_.uav1.follower_to_leader_pos(2));
    util_log("uav2 follow to leader = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav2.follower_to_leader_pos(0),
             multi_vehicle_.uav2.follower_to_leader_pos(1), multi_vehicle_.uav2.follower_to_leader_pos(2));
    util_log("uav3 follow to leader = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav3.follower_to_leader_pos(0),
             multi_vehicle_.uav3.follower_to_leader_pos(1), multi_vehicle_.uav3.follower_to_leader_pos(2));
    util_log("uav4 follow to leader = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav4.follower_to_leader_pos(0),
             multi_vehicle_.uav4.follower_to_leader_pos(1), multi_vehicle_.uav4.follower_to_leader_pos(2));

}

void DataMan::PrintDroneFormationKeep() {
    util_log("uav1 formation keep local pos = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav1.follower_keep_pos(0),
             multi_vehicle_.uav1.follower_keep_pos(1), multi_vehicle_.uav1.follower_keep_pos(2));
    util_log("uav2 formation keep local pos  = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav2.follower_keep_pos(0),
             multi_vehicle_.uav2.follower_keep_pos(1), multi_vehicle_.uav2.follower_keep_pos(2));
    util_log("uav3 formation keep local pos  = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav3.follower_keep_pos(0),
             multi_vehicle_.uav3.follower_keep_pos(1), multi_vehicle_.uav3.follower_keep_pos(2));
    util_log("uav4 formation keep local pos  = (%.2f, %.2f, %.2f) ", multi_vehicle_.uav4.follower_keep_pos(0),
             multi_vehicle_.uav4.follower_keep_pos(1), multi_vehicle_.uav4.follower_keep_pos(2));
}

void DataMan::PrintUSVFormationData() {
    util_log("is usv formation = %d, usv formation leader = %d,", multi_vehicle_.leader_usv.is_formation, multi_vehicle_.leader_usv.drone_id);
    util_log("usv1 follow to leader = (%.2f, %.2f, %.2f) ", multi_vehicle_.usv1.follower_to_leader_pos(0),
            multi_vehicle_.usv1.follower_to_leader_pos(1), multi_vehicle_.usv1.follower_to_leader_pos(2));
    util_log("usv2 follow to leader = (%.2f, %.2f, %.2f) ",  multi_vehicle_.usv2.follower_to_leader_pos(0),
             multi_vehicle_.usv2.follower_to_leader_pos(1), multi_vehicle_.usv2.follower_to_leader_pos(2));
    util_log("usv3 follow to leader = (%.2f, %.2f, %.2f) ",  multi_vehicle_.usv3.follower_to_leader_pos(0),
             multi_vehicle_.usv3.follower_to_leader_pos(1), multi_vehicle_.usv3.follower_to_leader_pos(2));
}

void DataMan::PrintUSVFormationKeep() {
    util_log("usv1 follow to keep = (%.2f, %.2f, %.2f) ", multi_vehicle_.usv1.follower_keep_pos(0),
             multi_vehicle_.usv1.follower_keep_pos(1), multi_vehicle_.usv1.follower_keep_pos(2));
    util_log("usv2 follow to keep = (%.2f, %.2f, %.2f) ",  multi_vehicle_.usv2.follower_keep_pos(0),
             multi_vehicle_.usv2.follower_keep_pos(1), multi_vehicle_.usv2.follower_keep_pos(2));
    util_log("usv3 follow to keep = (%.2f, %.2f, %.2f) ",  multi_vehicle_.usv3.follower_keep_pos(0),
             multi_vehicle_.usv3.follower_keep_pos(1), multi_vehicle_.usv3.follower_keep_pos(2));
}

void DataMan::PrintData() {
    util_log("\n");
    util_log("---------------flight data-------------");
    PrinrDorneFlightDate();
    PrintDroneTargetPosData();
    PrintAvoidanceData();

    PrintBoatData();
    PrintBoatTargetPosData();
    if (leader_uav_) {
        PrintDroneFormationData();
        PrintDroneFormationKeep();
    }
    PrintUSVFormationData();
    PrintUSVFormationKeep();
    util_log("---------------data end-----------------");
}

multi_vehicle &DataMan::GetData() {
    return multi_vehicle_;
}

void DataMan::SetCallBack(IFlightDataCallback *dataCallback) {
    callback_ = dataCallback;
}

void DataMan::setCommand(int command) {
    command_ = command;
    multi_vehicle_.user_command = command;
    util_log("set command = %d", multi_vehicle_.user_command);
    if (callback_) {
        callback_->OnFlightDataUpdate(FDATA_MANUAL_COMMAND);
    }
}
void DataMan::getCommand(int& command) {
    command = command_;
}


