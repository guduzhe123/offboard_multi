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
void DataMan::OnInit(IMsgRosManager *msg_ros, bool is_uav, bool is_usv) {
    msg_config_ = msg_ros;
    is_uav_ = is_uav;
    is_usv_ = is_usv;
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
        chlog::info("data","[DataMan]:data man is lead usv in formation ", is_formation);
    }
}


void DataMan::SetUAVState(mavros_msgs::SetMode &m_mode) {
    msg_config_->SetUAVState(m_mode);
}

void DataMan::SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) {
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
    TVec3 target_pos{multi_vehicle_.usv1.target_local_pos_sp.pose.position.x,
                     multi_vehicle_.usv1.target_local_pos_sp.pose.position.y,
                     0};
    //chlog::info("motion_plan", "222 target_pos = ", toStr(target_pos));
    msg_config_->PublishBoatPosControl(m_multi_vehicles);
}

void DataMan::SetUSV1MPControlData(const multi_vehicle & m_multi_vehicles) {
    multi_vehicle_.usv1.target_local_pos_sp = m_multi_vehicles.usv1.target_local_pos_sp;
    msg_config_->PublishUSV1PosControl(m_multi_vehicles);
}

void DataMan::SetUUVControlData(const multi_vehicle &m_multi_vehicles) {
    {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        multi_vehicle_.uuv1.target_local_pos_sp = m_multi_vehicles.uuv1.target_local_pos_sp;
        multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicles.leader_usv.target_local_pos_sp;
    }
    msg_config_->PublishUUVPosControl(m_multi_vehicles);
}

void DataMan::SetUSVAvoData(const bool usv1_crash, const bool usv2_crash, const bool usv3_crash) {
    msg_config_->SetUSVAvoData(usv1_crash, usv2_crash, usv3_crash);
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

void DataMan::setOctomap(const octomap_msgs::Octomap &octmsg) {

}

void DataMan::PrinrDorneFlightDate() {
    chlog::info("data","uav1 current pos = (", multi_vehicle_.uav1.current_local_pos.pose.position.x, ", ",
             multi_vehicle_.uav1.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.uav1.current_local_pos.pose.position.z , ")");
    chlog::info("data","uav2 current pos = (", multi_vehicle_.uav2.current_local_pos.pose.position.x,", ",
             multi_vehicle_.uav2.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.uav2.current_local_pos.pose.position.z, ")");
    chlog::info("data","uav3 current pos = (", multi_vehicle_.uav3.current_local_pos.pose.position.x,", ",
             multi_vehicle_.uav3.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.uav3.current_local_pos.pose.position.z, ")");
    chlog::info("data","uav4 current pos = (", multi_vehicle_.uav4.current_local_pos.pose.position.x,", ",
             multi_vehicle_.uav4.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.uav4.current_local_pos.pose.position.z, ")");
    chlog::info("data","leader uav current pos = (", multi_vehicle_.leader_uav.current_local_pos.pose.position.x,", ",
             multi_vehicle_.leader_uav.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.leader_uav.current_local_pos.pose.position.z, ")");
}

void DataMan::PrintBoatData() {
    chlog::info("data","[DataMan]:usv1 current pos = (", multi_vehicle_.usv1.current_local_pos.pose.position.x, ", ",
             multi_vehicle_.usv1.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.usv1.current_local_pos.pose.position.z, ")");
    chlog::info("data","[DataMan]:usv2 current pos = (", multi_vehicle_.usv2.current_local_pos.pose.position.x,", ",
             multi_vehicle_.usv2.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.usv2.current_local_pos.pose.position.z, ")");
    chlog::info("data","[DataMan]:usv3 current pos = (", multi_vehicle_.usv3.current_local_pos.pose.position.x, ", ",
             multi_vehicle_.usv3.current_local_pos.pose.position.y, ", ",
             multi_vehicle_.usv3.current_local_pos.pose.position.z, ")");

}

void DataMan::PrintDroneTargetPosData() {
    chlog::info("data","[DataMan]:uav1 current pos = (", multi_vehicle_.uav1.target_local_pos_sp.pose.position.x, ", ",
                multi_vehicle_.uav1.target_local_pos_sp.pose.position.y, ", ",
                multi_vehicle_.uav1.target_local_pos_sp.pose.position.z, ")");
    chlog::info("data","[DataMan]:uav2 current pos = (", multi_vehicle_.uav2.target_local_pos_sp.pose.position.x,", ",
                multi_vehicle_.uav2.target_local_pos_sp.pose.position.y, ", ",
                multi_vehicle_.uav2.target_local_pos_sp.pose.position.z, ")");
    chlog::info("data","[DataMan]:uav3 current pos = (", multi_vehicle_.uav3.target_local_pos_sp.pose.position.x, ", ",
                multi_vehicle_.uav3.target_local_pos_sp.pose.position.y, ", ",
                multi_vehicle_.uav3.target_local_pos_sp.pose.position.z, ")");
    chlog::info("data","[DataMan]:uav4 current pos = (", multi_vehicle_.uav4.target_local_pos_sp.pose.position.x, ", ",
                multi_vehicle_.uav4.target_local_pos_sp.pose.position.y, ", ",
                multi_vehicle_.uav4.target_local_pos_sp.pose.position.z, ")");
}

void DataMan::PrintBoatTargetPosData() {
    chlog::info("data","[DataMan]:usv1 target local pos = (", multi_vehicle_.usv1.target_local_pos_sp.pose.position.x, ", ",
             multi_vehicle_.usv1.target_local_pos_sp.pose.position.y, ", ",
             multi_vehicle_.usv1.target_local_pos_sp.pose.position.z, ")");
    chlog::info("data","[DataMan]:usv2 target local pos = (", multi_vehicle_.usv2.target_local_pos_sp.pose.position.x, ", ",
             multi_vehicle_.usv2.target_local_pos_sp.pose.position.y, ", ",
             multi_vehicle_.usv2.target_local_pos_sp.pose.position.z, ")");
    chlog::info("data","[DataMan]:usv3 target local pos = (", multi_vehicle_.usv3.target_local_pos_sp.pose.position.x,", ",
             multi_vehicle_.usv3.target_local_pos_sp.pose.position.y, ", ",
             multi_vehicle_.usv3.target_local_pos_sp.pose.position.z, ")");
}

void DataMan::PrintAvoidanceData() {
/*    chlog::info("data","[DataMan]:get output height_avoidance_uav1_ = , height_avoidance_uav2_ = %.2f, "
             "height_avoidance_uav3_ = %.2f, height_avoidance_uav4_ = %.2f",
             multi_vehicle_.uav1.avoidance_pos.z(), multi_vehicle_.uav2.avoidance_pos.z(),
             multi_vehicle_.uav3.avoidance_pos.z(), multi_vehicle_.uav4.avoidance_pos.z());*/
}

void DataMan::PrintDroneFormationData() {
    chlog::info("data","[DataMan]:is uav formation = ", multi_vehicle_.leader_uav.is_formation, ", uav formation leader = ",
            leader_uav_);
    chlog::info("data","[DataMan]:uav1 follow to leader = ( ",multi_vehicle_.uav1.follower_to_leader_pos(0),", ",
             multi_vehicle_.uav1.follower_to_leader_pos(1),", ",
             multi_vehicle_.uav1.follower_to_leader_pos(2), ")");
    chlog::info("data","[DataMan]:uav2 follow to leader = ( ", multi_vehicle_.uav2.follower_to_leader_pos(0),", ",
             multi_vehicle_.uav2.follower_to_leader_pos(1), ", ",
             multi_vehicle_.uav2.follower_to_leader_pos(2), ")");
    chlog::info("data","[DataMan]:uav3 follow to leader = ( ", multi_vehicle_.uav3.follower_to_leader_pos(0),", ",
             multi_vehicle_.uav3.follower_to_leader_pos(1), ", ",
             multi_vehicle_.uav3.follower_to_leader_pos(2), ")");
    chlog::info("data","[DataMan]:uav4 follow to leader = ( ", multi_vehicle_.uav4.follower_to_leader_pos(0),", ",
             multi_vehicle_.uav4.follower_to_leader_pos(1), ", ",
             multi_vehicle_.uav4.follower_to_leader_pos(2), ")");

}

void DataMan::PrintDroneFormationKeep() {
    chlog::info("data","[DataMan]:uav1 formation keep local pos = ( ", multi_vehicle_.uav1.follower_keep_pos(0), ", ",
             multi_vehicle_.uav1.follower_keep_pos(1),  ", ",multi_vehicle_.uav1.follower_keep_pos(2), ")");
    chlog::info("data","[DataMan]:uav2 formation keep local pos  = ( ", multi_vehicle_.uav2.follower_keep_pos(0), ", ",
             multi_vehicle_.uav2.follower_keep_pos(1),  ", ",multi_vehicle_.uav2.follower_keep_pos(2), ")");
    chlog::info("data","[DataMan]:uav3 formation keep local pos  = ( ", multi_vehicle_.uav3.follower_keep_pos(0), ", ",
             multi_vehicle_.uav3.follower_keep_pos(1),  ", ",multi_vehicle_.uav3.follower_keep_pos(2), ")");
    chlog::info("data","[DataMan]:uav4 formation keep local pos  = ( ", multi_vehicle_.uav4.follower_keep_pos(0), ", ",
             multi_vehicle_.uav4.follower_keep_pos(1),  ", ",multi_vehicle_.uav4.follower_keep_pos(2), ")");
}

void DataMan::PrintUSVFormationData() {
    chlog::info("data","[DataMan]:is usv formation = ", multi_vehicle_.leader_usv.is_formation,
            ", usv formation leader = ", multi_vehicle_.leader_usv.drone_id);
    chlog::info("data","[DataMan]:usv1 follow to leader = ( ", multi_vehicle_.usv1.follower_to_leader_pos(0), ", ",
            multi_vehicle_.usv1.follower_to_leader_pos(1),
            ", ",multi_vehicle_.usv1.follower_to_leader_pos(2), ")");
    chlog::info("data","[DataMan]:usv2 follow to leader = ( ",  multi_vehicle_.usv2.follower_to_leader_pos(0), ", ",
             multi_vehicle_.usv2.follower_to_leader_pos(1),
             ", ",multi_vehicle_.usv2.follower_to_leader_pos(2), ")");
    chlog::info("data","[DataMan]:usv3 follow to leader = ( ",  multi_vehicle_.usv3.follower_to_leader_pos(0), ", ",
             multi_vehicle_.usv3.follower_to_leader_pos(1),
             ", ",multi_vehicle_.usv3.follower_to_leader_pos(2), ")");
}

void DataMan::PrintUSVFormationKeep() {
    chlog::info("data","[DataMan]:usv1 follow to keep = ( ", multi_vehicle_.usv1.follower_keep_pos(0), ", ",
             multi_vehicle_.usv1.follower_keep_pos(1),
             ", ",multi_vehicle_.usv1.follower_keep_pos(2), ")");
    chlog::info("data","[DataMan]:usv2 follow to keep = ( ",  multi_vehicle_.usv2.follower_keep_pos(0), ", ",
             multi_vehicle_.usv2.follower_keep_pos(1),  ", ",multi_vehicle_.usv2.follower_keep_pos(2), ")");
    chlog::info("data","[DataMan]:usv3 follow to keep = ( ",  multi_vehicle_.usv3.follower_keep_pos(0), ", ",
             multi_vehicle_.usv3.follower_keep_pos(1),  ", ",multi_vehicle_.usv3.follower_keep_pos(2), ")");
}

void DataMan::PrintData() {
    chlog::info("data","[DataMan]:\n");
    chlog::info("data","[DataMan]:---------------flight data-------------");
    if (is_uav_) {
        PrinrDorneFlightDate();
        PrintDroneTargetPosData();
        PrintAvoidanceData();
        if (leader_uav_) {
            PrintDroneFormationData();
            PrintDroneFormationKeep();
        }
    }

    if (is_usv_) {
        PrintBoatData();
        PrintBoatTargetPosData();
        PrintUSVFormationData();
        PrintUSVFormationKeep();
    }

    chlog::info("data","[DataMan]:---------------data end-----------------");
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
    chlog::info("data","[DataMan]:set command = ", multi_vehicle_.user_command);
    if (callback_) {
        callback_->OnFlightDataUpdate(FDATA_MANUAL_COMMAND);
    }
}
void DataMan::getCommand(int& command) {
    command = command_;
}


