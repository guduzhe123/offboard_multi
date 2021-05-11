//
// Created by zhouhua on 2020/1/23.
//

#ifndef OFFBOARD_DATAMAN_HPP
#define OFFBOARD_DATAMAN_HPP

#include "Cinc.hpp"
#include "IMsgRosManager.hpp"

class IFlightDataCallback {
public:
    virtual void OnFlightDataUpdate(FDATA_TYPE data_type) = 0;
};

class DataMan {
public:
    static DataMan* getInstance();

    void OnInit(IMsgRosManager *msg_ros, bool is_uav, bool is_usv);

    void SetDroneData(const M_Drone &mDrone);

    void PrinrDorneFlightDate();

    void PrintBoatData();

    void PrintDroneTargetPosData();

    void PrintBoatTargetPosData();

    void PrintAvoidanceData();

    void PrintDroneFormationData();

    void PrintDroneFormationKeep();

    void PrintData();

    void
    SetUAVFormationData(bool is_formation, int leader_uav_id_, const TVec3 &follow_uav1, const TVec3 &follow_uav2,
                        const TVec3 &follow_uav3);

    void
    SetUAVFormationKeepData(const TVec3& follow_uav1, const TVec3& follow_uav2, const TVec3& follow_uav3, const TVec3& follow_uav4);

    void SetUSVFormationData(const multi_vehicle &m_multi_vehicles, bool is_formation);

    void SetAvoidanceData(const M_Drone_Avoidace& uav1, const M_Drone_Avoidace& uav2, const M_Drone_Avoidace& uav3, const M_Drone_Avoidace& uav4);

    void SetDroneControlData(const multi_vehicle &m_multi_vehicles);

    void PublishDroneControlData(const multi_vehicle &m_multi_vehicles);

    void SetBoatControlData(const multi_vehicle &m_multi_vehicles);

    void SetUSVMPControlData(const multi_vehicle &m_multi_vehicles, const int id);

    void SetUUVControlData(const multi_vehicle &m_multi_vehicles);

    void SetUSVAvoData(const bool usv1_usv2_crash, const bool usv1_usv3_crash, const bool usv2_usv3_crash);

    void SetUAVState(mavros_msgs::SetMode &m_mode);

    void SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id);

    void SetUAVLeader(M_Drone &leader_uav);

    void SetUSVLeader(M_Drone &leader_usv);

    void SetUUVLeader(M_Drone &leader_uuv);

    void SetUserCommand(const int value);

    void SetCallBack(IFlightDataCallback *dataCallback);

    void setCommand(int command);
    void getCommand(int& command);

    void setUSVHomePosition();

    void setOctomap(const octomap_msgs::Octomap &octmsg);

    multi_vehicle &GetData();

private:
    void PrintUSVFormationData();
    void PrintUSVFormationKeep();

    static DataMan* l_singleton;

    multi_vehicle multi_vehicle_;
    boost::mutex m_mutex;

    int leader_uav_ = 0;
    IMsgRosManager * msg_config_;
    IFlightDataCallback *callback_;
    int command_ = -1;

    bool is_uav_;
    bool is_usv_;
};


#endif //OFFBOARD_DATAMAN_HPP
