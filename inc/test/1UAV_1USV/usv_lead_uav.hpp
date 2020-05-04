//
// Created by zhouhua on 2020/5/3.
//

#ifndef OFFBOARD_USV_LEAD_UAV_HPP
#define OFFBOARD_USV_LEAD_UAV_HPP

#include "Cinc.hpp"
#include "uav_ros_Manager.hpp"
#include "usv_ros_Manager.hpp"
#include "DataMan.hpp"

class usv_lead_uav {
public:
    usv_lead_uav();
    ~usv_lead_uav() = default;
    void onInit();
    void getData();
    void doProgress();
    static usv_lead_uav* getInstance();

    enum UAV_TYPE {
        TAKEOFF,
        FOLLOW,
    };

private:
    void usvLocalPositionSp();
    void usvlocalControl();
    void uavlocalControl();
    bool pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos);

    uav_ros_Manager::Ptr uav_control_;
    usv_ros_Manager::Ptr usv_control_;

    multi_vehicle multiVehicle;
    static usv_lead_uav* l_pInst;

    std::vector<geometry_msgs::PoseStamped> usv_way_points;
    geometry_msgs::PoseStamped way_point;
    geometry_msgs::PoseStamped uav_way_point;
    geometry_msgs::PoseStamped current_usv_local_pos_;
    geometry_msgs::PoseStamped current_uav_local_pos_;
    int uav_state_;
    bool uav_reached_;
};
#endif //OFFBOARD_USV_LEAD_UAV_HPP
