//
// Created by zhouhua on 2020/5/3.
//

#ifndef OFFBOARD_USV_LEAD_UAV_HPP
#define OFFBOARD_USV_LEAD_UAV_HPP

#include "Cinc.hpp"
#include "test/2uav-usv/uav1_ros_Manager.hpp"
#include "test/2uav-usv/uav2_ros_Manager.hpp"
#include "test/2uav-usv/usv1_ros_Manager.hpp"
#include "DataMan.hpp"
#include "Calculate.hpp"

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
        FORMATION,
        FOLLOW,
        RETURN
    };

private:
    void usvLocalPositionSp();
    void usvlocalControl();
    void uavlocalControl();
    void GetTakeoffPos(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_first_local);
    bool pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos);
    void formation(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_formation);
    void checkCollision(double danger_distance);

    uav_ros_Manager::Ptr uav1_control_;
    uav2_ros_Manager::Ptr uav2_control_;
    usv1_ros_Manager::Ptr usv_control_;

    multi_vehicle multiVehicle;
    static usv_lead_uav* l_pInst;
    GlobalPosition master_start_gps_, slave_takeoff_gps_;

    std::vector<geometry_msgs::PoseStamped> usv_way_points;
    geometry_msgs::PoseStamped way_point;
    geometry_msgs::PoseStamped uav_way_point;
    geometry_msgs::PoseStamped current_usv_local_pos_;
    geometry_msgs::PoseStamped current_uav_local_pos_;
    TVec3 follow_slave_first_local_;
    TVec3 follow_slave_formation_;
    int uav_state_;
    bool uav_reached_;
    bool is_get_takeoff_pos_;
    bool is_avoidance_;
    double formation_distance_;
    int command_ ;
    double danger_distance_;
};
#endif //OFFBOARD_USV_LEAD_UAV_HPP
