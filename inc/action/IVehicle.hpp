//
// Created by zhouhua on 2020/1/4.
//

#ifndef OFFBOARD_IVEHICLE_HPP
#define OFFBOARD_IVEHICLE_HPP

#include "Cinc.hpp"
class Ivehicle {
public:
    Ivehicle();

    virtual ~Ivehicle() = default;

    virtual void uav_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) = 0;

    virtual void uav_state_cb(const mavros_msgs::State::ConstPtr& msg) = 0;

    virtual void uav_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) = 0;

    virtual void uav_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg) = 0;

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local_pos;
    geometry_msgs::PoseStamped target_pose;
    sensor_msgs::NavSatFix current_global_pos;

    ros::Subscriber state_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber multi_formation_sub;
    ros::Subscriber global_pos_sub;

    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    ros::Publisher local_pos_pub;
    ros::Publisher global_pos_pub;
};
#endif //OFFBOARD_IVEHICLE_HPP
