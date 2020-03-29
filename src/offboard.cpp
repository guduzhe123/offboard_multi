/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include "math.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
//#include <mavros_msgs/>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
//    ROS_INFO("state: %d",current_state.armed);
}

mavros_msgs::State current_state_uav;
void state_cb_uav(const mavros_msgs::State::ConstPtr& msg){
    current_state_uav = *msg;
//    ROS_INFO("state: %d",current_state.armed);
}


mavros_msgs:: VFR_HUD current_vfr_hud;
float curr_altitude;

void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_vfr_hud = *msg;
//    curr_altitude = current_vfr_hud.altitude;
//    ROS_INFO("altitude = %d",  current_vfr_hud.altitude);
}

geometry_msgs::PoseStamped current_local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = *msg;
    curr_altitude = current_local_pos.pose.position.z;
//    ROS_INFO("pose.z = %.2f", current_local_pos.pose.position.z);
}

bool pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < 2.0f;
}

int main(int argc, char **argv)
{
    ROS_INFO("publish");
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber state_sub_uav = nh.subscribe<mavros_msgs::State>
            ("uav/mavros/state", 10, state_cb_uav);

    ros::Publisher test = nh.advertise<mavros_msgs::State>
            ("uav/mavros/state", 10);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 5);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10, vrf_hud_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Publisher gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 1000);
    ros::Publisher global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1000);
    ros::Publisher g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 100);
/*    ros::Subscriber pos_local = nh.subscribe<geometry_msgs::PoseStamped>
            ()*/

    ros::Rate rate(10.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 10;
    pose.pose.position.y = 10;
    pose.pose.position.z = 10;

    std::vector<geometry_msgs::PoseStamped> usv_way_points;
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 10;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 10;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = -30;
    way_point.pose.position.z = 10;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 10;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
    

    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.twist.linear.x = 1;
    vel_cmd.twist.linear.y = 1;
    vel_cmd.twist.linear.z = 1;

    vel_cmd.twist.angular.x = 0.1;
    vel_cmd.twist.angular.y = 0.2;
    vel_cmd.twist.angular.z = 0.5;



    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int count = 0;


    while(ros::ok()){
/*        gps_global_pos_pub.publish(gps_pos);
        global_pos_pub.publish(gps_pos);*/
//        g_speed_control_pub.publish(vel_cmd);


        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (!usv_way_points.empty()) {
            way_point = usv_way_points.back();
            if (pos_reached(current_local_pos, usv_way_points.back())) {
                ROS_INFO("Finished one way point = (%.2f, %.2f, %.2f)",
                         usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                         usv_way_points.back().pose.position.z);
                usv_way_points.pop_back();

                if (!usv_way_points.empty()) {
                    ROS_INFO("Goto next way point = (%.2f, %.2f, %.2f)",
                             usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                             usv_way_points.back().pose.position.z);
                } else {
                    ROS_INFO("Finish all target points!");
                }
            }
        }
        test.publish(current_state);
        local_pos_pub.publish(way_point);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
