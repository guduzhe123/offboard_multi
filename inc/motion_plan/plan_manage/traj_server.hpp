//
// Created by zhouhua on 2020/8/31.
//

#ifndef OFFBOARD_TRAJ_SERVER_HPP
#define OFFBOARD_TRAJ_SERVER_HPP
#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "wa_ros_msgs/Bspline.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <wa_ros_msgs/DroneSpeedCtrl.h>
#include <ros/ros.h>


namespace fast_planner {
    class traj_server {
    public:
        traj_server();
        ~traj_server() = default;

        int onInit();
        void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                                  int id);
        void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                     const Eigen::Vector4d& color);
        void bsplineCallback(const wa_ros_msgs::BsplineConstPtr msg);
        void replanCallback(const std_msgs::Empty msg);
        void newCallback(const std_msgs::Empty msg);
        void odomCallbck(const nav_msgs::Odometry& msg);
        void visCallback(const ros::TimerEvent& e);
        void cmdCallback(const ros::TimerEvent& e);

    private:
        ros::Publisher cmd_vis_pub_, pos_cmd_pub, traj_pub_, drone_vel_sp_pub_;
        ros::Subscriber bspline_sub_, replan_sub_, new_sub_, odom_sub_;

        nav_msgs::Odometry odom;
        double pos_gain[3] = { 5.7, 5.7, 6.2 };
        double vel_gain[3] = { 3.4, 3.4, 4.0 };

        bool receive_traj_ = false;
        vector<NonUniformBspline> traj_;
        double traj_duration_;
        ros::Time start_time_;
        int traj_id_;
        double last_yaw_;
        double time_forward_;
        bool is_wind_application_;
        vector<Eigen::Vector3d> traj_cmd_, traj_real_;
    };
}

#endif //OFFBOARD_TRAJ_SERVER_HPP
