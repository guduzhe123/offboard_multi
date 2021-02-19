//
// Created by zhouhua on 2020/12/9.
//

#ifndef WINDAPPCORE_IPATHFINDER_H
#define WINDAPPCORE_IPATHFINDER_H

#include "Cinc.hpp"
#include <ros/ros.h>
#include "motion_plan/plan_manage/plan_container.hpp"
#include "PCL/IMap.hpp"

namespace fast_planner{
    struct MP_Config{
        TVec3 start_pos;
        TVec3 end_pos;
        float exp_vel;
        float max_vel;
        float max_acc;
        float safe_zone_r; // the flight corrider radius.
        float m_drone_heading;
        float target_heading;
        bool is_enable = false;
        int control_mode;
        MotionPlanState mp_plan_state;
        double safe_dist;
        double replan_thresh;
        double plan_horizon; // fitting line planning length
        TVec3 m_toward_point;
        bool is_track_point = false;
        bool is_speed_mode = false;
    };

    class IPathFinder{
    public:
        IPathFinder() = default;

        virtual ~IPathFinder() {};

        virtual bool replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                            Eigen::Vector3d end_pt, Eigen::Vector3d end_vel) = 0;

        virtual void planYaw(const Eigen::Vector3d& start_yaw) = 0;

        virtual void initPlanModules(const MP_Config &config, Sp<IMap> &map) = 0;

        virtual bool checkTrajCollision(double& distance) = 0;

        virtual LocalTrajData& getLocaldata() = 0;

        virtual MidPlanData& getPlanData() = 0;

        virtual void setGlobalWaypoints(const TVec3 &waypoints, const TVec3 &start_point,
                                                    const TVec3 &cur_pos) = 0;

        virtual void updateSpeedLimit(const float max_speed, const float max_acc) = 0;
    };
}

#endif //WINDAPPCORE_IPATHFINDER_H
