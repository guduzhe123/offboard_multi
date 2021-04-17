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
    class IPathFinder{
    public:
        IPathFinder() = default;

        virtual ~IPathFinder() {};

        virtual bool replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                            Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool collide) = 0;

        virtual bool planGlobalTraj(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos) = 0;

        virtual void planYaw(const Eigen::Vector3d& start_yaw) = 0;

        virtual void initPlanModules(const MP_Config &config, Sp<IMap> &map) = 0;

        virtual bool checkTrajCollision(double& distance) = 0;

        virtual LocalTrajData& getLocaldata() = 0;

        virtual MidPlanData& getPlanData() = 0;

        virtual void setGlobalWaypoints(const TVec3 &waypoints) = 0;

        virtual void updateSpeedLimit(const float max_speed, const float max_acc) = 0;

        LocalTrajData local_data_;
        GlobalTrajData global_data_;
        MidPlanData plan_data_;
    };
}

#endif //WINDAPPCORE_IPATHFINDER_H
