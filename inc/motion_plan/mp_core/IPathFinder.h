//
// Created by zhouhua on 2020/12/9.
//

#ifndef OFFBOARD_IPATHFINDER_H
#define OFFBOARD_IPATHFINDER_H

#include "Cinc.hpp"
#include <ros/ros.h>
#include "motion_plan/plan_manage/plan_container.hpp"
#include "motion_plan/plan_env/IMap.hpp"

namespace fast_planner{
    class IPathFinder{
    public:
        IPathFinder() = default;

        virtual ~IPathFinder() {};

        virtual bool replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                            Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool collide) = 0;

        virtual bool
        planGlobalTraj(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos, const int formation_type,
                       const float formation_distance, const int usv_id, const PolynomialTraj gl_traj) = 0;

        virtual void planYaw(const Eigen::Vector3d& start_yaw) = 0;

        virtual void initPlanModules(const MP_Config &config, Sp<IMap> &map) = 0;

        virtual bool checkTrajCollision(double& distance) = 0;

        virtual bool checkLineAviable(TVec3 start_pt, TVec3 end_pt) = 0;

        virtual LocalTrajData& getLocaldata() = 0;

        virtual MidPlanData& getPlanData() = 0;

        virtual GlobalTrajData& getGlobalData() = 0;

        virtual void setGlobalWaypoints(const vector<TVec3> &waypoints) = 0;

        virtual void updateSpeedLimit(const float max_speed, const float max_acc) = 0;

        virtual PolynomialTraj& getUSV2PolynomialTraj() = 0;
        virtual PolynomialTraj& getUSV3PolynomialTraj() = 0;
    };
}

#endif //OFFBOARD_IPATHFINDER_H
