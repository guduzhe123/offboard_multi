#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include "motion_plan/bspline_opt/bspline_optimizer.h"
#include "motion_plan/bspline/non_uniform_bspline.h"
#include "motion_plan/path_searching/kinodynamic_astar.h"
#include "motion_plan/plan_env/edt_environment.h"
#include "motion_plan/plan_manage/plan_container.hpp"
#include "PCL/OctoMap.hpp"
#include <ros/ros.h>
#include "IPathFinder.h"

namespace fast_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

    class FastPathFinder : public IPathFinder{
        // SECTION stable
    public:
        FastPathFinder();

        ~FastPathFinder();

        /* main planning interface */
        bool replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                               Eigen::Vector3d end_pt, Eigen::Vector3d end_vel) override ;
        bool planGlobalTraj(const Eigen::Vector3d& start_pos);

        void planYaw(const Eigen::Vector3d& start_yaw) override ;

        void initPlanModules(const MP_Config &config, Sp<IMap> &map) override ;
        void setGlobalWaypoints(const TVec3 &waypoints, const TVec3 &start_point, const TVec3 &cur_pos) override;

        bool checkTrajCollision(double& distance) override ;

        LocalTrajData& getLocaldata() override ;
        MidPlanData& getPlanData() override;

        void updateSpeedLimit(const float max_speed, const float max_acc) override;

        PlanParameters pp_;
        LocalTrajData local_data_;
        MidPlanData plan_data_;
        EDTEnvironment::Ptr edt_environment_;
        
    private:
        ros::Subscriber plan_state_sub_;
        /* main planning algorithms & modules */
        SDFMap::Ptr sdf_map_;
        GlobalTrajData global_data_;

        unique_ptr<KinodynamicAstar> kino_path_finder_;
        vector<BsplineOptimizer::Ptr> bspline_optimizers_;
        OctoMap::Ptr octo_map_;
        MP_Config mp_config_;

        void updateTrajInfo();

        // topology guided optimization

        void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                                vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

        Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
        Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

        void selectBestTraj(NonUniformBspline& traj);
        void refineTraj(NonUniformBspline& best_traj, double& time_inc);
        void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                            double& time_inc);

        // heading planning
        void calcNextYaw(const double& last_yaw, double& yaw);

        // !SECTION stable

        // SECTION developing

    public:
        typedef unique_ptr<FastPathFinder> Ptr;

        // !SECTION
    };
}  // namespace fast_planner

#endif