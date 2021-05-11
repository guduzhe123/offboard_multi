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
                    Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool collide) override ;
        bool planGlobalTraj(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos, const int formation_type,
                            const float formation_distance, const int usv_id, const PolynomialTraj gl_traj);

        void planYaw(const Eigen::Vector3d& start_yaw) override ;

        void initPlanModules(const MP_Config &config, Sp<IMap> &map) override ;
        void setGlobalWaypoints(const vector<TVec3> &waypoints) override;

        bool checkTrajCollision(double& distance) override ;
        bool checkLineAviable(TVec3 start_pt, TVec3 end_pt) override;

        LocalTrajData& getLocaldata() override ;
        MidPlanData& getPlanData() override;
        GlobalTrajData& getGlobalData() override;
        PolynomialTraj& getUSV2PolynomialTraj() override;
        PolynomialTraj& getUSV3PolynomialTraj() override;

        void updateSpeedLimit(const float max_speed, const float max_acc) override;

        PlanParameters pp_;
        LocalTrajData local_data_;
        MidPlanData plan_data_;
        EDTEnvironment::Ptr edt_environment_;
        GlobalTrajData global_data_;
        
    private:
        ros::Subscriber plan_state_sub_;
        /* main planning algorithms & modules */
        SDFMap::Ptr sdf_map_;

        unique_ptr<KinodynamicAstar> kino_path_finder_;
        vector<BsplineOptimizer::Ptr> bspline_optimizers_;
        OctoMap::Ptr octo_map_;
        MP_Config mp_config_;

        TVec3 drone_usv2_, drone_usv3_;
        PolynomialTraj usv2_gl_traj_;
        PolynomialTraj usv3_gl_traj_;

        void updateTrajInfo();

        // topology guided optimization

        void findCollisionRange(vector<Eigen::Vector3f>& colli_start, vector<Eigen::Vector3f>& colli_end);

        Eigen::MatrixXd reparamLocalTraj(double start_t, double &dt, double &duration, int step);
        Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double &dt,
                                         vector<Eigen::Vector3d> &start_end_derivative);

        void selectBestTraj(NonUniformBspline& traj);
        void refineTraj(NonUniformBspline& best_traj, double& time_inc);
        void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                            double& time_inc);

        // heading planning
        void calcNextYaw(const double& last_yaw, double& yaw);

        void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id,
                                 vector<Eigen::Vector3d> start_end_points);

        void planUSV2GlobalTraj(vector<Eigen::Vector3d> &leader_pos, Eigen::VectorXd &time);
        void planUSV3GlobalTraj(vector<Eigen::Vector3d> &leader_pos, Eigen::VectorXd &time);

        void initFormation();

        void
        calcMiniSnap(vector<Eigen::Vector3d> &inter_points, PolynomialTraj &gl_traj, Eigen::VectorXd &time, int usv_id);

        // !SECTION stable

        // SECTION developing

    public:
        typedef unique_ptr<FastPathFinder> Ptr;

        // !SECTION
    };
}  // namespace fast_planner

#endif