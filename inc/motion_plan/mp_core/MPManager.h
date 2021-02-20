//
// Created by zhouhua on 2020/12/6.
//

#ifndef WINDAPPCORE_MPMANAGER_H
#define WINDAPPCORE_MPMANAGER_H


#include "Cinc.hpp"
#include "motion_plan/bspline/non_uniform_bspline.h"
#include "FastPathFinder.h"
#include "MPPublisher.h"
#include "PCL/IMap.hpp"
#include "PCL/OctoMap.hpp"
#include "offboard/Bspline.h"


using namespace fast_planner;

enum REPLAN_MODE {
    POSITION_CUR = 0,
    POSITION_WITHOUT_CUR = 1,
    VELOCITY_WITH_CUR = 2,
    VELOCITY_WITHOUT_CUR = 3
};

struct Drone_State{
    TVec3 drone_pos;
    TVec3 drone_vel;
    TVec3 drone_acc;
    TVec3 drone_attitude;
};

class MPManager{
public:
    MPManager(const MP_Config& config);

    void SetMpEnable(bool is_enable);

    void OnUpdateDroneStatus(const TVec3 &drone_pos, const TVec3 &drone_vel, const TVec3 &drone_acc,
                             const TVec3 &drone_attitude);

    void OnUpdateTargetPos(const TVec3& target_pos);

    void OnUpdateMaxSpeed(float speed_max);

    void OnUpdateDroneHeading(float drone_heading);

    bool GetControlOutput(TVec3& vector_eus);

    void ProcessState();

    void updateMotionPlan(const float dist, const TVec3 &insp_vec,
                          const vector<TVec3> &waypoints);

    void bsplineResult(const offboard::Bspline& msg);

    void OnUpdateOctomap(const octomap_msgs::Octomap &msg);

    void updateEndVel(const TVec3 &end_vel);
private:
    enum MP_EXEC_STATE {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        REPLAN_NEW
    };

    ros::NodeHandle nh_;
    MP_Config mp_config_;
    Drone_State drone_st_;
    MP_EXEC_STATE mp_state_;
    TVec3 start_pt_,start_vel_,start_acc_, end_pos_, end_vel_;
    TVec3 pre_end_pt_;
    ros::Time start_time_;
    vector<NonUniformBspline> real_traj_;
    vector<NonUniformBspline> plan_traj_;
    double traj_duration_;

    float dist_config_;
    TVec3 insp_vec_ENU_;
    float min_dist_to_line_ = 100000;

    Sp<IPathFinder> path_finder_;
    Sp<MPPublisher> mp_publisher_;
    Sp<IMap> mp_map_;
    //Sp<FastPathFinder> planner_manager_;
    //PlanningVisualization::Ptr visualization_;

    vector<TVec3> traj_cmd_;

    bool receive_traj_;
    bool has_drone_update_;
    TVec3 init_target_pos_;
    int path_find_fail_timer_;

    // private functions
    void CalcDistToCenter(float &dist, const TVec3 &cur_pos, TVec3 start_pos, TVec3 end_pos);

    bool CallKinodynamicReplan();

    void ChangeExecState(MP_EXEC_STATE new_state, string pos_call);

    void initAfterTriggered(const TVec3& target_pos);

    void changeToTurbineFrame(TVec3 &pnt, TVec3 &pos_in_turbine_EUS);
};

#endif //WINDAPPCORE_MPMANAGER_H