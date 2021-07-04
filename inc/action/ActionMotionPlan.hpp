//
// Created by zhouhua on 2020/6/1.
//

#ifndef OFFBOARD_ACTIONMOTIONPLAN_HPP
#define OFFBOARD_ACTIONMOTIONPLAN_HPP

#include "IControlFunction.hpp"
#include "motion_plan/mp_core/MPManager.h"
#include "DataMan.hpp"
#include "USV2ActionMotionPlan.hpp"
#include "USV3ActionMotionPlan.hpp"

class ActionMotionPlan : public IControlFunction {
public:
    enum PlanState {
        ST_INIT,
        ST_RUNNING
    };

    /// @brief Constructor
    ///
    /// Constructor of the class ActionMotionPlan.
    /// @param name the name of this class
    ActionMotionPlan();

    /// @brief Destructor
    ///
    /// Destructor of the class ActionDroneMoving.
    ///
    ~ActionMotionPlan() override = default;

    /// @brief Initialization Function
    ///
    /// Initialize the class members
    /// init and update target position and heading status

    void Oninit(const int config) override ;

    bool initMP(const MP_Config& mpConfig);

    /// @brief Start the program
    void DoProgress() override;

    /// @brief Get Output
    ///
    /// This function used to update drone action output
    /// @param output Get the output value
    void SetFunctionOutPut() override;

    /// @brief get drone status
    ///
    /// get current status of drone and calculate control outputs.
    /// @param drone_pos The position of drone
    /// @param drone_speed The speed of drone
    /// @param heading The heading of drone
    void GetData() override;

    void updateSpeedLimit(const float& speed_limit);

    void updateHeadingControl(const TVec3 &tip_center);

    void OnUpdateTargetPoint(const TVec3 &new_target_point);

    void updateEndVel(const TVec3 &end_vel);

    void updateMotionPlan(const float dist, const TVec3 &insp_vec,
                          const vector<TVec3> &waypoints);

    void updateFlightData();

    void updateCirclePoint(const TVec3& tip_pos);

    void setEnable(bool enable);

    void SetStatus(const TVec3 &drone_pos, const TVec3 &drone_speed, float heading);

    void getUSV1GlobalTraj(const GlobalTrajData &global_data);

    void initNh(ros::NodeHandle& nh);

    void initFormation(TVec3 &usv2_form, TVec3 &usv3_form);

    static ActionMotionPlan* getInstance();


protected:
    string name_;

private:
    Sp<MPManager> mp_manager_;
    TDroneOutput output_;
    MP_Config mp_config_;
    M_Drone drone_state_;

    TVec3 usv1_drone_pos_;
    multi_vehicle m_multi_vehicle_;
    bool is_enable_;
    GlobalTrajData usv1_global_data_;
    ros::NodeHandle nh_;
    ros::Time time_;

    PolynomialTraj gl_traj;
    bool init_follower_;
    bool init_usv2_;
    bool init_usv3_;
    int m_state_;
    TVec3 start_pos_;
    TVec3 drone_usv2_;
    TVec3 drone_usv3_;
    bool init_time_;
    ros::Time t0, t1;

    vector<TVec3> goals_;
    void calcuLineDir(const vector<TVec3> &goal, TVec3 &usv2_pos_sp, TVec3 &usv3_pos_sp);
    void getFollowerSp(const TVec3 &drone_move, const Eigen::Matrix3f &rotMatrix, TVec3 &target_pos);
};

class ActionMotionPlanFactory : public IFunctionFactory {
public:
    ~ActionMotionPlanFactory() {};

    IControlFunction* FunctionCreator()  {
        return ActionMotionPlan::getInstance();
    }
};

#endif //OFFBOARD_ACTIONMOTIONPLAN_HPP
