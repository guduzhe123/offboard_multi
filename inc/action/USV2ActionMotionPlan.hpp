//
// Created by zhouhua on 2020/6/1.
//

#ifndef WINDAPPCORE_USV2_ACTIONMOTIONPLAN_HPP
#define WINDAPPCORE_USV2_ACTIONMOTIONPLAN_HPP

#include "IControlFunction.hpp"
#include "motion_plan/mp_core/MPManager.h"
#include "DataMan.hpp"

class USV2ActionMotionPlan : public IControlFunction {
public:
    /// @brief Constructor
    ///
    /// Constructor of the class USV2ActionMotionPlan.
    /// @param name the name of this class
    USV2ActionMotionPlan();

    /// @brief Destructor
    ///
    /// Destructor of the class ActionDroneMoving.
    ///
    ~USV2ActionMotionPlan() override = default;

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
    void initNh(ros::NodeHandle &nh, const shared_ptr<IMap> &IMap);

    void setPolyTraj(PolynomialTraj& poly_traj);

    static USV2ActionMotionPlan* getInstance();


protected:
    string name_;

private:
    Sp<MPManager> mp_manager_;
    TDroneOutput output_;
    MP_Config mp_config_;
    M_Drone drone_state_;

    TVec3 drone_pos_;
    multi_vehicle m_multi_vehicle_;
    bool is_enable_;
    GlobalTrajData usv1_global_data_;
    ros::NodeHandle nh_;
    shared_ptr<IMap> IMap_;
    int test_num;
};

class USV2ActionMotionPlanFactory : public IFunctionFactory {
public:
    ~USV2ActionMotionPlanFactory() {};

    IControlFunction* FunctionCreator()  {
        return USV2ActionMotionPlan::getInstance();
    }
};

#endif //WINDAPPCORE_USV2_ACTIONMOTIONPLAN_HPP