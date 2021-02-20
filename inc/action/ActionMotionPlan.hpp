//
// Created by zhouhua on 2020/6/1.
//

#ifndef WINDAPPCORE_ACTIONMOTIONPLAN_HPP
#define WINDAPPCORE_ACTIONMOTIONPLAN_HPP

#include "IActionDrone.hpp"
#include "MPManager.h"

namespace afcore {
    class ActionMotionPlan : public IActionDrone {
    public:
        /// @brief Constructor
        ///
        /// Constructor of the class ActionMotionPlan.
        /// @param name the name of this class
        ActionMotionPlan(const string& name = "drone MP");

        /// @brief Destructor
        ///
        /// Destructor of the class ActionDroneMoving.
        ///
        ~ActionMotionPlan() override = default;

        /// @brief Initialization Function
        ///
        /// Initialize the class members
        /// init and update target position and heading status
        bool OnInit(const MP_Config& mpConfig);

        /// @brief Start the program
        void Run() override;

        /// @brief Stop the program
        void Stop() override;

        /// @brief Get Output
        ///
        /// This function used to update drone action output
        /// @param output Get the output value
        void GetOutput(TDroneOutput &output) override;

        /// @brief get drone status
        ///
        /// get current status of drone and calculate control outputs.
        /// @param drone_pos The position of drone
        /// @param drone_speed The speed of drone
        /// @param heading The heading of drone
        void SetStatus(const TVec3 &drone_pos, const TVec3 &drone_speed, float heading) override;

        void updateSpeedLimit(const float& speed_limit);

        void updateHeadingControl(const TVec3 &tip_center);

        void OnUpdateTargetPoint(const TVec3 &new_target_point);

        void updateEndVel(const TVec3 &end_vel);

        void updateMotionPlan(const float dist, const TVec3 &insp_vec,
                              const vector<TVec3> &waypoints);

        void updateFlightData(const TFlightData &data);

        void updateCirclePoint(const TVec3& tip_pos);

        /// @brief Get Action Type
        ActionType GetType() override {
            return ACTION_DRONE_MOTION_PLAN;
        }

    protected:
        string name_;

    private:
        Sp<MPManager> mp_manager_;
        TDroneOutput output_;
        MP_Config mp_config_;
        Drone_State drone_state_;
        RoPID m_yaw_pid_ = PID(0.5, -0.5, 0.25, 0, 0.01);
    };

}
#endif //WINDAPPCORE_ACTIONMOTIONPLAN_HPP
