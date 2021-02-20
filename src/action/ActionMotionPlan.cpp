//
// Created by zhouhua on 2020/6/1.
//

#include "ActionMotionPlan.hpp"

namespace afcore {
    ActionMotionPlan::ActionMotionPlan(const string &name) :
            IActionDrone(name) {
        name_ = name;
    }

    bool ActionMotionPlan::OnInit(const MP_Config &mpConfig) {
        chlog::info("af_core", "motion_plan init! target_pos = " + toStr(mpConfig.end_pos)
                               + ", target_heading = " + to_string2(mpConfig.target_heading));
        mp_config_ = mpConfig;
        mp_manager_ = makeSp<MPManager>(mp_config_);
        output_.target_heading = mpConfig.m_drone_heading;
        mp_manager_->SetMpEnable(true);
        return false;
    }

    void ActionMotionPlan::Run() {
        if (IsEnable()) {
            mp_manager_->ProcessState();
        } else {
            mp_manager_->SetMpEnable(false);
        }
    }

    void ActionMotionPlan::Stop() {
        SetEnable(false);
    }

    void ActionMotionPlan::GetOutput(TDroneOutput &output) {
        if (mp_config_.is_track_point && (!isnan(drone_state_.drone_pos.x())
                                          || !isnan(drone_state_.drone_pos.y()) ||
                                          !isnan(drone_state_.drone_pos.z()))) {
            TLine line(drone_state_.drone_pos, mp_config_.m_toward_point);
            TVec3 d_pos = line.dv();
            TGeoRot geoRot;
            geoRot.fromVec(d_pos);
            output_.target_heading = geoRot.h;
            chlog::info("af_data", "Action motion plan mp_config_.is_track_point = ", mp_config_.is_track_point,
                        ", mp_config_.m_toward_point = " + toStr(mp_config_.m_toward_point));
        } else {
            output_.target_heading = mp_config_.target_heading;
        }
        output = output_;
        output_.is_speed_mode = mp_config_.is_speed_mode;

        // stop if cannot get motion plan result;
        if (!mp_manager_->GetControlOutput(output_.m_vector)) {
            output_.m_vector = TVec3{0, 0, 0};
            output_.is_speed_mode = true;
            return;
        }

        if (mp_config_.mp_plan_state == ROOTTURN) {
            output_.is_turn_heading = true;
            output_.turn_rate = mp_config_.turn_rate;
        }
        output_.m_pos_ctrl_speed = mp_config_.max_vel;
        output = output_;
        chlog::info("af_data",
                    "ActionDroneMotionPlan --- GetOutput m_vector = " + toStr(output.m_vector) + ", is_speed_mode = " +
                    to_string(output.is_speed_mode) + ", m_pos_ctrl_speed = " + to_string2(output.m_pos_ctrl_speed) +
                    ", target_heading = ", output.target_heading);
    }

    void ActionMotionPlan::SetStatus(const TVec3 &drone_pos, const TVec3 &drone_speed, float heading) {
        // c_pos_ = drone_pos;
        drone_state_.drone_pos = drone_pos;
        TVec3 attitude = {0, 0, heading}, acc = {0, 0, 0};
        mp_manager_->OnUpdateDroneStatus(drone_pos, drone_speed, acc, attitude);
    }

    void ActionMotionPlan::updateSpeedLimit(const float &speed_limit) {
        mp_config_.max_vel = speed_limit;
        mp_manager_->OnUpdateMaxSpeed(speed_limit);
        //chlog::info("af_core", "motion plan update speed limit = " + to_string2(speed_limit));
    }

    void ActionMotionPlan::OnUpdateTargetPoint(const TVec3 &new_target_point) {
        TVec3 end_pos = new_target_point;
        chlog::info("af_core", "motion_plan init! new_target_point = " + toStr(end_pos));
        mp_manager_->OnUpdateTargetPos(end_pos);
    }

    void ActionMotionPlan::updateEndVel(const TVec3 &end_vel) {
        TVec3 end_vel_ENU = end_vel;
        mp_manager_->updateEndVel(end_vel_ENU);
    }

    void ActionMotionPlan::updateMotionPlan(const float dist,const TVec3 &insp_vec,
                                            const vector<TVec3> &waypoints) {
        mp_manager_->updateMotionPlan(dist, insp_vec, waypoints);
    }


    void ActionMotionPlan::updateFlightData(const TFlightData &data) {
        mp_manager_->updateFlightData(data);
    }

    void ActionMotionPlan::updateCirclePoint(const TVec3 &tip_pos) {
        mp_config_.m_toward_point = tip_pos;
    }

}
