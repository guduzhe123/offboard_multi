//
// Created by zhouhua on 2020/6/1.
//

#include "ActionMotionPlan.hpp"


ActionMotionPlan::ActionMotionPlan() :
        is_enable_(false)
{
}

void ActionMotionPlan::Oninit(const int config) {

}

bool ActionMotionPlan::initMP(const MP_Config &mpConfig) {
    chlog::info("motion_plan", "motion_plan init! target_pos = " + toStr(mpConfig.end_pos)
                           + ", target_heading = " + to_string2(mpConfig.target_heading));
    mp_config_ = mpConfig;
    mp_manager_ = makeSp<MPManager>(mp_config_);
    output_.target_heading = mpConfig.m_drone_heading;
    mp_manager_->SetMpEnable(true);
    return false;
}

void ActionMotionPlan::GetData() {

}

void ActionMotionPlan::DoProgress() {
    if (mp_manager_ == NULL) return;
    chlog::info("motion_plan", "is_enable = ", is_enable_);
    if (is_enable_) {
        mp_manager_->ProcessState();
        TVec3 drone_data, drone_speed;
//        drone_data =
//        SetStatus()
    } else {
        mp_manager_->SetMpEnable(false);
    }
}

void ActionMotionPlan::setEnable(bool enable) {
    is_enable_ = enable;
}

void ActionMotionPlan::SetFunctionOutPut() {
 /*   if (mp_config_.is_track_point && (!isnan(drone_state_.drone_pos.x())
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


    output_.m_pos_ctrl_speed = mp_config_.max_vel;
    output = output_;
    chlog::info("af_data",
                "ActionDroneMotionPlan --- GetOutput m_vector = " + toStr(output.m_vector) + ", is_speed_mode = " +
                to_string(output.is_speed_mode) + ", m_pos_ctrl_speed = " + to_string2(output.m_pos_ctrl_speed) +
                ", target_heading = ", output.target_heading);*/
}

void ActionMotionPlan::SetStatus(const TVec3 &drone_pos, const TVec3 &drone_speed, float heading) {
    TVec3 attitude = {0, 0, heading}, acc = {0, 0, 0};
    mp_manager_->OnUpdateDroneStatus(drone_pos, drone_speed, acc, attitude);
}

void ActionMotionPlan::updateSpeedLimit(const float &speed_limit) {
    mp_config_.max_vel = speed_limit;
    mp_manager_->OnUpdateMaxSpeed(speed_limit);
    //chlog::info("motion_plan", "motion plan update speed limit = " + to_string2(speed_limit));
}

void ActionMotionPlan::OnUpdateTargetPoint(const TVec3 &new_target_point) {
    TVec3 end_pos = new_target_point;
    chlog::info("motion_plan", "motion_plan init! new_target_point = " + toStr(end_pos));
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


void ActionMotionPlan::updateCirclePoint(const TVec3 &tip_pos) {
    mp_config_.m_toward_point = tip_pos;
}

ActionMotionPlan* ActionMotionPlan::getInstance() {
    static ActionMotionPlan *action_mp = NULL;
    if (action_mp == NULL) {
        action_mp = new ActionMotionPlan();
    }
    return action_mp;
}

