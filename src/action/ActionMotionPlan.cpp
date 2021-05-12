//
// Created by zhouhua on 2020/6/1.
//

#include "ActionMotionPlan.hpp"


ActionMotionPlan::ActionMotionPlan() :
        is_enable_(false),
        init_follower_(false),
        init_usv2_(false),
        init_usv3_(false)
{
}

void ActionMotionPlan::Oninit(const int config) {

}

void ActionMotionPlan::initNh(ros::NodeHandle& nh) {
    nh_ = nh;
}

bool ActionMotionPlan::initMP(const MP_Config &mpConfig) {
    mp_config_ = mpConfig;
    mp_config_.log_path = "USV1_MP";
    mp_config_.nh = nh_;
    mp_config_.drone_id = 1;
    output_.target_heading = mpConfig.m_drone_heading;
    mp_manager_ = makeSp<MPManager>(mp_config_);
    mp_manager_->SetMpEnable(true);
    chlog::info(mp_config_.log_path, "[Action MP]: motion_plan init! target_pos = " + toStr(mpConfig.end_pos)
                               + ", target_heading = " + to_string2(mpConfig.target_heading));
    return false;
}

void ActionMotionPlan::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    if (mp_manager_ == NULL) return;
    usv1_drone_pos_ << m_multi_vehicle_.usv1.current_local_pos.pose.position.x,
                      m_multi_vehicle_.usv1.current_local_pos.pose.position.y,
                      m_multi_vehicle_.usv1.current_local_pos.pose.position.z;

    usv2_drone_pos_ << m_multi_vehicle_.usv2.current_local_pos.pose.position.x,
                      m_multi_vehicle_.usv2.current_local_pos.pose.position.y,
                      m_multi_vehicle_.usv2.current_local_pos.pose.position.z;

    usv3_drone_pos_ << m_multi_vehicle_.usv3.current_local_pos.pose.position.x,
                      m_multi_vehicle_.usv3.current_local_pos.pose.position.y,
                      m_multi_vehicle_.usv3.current_local_pos.pose.position.z;
    TVec3 attitude = {0, 0, m_multi_vehicle_.usv1.yaw}, acc = {0, 0, 0};
    TVec3 drone_speed = m_multi_vehicle_.usv1.velocity;
    mp_manager_->OnUpdateDroneStatus(usv1_drone_pos_, drone_speed, acc, attitude);
}

void ActionMotionPlan::DoProgress() {
    if (mp_manager_ == NULL) return;
//    chlog::info(mp_config_.log_path, "is_enable = ", is_enable_);
    if (is_enable_) {
        mp_manager_->ProcessState();
        TVec3 drone_data, drone_speed;
        SetFunctionOutPut();

        PolynomialTraj usv2_traj, usv3_traj;
        if (mp_manager_->getPolyTraj(usv2_traj, usv3_traj)) {
            if (mp_config_.formation_type != VF_USV_LINE_VERTICAL) {
                if (!init_follower_) {
                    USV2ActionMotionPlan::getInstance()->initMP(mp_config_);
                    USV2ActionMotionPlan::getInstance()->setPolyTraj(usv2_traj);
                    USV3ActionMotionPlan::getInstance()->initMP(mp_config_);
                    USV3ActionMotionPlan::getInstance()->setPolyTraj(usv3_traj);
                    USV2ActionMotionPlan::getInstance()->setEnable(true);
                    USV3ActionMotionPlan::getInstance()->setEnable(true);
                    init_follower_ = true;
                }
            } else {
                float usv1_length = (output_.m_vector).norm();
                chlog::info(mp_config_.log_path, "usv1_length = ", usv1_length);
                if (usv1_length > K_multi_usv_formation_distance && !init_usv2_) {
                    USV2ActionMotionPlan::getInstance()->initMP(mp_config_);
                    USV2ActionMotionPlan::getInstance()->setPolyTraj(usv2_traj);
                    USV2ActionMotionPlan::getInstance()->setEnable(true);
                    init_usv2_ = true;
                }
                if (usv1_length > 2 * K_multi_usv_formation_distance && !init_usv3_) {
                    USV3ActionMotionPlan::getInstance()->initMP(mp_config_);
                    USV3ActionMotionPlan::getInstance()->setPolyTraj(usv3_traj);
                    USV3ActionMotionPlan::getInstance()->setEnable(true);
                    init_usv3_ = true;
                }

            }
        }
    } else {
        mp_manager_->SetMpEnable(false);
    }
}

void ActionMotionPlan::setEnable(bool enable) {
    is_enable_ = enable;
}

void ActionMotionPlan::SetFunctionOutPut() {
    if (mp_config_.is_track_point && (!isnan(usv1_drone_pos_.x())
                                      || !isnan(usv1_drone_pos_.y()) ||
                                      !isnan(usv1_drone_pos_.z())) && mp_manager_ != NULL) {

        output_.target_heading = mp_config_.target_heading;
        output_.is_speed_mode = mp_config_.is_speed_mode;

        // stop if cannot get motion plan result;
        if (!mp_manager_->GetControlOutput(output_.m_vector)) {
            output_.m_vector = TVec3{0, 0, 0};
            output_.is_speed_mode = true;
            return;
        }

        output_.m_pos_ctrl_speed = mp_config_.max_vel;
        m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.x = output_.m_vector.x();
        m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.y = output_.m_vector.y();
        m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.z = output_.m_vector.z();
        DataMan::getInstance()->SetUSVMPControlData(m_multi_vehicle_, 1);
        chlog::info("af_data",
                    "[USV1 ActionDroneMotionPlan] --- GetOutput m_vector = " + toStr(output_.m_vector));
    }
}

void ActionMotionPlan::updateSpeedLimit(const float &speed_limit) {
    mp_config_.max_vel = speed_limit;
    mp_manager_->OnUpdateMaxSpeed(speed_limit);
    //chlog::info(mp_config_.log_path, "motion plan update speed limit = " + to_string2(speed_limit));
}

void ActionMotionPlan::OnUpdateTargetPoint(const TVec3 &new_target_point) {
    TVec3 end_pos = new_target_point;
    chlog::info(mp_config_.log_path, "[Action MP]:motion_plan init! new_target_point = " + toStr(end_pos));
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

