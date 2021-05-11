//
// Created by zhouhua on 2020/6/1.
//

#include "USV2ActionMotionPlan.hpp"


USV2ActionMotionPlan::USV2ActionMotionPlan() :
        is_enable_(false),
        test_num(0)
{
}

void USV2ActionMotionPlan::Oninit(const int config) {

}
void USV2ActionMotionPlan::initNh(ros::NodeHandle &nh, const shared_ptr<IMap> &IMap) {
    nh_ = nh;
    IMap_ = IMap;
    test_num = 66;
    chlog::info("USV2_MP", "[USV2 Action MP]: usv2 usv2_mp init! test_num = ", test_num);
}

bool USV2ActionMotionPlan::initMP(const MP_Config &mpConfig) {
    mp_config_ = mpConfig;
    mp_config_.log_path = "USV2_MP";
    mp_config_.nh = nh_;
    mp_config_.drone_id = 2;
    mp_config_.mp_map = IMap_;
    mp_manager_ = makeSp<MPManager>(mp_config_);
    output_.target_heading = mpConfig.m_drone_heading;
    mp_manager_->SetMpEnable(true);
    chlog::info(mp_config_.log_path, "[USV2 Action MP]: usv2 usv2_mp init! target_pos = " + toStr(mpConfig.end_pos)
                           + ", target_heading = " + to_string2(mpConfig.target_heading),
                           ", test num = ", test_num);
    return false;
}

void USV2ActionMotionPlan::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    if (mp_manager_ == NULL) return;
    drone_pos_ << m_multi_vehicle_.usv2.current_local_pos.pose.position.x,
                      m_multi_vehicle_.usv2.current_local_pos.pose.position.y,
                      m_multi_vehicle_.usv2.current_local_pos.pose.position.z;
    TVec3 attitude = {0, 0, m_multi_vehicle_.usv2.yaw}, acc = {0, 0, 0};
    TVec3 drone_speed = m_multi_vehicle_.usv2.velocity;
    mp_manager_->OnUpdateDroneStatus(drone_pos_, drone_speed, acc, attitude);
}

void USV2ActionMotionPlan::DoProgress() {
    if (mp_manager_ == NULL) return;
    if (is_enable_) {
        mp_manager_->ProcessState();
        TVec3 drone_data, drone_speed;
        SetFunctionOutPut();
    } else {
        mp_manager_->SetMpEnable(false);
    }
}

void USV2ActionMotionPlan::setEnable(bool enable) {
    is_enable_ = enable;
}

void USV2ActionMotionPlan::SetFunctionOutPut() {
    if (mp_config_.is_track_point && (!isnan(drone_pos_.x())
                                      || !isnan(drone_pos_.y()) ||
                                      !isnan(drone_pos_.z())) && mp_manager_ != NULL) {

        output_.target_heading = mp_config_.target_heading;
        output_.is_speed_mode = mp_config_.is_speed_mode;

        // stop if cannot get motion plan result;
        if (!mp_manager_->GetControlOutput(output_.m_vector)) {
            output_.m_vector = TVec3{0, 0, 0};
            output_.is_speed_mode = true;
            return;
        }

        output_.m_pos_ctrl_speed = mp_config_.max_vel;
        m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.x = output_.m_vector.x();
        m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.y = output_.m_vector.y();
        m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.z = output_.m_vector.z();
        DataMan::getInstance()->SetUSVMPControlData(m_multi_vehicle_, 2);
        chlog::info(mp_config_.log_path,
                    "ActionDroneMotionPlan --- GetOutput m_vector = " + toStr(output_.m_vector));
    }
}

void USV2ActionMotionPlan::updateSpeedLimit(const float &speed_limit) {
    mp_config_.max_vel = speed_limit;
    mp_manager_->OnUpdateMaxSpeed(speed_limit);
    //chlog::info(mp_config_.log_path, "motion plan update speed limit = " + to_string2(speed_limit));
}

void USV2ActionMotionPlan::OnUpdateTargetPoint(const TVec3 &new_target_point) {
    TVec3 end_pos = new_target_point;
    chlog::info(mp_config_.log_path, "[Action MP]:usv2_mp init! new_target_point = " + toStr(end_pos));
    mp_manager_->OnUpdateTargetPos(end_pos);
}

void USV2ActionMotionPlan::updateEndVel(const TVec3 &end_vel) {
    TVec3 end_vel_ENU = end_vel;
    mp_manager_->updateEndVel(end_vel_ENU);
}

void USV2ActionMotionPlan::updateMotionPlan(const float dist,const TVec3 &insp_vec,
                                        const vector<TVec3> &waypoints) {
    mp_manager_->updateMotionPlan(dist, insp_vec, waypoints);
}


void USV2ActionMotionPlan::updateCirclePoint(const TVec3 &tip_pos) {
    mp_config_.m_toward_point = tip_pos;
}

void USV2ActionMotionPlan::setPolyTraj(PolynomialTraj& poly_traj) {
    mp_manager_->setPolyTraj(poly_traj);
}


USV2ActionMotionPlan* USV2ActionMotionPlan::getInstance() {
    static USV2ActionMotionPlan *action_mp = NULL;
    if (action_mp == NULL) {
        action_mp = new USV2ActionMotionPlan();
    }
    return action_mp;
}

