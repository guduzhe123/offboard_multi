//
// Created by zhouhua on 2020/6/1.
//

#include "ActionMotionPlan.hpp"


ActionMotionPlan::ActionMotionPlan() :
        is_enable_(false),
        init_follower_(false),
        init_usv2_(false),
        init_usv3_(false),
        m_state_(ST_INIT),
        init_time_(false)
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
    chlog::info(mp_config_.log_path, "[Action MP]: motion_plan init! target_pos = " + toStr(mpConfig.end_pos)
                               + ", target_heading = " + to_string2(mpConfig.target_heading));
    init_follower_ = false;
    init_usv2_ = false;
    init_usv3_ = false;
    m_state_ = ST_INIT;
    TVec3 usv2_form, usv3_form;
    initFormation(usv2_form, usv3_form);
    init_time_ = false;
    return false;
}

void ActionMotionPlan::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    if (mp_manager_ == NULL) return;
    usv1_drone_pos_ << m_multi_vehicle_.usv1.current_local_pos.pose.position.x,
                      m_multi_vehicle_.usv1.current_local_pos.pose.position.y,
                      m_multi_vehicle_.usv1.current_local_pos.pose.position.z;
    TVec3 attitude = {0, 0, m_multi_vehicle_.usv1.yaw}, acc = {0, 0, 0};
    TVec3 drone_speed = m_multi_vehicle_.usv1.velocity;
    mp_manager_->OnUpdateDroneStatus(usv1_drone_pos_, drone_speed, acc, attitude);

}

void ActionMotionPlan::DoProgress() {
    if (mp_manager_ == NULL) return;
//    chlog::info(mp_config_.log_path, "is_enable = ", is_enable_);

    if (is_enable_) {
        switch(m_state_) {
            case ST_INIT: {
                goals_ = mp_config_.targets;
                start_pos_ = usv1_drone_pos_;
                goals_.insert(goals_.begin(), start_pos_);
                chlog::info(mp_config_.log_path, "start pos = ", toStr(start_pos_));
                m_state_ = ST_RUNNING;
                break;
            }

            case ST_RUNNING: {
                mp_manager_->ProcessState();
                SetFunctionOutPut();
                TVec3 usv2_pos_sp{0,0,0}, usv3_pos_sp{0,0,0};
                if (mp_config_.formation_type == VF_USV_TRIANGLE
                    || mp_config_.formation_type == VF_USV_INVERSION_TRIANGLE) {
                    calcuLineDir(goals_, usv2_pos_sp, usv3_pos_sp);
                }

                PolynomialTraj usv2_traj, usv3_traj;
                if (mp_manager_->getPolyTraj(usv2_traj, usv3_traj)) {
                    if (!init_time_) {
                        t0 = ros::Time::now();
                        init_time_ = true;

                    }
                    chlog::info(mp_config_.log_path, "init_time_ = ", init_time_, ", t0 = ", t0.toSec());
                    if (mp_config_.formation_type != VF_USV_LINE_VERTICAL) {
                        if (!init_follower_) {
                            mp_manager_->SetMpEnable(true);
                            USV2ActionMotionPlan::getInstance()->initMP(mp_config_);
                            USV2ActionMotionPlan::getInstance()->setPolyTraj(usv2_traj);
                            USV3ActionMotionPlan::getInstance()->initMP(mp_config_);
                            USV3ActionMotionPlan::getInstance()->setPolyTraj(usv3_traj);
                            USV2ActionMotionPlan::getInstance()->setEnable(true);
                            USV3ActionMotionPlan::getInstance()->setEnable(true);
                            init_follower_ = true;
                            break;
                        }

                        USV2ActionMotionPlan::getInstance()->setFormationTarget(usv2_pos_sp);
                        USV3ActionMotionPlan::getInstance()->setFormationTarget(usv3_pos_sp);
                    } else {
                        t1 = ros::Time::now();
                        double time_err = (t1 - t0).toSec();
                        float usv1_length = time_err * mp_config_.max_vel;
                        chlog::info(mp_config_.log_path, "usv1_length = ", usv1_length, ", time_err = ", time_err,
                                    ", t1 =", t1.toSec(), ", t0 = ", t0.toSec());
                        if (usv1_length > K_multi_usv_formation_distance && !init_usv2_) {
                            mp_manager_->SetMpEnable(true);
                            USV2ActionMotionPlan::getInstance()->initMP(mp_config_);
                            USV2ActionMotionPlan::getInstance()->setPolyTraj(usv2_traj);
                            USV2ActionMotionPlan::getInstance()->setEnable(true);
                            init_usv2_ = true;
                        }
                        if (usv1_length > 2 * K_multi_usv_formation_distance && !init_usv3_) {
                            mp_manager_->SetMpEnable(true);
                            USV3ActionMotionPlan::getInstance()->initMP(mp_config_);
                            USV3ActionMotionPlan::getInstance()->setPolyTraj(usv3_traj);
                            USV3ActionMotionPlan::getInstance()->setEnable(true);
                            init_usv3_ = true;
                        }

                    }
                }
                break;
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
        chlog::info(mp_config_.log_path,
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

void ActionMotionPlan::calcuLineDir(const vector<TVec3> &goal, TVec3 &usv2_pos_sp, TVec3 &usv3_pos_sp) {
    TVec3 drone2_pos_sp{0,0,0}, drone3_pos_sp{0,0,0};
    for (int i = 0; i < goal.size() - 1; i++) {
        TVec3 dir_start = (goal[i] - output_.m_vector).normalized();
        TVec3 dir_end = (output_.m_vector - goal[i + 1]).normalized();
        if ((goal[i] - output_.m_vector).norm() < 0.00001 || (output_.m_vector - goal[i + 1]).norm() < 0.00001 ) return;

        float value = dir_start.dot(dir_end);
        if (fabs(value - 1) < 0.00001) {
            chlog::info(mp_config_.log_path, "in the line start i = ", i, ", goal = ", toStr(goal[i]),
                        ", goal2 = ", toStr(goal[i+1]), ", dir_start = ", toStr(dir_start), ", dir_end = ",
                        toStr(dir_end), ", value = ", value);
            if ((output_.m_vector - goal[i + 1]).norm() < K_multi_usv_formation_distance) return;
            if (i > 0 && (output_.m_vector - goal[i]).norm() < K_multi_usv_formation_distance) return;
            TVec3 line_dir = ((goal.at(i + 1) - goal.at(i)).normalized()).cast<float>();

            TVec3 line_dir_norm(1, 0, 0);
            Eigen::Matrix3f rotMatrix = Eigen::Quaternionf::FromTwoVectors(line_dir_norm,
                                                                           line_dir).toRotationMatrix();
            getFollowerSp(drone_usv2_, rotMatrix, drone2_pos_sp);
            getFollowerSp(drone_usv3_, rotMatrix, drone3_pos_sp);
        }
        usv2_pos_sp = drone2_pos_sp;
        usv3_pos_sp = drone3_pos_sp;
    }
}

void ActionMotionPlan::getFollowerSp(const TVec3 &drone_move, const Eigen::Matrix3f &rotMatrix, TVec3 &target_pos) {
    TVec3 res = rotMatrix * drone_move;
    target_pos = output_.m_vector + res;
    chlog::info(mp_config_.log_path, "pos = ", toStr(target_pos), ", res = ", toStr(res));
}

void ActionMotionPlan::initFormation(TVec3 &usv2_form, TVec3 &usv3_form) {
    switch (mp_config_.formation_type) {
        case VF_USV_TRIANGLE: {
            chlog::info(mp_config_.log_path,"[Action MP]: usv Formation call! Triangle!");
            drone_usv2_ = TVec3(-K_multi_usv_formation_distance, -K_multi_usv_formation_distance , 0);
            drone_usv3_ = TVec3(-K_multi_usv_formation_distance, K_multi_usv_formation_distance , 0);
            chlog::info(mp_config_.log_path, "[Action MP]: drone_usv2_ = ", toStr(drone_usv2_),
                        ", drone_usv3 = ", toStr(drone_usv3_));
        }
            break;

        case VF_USV_INVERSION_TRIANGLE: {
            chlog::info(mp_config_.log_path,"[Action MP]: usv Formation call! INVERSION Triangle!");
            drone_usv2_ = TVec3(K_multi_usv_formation_distance, K_multi_usv_formation_distance , 0);
            drone_usv3_ = TVec3(K_multi_usv_formation_distance , -K_multi_usv_formation_distance, 0);
        }
            break;


        case VF_USV_LINE_HORIZONTAL : {
            chlog::info(mp_config_.log_path,"[Action MP]: usv Formation call! Line horizontal!");
            drone_usv2_ = TVec3( -K_multi_usv_formation_distance , 0, 0);
            drone_usv3_ = TVec3( -2 * K_multi_usv_formation_distance, 0, 0);
        }
            break;

        case VF_USV_LINE_VERTICAL : {
            chlog::info(mp_config_.log_path,"[Action MP]: usv Formation call! Line Vertical!");
            drone_usv2_ = TVec3(0 , K_multi_usv_formation_distance,  0);
            drone_usv3_ = TVec3(0,2 * K_multi_usv_formation_distance, 0);
        }
            break;

        case VF_USV_ALL_RETURN: {
            chlog::info(mp_config_.log_path,"[Action MP]: usv Formation call! All USVs Return!");
        }
            break;

        default:
            break;

    }
    usv2_form = drone_usv2_;
    usv3_form = drone_usv3_;
}

ActionMotionPlan* ActionMotionPlan::getInstance() {
    static ActionMotionPlan *action_mp = NULL;
    if (action_mp == NULL) {
        action_mp = new ActionMotionPlan();
    }
    return action_mp;
}

