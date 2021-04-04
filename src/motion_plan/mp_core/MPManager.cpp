//
// Created by zhouhua on 2020/12/6.
//

#include "motion_plan/mp_core/MPManager.h"

MPManager::MPManager(const MP_Config &config) :
        drone_st_{},
        mp_state_(INIT),
        end_vel_{},
        receive_traj_(false),
        has_drone_update_(false),
        init_target_pos_{},
        path_find_fail_timer_(0) {
    chlog::info("motion_plan", "\n");
    chlog::info("motion_plan", "[MP Manager]: mp manager init!");

    mp_config_ = config;
    mp_publisher_ = makeSp<MPPublisher>();
    mp_publisher_->OnInit(mp_config_.mp_plan_state);

    path_finder_ = makeSp<FastPathFinder>();
    path_finder_->initPlanModules(mp_config_, mp_config_.mp_map);
    receive_traj_ = false;
    mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 1.0));

    chlog::info("motion_plan", "[MP Mananger]: Triggered! end_pt_ = " + toStr(mp_config_.end_pos), "max_vel = "
                                                                                                   + to_string(mp_config_.max_vel));
    ChangeExecState(WAIT_TARGET, "FSM");
    end_vel_.setZero();
    init_target_pos_ = mp_config_.end_pos;
}


void MPManager::SetMpEnable(bool is_enable) {
    mp_config_.is_enable = is_enable;
}

void MPManager::OnUpdateDroneStatus(const TVec3 &drone_pos, const TVec3 &drone_vel, const TVec3 &drone_acc,
                                    const TVec3 &drone_attitude) {
    drone_st_.drone_attitude = drone_attitude;
    drone_st_.drone_pos = drone_pos;
    drone_st_.drone_vel = drone_vel;
    drone_st_.drone_acc = drone_acc;

    mp_publisher_->updateDroneData(drone_st_.drone_pos);
    has_drone_update_ = true;

    traj_cmd_.push_back(drone_st_.drone_pos);
    if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
    mp_publisher_->displayTrajWithColor(traj_cmd_, 0.1, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void MPManager::OnUpdateTargetPos(const TVec3 &target_pos) {
    if ((target_pos - mp_config_.end_pos).norm() < 0.5) return;
    mp_config_.end_pos = target_pos;
    mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 1.0));
    chlog::info("motion_plan", "[MP Manager]: update motion plan target = ", toStr(mp_config_.end_pos));
    if (mp_state_ == EXEC_TRAJ) {
        ChangeExecState(REPLAN_TRAJ, "FSM");
    }
}

void MPManager::updateEndVel(const TVec3 &end_vel) {
    end_vel_ = end_vel;
}

void MPManager::updateMotionPlan(const float dist, const TVec3 &insp_vec,
                                 const vector<TVec3> &waypoints) {
    vector<TVec3> points;
    points.clear();
    points.push_back(waypoints.back());
    init_target_pos_ = waypoints.back();
    mp_publisher_->updateMPTarget(dist, points);
    dist_config_ = dist;
    insp_vec_ENU_ = insp_vec.normalized();
    chlog::info("motion_plan", "[MP Manager]: insp_vec = ", toStr(insp_vec), ", insp_vec_ENU_ = ",
                toStr(insp_vec_ENU_));
}


bool MPManager::CallKinodynamicReplan() {
    bool plan_success = path_finder_->replan(start_pt_.cast<double>(), start_vel_.cast<double>(),
                                             start_acc_.cast<double>(),
                                             mp_config_.end_pos.cast<double>(), end_vel_.cast<double>());
    if (plan_success) {
        auto info = &path_finder_->getLocaldata();

        /* get traj result */
        offboard::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

        for (int i = 0; i < pos_pts.rows(); ++i) {
            geometry_msgs::Point pt;
            pt.x = pos_pts(i, 0);
            pt.y = pos_pts(i, 1);
            pt.z = pos_pts(i, 2);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        for (int i = 0; i < knots.rows(); ++i) {
            bspline.knots.push_back(knots(i));
        }

        bsplineResult(bspline);

        plan_traj_.clear();
        plan_traj_.push_back(info->position_traj_);
        plan_traj_.push_back(plan_traj_[0].getDerivative());
        plan_traj_.push_back(plan_traj_[1].getDerivative());

        traj_duration_ = plan_traj_[0].getTimeSum();

        receive_traj_ = true;

        /* visulization */
        auto plan_data = &path_finder_->getPlanData();
        mp_publisher_->drawGeometricPath(plan_data->kino_path_);
        mp_publisher_->drawBspline(info->position_traj_);

        return true;
    } else {
        chlog::info("motion_plan", "[MP Manager]:  generate new traj fail.");
        return false;
    }
}

void MPManager::bsplineResult(const offboard::Bspline &msg) {
    // parse pos traj

    Eigen::MatrixXd pos_pts(msg.pos_pts.size(), 3);

    Eigen::VectorXd knots(msg.knots.size());
    for (unsigned int i = 0; i < msg.knots.size(); ++i) {
        knots(i) = msg.knots[i];
    }

    for (unsigned int i = 0; i < msg.pos_pts.size(); ++i) {
        pos_pts(i, 0) = msg.pos_pts[i].x;
        pos_pts(i, 1) = msg.pos_pts[i].y;
        pos_pts(i, 2) = msg.pos_pts[i].z;
    }

    NonUniformBspline pos_traj(pos_pts, msg.order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    Eigen::MatrixXd yaw_pts(msg.yaw_pts.size(), 1);
    for (unsigned int i = 0; i < msg.yaw_pts.size(); ++i) {
        yaw_pts(i, 0) = msg.yaw_pts[i];
    }

    NonUniformBspline yaw_traj(yaw_pts, msg.order, msg.yaw_dt);

    start_time_ = msg.start_time;

    plan_traj_.clear();
    plan_traj_.push_back(pos_traj);
    plan_traj_.push_back(plan_traj_[0].getDerivative());
    plan_traj_.push_back(plan_traj_[1].getDerivative());

    traj_duration_ = plan_traj_[0].getTimeSum();

    receive_traj_ = true;
}

void MPManager::ChangeExecState(MP_EXEC_STATE new_state, string pos_call) {
    string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
    int pre_s = int(mp_state_);
    mp_state_ = new_state;
    chlog::info("motion_plan", "[MP Manager]: [" + pos_call + "]: from " + state_str[pre_s]
                               + " to " + state_str[int(new_state)]);
}

bool MPManager::GetControlOutput(TVec3 &vector_eus) {
//        chlog::info("motion_plan", "[MP Manager]: received traj = ", receive_traj_);
    if (!receive_traj_) return false;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();

    Eigen::Vector3d pos = {0, 0, 0}, vel = {0, 0, 0}, acc = {0, 0, 0};
    double yaw = 0;

    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        pos = plan_traj_[0].evaluateDeBoorT(t_cur);
        vel = plan_traj_[1].evaluateDeBoorT(t_cur);
        acc = plan_traj_[2].evaluateDeBoorT(t_cur);
    } else if (t_cur >= traj_duration_) {
        //hover when finish traj_
        pos = plan_traj_[0].evaluateDeBoorT(traj_duration_);
        vel = plan_traj_[1].evaluateDeBoorT(traj_duration_);
        acc.setZero();
    } else {
        chlog::info("motion_plan", "[MP Manager]: invalid time!");
    }

    if (mp_config_.is_speed_mode) {
        vector_eus = vel.cast<float>();
    } else {
        vector_eus = pos.cast<float>();
    }

    TVec3 dir(cos(yaw), sin(yaw), 0.0);
    mp_publisher_->DrawTrajCommand(pos.cast<float>(), 2 * dir, Eigen::Vector4d(1, 1, 0, 0.7), 2);
/*    chlog::info("motion_plan", "[MP Manager]: mp pos command = ", pos.x(), ",", pos.y(), ",", pos.z(),
                ", is speed mode = ", mp_config_.is_speed_mode);*/
    // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));
    return true;
}

void MPManager::OnUpdateMaxSpeed(float speed_max) {
    mp_config_.max_vel = speed_max;
    mp_config_.max_acc = speed_max;
    path_finder_->updateSpeedLimit(mp_config_.max_vel, mp_config_.max_acc);
    chlog::info("motion_plan", "[MP Manager]: update_speed limit:", to_string(mp_config_.max_vel));
}

void MPManager::OnUpdateDroneHeading(float drone_heading) {
    mp_config_.m_drone_heading = drone_heading;
}


void MPManager::ProcessState() {
//    chlog::info("motion_plan", "[MP Manager]: mp_state_ = " , mp_state_);
    switch (mp_state_) {
        case INIT: {
            ChangeExecState(WAIT_TARGET, "FSM");
            break;
        }

        case WAIT_TARGET: {
            receive_traj_ = false;
            if (!mp_config_.is_enable || !has_drone_update_)
                return;
            else {
                ChangeExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case GEN_NEW_TRAJ: {
            start_pt_ = drone_st_.drone_pos;
            start_vel_ = drone_st_.drone_vel;
            start_acc_.setZero();

            bool success = CallKinodynamicReplan();
            if (success) {
                ChangeExecState(EXEC_TRAJ, "FSM");
            } else {
                receive_traj_ = false;
                path_find_fail_timer_++;
                if (path_find_fail_timer_ < 5) {
                    ChangeExecState(GEN_NEW_TRAJ, "FSM");
                } else {
                    ChangeExecState(WAIT_TARGET, "FSM");
                    path_find_fail_timer_ = 0;
                    chlog::info("motion_plan", "[MP Manager]: A star search failed too many times!");
                }
            }
            break;
        }

        case EXEC_TRAJ: {
            /* determine if need to replan */
            LocalTrajData *info = &path_finder_->getLocaldata();
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();
            t_cur = min(info->duration_, t_cur);

            Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

            if ((mp_config_.end_pos - drone_st_.drone_pos).norm() < 0.8) {
                ChangeExecState(WAIT_TARGET, "FSM");
                return;

            } else if (t_cur > info->duration_ - 5e-2 && mp_config_.mp_plan_state == MotionPlanState::TRACKING) {
                ChangeExecState(GEN_NEW_TRAJ, "FSM");
                chlog::info("motion_plan", "[MP Manager]: motion plan time out!");
                return;
            } else if ((info->start_pos_ - pos).norm() < 1.5 ) {
                chlog::info("motion_plan", "[MP Manager]: close to start pos!");
                return;

            } else {
                ChangeExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case REPLAN_TRAJ: {
            LocalTrajData *info = &path_finder_->getLocaldata();
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();

            if (mp_config_.control_mode == VELOCITY_WITHOUT_CUR) {
                start_pt_ = drone_st_.drone_pos;
                start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur).cast<float>();
            } else if (mp_config_.control_mode == POSITION_WITHOUT_CUR) {
                start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur).cast<float>();
                start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur).cast<float>();
                if ((start_pt_ - mp_config_.end_pos).norm() < 0.8) {
                    chlog::info("motion_plan", "[MP Manager]: near target, change goal");
                    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur).cast<float>();
                    start_vel_ = drone_st_.drone_vel;
                }
            } else if (mp_config_.control_mode == POSITION_CUR || mp_config_.control_mode == VELOCITY_WITH_CUR) {
                start_pt_ = drone_st_.drone_pos;
                start_vel_ = drone_st_.drone_vel;
            }

            start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur).cast<float>();

            bool success = CallKinodynamicReplan();
            if (success) {
                ChangeExecState(EXEC_TRAJ, "FSM");
            } else {
                ChangeExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }
        default:
            break;
    }
}
