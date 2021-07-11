//
// Created by zhouhua on 2020/12/6.
//

#include "motion_plan/mp_core/MPManager.h"

MPManager::MPManager(const MP_Config &config) :
        drone_st_{},
        mp_state_(INIT),
        end_vel_{},
        receive_traj_(false),
        have_target_(false),
        has_drone_update_(false),
        init_target_pos_{},
        path_find_fail_timer_(0),
        collide_(false),
        plan_success_(false),
        check_collision_state_(CHECK_COLLISION),
        time_add_sum_(0),
        time_add_sum_pre_(0){
    log = config.log_path;
    chlog::info(log, "~~~~~~~~\n");
    chlog::info(log, "[MP Manager]: mp manager init!");

    mp_config_ = config;
    mp_publisher_ = makeSp<MPPublisher>();
    mp_publisher_->OnInit(mp_config_.nh);

    path_finder_ = makeSp<FastPathFinder>();
    path_finder_->initPlanModules(mp_config_, mp_config_.mp_map);
    receive_traj_ = false;

    if (!mp_config_.targets.empty()) {
        mp_config_.end_pos = mp_config_.targets.front();
        path_finder_->setGlobalWaypoints(mp_config_.targets);
    }
    mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 1.0));

    chlog::info(log, "[MP Mananger]: Triggered! end_pt_ = " + toStr(mp_config_.end_pos), "max_vel = "
                                                                                                   + to_string(mp_config_.max_vel));
    ChangeExecState(WAIT_TARGET, "FSM");
    end_vel_.setZero();
    init_target_pos_ = mp_config_.end_pos;
    have_target_ = true;
    formationCall(mp_config_.formation_type);
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
    init_target_pos_ = mp_config_.end_pos;
    mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 1.0));
    chlog::info(log, "[MP Manager]: update motion plan target = ", toStr(mp_config_.end_pos));
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
    chlog::info(log, "[MP Manager]: insp_vec = ", toStr(insp_vec), ", insp_vec_ENU_ = ",
                toStr(insp_vec_ENU_));
    have_target_ = true;
}


void MPManager::setPolyTraj(const PolynomialTraj& poly_traj) {
    gl_traj_ = poly_traj;
}

bool MPManager::getPolyTraj(PolynomialTraj& usv2_poly_traj, PolynomialTraj& usv3_poly_traj) {
    if (plan_success_) {
        usv2_poly_traj = path_finder_->getUSV2PolynomialTraj();
        usv3_poly_traj = path_finder_->getUSV3PolynomialTraj();
    }
    return plan_success_;
}

bool MPManager::CallKinodynamicReplan(int step) {
    plan_success_ = false;
    chlog::info(log, "step = ", step, ", collide_ = ", collide_);
    if (step == 1) {
        plan_success_ = path_finder_->planGlobalTraj(start_pt_, mp_config_.end_pos,
                                                     mp_config_.formation_type, mp_config_.formation_distance,
                                                     mp_config_.drone_id, gl_traj_);
        chlog::info(log, "plan success = ", plan_success_);

    } else {
        plan_success_ = path_finder_->replan(start_pt_.cast<double>(), start_vel_.cast<double>(),
                                            start_acc_.cast<double>(),
                                            mp_config_.end_pos.cast<double>(), end_vel_.cast<double>(),
                                            collide_);
    }

    if (plan_success_) {
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

        /* visulization */
        auto plan_data = &path_finder_->getPlanData();
        auto global_data = &path_finder_->getGlobalData();
        mp_publisher_->drawPolynomialTraj(global_data->global_traj_, 0.1,
                                           Eigen::Vector4d(0, 0, 0, 1), 0);
        mp_publisher_->drawGeometricPath(plan_data->kino_path_);
        mp_publisher_->drawBspline(info->position_traj_);

        return true;
    } else {
        chlog::info(log, "[MP Manager]:  generate new traj fail.");
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
    chlog::info(log, "[MP Manager]: [" + pos_call + "]: from " + state_str[pre_s]
                               + " to " + state_str[int(new_state)]);
}

void MPManager::formationCall(int formation_type) {
    switch (formation_type) {
        case VF_USV_TRIANGLE: {
            chlog::info(log, "[MP Manager]: VF_USV_TRIANGLE");
            break;
        }
        case VF_USV_LINE_HORIZONTAL: {
            chlog::info(log, "[MP Manager]: VF_USV_LINE_HORIZONTAL");
            break;
        }
        case VF_USV_LINE_VERTICAL: {
            chlog::info(log, "[MP Manager]: VF_USV_LINE_VERTICAL");
            break;
        }
        case VF_USV_INVERSION_TRIANGLE: {
            chlog::info(log, "[MP Manager]: VF_USV_INVERSION_TRIANGLE");
            break;
        }

    }

}

bool MPManager::GetControlOutput(TVec3 &vector_eus) {
    if (!receive_traj_) return false;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();

    t_cur += time_add_sum_;
    Eigen::Vector3d pos = {0, 0, 0}, vel = {0, 0, 0}, acc = {0, 0, 0};
    double yaw = 0;

    chlog::info(log, "t_cur = ", t_cur);

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
        chlog::info(log, "[MP Manager]: invalid time!");
        return false;
    }

    if (formation_target_.norm() > 0 && (formation_target_ - pos.cast<float>()).norm() > 2.0) {

        chlog::info(mp_config_.log_path, "command pos before = ", toStr(pos.cast<float>()));
        chlog::info(mp_config_.log_path, "distance to formation target =  ", (formation_target_ - pos.cast<float>()).norm());
    }

    if (mp_config_.is_speed_mode) {
        vector_eus = vel.cast<float>();
    } else {
        vector_eus = pos.cast<float>();
    }

    TVec3 dir(cos(yaw), sin(yaw), 0.0);
    mp_publisher_->DrawTrajCommand(pos.cast<float>(), 2 * dir, Eigen::Vector4d(1, 1, 0, 0.7), 2);
    return true;
}

void MPManager::OnUpdateMaxSpeed(float speed_max) {
    mp_config_.max_vel = speed_max;
    mp_config_.max_acc = speed_max;
    path_finder_->updateSpeedLimit(mp_config_.max_vel, mp_config_.max_acc);
    chlog::info(log, "[MP Manager]: update_speed limit:", to_string(mp_config_.max_vel));
}

void MPManager::OnUpdateDroneHeading(float drone_heading) {
    mp_config_.m_drone_heading = drone_heading;
}

void MPManager::checkCollisionReplan(TVec3& cur_pos) {
    if (!mp_config_.mp_map) return;
    chlog::info(log, "check_collision_state_ = ", check_collision_state_);
    switch (check_collision_state_) {
        case CHECK_COLLISION: {
            if (mp_config_.mp_map && !mp_config_.mp_map->isStateValid(cur_pos, false)) {
                check_collision_state_ = COLLOSION_INIT;
                collide_ = true;
            }
            break;
        }
        case COLLOSION_INIT: {
            check_collision_state_ = REPLAN_TARGET;
            break;
        }

        case REPLAN_TARGET: {
            const double dr = 0.5, dtheta = 30, dz = 0.3;
            double new_x, new_y, new_z, max_dist = -1.0;
            TVec3 goal;
            float new_min_dist;

            for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
                for (double theta = -90; theta <= 270; theta += dtheta) {

                    new_x = mp_config_.end_pos(0) + r * cos(theta / 57.3);
                    new_y = mp_config_.end_pos(1) + r * sin(theta / 57.3);

                    TVec3 new_pt(new_x, new_y, 0); // EUS
                    TVec3 new_pos_ENU;
                    mp_config_.mp_map->getMinDistance(new_pt, new_min_dist);

                    if (new_min_dist > max_dist) {
                        /* reset end_pt_ */
                        goal = new_pt;
                        max_dist = new_min_dist;
                    }
                }
            }

            if (max_dist > 0.3) {
                chlog::info(log, "change goal, replan. goal = " + toStr(goal));
                mp_config_.end_pos = goal;
                end_vel_.setZero();

                if (mp_state_ == EXEC_TRAJ) {
                    ChangeExecState(REPLAN_TRAJ, "SAFETY");
                }
                check_collision_state_ = REPLANNING;
                break;
            }
            mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 0.70));

            break;
        }

        case REPLANNING: {
            if (mp_config_.mp_map && mp_config_.mp_map->isStateValid(cur_pos, false)) {
                check_collision_state_ = ORIGINAL_TARGET;
            }
        }
            break;

        case ORIGINAL_TARGET: {
            mp_config_.end_pos = init_target_pos_;
            if (mp_state_ == EXEC_TRAJ) {
                ChangeExecState(REPLAN_TRAJ, "SAFETY");
            }
            check_collision_state_ = CHECK_COLLISION;
        }
            break;
    }

    /* ---------- check trajectory ---------- */
    if (mp_state_ == EXEC_TRAJ || mp_state_ == REPLAN_TRAJ) {

        double dist;
        bool   safe = path_finder_->checkTrajCollision(dist);
        if (!safe) {
            if (dist > 0.1) {
                chlog::info(log, "current traj: ", dist, "  m to collision" );
                collide_ = true;

                ChangeExecState(REPLAN_TRAJ, "SAFETY");
            }
        }
    }
}

void MPManager::checkEndPos() {
    if (mp_config_.targets.empty()) return;
    vector<TVec3> targets = mp_config_.targets;
    reverse(targets.begin(), targets.end());
    if ((drone_st_.drone_pos - mp_config_.end_pos).norm() < 2.0) { // reach the end pos
        targets.pop_back();
        mp_config_.end_pos = targets.back();
        chlog::info(mp_config_.log_path, "end pos = ", toStr(mp_config_.end_pos));
    }
}

void MPManager::setFormationTarget(const TVec3 &formation_pos_target) {
    formation_target_ = formation_pos_target;
    chlog::info(mp_config_.log_path, "usv get formation target = ", toStr(formation_target_));

}

void MPManager::ProcessState() {

    checkEndPos();
    checkCollisionReplan(drone_st_.drone_pos);
    switch (mp_state_) {
        case INIT: {
            ChangeExecState(WAIT_TARGET, "FSM");
            break;
        }

        case WAIT_TARGET: {
            receive_traj_ = false;
            if (!mp_config_.is_enable || !has_drone_update_ || !have_target_)
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

            bool success = CallKinodynamicReplan(1); // TODO test A* use 2
            if (success) {
                exec_start_pos_ = drone_st_.drone_pos;
                ChangeExecState(EXEC_TRAJ, "FSM");
            } else {
                receive_traj_ = false;
                ChangeExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case EXEC_TRAJ: {
            /* determine if need to replan */
            ros::Time time;
            double t0, t1, t2;
            time = ros::Time::now();
            LocalTrajData *info = &path_finder_->getLocaldata();
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();
            GlobalTrajData* global_data = &path_finder_->getGlobalData();

            t0 =  (ros::Time::now() - time).toSec();
            time = ros::Time::now();
            Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
            t1 =  (ros::Time::now() - time).toSec();

            chlog::info(log, "[MP Manager]: mp_config_.end_pos = ", toStr(mp_config_.end_pos),
                    ", drone_st_.drone_pos = ", toStr(drone_st_.drone_pos), ", t_cur = ",
                        t_cur, ", global_duration_ = ", global_data->global_duration_);
            chlog::info(log, "[MP Manager]: t0 = ", to_string(t0), ", t1 = ", to_string(t1));
            chlog::info(log, "(info->start_pos_ - pos).norm() = ", (info->start_pos_ - pos).norm());

            float err_target;
            if (mp_config_.targets.size() > 0) {
                err_target = (mp_config_.targets.back() - drone_st_.drone_pos).norm();
            } else {
                err_target = (mp_config_.end_pos - drone_st_.drone_pos).norm();
            }
            if (t_cur > global_data->global_duration_ - 1e-2 && err_target > 2.0) {
                ChangeExecState(GEN_NEW_TRAJ, "FSM");
                have_target_ = false;
                return;
            }  else if  ((info->start_pos_ - pos).norm() < 3.0 &&
                (exec_start_pos_ - drone_st_.drone_pos).norm() < 3.0 /*&& !collide_*/) {
//                chlog::info(log, "[MP Manager]: close to start pos!");
                return;

            } else {
                ChangeExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case REPLAN_TRAJ: {
            collide_ = true; // test
            LocalTrajData *info = &path_finder_->getLocaldata();
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();

            if (mp_config_.control_mode == VELOCITY_WITHOUT_CUR) {
                start_pt_ = drone_st_.drone_pos;
                start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur ).cast<float>();
            } else if (mp_config_.control_mode == POSITION_WITHOUT_CUR) {
                start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur ).cast<float>();
                start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur ).cast<float>();
                if ((start_pt_ - mp_config_.end_pos).norm() < 1) {
                    chlog::info(log, "[MP Manager]: near target, change goal");
                    have_target_ = false;
                    ChangeExecState(WAIT_TARGET, "FSM");
                }
            } else if (mp_config_.control_mode == POSITION_CUR || mp_config_.control_mode == VELOCITY_WITH_CUR) {
                start_pt_ = drone_st_.drone_pos;
                start_vel_ = drone_st_.drone_vel;
            }

            start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur).cast<float>();

            bool success = CallKinodynamicReplan(2);
            if (success) {
                exec_start_pos_ = drone_st_.drone_pos;
                ChangeExecState(EXEC_TRAJ, "FSM");
                path_find_fail_timer_ = 0;
            } else {
                path_find_fail_timer_++;
                if (path_find_fail_timer_ < 100) {
                    start_pt_ = drone_st_.drone_pos;
                    success = CallKinodynamicReplan(2);
                    chlog::info(log, "Replan fail, retrying...");
                    if (success) {
                        exec_start_pos_ = drone_st_.drone_pos;
                        ChangeExecState(EXEC_TRAJ, "FSM");
                    }

                }  else {
                    ChangeExecState(WAIT_TARGET, "FSM");
                    path_find_fail_timer_ = 0;
                    chlog::info(log, "[MP Manager]: A star search failed too many times!");
                }
            }
            break;
        }
        default:
            break;
    }
}
