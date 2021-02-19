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
    fp_config_.max_vel = mp_config_.max_vel;
    fp_config_.max_acc = mp_config_.max_acc;
    fp_config_.mp_plan_state = mp_config_.mp_plan_state;
    fp_config_.m_toward_point = mp_config_.m_toward_point;
    path_finder_->initPlanModules(fp_config_, mp_config_.mp_map);
    receive_traj_ = false;
    mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 1.0));

    chlog::info("motion_plan", "[MP Mananger]: Triggered! end_pt_ = " + toStr(mp_config_.end_pos), "max_vel = "
                                                                                                   + to_string(fp_config_.max_vel));
    ChangeExecState(WAIT_TARGET, "FSM");
    end_vel_.setZero();
    init_target_pos_ = mp_config_.end_pos;
}

void MPManager::updateFlightData(const TFlightData &data) {
    flight_data_ = data;
    OnUpdateBladeFittingLine(data.m_mp_data.fitting_line_dots);
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

void MPManager::OnUpdateBladeFittingLine(const wa_ros_msgs::BezierCurvePnts &line_dot) {
//        chlog::info("motion_plan", "line_dot.bezier_curve_points.size() = ", line_dot.bezier_curve_points.size());
    if (line_dot.bezier_curve_points.size() > 0 && mp_state_ > WAIT_TARGET) {
        wa_ros_msgs::BezierCurvePnts path;
        wa_ros_msgs::FittedPoints pnt1, pnt2;
        wa_ros_msgs::FittedPoints pnt;
        chlog::info("motion_plan", "[MP Manager]: update blade fitting line!");

        for (auto &bezier_curve_point : line_dot.bezier_curve_points) {
            TVec3 pos_cur;
            pos_cur.x() = bezier_curve_point.x;
            pos_cur.y() = bezier_curve_point.y;
            pos_cur.z() = bezier_curve_point.z;
            TVec3 pos_in_turbine_EUS;
            changeToTurbineFrame(pos_cur, pos_in_turbine_EUS);

            TVec3 pos = pos_in_turbine_EUS - dist_config_ * insp_vec_ENU_; // TODO update + -
            pnt.x = pos.x();
            pnt.y = pos.y();
            pnt.z = pos.z();
            path.bezier_curve_points.push_back(pnt);
        }

        mp_publisher_->DrawBladePath(line_dot);
        mp_publisher_->DrawDronePath(path);
        if (!mp_publisher_->checkPathAvailable(path)) {
            chlog::info("motion_plan", "[MP Manager]: blade line false!!!!!!!");
            return;
        }

        pnt1 = path.bezier_curve_points.front();
        pnt2 = path.bezier_curve_points.back();
        fitting_line_.m_p0.x() = pnt1.x;
        fitting_line_.m_p0.y() = pnt1.y;
        fitting_line_.m_p0.z() = pnt1.z;

        fitting_line_.m_p1.x() = pnt2.x;
        fitting_line_.m_p1.y() = pnt2.y;
        fitting_line_.m_p1.z() = pnt2.z;
        TVec3 dir = fitting_line_.dv();
        TLine cur_tar_origin(drone_st_.drone_pos, init_target_pos_);
        float angle = acos(cur_tar_origin.normal().dot(fitting_line_.normal()));
        chlog::info("motion_plan", "[MP Manager]: init_target_pos_ = ", toStr(init_target_pos_), ", angle = ",
                    angle * RAD2DEG);
        if (angle * RAD2DEG > 90) {
            dir = -dir;
            swap(fitting_line_.m_p0, fitting_line_.m_p1);
        }/* else if (angle * RAD2DEG > 40 && angle * RAD2DEG < 90) {
                chlog::info("motion_plan", "[MPManger]: return for large angle");
                return;
            }*/

        float dist = 100000;
        int min_i = 0;
        TVec3 cur_point;
        for (int i = 0; i < (int) path.bezier_curve_points.size(); i++) {
            cur_point = TVec3(path.bezier_curve_points[i].x, path.bezier_curve_points[i].y,
                              path.bezier_curve_points[i].z);

            if ((cur_point - drone_st_.drone_pos).norm() < dist) {
                dist = (cur_point - drone_st_.drone_pos).norm();
                min_i = i;
            }
        }

        TLine start_end(fitting_line_.m_p0, fitting_line_.m_p1);
        min_dist_to_line_ = start_end.dist(drone_st_.drone_pos);

        TVec3 min_to_cur = TVec3(path.bezier_curve_points[min_i].x, path.bezier_curve_points[min_i].y,
                                 path.bezier_curve_points[min_i].z);

        float target_length;
        if (min_dist_to_line_ > 2) {
            target_length = 7;
        } else {
            target_length = 15;
        }

        if (mp_config_.mp_plan_state == ROOTTURN) {
            target_length = 3;
        }

        mp_config_.end_pos = min_to_cur + target_length * dir.normalized();
        chlog::info("motion_plan", "[MP Manager]: line fitting update end_pos = " + toStr(mp_config_.end_pos));
        mp_publisher_->drawGoal(mp_config_.end_pos, 1, Eigen::Vector4d(1, 0, 0, 1.0));

        end_vel_ = mp_config_.max_vel * fitting_line_.normal();
        path_finder_->setGlobalWaypoints(fitting_line_.m_p1, fitting_line_.m_p0, drone_st_.drone_pos);
    } else {
        min_dist_to_line_ = 0;
    }
}

bool MPManager::CallKinodynamicReplan() {
    bool plan_success = path_finder_->replan(start_pt_.cast<double>(), start_vel_.cast<double>(),
                                             start_acc_.cast<double>(),
                                             mp_config_.end_pos.cast<double>(), end_vel_.cast<double>());
    if (!plan_success) {
        start_vel_ = TVec3(0, 0, 0);
        path_finder_->replan(start_pt_.cast<double>(), start_vel_.cast<double>(), start_acc_.cast<double>(),
                             mp_config_.end_pos.cast<double>(), end_vel_.cast<double>());
    }
    if (plan_success) {
        auto info = &path_finder_->getLocaldata();

        /* get traj result */
        wa_ros_msgs::Bspline bspline;
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

void MPManager::bsplineResult(const wa_ros_msgs::Bspline &msg) {
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
    double yaw = (flight_data_.m_drone.drone_heading + flight_data_.m_turbine.heading) * M_PI / 180.0f;

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
    chlog::info("motion_plan", "[MP Manager]: mp pos command = ", pos.x(), ",", pos.y(), ",", pos.z(),
                ", is speed mode = ", mp_config_.is_speed_mode);
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

void MPManager::changeToTurbineFrame(TVec3 &pnt, TVec3 &pos_in_turbine_EUS) {
    TVec3 pos_in_local_EUS = ENU2EUS(pnt); // pos in local EUS
    pos_in_turbine_EUS =
            FrameTransform::GetInstance()->TransFrame(pos_in_local_EUS, LocalEUS, TurbineEUS);
}

void MPManager::ProcessState() {
//        chlog::info("motion_plan", "[MP Manager]: mp_state_ = " , mp_state_);
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
                if (mp_config_.mp_plan_state == TRACKING ||
                    mp_config_.mp_plan_state == ROOTTURN || mp_config_.mp_plan_state == POINTTOPOINT) {
                    path_finder_->setGlobalWaypoints(mp_config_.end_pos, drone_st_.drone_pos, drone_st_.drone_pos);
                    chlog::info("motion_plan", "[MP Manager]: update flight corridor!");
                }

                ChangeExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case GEN_NEW_TRAJ: {
            start_pt_ = drone_st_.drone_pos;
            start_vel_ = drone_st_.drone_vel;
            start_acc_.setZero();
            if ((min_dist_to_line_ > 1.3 && mp_config_.mp_plan_state == TRACKING) ||
                mp_config_.mp_plan_state == ROOTTURN || mp_config_.mp_plan_state == POINTTOPOINT) {
                path_finder_->setGlobalWaypoints(mp_config_.end_pos, drone_st_.drone_pos, drone_st_.drone_pos);
                chlog::info("motion_plan", "[MP Manager]: update flight corridor!");
            }
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
            } else if ((info->start_pos_ - pos).norm() < 1.5 &&
                       mp_config_.mp_plan_state != MotionPlanState::ROOTTURN) {
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

//                planner_manager_->setGlobalWaypoints(end_pt_, start_pt_);
            if (min_dist_to_line_ > 2 &&
                (mp_config_.mp_plan_state == TRACKING || mp_config_.mp_plan_state == ROOTTURN)) {
                path_finder_->setGlobalWaypoints(mp_config_.end_pos, drone_st_.drone_pos, drone_st_.drone_pos);
            }
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
