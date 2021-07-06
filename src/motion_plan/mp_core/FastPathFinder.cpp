// #include <fstream>
#include <motion_plan/mp_core/FastPathFinder.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

    FastPathFinder::FastPathFinder() {}

    FastPathFinder::~FastPathFinder() { }

    void FastPathFinder::initPlanModules(const MP_Config &config, Sp<IMap> &map) {
        /* read algorithm parameters */
        ros::NodeHandle nh;
        nh = config.nh;
        nh.param("/manager/max_jerk", pp_.max_jerk_, 4.0);
        nh.param("/manager/dynamic_environment", pp_.dynamic_, -1);
        nh.param("/manager/clearance_threshold", pp_.clearance_, 0.2);
        nh.param("/manager/local_segment_length", pp_.local_traj_len_, 6.0);
        nh.param("/manager/control_points_distance", pp_.ctrl_pt_dist, 0.5);

        bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization;
        nh.param("/manager/use_geometric_path", use_geometric_path, false);
        nh.param("/manager/use_kinodynamic_path", use_kinodynamic_path, true);
        nh.param("/manager/use_topo_path", use_topo_path, false);
        nh.param("/manager/use_optimization", use_optimization, true);

        mp_config_ = config;
        pp_.max_vel_ = mp_config_.max_vel;
        pp_.max_acc_ = mp_config_.max_acc;
        local_data_.traj_id_ = 0;
        sdf_map_.reset(new SDFMap);
        sdf_map_->initMap(nh);
        edt_environment_.reset(new EDTEnvironment);
        edt_environment_->setMap(sdf_map_);

        chlog::info(mp_config_.log_path, "[plan man]: pp_.max_vel_ = " + to_string2(pp_.max_vel_) + ", max acc_: " +
                                         to_string2(pp_.max_acc_));
        kino_path_finder_.reset(new KinodynamicAstar);
        kino_path_finder_->setParam(nh, mp_config_.log_path);
        kino_path_finder_->setEnvironment(map, edt_environment_);
        kino_path_finder_->setSpeedLimit(pp_.max_vel_, pp_.max_acc_);
        kino_path_finder_->init();

        bspline_optimizers_.resize(2);
        for (int i = 0; i < 2; ++i) {
            bspline_optimizers_[i].reset(new BsplineOptimizer);
            bspline_optimizers_[i]->setParam(nh);
            bspline_optimizers_[i]->setEnvironment(map);
            bspline_optimizers_[i]->setSpeedLimit(pp_.max_vel_, pp_.max_acc_);
        }
        global_data_.setMaxVel(pp_.max_vel_, pp_.max_acc_);

        initFormation();
    }

    void FastPathFinder::initFormation() {
        switch (mp_config_.formation_type) {
            case VF_USV_TRIANGLE: {
                chlog::info(mp_config_.log_path,"[USV Formation]: usv Formation call! Triangle!");
                drone_usv2_ = TVec3(-K_multi_usv_formation_distance, -K_multi_usv_formation_distance , 0);
                drone_usv3_ = TVec3(-K_multi_usv_formation_distance, K_multi_usv_formation_distance , 0);
                chlog::info(mp_config_.log_path, "[USV Formation]: drone_usv2_ = ", toStr(drone_usv2_),
                            ", drone_usv3 = ", toStr(drone_usv3_));
            }
                break;

            case VF_USV_INVERSION_TRIANGLE: {
                chlog::info(mp_config_.log_path,"[USV Formation]: usv Formation call! INVERSION Triangle!");
                drone_usv2_ = TVec3(K_multi_usv_formation_distance, K_multi_usv_formation_distance , 0);
                drone_usv3_ = TVec3(K_multi_usv_formation_distance , -K_multi_usv_formation_distance, 0);
            }
                break;


            case VF_USV_LINE_HORIZONTAL : {
                chlog::info(mp_config_.log_path,"[USV Formation]: usv Formation call! Line horizontal!");
                drone_usv2_ = TVec3( -K_multi_usv_formation_distance , 0, 0);
                drone_usv3_ = TVec3( -2 * K_multi_usv_formation_distance, 0, 0);
            }
                break;

            case VF_USV_LINE_VERTICAL : {
                chlog::info(mp_config_.log_path,"[USV Formation]: usv Formation call! Line Vertical!");
                drone_usv2_ = TVec3(0 , K_multi_usv_formation_distance,  0);
                drone_usv3_ = TVec3(0,2 * K_multi_usv_formation_distance, 0);
            }
                break;

            case VF_USV_ALL_RETURN: {
                chlog::info(mp_config_.log_path,"[USV Formation]: usv Formation call! All USVs Return!");
            }
                break;

            default:
                break;

        }
    }

    void FastPathFinder::setGlobalWaypoints(const vector<TVec3> &waypoints) {
        plan_data_.global_waypoints_.clear();
        for (auto i : waypoints) {
            plan_data_.global_waypoints_.push_back(i.cast<double>());
        }
    }

    void FastPathFinder::updateSpeedLimit(const float max_speed, const float max_acc) {
        pp_.max_vel_ = max_speed;
        pp_.max_acc_ = max_acc;
        kino_path_finder_->setSpeedLimit(pp_.max_vel_, pp_.max_acc_);
        bspline_optimizers_[0]->setSpeedLimit(pp_.max_vel_, pp_.max_acc_);
        global_data_.setMaxVel(pp_.max_vel_, pp_.max_acc_);
    }


    bool FastPathFinder::checkTrajCollision(double& distance) {

        double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

        double tm, tmp;
        local_data_.position_traj_.getTimeSpan(tm, tmp);
        Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

        double          radius = 0.0;
        Eigen::Vector3d fut_pt;
        double          fut_t = 0.02;

        while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
            fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);
            if (mp_config_.mp_map && !mp_config_.mp_map->isStateValid(fut_pt.cast<float>(), false)) {
                distance = radius;
                return false;
            }

            radius = (fut_pt - cur_pt).norm();
            fut_t += 0.02;
        }

        return true;
    }

    bool FastPathFinder::checkLineAviable(TVec3 start_pt, TVec3 end_pt) {
        TVec3 dir = (start_pt - end_pt).normalized();
        float dt_len = 0.5;
        TVec3 center_pos = start_pt;
        chlog::info(mp_config_.log_path, "start_pt = ", toStr(start_pt), ", end_pt = ", toStr(end_pt));
        while ((center_pos - end_pt).norm() > 0.8) {
            chlog::info(mp_config_.log_path, "center_pos = ", toStr(center_pos));
            if (mp_config_.mp_map && !mp_config_.mp_map->isStateValid(center_pos, false)) {
                return false;
            }
            center_pos -= dt_len * dir;
        }
        return true;
    }
    // SECTION topological replanning

    bool FastPathFinder::planGlobalTraj(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos,
                                        const int formation_type,
                                        const float formation_distance, const int usv_id,
                                        const PolynomialTraj poly_traj) {
        PolynomialTraj gl_traj;
        if (usv_id == 1) {
            plan_data_.clearTopoPaths();
            // generate global reference trajectory

            vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
            if (points.size() == 0)
            {
                chlog::info(mp_config_.log_path,  "no global waypoints!");
                return false;
            }

            points.insert(points.begin(), start_pos.cast<double>());

            // insert intermediate points if too far
            vector<Eigen::Vector3d> inter_points;
            const double            dist_thresh = 4.0;

            inter_points.push_back(points.front());
            for (int i = 0; i < points.size() - 1; ++i) {
                double dist = (points.at(i + 1) - points.at(i)).norm();
                if (dist > dist_thresh) {
                    int id_num = floor(dist / dist_thresh) + 1;

                    for (int j = 1; j < id_num; ++j) {
                        Eigen::Vector3d inter_pt =
                                points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                        inter_points.push_back(inter_pt);
                    }
                }
            }

            inter_points.push_back(points.back());
            if (inter_points.size() == 2) {
                Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
                inter_points.insert(inter_points.begin() + 1, mid);
            }

            // write position matrix
            int             pt_num = inter_points.size();
            Eigen::VectorXd time(pt_num - 1);
            calcMiniSnap(inter_points, gl_traj, time, 1);

            if (mp_config_.formation_type == VF_USV_LINE_VERTICAL) {
                usv2_gl_traj_ = gl_traj;
                usv3_gl_traj_ = gl_traj;
            } else {
                planFollowerGlobalTraj(inter_points, time, drone_usv2_, usv2_gl_traj_);
                planFollowerGlobalTraj(inter_points, time, drone_usv3_, usv3_gl_traj_);
            }

        } else {
            gl_traj = poly_traj;
        }

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        // truncate a local trajectory
        double            dt, duration;
        Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration, duration);
        NonUniformBspline bspline(ctrl_pts, 3, dt);

        global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
        local_data_.position_traj_ = bspline;
        local_data_.start_time_    = time_now;
        chlog::info(mp_config_.log_path, "global trajectory generated.");

        updateTrajInfo();

        return true;
    }

    void
    FastPathFinder::calcMiniSnap(vector<Eigen::Vector3d> &inter_points, PolynomialTraj &gl_traj, Eigen::VectorXd &time,
                                 int usv_id) {
        if (inter_points.size() - 1 != time.size()) {
            chlog::info(mp_config_.log_path, "Error! wrong mini snap inputs! pos size = ", inter_points.size(),
                        ", time size = ", time.size());
            return;
        }

        int             pt_num = inter_points.size();
        Eigen::MatrixXd pos(pt_num, 3);
        for (int i = 0; i < pt_num; ++i) {
            pos.row(i) = inter_points[i];
            TVec3 pos_i = inter_points[i].cast<float>();
            chlog::info(mp_config_.log_path, "33333333333, inter_points[", i, "] = ", toStr(pos_i));
        }

        Eigen::Vector3d zero(0, 0, 0);
        if (usv_id == 1) {
            for (int i = 0; i < pt_num - 1; ++i) {
                time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
                if (i > 0) {
                    if (fabs(time(i) - time(0)) > 0.2)  time(i) = 2 * time(0);
                }
                chlog::info(mp_config_.log_path, "leader time = ", time(i));
            }

            time(0) *= 2.0;
            time(0) = max(1.0, time(0));
            time(time.rows() - 1) *= 2.0;
            time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

        }
        gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);
    }


    void FastPathFinder::planFollowerGlobalTraj(vector<Eigen::Vector3d> &leader_pos, Eigen::VectorXd &time,
                                                const TVec3 &dorne_move,
                                                PolynomialTraj &gl_traj) {
        TVec3 line_dir_norm(1, 0, 0);
        vector<Eigen::Vector3d> follower_pos;
        TVec3 res;
        if (mp_config_.formation_type == VF_USV_TRIANGLE || mp_config_.formation_type == VF_USV_INVERSION_TRIANGLE) {
            for (int i = 0; i < leader_pos.size() - 1; i++) {
                TVec3 line_dir = ((leader_pos.at(i + 1) - leader_pos.at(i)).normalized()).cast<float>();
                Eigen::Matrix3f rotMatrix = Eigen::Quaternionf::FromTwoVectors(line_dir_norm,
                                                                               line_dir).toRotationMatrix();
                cout << "Rotation_usv 2 = " << endl << rotMatrix << endl;
                res = rotMatrix * dorne_move;
                Eigen::Vector3d pos = leader_pos.at(i) + res.cast<double>();
                chlog::info(mp_config_.log_path, "pos = ", toStr(pos.cast<float>()), ", leader pos = ",
                            toStr(leader_pos.at(i).cast<float>()), ", res = ", toStr(res));
                follower_pos.push_back(pos);
            }

            Eigen::Vector3d pos = leader_pos.back() + res.cast<double>();
            follower_pos.push_back(pos);
        } else if (mp_config_.formation_type == VF_USV_LINE_HORIZONTAL) {
            for (int i = 0; i < leader_pos.size() ; i++) {
                Eigen::Vector3d pos = leader_pos.at(i) + dorne_move.cast<double>();
                chlog::info(mp_config_.log_path, "pos = ", toStr(pos.cast<float>()), ", leader pos = ",
                            toStr(leader_pos.at(i).cast<float>()), ", res = ", toStr(res));
                follower_pos.push_back(pos);
            }
        }

        calcMiniSnap(follower_pos, gl_traj, time, 2);
    }

    Eigen::MatrixXd FastPathFinder::reparamLocalTraj(double start_t, double &dt, double &duration, int step) {
        /* get the sample points local traj within radius */

        vector<Eigen::Vector3d> point_set;
        vector<Eigen::Vector3d> start_end_derivative;

        global_data_.setSetp(step);
        global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                                     start_end_derivative, dt, duration);

        /* parameterization of B-spline */

        Eigen::MatrixXd ctrl_pts;
        NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
        plan_data_.local_start_end_derivative_ = start_end_derivative;
        // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

        return ctrl_pts;
    }
// !SECTION

// SECTION kinodynamic replanning

    void FastPathFinder::findCollisionRange(vector<Eigen::Vector3f>& colli_start,
                                            vector<Eigen::Vector3f>& colli_end) {
        bool               last_safe = true, safe;
        double             t_m, t_mp;
        NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
        initial_traj->getTimeSpan(t_m, t_mp);

        /* find range of collision */
        double t_s = -1.0, t_e = -1.0;
        for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

            Eigen::Vector3f ptc = initial_traj->evaluateDeBoor(tc).cast<float>();
            safe = mp_config_.mp_map && mp_config_.mp_map->isStateValid(ptc, false);
            /*chlog::info(mp_config_.log_path, "find safe traj pos = ", toStr(ptc), ", is safe = ",
                        safe);*/

            if (last_safe && !safe) {
                colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05).cast<float>());
                TVec3 cur_coll_start = colli_start.back();
                chlog::info(mp_config_.log_path, "collision start = ", toStr(cur_coll_start));
                if (t_s < 0.0) t_s = tc - 0.05;
            } else if (!last_safe && safe) {
                colli_end.push_back(ptc);
                t_e = tc;
            }

            last_safe = safe;
        }

        if (colli_end.size() == 0) {
            TVec3 ptc = initial_traj->evaluateDeBoor(t_mp).cast<float>();
            colli_end.push_back(ptc);
        }

        if (colli_start.size() == 0) return;

        if (colli_start.size() == 1 && colli_end.size() == 0) return;
    }

    bool FastPathFinder::replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                                Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool collide) {
        chlog::info(mp_config_.log_path, "[plan man]: plan manager start point: (" + to_string2(start_pt(0)) +
                                         ", " + to_string2(start_pt(1)) +
                                         ", " + to_string2(start_pt(2)) + ")" +
                                         ", start_vel, " + to_string2(start_vel(0)) +
                                         ", " + to_string2(start_vel(1)) +
                                         ", " + to_string2(start_vel(2)) + ")");

        chlog::info(mp_config_.log_path, "[plan man]: , end_pt, " + to_string2(end_pt(0)) +
                                         ", " + to_string2(end_pt(1)) +
                                         ", " + to_string2(end_pt(2)) + ")" +
                                         ", end_vel, " + to_string2(end_vel(0)) +
                                         ", " + to_string2(end_vel(1)) +
                                         ", " + to_string2(end_vel(2)) +
                                         ", collide = ", collide);
        ros::Time t1, t2;
        ros::Time time_now = ros::Time::now();
        double    t_now    = (time_now - global_data_.global_start_time_).toSec() ;
/*        if (mp_config_.drone_id == 2) {
            t_now += 1.0;
            time_now += ros::Duration(1.0);
        }*/
        chlog::info(mp_config_.log_path, "time_now = ", time_now.toSec(), ", global_data_.global_start_time_ = ",
                    global_data_.global_start_time_.toSec(), ", t_now = ", t_now);
        double    local_traj_dt, local_traj_duration;
        double    time_inc = 0.0;
        Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration, 2);
        if (ctrl_pts.rows() == 0 || ctrl_pts.cols() == 0) return false;
        NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
        local_data_.start_time_ = time_now;

        if (!collide) {
            /* truncate a new local segment for replanning */
            local_data_.position_traj_ = init_traj;
            global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);
        } else {
            double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

            plan_data_.initial_local_segment_ = init_traj;
/*            vector<Eigen::Vector3f> colli_start, colli_end;
            findCollisionRange(colli_start, colli_end);*/
            // kinodynamic path searching

            t1 = ros::Time::now();

            kino_path_finder_->reset();

            int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt,
                                                   end_vel, mp_config_.mp_plan_state, true, true);

            if (status == KinodynamicAstar::NO_PATH) {
                chlog::info(mp_config_.log_path, "[plan man]: kinodynamic search fail!");

                // retry searching with discontinuous initial state
                kino_path_finder_->reset();
                status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, mp_config_.mp_plan_state, false, false);

                if (status == KinodynamicAstar::NO_PATH) {
                    chlog::info(mp_config_.log_path, "[plan man]: Can't find path.");
                    return false;
                } else {
                    chlog::info(mp_config_.log_path, "[plan man]: retry search success.");
                }

            } else {
                chlog::info(mp_config_.log_path, "[plan man]: kinodynamic search success.");
            }

            plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

            t_search = (ros::Time::now() - t1).toSec();

            // parameterize the path to bspline

            double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
            vector<Eigen::Vector3d> point_set, start_end_derivatives;
            kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

            NonUniformBspline best_traj;
            if (true) {
                Eigen::MatrixXd ctrl_pts;
                NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                NonUniformBspline init(ctrl_pts, 3, ts);

                // bspline trajectory optimization

                t1 = ros::Time::now();

                Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[0]->BsplineOptimizeTraj(
                        ctrl_pts, ts, BsplineOptimizer::SMOOTH, 1, 1);

                Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[0]->BsplineOptimizeTraj(
                        opt_ctrl_pts1, ts, BsplineOptimizer::NORMAL_PHASE,1, 1);

                t_opt = (ros::Time::now() - t1).toSec();

                // iterative time adjustment

                t1                    = ros::Time::now();
                NonUniformBspline pos = NonUniformBspline(opt_ctrl_pts2, 3, ts);

                double to = pos.getTimeSum();
                pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
                bool feasible = pos.checkFeasibility(false);

                int iter_num = 0;
                while (!feasible && ros::ok()) {

                    feasible = pos.reallocateTime();

                    if (++iter_num >= 3) break;
                }

                // pos.checkFeasibility(true);
                // cout << "[Main]: iter num: " << iter_num << endl;

                double tn = pos.getTimeSum();

                chlog::info(mp_config_.log_path, "[plan man]: Reallocate ratio: " + to_string2( tn / to ));
                if (tn / to > 3.0) ROS_ERROR("reallocate error.");

                t_adjust = (ros::Time::now() - t1).toSec();

                // save planned results

                local_data_.start_time_ = ros::Time::now();
                local_data_.position_traj_ = pos;

                double t_total = t_search + t_opt + t_adjust;
                chlog::info(mp_config_.log_path, "[plan man]: time: " + to_string(t_total) + ", search: " +
                                                 to_string(t_search) + ", optimize: " + to_string(t_opt) +
                                                 ", adjust time:" + to_string(t_adjust));

                time_inc = t_total;

                chlog::info(mp_config_.log_path,  "get position, local_start_time_ = ", t_now, ", local_end_time_ = "
                        , local_traj_duration + time_inc + t_now , ", time_inc = " , time_inc,
                            ",local_traj_duration = ", local_traj_duration);

            } else {
                plan_data_.topo_traj_pos1_.resize(1);
                plan_data_.topo_traj_pos2_.resize(1);
                vector<thread> optimize_threads;
                for (int i = 0; i < 1; ++i) {
                    optimize_threads.emplace_back(&FastPathFinder::optimizeTopoBspline, this, t_now,
                                                  local_traj_duration, point_set, i, start_end_derivatives);
                }
                for (int i = 0; i < 1; ++i) {

                    optimize_threads[i].join();
                    chlog::info(mp_config_.log_path, "optimize i " , i );
                }

                t_opt = (ros::Time::now() - t1).toSec();
                chlog::info(mp_config_.log_path, "[planner]: optimization time: ", t_opt );
                best_traj = plan_data_.topo_traj_pos2_[0];
                refineTraj(best_traj, time_inc);
            }
        }

        global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                  local_traj_duration + time_inc + t_now, time_inc);
        updateTrajInfo();

        return true;
    }

    void FastPathFinder::updateTrajInfo() {
        local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_          = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

// !SECTION

    void FastPathFinder::planYaw(const Eigen::Vector3d& start_yaw) {
        chlog::info(mp_config_.log_path, "[plan man]: plan yaw");
        //auto t1 = ros::Time::now();
        // calculate waypoints of heading

        auto&  pos      = local_data_.position_traj_;
        double duration = pos.getTimeSum();

        //Eigen::Vector3d new_start_yaw(0,0,0);

        double dt_yaw  = 0.3;
        int    seg_num = ceil(duration / dt_yaw);
        dt_yaw         = duration / seg_num;

        //const double            forward_t = 2.0;
        //double                  last_yaw  = start_yaw(0);
        vector<Eigen::Vector3d> waypts;
        vector<int>             waypt_idx;

        // seg_num -> seg_num - 1 points for constraint excluding the boundary states

        for (int i = 0; i < seg_num; ++i) {
            waypts.push_back(start_yaw);
            waypt_idx.push_back(i);
        }


        // calculate initial control points with boundary state constraints

        Eigen::MatrixXd yaw(seg_num + 3, 1);
        // Eigen::MatrixXd yaw(6, 1);
        yaw.setZero();

        Eigen::Matrix3d states2pts;
        states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
                dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
        yaw.block(0, 0, 3, 1) = states2pts * start_yaw;
//    yaw.block(0, 0, 3, 1) = start_yaw;

        Eigen::Vector3d end_yaw = start_yaw;
        //Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
        //  Eigen::Vector3d end_v(0, 0, 0);
        //Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
        //calcNextYaw(last_yaw, end_yaw(0));
        yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;
        //  yaw.block(3, 0, 3, 1) = end_yaw;

        // solve
        bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
        int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
//    int cost_func = BsplineOptimizer::YAW_SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
        yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

        // update traj info
        local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
        local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
        local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

        vector<double> path_yaw;
        for (unsigned int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
        plan_data_.path_yaw_    = path_yaw;
        plan_data_.dt_yaw_      = dt_yaw;
        plan_data_.dt_yaw_path_ = dt_yaw;

//        std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
    }

    LocalTrajData &FastPathFinder::getLocaldata() {
        return local_data_;
    }

    MidPlanData& FastPathFinder::getPlanData() {
        return plan_data_;
    }

    GlobalTrajData& FastPathFinder::getGlobalData() {
        return global_data_;
    }

    PolynomialTraj& FastPathFinder::getUSV2PolynomialTraj() {
        return usv2_gl_traj_;
    };

    PolynomialTraj& FastPathFinder::getUSV3PolynomialTraj() {
        return usv3_gl_traj_;
    };

    void FastPathFinder::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
        ros::Time t1 = ros::Time::now();
        time_inc     = 0.0;
        double    dt, t_inc;
        const int max_iter = 1;

        // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
        Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
        int             cost_function = BsplineOptimizer::NORMAL_PHASE;

        best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
        double ratio = best_traj.checkRatio();
        chlog::info(mp_config_.log_path, "[Refine]: ratio: " , ratio );
        reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
        time_inc += t_inc;

        ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
        best_traj = NonUniformBspline(ctrl_pts, 3, dt);
        chlog::info(mp_config_.log_path, "[Refine]: cost " ,  (ros::Time::now() - t1).toSec()
                ,  " seconds, time change is: " , time_inc);
    }

    void FastPathFinder::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
        int    prev_num    = bspline.getControlPoint().rows();
        double time_origin = bspline.getTimeSum();
        int    seg_num     = bspline.getControlPoint().rows() - 3;
        // double length = bspline.getLength(0.1);
        // int seg_num = ceil(length / pp_.ctrl_pt_dist);

        ratio = min(1.01, ratio);
        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt              = duration / double(seg_num);
        time_inc        = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt) {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }
        NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                                 ctrl_pts);
        // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
    }

    void FastPathFinder::optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                                             int traj_id,
                                             vector<Eigen::Vector3d> start_end_points) {
        ros::Time t1;
        double    tm1, tm2, tm3;

        t1 = ros::Time::now();

        // parameterize B-spline according to the length of guide path
        int             seg_num = kino_path_finder_->pathLength(guide_path) / pp_.ctrl_pt_dist;
        Eigen::MatrixXd ctrl_pts;
        double          dt;

        ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt, start_end_points);
        // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

        // discretize the guide path and align it with B-spline control points
        vector<Eigen::Vector3d> guide_pt;
        guide_pt = kino_path_finder_->discretizePath(guide_path, int(ctrl_pts.rows()) - 2);

        guide_pt.pop_back();
        guide_pt.pop_back();
        guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

        // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
        if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

        tm1 = (ros::Time::now() - t1).toSec();
        t1  = ros::Time::now();

        // first phase, path-guided optimization

        bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
        Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
                ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

        plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

        tm2 = (ros::Time::now() - t1).toSec();
        t1  = ros::Time::now();

        // second phase, normal optimization

        Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
                opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

        plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

        tm3 = (ros::Time::now() - t1).toSec();
        chlog::info(mp_config_.log_path, "optimization ", traj_id,
                    " cost ", tm1, ", ", tm2, ", ", tm3, "seconds");
    }

    Eigen::MatrixXd FastPathFinder::reparamLocalTraj(double start_t, double duration, int seg_num, double &dt,
                                                     vector<Eigen::Vector3d> &start_end_derivative) {
        vector<Eigen::Vector3d> point_set;

        dt = duration / seg_num;
        global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
        plan_data_.local_start_end_derivative_ = start_end_derivative;

        /* parameterization of B-spline */
        Eigen::MatrixXd ctrl_pts;
        NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
        // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

        return ctrl_pts;
    }

}  // namespace fast_planner
