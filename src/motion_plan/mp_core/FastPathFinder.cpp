// #include <fstream>
#include <motion_plan/mp_core/FastPathFinder.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

    FastPathFinder::FastPathFinder() {}

    FastPathFinder::~FastPathFinder() { chlog::info("motion_plan", "des manager" ) ; }

    void FastPathFinder::initPlanModules(const MP_Config &config, Sp<IMap> &map) {
        /* read algorithm parameters */
        ros::NodeHandle nh;
        nh.param("manager/max_jerk", pp_.max_jerk_, 4.0);
        nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
        nh.param("manager/clearance_threshold", pp_.clearance_, 0.2);
        nh.param("manager/local_segment_length", pp_.local_traj_len_, 6.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, 0.5);

        bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization;
        nh.param("manager/use_geometric_path", use_geometric_path, false);
        nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, true);
        nh.param("manager/use_topo_path", use_topo_path, false);
        nh.param("manager/use_optimization", use_optimization, true);

        mp_config_ = config;
        pp_.max_vel_ = mp_config_.max_vel;
        pp_.max_acc_ = mp_config_.max_acc;
        local_data_.traj_id_ = 0;
        sdf_map_.reset(new SDFMap);
        sdf_map_->initMap(nh);
        edt_environment_.reset(new EDTEnvironment);
        edt_environment_->setMap(sdf_map_);

        chlog::info("motion_plan", "[plan man]: pp_.max_vel_ = " + to_string2(pp_.max_vel_) + ", max acc_: " +
                                   to_string2(pp_.max_acc_));
        kino_path_finder_.reset(new KinodynamicAstar);
        kino_path_finder_->setParam(nh);
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
    }

    void FastPathFinder::setGlobalWaypoints(const TVec3 &waypoints) {
        plan_data_.global_waypoints_.push_back(waypoints.cast<double>());
    }

    void FastPathFinder::updateSpeedLimit(const float max_speed, const float max_acc) {
        pp_.max_vel_ = max_speed;
        pp_.max_acc_ = max_acc;
        kino_path_finder_->setSpeedLimit(pp_.max_vel_, pp_.max_acc_);
        bspline_optimizers_[0]->setSpeedLimit(pp_.max_vel_, pp_.max_acc_);
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
            float dist;
            mp_config_.mp_map->getMinDistance(fut_pt.cast<float>(), dist);
            if (dist < 0.3) {
                distance = radius;
                return false;
            }

            radius = (fut_pt - cur_pt).norm();
            fut_t += 0.02;
        }

        return true;
    }

    // SECTION topological replanning

    bool FastPathFinder::planGlobalTraj(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos) {
        plan_data_.clearTopoPaths();

        // generate global reference trajectory

        vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
        if (points.size() == 0)
        {
            std::cout << "no global waypoints!" << std::endl;
            return false;
        }

        points.insert(points.begin(), start_pos.cast<double>());

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        const double            dist_thresh = 4.0;

        for (int i = 0; i < points.size() - 1; ++i) {
            inter_points.push_back(points.at(i));
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
        Eigen::MatrixXd pos(pt_num, 3);
        for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i) {
            time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(0) = max(1.0, time(0));
        time(time.rows() - 1) *= 2.0;
        time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

        PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        // truncate a local trajectory

        double            dt, duration;
        Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
        NonUniformBspline bspline(ctrl_pts, 3, dt);

        global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
        local_data_.position_traj_ = bspline;
        local_data_.start_time_    = time_now;
        ROS_INFO("global trajectory generated.");

        updateTrajInfo();

        return true;
    }

    Eigen::MatrixXd FastPathFinder::reparamLocalTraj(double start_t, double& dt, double& duration) {
        /* get the sample points local traj within radius */

        vector<Eigen::Vector3d> point_set;
        vector<Eigen::Vector3d> start_end_derivative;

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

    bool FastPathFinder::replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                                Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool collide) {
        chlog::info("motion_plan", "[plan man]: plan manager start point: (" + to_string2(start_pt(0)) +
                                       ", " + to_string2(start_pt(1)) +
                                       ", " + to_string2(start_pt(2)) + ")" +
                                       ", start_vel, " + to_string2(start_vel(0)) +
                                       ", " + to_string2(start_vel(1)) +
                                       ", " + to_string2(start_vel(2)) + ")" +
                                       ", end_pt, " + to_string2(end_pt(0)) +
                                       ", " + to_string2(end_pt(1)) +
                                       ", " + to_string2(end_pt(2)) + ")" +
                                       ", end_vel, " + to_string2(end_vel(0)) +
                                       ", " + to_string2(end_vel(1)) +
                                       ", " + to_string2(end_vel(2)) );

        if (!collide) {
            /* truncate a new local segment for replanning */
            ros::Time time_now = ros::Time::now();
            double    t_now    = (time_now - global_data_.global_start_time_).toSec();
            double    local_traj_dt, local_traj_duration;
            double    time_inc = 0.0;

            Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
            NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
            local_data_.start_time_ = time_now;
            refineTraj(init_traj, time_inc);
            local_data_.position_traj_ = init_traj;
            global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);
        } else {

            ros::Time t1, t2;

            local_data_.start_time_ = ros::Time::now();
            double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

            // kinodynamic path searching

            t1 = ros::Time::now();

            kino_path_finder_->reset();

            int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, mp_config_.mp_plan_state, true, true);

            if (status == KinodynamicAstar::NO_PATH) {
                chlog::info("motion_plan", "[plan man]: kinodynamic search fail!");

                // retry searching with discontinuous initial state
                kino_path_finder_->reset();
                status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, mp_config_.mp_plan_state, false, false);

                if (status == KinodynamicAstar::NO_PATH) {
                    chlog::info("motion_plan", "[plan man]: Can't find path.");
                    return false;
                } else {
                    chlog::info("motion_plan", "[plan man]: retry search success.");
                }

            } else {
                chlog::info("motion_plan", "[plan man]: kinodynamic search success.");
            }

            plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

            t_search = (ros::Time::now() - t1).toSec();

            // parameterize the path to bspline

            double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
            vector<Eigen::Vector3d> point_set, start_end_derivatives;
            kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

            Eigen::MatrixXd ctrl_pts;
            NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
            NonUniformBspline init(ctrl_pts, 3, ts);

            // bspline trajectory optimization

            t1 = ros::Time::now();

            int cost_function;
            chlog::info("motion_plan", "[plan man]: mp_config_.mp_plan_state = ", mp_config_.mp_plan_state);

            cost_function = BsplineOptimizer::SAFE;
            if (status != KinodynamicAstar::REACH_END) {
                cost_function |= BsplineOptimizer::ENDPOINT;
            }

            ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

            t_opt = (ros::Time::now() - t1).toSec();

            // iterative time adjustment

            t1                    = ros::Time::now();
            NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

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

            chlog::info("motion_plan", "[plan man]: Reallocate ratio: " + to_string2( tn / to ));
            if (tn / to > 3.0) ROS_ERROR("reallocate error.");

            t_adjust = (ros::Time::now() - t1).toSec();

            // save planned results

            local_data_.position_traj_ = pos;

            double t_total = t_search + t_opt + t_adjust;
            chlog::info("motion_plan", "[plan man]: time: " + to_string(t_total) + ", search: " +
                                       to_string(t_search) + ", optimize: " + to_string(t_opt) +
                                       ", adjust time:" + to_string(t_adjust));

            pp_.time_search_   = t_search;
            pp_.time_optimize_ = t_opt;
            pp_.time_adjust_   = t_adjust;
        }

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
        chlog::info("motion_plan", "[plan man]: plan yaw");
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
        std::cout << "ratio: " << ratio << std::endl;
        reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
        time_inc += t_inc;

        ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
        best_traj = NonUniformBspline(ctrl_pts, 3, dt);
        ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                          << " seconds, time change is: " << time_inc);
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

}  // namespace fast_planner
