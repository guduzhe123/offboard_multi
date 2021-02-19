#include "motion_plan/path_searching/kinodynamic_astar.h"
#include <sstream>

using namespace std;
using namespace Eigen;

namespace fast_planner {
    KinodynamicAstar::~KinodynamicAstar() {
        for (int i = 0; i < allocate_num_; i++) {
            delete path_node_pool_[i];
        }
    }

    void KinodynamicAstar::setParam(ros::NodeHandle& nh) {
        nh.param("search/max_tau", max_tau_, 0.3);
        nh.param("search/init_max_tau", init_max_tau_, 0.4);
        nh.param("search/w_time", w_time_, 10.0);
        nh.param("search/horizon", horizon_, 7.0);
        nh.param("search/resolution_astar", resolution_, 0.1);
        nh.param("search/time_resolution", time_resolution_, 0.8);
        nh.param("search/lambda_heu", lambda_heu_, 5.0);
        nh.param("search/margin", margin_, 0.3);
        nh.param("search/allocate_num", allocate_num_, 10000);
        nh.param("search/check_num", check_num_, 5);
        nh.param("search/dist_radius", check_radius_, 2.0);
        nh.param("search/dist_lambda", dist_lambda_, 20.0);
        nh.param("search/safe_dist_tip", safe_dist_tip_, 7.0);

    }

    void KinodynamicAstar::setSpeedLimit(const double max_vel, const double max_acc) {
        max_vel_ = max_vel;
        max_acc_ = max_acc;
        chlog::info("motion_plan", "[Kino Astar] margin:" + to_string2(margin_), ", max_vel =", max_vel_);
    }

    void KinodynamicAstar::setTargetPoint(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &start_point
                                                    , const Eigen::Vector3d &cur_pos) {
        target_pos_ = target_pos;
        start_pos_ = start_point;
        odom_pos_ = cur_pos;
        chlog::info("motion_plan", "[Kino Astar] target pos: (" + to_string2(target_pos_(0)) +
                                   ", " + to_string2(target_pos_(1)) +
                                   ", " + to_string2(target_pos_(2)) + ")" +
                                   ", start point: (" + to_string2(start_point(0)) +
                                   ", " + to_string2(start_point(1)) +
                                   ", " + to_string2(start_point(2)) + ")");
    }

    void KinodynamicAstar::setCirclePoint(const TVec3 &mp_circle_point) {
        circle_pos_ = mp_circle_point.cast<double>();
        chlog::info("motion_plan", "[Kino Astar]: Circle! circle center point = " + toStr(mp_circle_point));
    }

    void
    KinodynamicAstar::calcDistToCenter(double &dist, const Eigen::Vector3d &cur_pos, const Eigen::Vector3d &start_pos, bool show) {
        // start_pos 目标曲线的起始点
        // cur_pos  A*生成的点
        TLine start_end(start_pos.cast<float>(), target_pos_.cast<float>());

        dist = start_end.dist(cur_pos.cast<float>());
    }

    int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                                 Eigen::Vector3d end_pt, Eigen::Vector3d end_v,
                                 const MotionPlanState& afc_state,
                                 bool init, bool init_search_success,
                                 bool dynamic, double time_start) {
        start_vel_ = start_v;
        start_acc_ = start_a;
        end_pos_ = end_pt;
//        odom_pos_ = start_pt;
        /* ---------- initialize ---------- */
        PathNodePtr cur_node    = path_node_pool_[0];
        cur_node->parent        = NULL;
        cur_node->state.head(3) = start_pt;
        cur_node->state.tail(3) = start_v;
        cur_node->index         = posToIndex(start_pt);
        cur_node->g_score       = 0.0;

        Eigen::VectorXd end_state(6);
        Eigen::Vector3i end_index;
        double          time_to_goal;

        end_state.head(3)    = end_pt;
        end_state.tail(3)    = end_v;
        end_index            = posToIndex(end_pt);
        cur_node->f_score    = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
        cur_node->node_state = IN_OPEN_SET;
        cur_node->input = {0,0,0};

        open_set_.push(cur_node);
        use_node_num_ += 1;

        expanded_nodes_.insert(cur_node->index, cur_node);

        PathNodePtr terminate_node = NULL;
        //bool        init_search    = init;
        bool        search_successed = init_search_success;
        const int   tolerance      = 1.5;

        /* ---------- search loop ---------- */
        while (!open_set_.empty()) {
            /* ---------- get lowest f_score node ---------- */
            cur_node = open_set_.top();

            /* ---------- determine termination ---------- */

/*            bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                            abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                            abs(cur_node->index(2) - end_index(2)) <= tolerance;*/
            bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
            bool near_end = (cur_node->state.head(3) - end_pt).norm() <= tolerance;

            if (reach_horizon || near_end) {
                chlog::info("motion_plan", " \n " );
                chlog::info("motion_plan", "[Kino Astar]:---------------------- " );
                chlog::info("motion_plan", "[Kino Astar] use node num: " + to_string(use_node_num_) +
                                                ", iter num: " +to_string(iter_num_), ", out corridor num = ", outof_corridor_mum_,
                                                ", in collosion num = ", in_collision_num_
                                                , ", in closeset num = ", in_closed_set_num_, ", vel_infeasible_num_ = ", vel_infeasible_num_
                                                ,", feasible_num_ = " , feasible_num_
                                                , ", total_tried_num_ = ", total_tried_num_);

                terminate_node = cur_node;
                retrievePath(terminate_node);
                has_path_ = true;

                if (near_end) {
                    chlog::info("motion_plan", "[Kino Astar]: near end.");

                    /* one shot trajectory */
                    estimateHeuristic(cur_node->state, end_state, time_to_goal);
                    computeShotTraj(cur_node->state, end_state, time_to_goal);

                    if (terminate_node->parent == NULL && !is_shot_succ_)
                        return NO_PATH;
                    else
                        return REACH_END;
                } else if (reach_horizon) {
                    chlog::info("motion_plan", "[Kino Astar]: Reach horizon_");
                    return REACH_HORIZON;
                }
            }

            /* ---------- pop node and add to close set ---------- */
            open_set_.pop();
            cur_node->node_state = IN_CLOSE_SET;
            iter_num_ += 1;

            /* ---------- init state propagation ---------- */
            double res = 1 / 2.0, time_res = 1 / 2.0/*, time_res_init = 1 / 2.0*/;

            Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
            Eigen::Matrix<double, 6, 1> pro_state;
            vector<PathNodePtr>         tmp_expand_nodes;
            Eigen::Vector3d             um;
            double                      pro_t;

            vector<Eigen::Vector3d> inputs;
            vector<double>          durations;


            float max_time;
            if (afc_state == MotionPlanState::CIRCLE && !search_successed) {
                max_acc_ = 5.0;
                max_time = 2.0;
                res = 1 /5.0, time_res = 1 / 10.0;
                chlog::info("motion_plan", "[Kino Astar]: blade circle big step! ");
            } else {
                max_acc_ = 1.5;
                max_time = max_tau_;
            }

            {
                for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
                    for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                        for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
                            um << ax, ay, az;
                            inputs.push_back(um);
                        }
                for (double tau = time_res * max_time; tau <= max_time; tau += time_res * max_time)
                    durations.push_back(tau);
            }

            /* ---------- state propagation loop ---------- */
            for (unsigned int i = 0; i < inputs.size(); ++i)
                for (unsigned int j = 0; j < durations.size(); ++j) {
                    //init_search = false;
                    um          = inputs[i];
                    double tau  = durations[j];
                    stateTransit(cur_state, pro_state, um, tau);
                    pro_t = cur_node->time + tau;
                    total_tried_num_++;
                    /* ---------- check if in free space ---------- */

                    double dist_to_center = 0/*, cur_dist*/;
                    if ( afc_state == MotionPlanState::TRACKING || afc_state == MotionPlanState::ROOTTURN
                        || afc_state == MotionPlanState::POINTTOPOINT) {
                        calcDistToCenter(dist_to_center, pro_state.head(3), start_pos_, true);
                        if (dist_to_center > check_radius_ ) {
                            outof_corridor_mum_++;
                            continue;
                        }
                    }

                    if (afc_state == MotionPlanState::CIRCLE) {
                        double dist_to_tip = (circle_pos_ - pro_state.head(3)).norm();
                        if (dist_to_tip < safe_dist_tip_) {
                            outof_corridor_mum_++;
                            continue;
                        }
                    }

                    /* not in close set */
                    Eigen::Vector3i pro_id   = posToIndex(pro_state.head(3));
                    int             pro_t_id = timeToIndex(pro_t);

                    PathNodePtr pro_node = expanded_nodes_.find(pro_id);

                    if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
                        // cout << "in closeset" << endl;
                        in_closed_set_num_++;
                        continue;
                    }

                    /* vel feasibe */
                    Eigen::Vector3d pro_v = pro_state.tail(3);
                    float vel_norm = pro_v.norm();
                    if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_
                        || fabs(pro_v(2)) > max_vel_ || vel_norm > max_vel_ + 0.05) {
                        // cout << "vel infeasible" << endl;
                        vel_infeasible_num_++;
                        continue;
                    }

                    /*check turbine obstacle distance*/
                    float min_dist;
                    if(afc_state == MotionPlanState::ROOTTURN
                       || afc_state == MotionPlanState::CIRCLE) {
                        if (!map_->isStateValid(pro_state.head(3).cast<float>(), min_dist)) {
                            // chlog::info("motion_plan", "the state is in collision!");
                            in_collision_num_++;
                            continue;
                        }
                    }
                    /* ---------- compute cost ---------- */
                    double time_to_goal, tmp_g_score, tmp_f_score, dist = 0;
                    if (afc_state == MotionPlanState::TRACKING || afc_state == MotionPlanState::ROOTTURN
                         || afc_state == MotionPlanState::POINTTOPOINT) {
                        calcDistToCenter(dist, pro_state.head(3), start_pos_, false);
                    } else if (afc_state == MotionPlanState::CIRCLE){
//                        double radius = (start_pt - circle_pos_).norm();
                        if ((circle_pos_ - pro_state.head(3)).norm() < 8) {
                            dist = fabs((circle_pos_ - pro_state.head(3)).norm() - 8);
                        }
                    } else {
                        dist = 0;
                    }

                    tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score + dist_lambda_ * dist;
                    tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

                    /* ---------- compare expanded node in this loop ---------- */

                    bool prune = false;
                    for (unsigned int j = 0; j < tmp_expand_nodes.size(); ++j) {
                        PathNodePtr expand_node = tmp_expand_nodes[j];
                        if ((pro_id - expand_node->index).norm() == 0 &&
                            ((!dynamic) || pro_t_id == expand_node->time_idx)) {

                            prune = true;

                            if (tmp_f_score < expand_node->f_score) {
                                expand_node->f_score  = tmp_f_score;
                                expand_node->g_score  = tmp_g_score;
                                expand_node->state    = pro_state;
                                expand_node->input    = um;
                                expand_node->duration = tau;
                                expand_node->dist_center = dist_to_center;
                            }

                            feasible_num_++;
                            break;
                        }
                    }

                    /* ---------- new neighbor in this loop ---------- */

                    if (!prune) {
                        if (pro_node == NULL) {
                            pro_node             = path_node_pool_[use_node_num_];
                            pro_node->index      = pro_id;
                            pro_node->state      = pro_state;
                            pro_node->f_score    = tmp_f_score;
                            pro_node->g_score    = tmp_g_score;
                            pro_node->input      = um;
                            pro_node->duration   = tau;
                            pro_node->parent     = cur_node;
                            pro_node->dist_center     = dist_to_center;
                            pro_node->node_state = IN_OPEN_SET;

                            open_set_.push(pro_node);

                            expanded_nodes_.insert(pro_id, pro_node);

                            tmp_expand_nodes.push_back(pro_node);

                            use_node_num_ += 1;
                            if (use_node_num_ == allocate_num_) {
                                chlog::info("motion_plan", "[Kino Astar] run out of memory. use node num: " , use_node_num_
                                        , ", iter num: " ,iter_num_, ", out corridor num = ", outof_corridor_mum_,
                                            ", in collosion num = ", in_collision_num_
                                        , ", in closeset num = ", in_closed_set_num_, ", vel_infeasible_num_ = ", vel_infeasible_num_
                                        ,", feasible_num_ = " , feasible_num_
                                        , ", total_tried_num_ = ", total_tried_num_);
                                return NO_PATH;
                            }
                        } else if (pro_node->node_state == IN_OPEN_SET) {
                            if (tmp_g_score < pro_node->g_score) {
                                // pro_node->index = pro_id;
                                pro_node->state    = pro_state;
                                pro_node->f_score  = tmp_f_score;
                                pro_node->g_score  = tmp_g_score;
                                pro_node->input    = um;
                                pro_node->duration = tau;
                                pro_node->parent   = cur_node;
                                pro_node->dist_center     = dist_to_center;
                            }
                            open_count_ ++;
                        } else {
                            chlog::info("motion_plan", "[Kino Astar] error type in searching: " + to_string2(pro_node->node_state));
                        }

                    }

                    /* ----------  ---------- */
                }
        }

        /* ---------- open set empty, no path ---------- */
        chlog::info("motion_plan", "[Kino Astar] open set empty, no path! use node num: " , use_node_num_
                                    , ", iter num: " ,iter_num_, ", out corridor num = ", outof_corridor_mum_,
                                ", in collosion num = ", in_collision_num_
                            , ", in closeset num = ", in_closed_set_num_, ", vel_infeasible_num_ = ", vel_infeasible_num_
                            ,", feasible_num_ = " , feasible_num_
                            , ", open_count_ = ", open_count_
                            , ", total_tried_num_ = ", total_tried_num_);
        return NO_PATH;
    }


    void KinodynamicAstar::retrievePath(PathNodePtr end_node) {
        PathNodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);

        while (cur_node->parent != NULL) {
            cur_node = cur_node->parent;
            path_nodes_.push_back(cur_node);
        }

        reverse(path_nodes_.begin(), path_nodes_.end());
    }
    double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                               double& optimal_time) {
        const Vector3d dp = x2.head(3) - x1.head(3);
        const Vector3d v0 = x1.segment(3, 3);
        const Vector3d v1 = x2.segment(3, 3);

        double c1 = -36 * dp.dot(dp);
        double c2 = 24 * (v0 + v1).dot(dp);
        double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
        double c4 = 0;
        double c5 = w_time_;

        std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

        double v_max = max_vel_;
        double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
        ts.push_back(t_bar);

        double cost = 100000000;
        double t_d  = t_bar;

        for (auto t : ts) {
            if (t < t_bar) continue;
            double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
            if (c < cost) {
                cost = c;
                t_d  = t;
            }
        }

        optimal_time = t_d;

        return 1.0 * (1 + tie_breaker_) * cost;
    }

    bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                           double time_to_goal) {
        /* ---------- get coefficient ---------- */
        const Vector3d p0  = state1.head(3);
        const Vector3d dp  = state2.head(3) - p0;
        const Vector3d v0  = state1.segment(3, 3);
        const Vector3d v1  = state2.segment(3, 3);
        const Vector3d dv  = v1 - v0;
        double         t_d = time_to_goal;
        MatrixXd       coef(3, 4);
        end_vel_ = v1;

        Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
        Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
        Vector3d c = v0;
        Vector3d d = p0;

        // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
        // a*t^3 + b*t^2 + v0*t + p0
        coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

        Vector3d coord, vel, acc;
        VectorXd poly1d, t, polyv, polya;
        Vector3i index;

        Eigen::MatrixXd Tm(4, 4);
        Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

        /* ---------- forward checking of trajectory ---------- */
        double t_delta = t_d / 10;
        for (double time = t_delta; time <= t_d; time += t_delta) {
            t = VectorXd::Zero(4);
            for (int j = 0; j < 4; j++) t(j) = pow(time, j);

            for (int dim = 0; dim < 3; dim++) {
                poly1d     = coef.row(dim);
                coord(dim) = poly1d.dot(t);
                vel(dim)   = (Tm * poly1d).dot(t);
                acc(dim)   = (Tm * Tm * poly1d).dot(t);

                if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_) {
                    // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
                    // return false;
                }
            }

            if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) ||
                coord(1) >= map_size_3d_(1) || coord(2) < origin_(2) || coord(2) >= map_size_3d_(2)) {
                return false;
            }

            if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
                return false;
            }
        }
        coef_shot_    = coef;
        t_shot_       = t_d;
        is_shot_succ_ = true;
        return true;
    }

    vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d) {
        vector<double> dts;

        double a2 = b / a;
        double a1 = c / a;
        double a0 = d / a;

        double Q = (3 * a1 - a2 * a2) / 9;
        double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
        double D = Q * Q * Q + R * R;
        if (D > 0) {
            double S = std::cbrt(R + sqrt(D));
            double T = std::cbrt(R - sqrt(D));
            dts.push_back(-a2 / 3 + (S + T));
            return dts;
        } else if (D == 0) {
            double S = std::cbrt(R);
            dts.push_back(-a2 / 3 + S + S);
            dts.push_back(-a2 / 3 - S);
            return dts;
        } else {
            double theta = acos(R / sqrt(-Q * Q * Q));
            dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
            return dts;
        }
    }

    vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e) {
        vector<double> dts;

        double a3 = b / a;
        double a2 = c / a;
        double a1 = d / a;
        double a0 = e / a;

        vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
        double         y1 = ys.front();
        double         r  = a3 * a3 / 4 - a2 + y1;
        if (r < 0) return dts;

        double R = sqrt(r);
        double D, E;
        if (R != 0) {
            D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
            E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        } else {
            D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
            E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
        }

        if (!std::isnan(D)) {
            dts.push_back(-a3 / 4 + R / 2 + D / 2);
            dts.push_back(-a3 / 4 + R / 2 - D / 2);
        }
        if (!std::isnan(E)) {
            dts.push_back(-a3 / 4 - R / 2 + E / 2);
            dts.push_back(-a3 / 4 - R / 2 - E / 2);
        }

        return dts;
    }

    void KinodynamicAstar::init() {
        /* ---------- map params ---------- */
        this->inv_resolution_ = 1.0 / resolution_;
        inv_time_resolution_  = 1.0 / time_resolution_;
        edt_environment_->getMapRegion(origin_, map_size_3d_);
/*        cout << "origin_: " << origin_.transpose() << endl;
        cout << "map size: " << map_size_3d_.transpose() << endl;*/

        /* ---------- pre-allocated node ---------- */
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) {
            path_node_pool_[i] = new PathNode;
        }

        phi_          = Eigen::MatrixXd::Identity(6, 6);
        use_node_num_ = 0;
        iter_num_     = 0;
        outof_corridor_mum_ = 0;
        in_collision_num_ = 0;
        total_tried_num_ = 0;
        in_closed_set_num_ = 0;
        vel_infeasible_num_ = 0;
        feasible_num_ = 0;
        open_count_ = 0;
    }

    void KinodynamicAstar::setEnvironment(const Sp<IMap> &env, const EDTEnvironment::Ptr &sdf) {
        this->map_ = env;
        this->edt_environment_ = sdf;
    }

    void KinodynamicAstar::reset() {
        expanded_nodes_.clear();
        path_nodes_.clear();

        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set_.swap(empty_queue);

        for (int i = 0; i < use_node_num_; i++) {
            PathNodePtr node = path_node_pool_[i];
            node->parent     = NULL;
            node->node_state = NOT_EXPAND;
        }

        use_node_num_ = 0;
        iter_num_     = 0;
        outof_corridor_mum_ = 0;
        in_collision_num_ = 0;
        total_tried_num_ = 0;
        in_closed_set_num_ = 0;
        vel_infeasible_num_ = 0;
        feasible_num_ = 0;
        open_count_ = 0;
        is_shot_succ_ = false;
    }

    std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t) {
        vector<Vector3d> state_list;

        /* ---------- get traj of searching ---------- */
        PathNodePtr          node = path_nodes_.back();
        Matrix<double, 6, 1> x0, xt;

        while (node->parent != NULL) {
            Vector3d ut       = node->input;
            double   duration = node->duration;
            x0                = node->parent->state;

            for (double t = duration; t >= -1e-5; t -= delta_t) {
                stateTransit(x0, xt, ut, t);
                state_list.push_back(xt.head(3));
            }
            node = node->parent;
        }
        reverse(state_list.begin(), state_list.end());

        /* ---------- get traj of one shot ---------- */
        if (is_shot_succ_) {
            Vector3d coord;
            VectorXd poly1d, time(4);

            for (double t = delta_t; t <= t_shot_; t += delta_t) {
                for (int j = 0; j < 4; j++) time(j) = pow(t, j);

                for (int dim = 0; dim < 3; dim++) {
                    poly1d     = coef_shot_.row(dim);
                    coord(dim) = poly1d.dot(time);
                }
                state_list.push_back(coord);
            }
        }

        return state_list;
    }

    void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                      vector<Eigen::Vector3d>& start_end_derivatives) {
        /* ---------- final trajectory time ---------- */
        double T_sum = 0.0;
        if (is_shot_succ_) T_sum += t_shot_;

        PathNodePtr node = path_nodes_.back();
        while (node->parent != NULL) {
            T_sum += node->duration;
            node = node->parent;
        }
        // cout << "final time:" << T_sum << endl;

        /* ---------- init for sampling ---------- */
        int K = floor(T_sum / ts);
        ts    = T_sum / double(K + 1);
        // cout << "K:" << K << ", ts:" << ts << endl;

        bool sample_shot_traj = is_shot_succ_;

        // Eigen::VectorXd sx(K + 2), sy(K + 2), sz(K + 2);
        // int sample_num = 0;
        node = path_nodes_.back();

        Eigen::Vector3d end_vel, end_acc;

        double t;
        if (sample_shot_traj) {
            t       = t_shot_;
            end_vel = end_vel_;

            for (int dim = 0; dim < 3; ++dim) {
                Vector4d coe = coef_shot_.row(dim);
                end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
            }

        } else {
            t       = node->duration;
            end_vel = node->state.tail(3);
            end_acc = node->input;
        }

        for (double ti = T_sum; ti > -1e-5; ti -= ts) {
            /* ---------- sample shot traj---------- */
            if (sample_shot_traj) {

                Vector3d coord;
                Vector4d poly1d, time;

                for (int j = 0; j < 4; j++) time(j) = pow(t, j);

                for (int dim = 0; dim < 3; dim++) {
                    poly1d     = coef_shot_.row(dim);
                    coord(dim) = poly1d.dot(time);
                }

                // sx(sample_num) = coord(0), sy(sample_num) = coord(1), sz(sample_num) = coord(2);
                // ++sample_num;
                point_set.push_back(coord);
                t -= ts;

                /* end of segment */
                if (t < -1e-5) {
                    sample_shot_traj = false;
                    if (node->parent != NULL) t += node->duration;
                }
            }
                /* ---------- sample search traj---------- */
            else {

                Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
                Eigen::Matrix<double, 6, 1> xt;
                Vector3d                    ut = node->input;

                stateTransit(x0, xt, ut, t);
                // sx(sample_num) = xt(0), sy(sample_num) = xt(1), sz(sample_num) = xt(2);
                // ++sample_num;

                point_set.push_back(xt.head(3));
                t -= ts;

                // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
                if (t < -1e-5 && node->parent->parent != NULL) {
                    node = node->parent;
                    t += node->duration;
                }
            }
        }

        /* ---------- return samples ---------- */
        // samples.col(K + 2) = start_vel_;
        // samples.col(K + 3) = end_vel_;
        // samples.col(K + 4) = node->input;

        reverse(point_set.begin(), point_set.end());

        start_end_derivatives.push_back(start_vel_);
        start_end_derivatives.push_back(end_vel);
        start_end_derivatives.push_back(node->input);
        start_end_derivatives.push_back(end_acc);
    }

    std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes() {
        vector<PathNodePtr> visited;
        visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
        return visited;
    }

    Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt) {
        Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

        // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
        // origin_(1)) * inv_resolution_),
        //     floor((pt(2) - origin_(2)) * inv_resolution_);

        return idx;
    }

    int KinodynamicAstar::timeToIndex(double time) {
        return  floor((time - time_origin_) * inv_time_resolution_);
    }

    void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                        Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                                        double tau) {
        for (int i = 0; i < 3; ++i) phi_(i, i + 3) = tau;

        Eigen::Matrix<double, 6, 1> integral;
        integral.head(3) = 0.5 * pow(tau, 2) * um;
        integral.tail(3) = tau * um;

        state1 = phi_ * state0 + integral;
    }

}  // namespace fast_planner
