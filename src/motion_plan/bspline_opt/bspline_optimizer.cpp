#include "motion_plan/bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

    const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
    const int BsplineOptimizer::DISTANCE    = (1 << 1);
    const int BsplineOptimizer::FEASIBILITY = (1 << 2);
    const int BsplineOptimizer::ENDPOINT    = (1 << 3);
    const int BsplineOptimizer::GUIDE       = (1 << 4);
    const int BsplineOptimizer::YAW_SMOOTHNESS = (1 << 5);
    const int BsplineOptimizer::WAYPOINTS   = (1 << 6);
    const int BsplineOptimizer::DIST_CENTER   = (1 << 7);

    const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
    const int BsplineOptimizer::NORMAL_PHASE =
            BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY | BsplineOptimizer::DIST_CENTER;
    const int BsplineOptimizer::SAFE =
            BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

    void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
        nh.param("optimization/lambda1", lambda1_, 10.0);
        nh.param("optimization/lambda2", lambda2_, 0.8);
        nh.param("optimization/lambda3", lambda3_, 0.00001);
        nh.param("optimization/lambda4", lambda4_, 0.01);
        nh.param("optimization/lambda5", lambda5_, -1.0);
        nh.param("optimization/lambda6", lambda6_, -1.0);
        nh.param("optimization/lambda7", lambda7_, 100.0);
        nh.param("optimization/lambda8", lambda8_, -1.0);
        nh.param("optimization/lambda9", lambda9_, 1.0);

        nh.param("optimization/dist0", dist0_, 5.0);
        nh.param("optimization/dist_min", dist_min_, 0.5);
        nh.param("optimization/dist_max", dist_max_, 2.0);
        nh.param("optimization/visib_min", visib_min_, -1.0);
        nh.param("optimization/dlmin", dlmin_, -1.0);
        nh.param("optimization/wnl", wnl_, -1.0);

        nh.param("optimization/max_iteration_num1", max_iteration_num_[0], 2);
        nh.param("optimization/max_iteration_num2", max_iteration_num_[1], 300);
        nh.param("optimization/max_iteration_num3", max_iteration_num_[2], 200);
        nh.param("optimization/max_iteration_num4", max_iteration_num_[3], 200);
        nh.param("optimization/max_iteration_time1", max_iteration_time_[0], 0.0001);
        nh.param("optimization/max_iteration_time2", max_iteration_time_[1], 0.005);
        nh.param("optimization/max_iteration_time3", max_iteration_time_[2], 0.003);
        nh.param("optimization/max_iteration_time4", max_iteration_time_[3], 0.003);

        nh.param("optimization/algorithm1", algorithm1_, 15);
        nh.param("optimization/algorithm2", algorithm2_, 11);
        nh.param("optimization/order", order_, 3);

    }

    void BsplineOptimizer::setEnvironment(const Sp<IMap>& env) {
        this->map_ = env;
    }

    void BsplineOptimizer::setSpeedLimit(const double max_vel, const double max_acc) {
        max_vel_ = max_vel;
        max_acc_ = max_acc;
    }

    void BsplineOptimizer::setTargetPoint(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &start_point) {
        target_pos_ = target_pos;
        start_pos_ = start_point;
        chlog::info("motion_plan", "[Bspline Optimize] target pos: (" + to_string2(target_pos_(0)) +
                                                                      ", " + to_string2(target_pos_(1)) +
                                                                      ", " + to_string2(target_pos_(2)) + ")" +
                                                                      ", start point: (" + to_string2(start_point(0)) +
                                                                      ", " + to_string2(start_point(1)) +
                                                                      ", " + to_string2(start_point(2)) + ")");
    }

    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
        control_points_ = points;
        dim_            = control_points_.cols();
    }

    void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

    void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
        max_num_id_  = max_num_id;
        max_time_id_ = max_time_id;
    }

    void BsplineOptimizer::setCostFunction(const int& cost_code) {
        cost_function_ = cost_code;

        // print optimized cost function
        string cost_str;
        if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
        if (cost_function_ & DISTANCE) cost_str += " dist  |";
        if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
        if (cost_function_ & ENDPOINT) cost_str += " endpt |";
        if (cost_function_ & GUIDE) cost_str += " guide |";
        if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
        if (cost_function_ & DIST_CENTER) cost_str += " dist_center |";

        chlog::info("motion_plan", "[Bspline Optimize] cost function: " + cost_str);
    }

    void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

    void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                        const vector<int>&             waypt_idx) {
        waypoints_ = waypts;
        waypt_idx_ = waypt_idx;
    }

    Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                          const int& cost_function, int max_num_id,
                                                          int max_time_id) {
        setControlPoints(points);
        setBsplineInterval(ts);
        setCostFunction(cost_function);
        setTerminateCond(max_num_id, max_time_id);

        optimize();
        return this->control_points_;
    }

    void BsplineOptimizer::optimize() {
        /* initialize solver */
        iter_num_        = 0;
        min_cost_        = std::numeric_limits<double>::max();
        const int pt_num = control_points_.rows();
        g_q_.resize(pt_num);
        g_smoothness_.resize(pt_num);
        g_distance_.resize(pt_num);
        g_feasibility_.resize(pt_num);
        g_endpoint_.resize(pt_num);
        g_waypoints_.resize(pt_num);
        g_guide_.resize(pt_num),
        g_dist_center_.resize(pt_num);

        if (cost_function_ & ENDPOINT) {
            variable_num_ = dim_ * (pt_num - order_);
            // end position used for hard constraint
            end_pt_ = (1 / 6.0) *
                      (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
                       control_points_.row(pt_num - 1));
        } else {
            variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
        }

        /* do optimization using NLopt slover */
        nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
        opt.set_min_objective(BsplineOptimizer::costFunction, this);
        opt.set_maxeval(max_iteration_num_[max_num_id_]);
        opt.set_maxtime(max_iteration_time_[max_time_id_]);
        opt.set_xtol_rel(1e-5);

        vector<double> q(variable_num_);
        for (int i = order_; i < pt_num; ++i) {
            if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
            for (int j = 0; j < dim_; j++) {
                q[dim_ * (i - order_) + j] = control_points_(i, j);
            }
        }

        if (dim_ != 1) {
            vector<double> lb(variable_num_), ub(variable_num_);
            const double   bound = 5.0;
            for (int i = 0; i < variable_num_; ++i) {
                lb[i] = q[i] - bound;
                ub[i] = q[i] + bound;
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        }

        try {
            // cout << fixed << setprecision(7);
            // vec_time_.clear();
            // vec_cost_.clear();
            // time_start_ = ros::Time::now();
            double        final_cost;
            nlopt::result result = opt.optimize(q, final_cost);
            chlog::info("motion_plan", "[Bspline Optimize] opt result: " + to_string(result));

            /* retrieve the optimization result */
            // cout << "Min cost:" << min_cost_ << endl;
        } catch (std::exception& e) {
            cout << e.what() << endl;
            chlog::info("motion_plan", "[Optimization]: nlopt exception: nlopt failure");
        }

        for (int i = order_; i < control_points_.rows(); ++i) {
            if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
            for (int j = 0; j < dim_; j++) {
                control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
            }
        }

        if (!(cost_function_ & GUIDE))
            chlog::info("motion_plan", "[Bspline Optimize]: iter num: " + to_string(iter_num_));
    }

    void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                              vector<Eigen::Vector3d>& gradient) {
        cost = 0.0;
        Eigen::Vector3d zero(0, 0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);
        Eigen::Vector3d jerk, temp_j;

        for (unsigned int i = 0; i < q.size() - order_; i++) {
            /* evaluate jerk */
            jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
            cost += jerk.squaredNorm();
            temp_j = 2.0 * jerk;
            /* jerk gradient */
            gradient[i + 0] += -temp_j;
            gradient[i + 1] += 3.0 * temp_j;
            gradient[i + 2] += -3.0 * temp_j;
            gradient[i + 3] += temp_j;
        }
    }

    void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                            vector<Eigen::Vector3d>& gradient) {
        cost = 0.0;
        Eigen::Vector3d zero(0, 0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        float          dist;
        Eigen::Vector3d dist_grad(1,1,1), g_zero(0, 0, 0);

        int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

        for (int i = order_; i < end_idx; i++) {
            if (dist_grad.norm() > 1e-4) dist_grad.normalize();

            if (!map_->isStateValid(q[i].cast<float>())) { // TODO check distance
                cost += pow(dist - dist0_, 2);
                gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
            }
        }
    }

    void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                               vector<Eigen::Vector3d>& gradient) {
        cost = 0.0;
        Eigen::Vector3d zero(0, 0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        /* abbreviation */
        double ts, vm2, am2, ts_inv2, ts_inv4;
        vm2 = max_vel_ * max_vel_;
        am2 = max_acc_ * max_acc_;

        ts      = bspline_interval_;
        ts_inv2 = 1 / ts / ts;
        ts_inv4 = ts_inv2 * ts_inv2;

        /* velocity feasibility */
        for (unsigned int i = 0; i < q.size() - 1; i++) {
            Eigen::Vector3d vi = q[i + 1] - q[i];

            for (int j = 0; j < 3; j++) {
                double vd = vi(j) * vi(j) * ts_inv2 - vm2;
                if (vd > 0.0) {
                    cost += pow(vd, 2);

                    double temp_v = 4.0 * vd * ts_inv2;
                    gradient[i + 0](j) += -temp_v * vi(j);
                    gradient[i + 1](j) += temp_v * vi(j);
                }
            }
        }

        /* acceleration feasibility */
        for (unsigned int i = 0; i < q.size() - 2; i++) {
            Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

            for (int j = 0; j < 3; j++) {
                double ad = ai(j) * ai(j) * ts_inv4 - am2;
                if (ad > 0.0) {
                    cost += pow(ad, 2);

                    double temp_a = 4.0 * ad * ts_inv4;
                    gradient[i + 0](j) += temp_a * ai(j);
                    gradient[i + 1](j) += -2 * temp_a * ai(j);
                    gradient[i + 2](j) += temp_a * ai(j);
                }
            }
        }
    }

    void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                            vector<Eigen::Vector3d>& gradient) {
        cost = 0.0;
        Eigen::Vector3d zero(0, 0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        // zero cost and gradient in hard constraints
        Eigen::Vector3d q_3, q_2, q_1, dq;
        q_3 = q[q.size() - 3];
        q_2 = q[q.size() - 2];
        q_1 = q[q.size() - 1];

        dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
        cost += dq.squaredNorm();

        gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
        gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
        gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
    }

    void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                             vector<Eigen::Vector3d>& gradient) {
        cost = 0.0;
        Eigen::Vector3d zero(0, 0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        Eigen::Vector3d q1, q2, q3, dq;

        // for (auto wp : waypoints_) {
        for (unsigned int i = 0; i < waypoints_.size(); ++i) {
            Eigen::Vector3d waypt = waypoints_[i];
            int             idx   = waypt_idx_[i];

            q1 = q[idx];
            q2 = q[idx + 1];
            q3 = q[idx + 2];

            dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
            cost += dq.squaredNorm();

            gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
            gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
            gradient[idx + 2] += dq * (2.0 / 6.0);
        }
    }

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
    void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
        cost = 0.0;
        Eigen::Vector3d zero(0, 0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        int end_idx = q.size() - order_;

        for (int i = order_; i < end_idx; i++) {
            Eigen::Vector3d gpt = guide_pts_[i - order_];
            cost += (q[i] - gpt).squaredNorm();
            gradient[i] += 2 * (q[i] - gpt);
        }
    }

    void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                       double& f_combine) {
        /* convert the NLopt format vector to control points. */

        // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
        // For 1D case, the second and third elements are zero, and similar for the 2D case.
        for (int i = 0; i < order_; i++) {
            for (int j = 0; j < dim_; ++j) {
                g_q_[i][j] = control_points_(i, j);
            }
            for (int j = dim_; j < 3; ++j) {
                g_q_[i][j] = 0.0;
            }
        }

        for (int i = 0; i < variable_num_ / dim_; i++) {
            for (int j = 0; j < dim_; ++j) {
                g_q_[i + order_][j] = x[dim_ * i + j];
            }
            for (int j = dim_; j < 3; ++j) {
                g_q_[i + order_][j] = 0.0;
            }
        }

        if (!(cost_function_ & ENDPOINT)) {
            for (int i = 0; i < order_; i++) {

                for (int j = 0; j < dim_; ++j) {
                    g_q_[order_ + variable_num_ / dim_ + i][j] =
                            control_points_(control_points_.rows() - order_ + i, j);
                }
                for (int j = dim_; j < 3; ++j) {
                    g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
                }
            }
        }

        f_combine = 0.0;
        grad.resize(variable_num_);
        fill(grad.begin(), grad.end(), 0.0);

        /*  evaluate costs and their gradient  */
        double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints, f_dist_center;
        f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = f_dist_center = 0.0;

        if (cost_function_ & SMOOTHNESS) {
            calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
            f_combine += lambda1_ * f_smoothness;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
        }
        if (cost_function_ & DISTANCE) {
            calcDistanceCost(g_q_, f_distance, g_distance_);
            f_combine += lambda2_ * f_distance;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
        }
        if (cost_function_ & FEASIBILITY) {
            calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
            f_combine += lambda3_ * f_feasibility;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
        }
        if (cost_function_ & ENDPOINT) {
            calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
            f_combine += lambda4_ * f_endpoint;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
        }
        if (cost_function_ & GUIDE) {
            calcGuideCost(g_q_, f_guide, g_guide_);
            f_combine += lambda5_ * f_guide;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
        }
        if (cost_function_ & DIST_CENTER) {
            calcDistCenterCost(g_q_, f_dist_center, g_dist_center_);
            f_combine += lambda9_ * f_dist_center;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_*i + j] += lambda9_ * g_dist_center_[i + order_](j);
        }

        if (cost_function_ & WAYPOINTS) {
            calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
            //lambda7_ = 0;
            f_combine += lambda7_ * f_waypoints;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
        }

        if (cost_function_ & YAW_SMOOTHNESS) {
            calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
            f_combine += 0;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += 0;
        }
        /*  print cost  */
        // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
        //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
        //        << ", waypt: " << lambda7_ * f_waypoints << endl;
        // }

        // if (optimization_phase_ == SECOND_PHASE) {
        //  << ", smooth: " << lambda1_ * f_smoothness
        //  << " , dist:" << lambda2_ * f_distance
        //  << ", fea: " << lambda3_ * f_feasibility << endl;
        // << ", end: " << lambda4_ * f_endpoint
        // << ", guide: " << lambda5_ * f_guide
        // }
    }

    double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                          void* func_data) {
        BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
        double            cost;
        opt->combineCost(x, grad, cost);
        opt->iter_num_++;

        /* save the min cost result */
        if (cost < opt->min_cost_) {
            opt->min_cost_      = cost;
            opt->best_variable_ = x;
        }
        //chlog::info("motion_plan", "[Bspline Optimize]: finnal cost: " + to_string(cost));
        return cost;

        // /* evaluation */
        // ros::Time te1 = ros::Time::now();
        // double time_now = (te1 - opt->time_start_).toSec();
        // opt->vec_time_.push_back(time_now);
        // if (opt->vec_cost_.size() == 0)
        // {
        //   opt->vec_cost_.push_back(f_combine);
        // }
        // else if (opt->vec_cost_.back() > f_combine)
        // {
        //   opt->vec_cost_.push_back(f_combine);
        // }
        // else
        // {
        //   opt->vec_cost_.push_back(opt->vec_cost_.back());
        // }
    }

    vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
        vector<Eigen::Vector3d> ctrl_q;
        for (int i = 0; i < ctrl_pts.rows(); ++i) {
            ctrl_q.push_back(ctrl_pts.row(i));
        }
        return ctrl_q;
    }

    Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

    bool BsplineOptimizer::isQuadratic() {
        if (cost_function_ == GUIDE_PHASE) {
            return true;
        } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
            return true;
        }
        return false;
    }

}  // namespace fast_planner