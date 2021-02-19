//
// Created by zhouhua on 2020/12/7.
//

#include "motion_plan/mp_core/MPPublisher.h"

namespace afcore {
    MPPublisher::MPPublisher() :
            dist_config_(0),
            drone_update_(false),
            KF_init_(false),
            traj_pub_time_(0){

    }
    void MPPublisher::OnInit(const float state) {
        ros::NodeHandle nh;
        motionPlanPub_ = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 50); // publish target way points.
        cmd_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/position_cmd_vis", 10); // mark
        traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/travel_traj", 10); // mark
        mp_drone_pos_update_pub_ = nh.advertise<nav_msgs::Odometry>("/visual_slam/odom", 100);
        mp_state_pub_ = nh.advertise<wa_ros_msgs::MotionPlanMsg>("/planning/state", 10);
        marker_blade_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/blade_fitting", 10);
        marker_path_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/path_fitting", 10);
        path_update_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/path_update", 10);
        fitting_path_pub_ = nh.advertise<wa_ros_msgs::BezierCurvePnts>("/ouster/path_fitting", 10);
        flight_corridor_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/tracking/Filter_region", 10);

        visualization_ = makeSp<PlanningVisualization>(nh);
        mp_state_ = state;

    }

    bool MPPublisher::updateMPTarget(const float dist, const vector<TVec3> &waypoints) {

        dist_config_ = dist;
        wayPoints_ = waypoints;
        return true;
    }

    void MPPublisher::updateDroneData(const TVec3 &drone_pos) {
        drone_pos_ = drone_pos;
        drone_update_ = true;
        float resolution;
        if (mp_state_ == MotionPlanState::TRACKING) {
            resolution = 6;
        } else if (mp_state_ == MotionPlanState::POINTTOPOINT) {
            resolution = 2;
        }  else {
            resolution = 10;
        }
        visualization_->drawDroneSpere(drone_pos.cast<double>(), resolution, Eigen::Vector4d(1, 0.1, 0.1, 0.5), 0, 6);
    }

    void MPPublisher::DrawTrajCommand(const TVec3 &pos, const TVec3 &vec, const Eigen::Vector4d &color, int id) {
        TVec3 pos_in_Local_EUS =
                FrameTransform::GetInstance()->TransFrame(pos, TurbineEUS, LocalEUS);
        TVec3 pos_in_Local_ENU = EUS2ENU(pos_in_Local_EUS);
        visualization_msgs::Marker mk_state;
        mk_state.header.frame_id = GLOBAL_FRAME_ID;
        mk_state.header.stamp = ros::Time::now();
        mk_state.id = id;
        mk_state.type = visualization_msgs::Marker::ARROW;
        mk_state.action = visualization_msgs::Marker::ADD;

        mk_state.pose.orientation.w = 1.0;
        mk_state.scale.x = 0.1;
        mk_state.scale.y = 0.2;
        mk_state.scale.z = 0.3;

        geometry_msgs::Point pt;
        pt.x = pos_in_Local_ENU(0);
        pt.y = pos_in_Local_ENU(1);
        pt.z = pos_in_Local_ENU(2);
        mk_state.points.push_back(pt);

        pt.x = pos_in_Local_ENU(0) + vec(0);
        pt.y = pos_in_Local_ENU(1) + vec(1);
        pt.z = pos_in_Local_ENU(2) + vec(2);
        mk_state.points.push_back(pt);

        mk_state.color.r = color(0);
        mk_state.color.g = color(1);
        mk_state.color.b = color(2);
        mk_state.color.a = color(3);

        cmd_vis_pub_.publish(mk_state);
    }

    void MPPublisher::DrawBladePath(const wa_ros_msgs::BezierCurvePnts &bezier_curve_points){
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = GLOBAL_FRAME_ID;
        line_strip.header.stamp =ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        geometry_msgs::Point p;
        for (unsigned int i = 0; i < bezier_curve_points.bezier_curve_points.size(); i++) {
            p.x = bezier_curve_points.bezier_curve_points[i].x;
            p.y = bezier_curve_points.bezier_curve_points[i].y;
            p.z = bezier_curve_points.bezier_curve_points[i].z;
            line_strip.points.push_back(p);
        }
        marker_blade_pub_.publish(line_strip);
    }

    void MPPublisher::DrawDronePath(const wa_ros_msgs::BezierCurvePnts &path) {
        chlog::info("motion_plan", "[MP Publisher]:~~~~ dist config = ", dist_config_);
        if (dist_config_ < 0.08) return;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = GLOBAL_FRAME_ID;
        line_strip.header.stamp =ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.color.g = 1.0;
        wa_ros_msgs::BezierCurvePnts line;
        geometry_msgs::Point p;
        for (auto &bezier_curve_point : path.bezier_curve_points) {
            TVec3 pnt, pnt_local;
            pnt << bezier_curve_point.x, bezier_curve_point.y, bezier_curve_point.z;
            changeToTurbineFrame(pnt, pnt_local);

            p.x = pnt_local.x();
            p.y = pnt_local.y();
            p.z = pnt_local.z();
            line_strip.points.push_back(p);
        }
        line = path;
        if (checkPathAvailable(line) ) {
            TVec3 dir;
            getPathVec(line, dir);
            if (!KF_init_) {
                onInitKF();
                x0_ << dir.x(), dir.y(), dir.z();
                kf_.InitSystemState(x0_);
                KF_init_ = true;
            } else {
                z_ << dir.x(), dir.y(), dir.z();
                kf_.UpdateState(z_, u_);
            }
            dir = kf_.GetCurrentEstimatedState();

            wa_ros_msgs::BezierCurvePnts path_update;
            updatePath(drone_pos_, dir, path_update);

            fitting_path_pub_.publish(path);
            marker_path_pub_.publish(line_strip);

            TVec3 start_point, end_point;
            start_point.x() = path.bezier_curve_points.front().x;
            start_point.y() = path.bezier_curve_points.front().y;
            start_point.z() = path.bezier_curve_points.front().z;

            end_point.x() = path.bezier_curve_points.back().x;
            end_point.y() = path.bezier_curve_points.back().y;
            end_point.z() = path.bezier_curve_points.back().z;
            DrawFlightCorridor(start_point, end_point);
        }
    }

    void MPPublisher::DrawFlightCorridor(TVec3 &start_point, TVec3 &end_point) {
        if (flight_corridor_pub_.getNumSubscribers() <= 0) return;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr hub_cylinder_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointXYZRGB pnc;
        pnc.r = 0;
        pnc.g = 150;
        pnc.b = 150;

        TVec3 direction = end_point - start_point;
        // TODO add direction and rotation matrix.
        for (float dist = -5.0; dist <= 5.0; dist += 0.5) {
            TVec3 cent_pnt = drone_pos_ + dist * direction.normalized();
            for (float theta = 0; theta < 2 * M_PI; theta += M_PI / 30) {
                pnc.x = 2 * cos(theta) + cent_pnt.x();
                pnc.y = 2 * sin(theta) + cent_pnt.y();
                pnc.z = cent_pnt.z();
                hub_cylinder_ptr->push_back(pnc);
            }
        }

        sensor_msgs::PointCloud2 ros_pcl;
        pcl::toROSMsg(*hub_cylinder_ptr, ros_pcl);
        ros_pcl.header.frame_id = GLOBAL_FRAME_ID;
        ros_pcl.header.stamp = ros::Time::now();
        flight_corridor_pub_.publish(ros_pcl);
    }

    void MPPublisher::displayTrajWithColor(vector<TVec3> &path, double resolution, const Eigen::Vector4d &color,
                                           int id) {
        visualization_msgs::Marker mk;
        mk.header.frame_id = GLOBAL_FRAME_ID;
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::LINE_STRIP;
        mk.action = visualization_msgs::Marker::DELETE;
        mk.id = id;

        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.r = color(0);
        mk.color.g = color(1);
        mk.color.b = color(2);
        mk.color.a = color(3);

        mk.scale.x = resolution;
        mk.scale.y = resolution;
        mk.scale.z = resolution;

        geometry_msgs::Point pt;
        for (auto & i : path) {
            TVec3 pnt, pnt_local;
            pnt << i(0), i(1), i(2);
            changeToTurbineFrame(pnt, pnt_local);

            pt.x = pnt_local.x();
            pt.y = pnt_local.y();
            pt.z = pnt_local.z();
            mk.points.push_back(pt);
        }

        while (mk.points.size() > 2000) {
            mk.points.erase(mk.points.begin());
        }
        
        traj_pub_time_++;
        if (traj_pub_time_ > 3) {
            traj_pub_.publish(mk);
            traj_pub_time_ = 0;
        }
    }

    void MPPublisher::calcDistToCenter(float &dist, const TVec3 &cur_pos, TVec3 start_pos, TVec3 end_pos) {
        TLine start_cur(start_pos, cur_pos.cast<float>());
        TLine start_end(start_pos, end_pos);
        float angle = acos(start_cur.normal().dot(start_end.normal()));
        dist = start_cur.length() * sin(angle);
    }

    bool MPPublisher::checkPathAvailable(wa_ros_msgs::BezierCurvePnts &path) {
        if (path.bezier_curve_points.size() < 10) {
            chlog::info("motion_plan", "[MP Publisher]: fitting path number is little =" + to_string(path.bezier_curve_points.size()));
            return false;
        }
        wa_ros_msgs::FittedPoints pnt1, pnt2;
        pnt1 = path.bezier_curve_points.front();
        pnt2 = path.bezier_curve_points.back();

        if (!drone_update_) return false;
        TVec3 start_pos, target_pos;

        start_pos.x() = pnt1.x;
        start_pos.y() = pnt1.y;
        start_pos.z() = pnt1.z;

        target_pos.x() = pnt2.x;
        target_pos.y() = pnt2.y;
        target_pos.z() = pnt2.z;
        TLine cur_tar(start_pos, target_pos);
/*        TVec3 dir = {0,0,-1};
        float angle = acos(dir.dot(cur_tar.normal()));*/

        if (wayPoints_.size() > 0) {
            TVec3 origin_target;
            origin_target = wayPoints_.back();
            chlog::info("motion_plan","[MP_publisher]: update waypints = ", toStr(origin_target));
            TLine cur_tar_origin(drone_pos_, origin_target);
            float angle = acos(cur_tar_origin.normal().dot(cur_tar.normal()));

            chlog::info("motion_plan", "[MP Publisher]: path calc roll = " + to_string(angle * rad2deg) + ", cur_pos = " + toStr(drone_pos_) +
                                   ", target_pos = " + toStr(target_pos) + ", start_pos = " + toStr(start_pos));

            float dist;
            calcDistToCenter(dist, drone_pos_, start_pos, target_pos);
            if (angle * rad2deg > 90) {
                angle = M_PI - angle;
            }
            chlog::info("motion_plan", "[MP Publisher]: angle = " + to_string2(angle * rad2deg) + ", dist to line = " + to_string2(dist));
            if ((angle * rad2deg < 50) && (dist < 10)) {
                return true;
            }

        }

        return false;

    }

    void MPPublisher::getPathVec(wa_ros_msgs::BezierCurvePnts &path, TVec3 &dir) {
        wa_ros_msgs::FittedPoints pnt1, pnt2;
        pnt1 = path.bezier_curve_points.front();
        pnt2 = path.bezier_curve_points.back();

        TVec3 Tpnt1, Tpnt2;
        Tpnt1 = TVec3{pnt1.x, pnt1.y, pnt1.z};
        Tpnt2 = TVec3{pnt2.x, pnt2.y, pnt2.z};
        dir = (Tpnt2 - Tpnt1).normalized();
    }

    void MPPublisher::updatePath(TVec3 &drone_cur, TVec3 &dir, wa_ros_msgs::BezierCurvePnts &path) {

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = GLOBAL_FRAME_ID;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1;
        // Line strip is blue
        line_strip.color.b = 0.5;
        line_strip.color.a = 1.0;
        line_strip.color.g = 0.5;
        line_strip.color.r = 1.0;

        wa_ros_msgs::FittedPoints pnt;
        geometry_msgs::Point p;
        float dist = 0.2;
        for (int i = -20; i < 20; i++) {
            TVec3 end_point, end_pnt_local;
            end_point = drone_cur + dir * dist * i;
            changeToTurbineFrame(end_point, end_pnt_local);

            pnt.x = end_pnt_local.x();
            pnt.y = end_pnt_local.y();
            pnt.z = end_pnt_local.z();

            p.x = end_pnt_local.x();
            p.y = end_pnt_local.y();
            p.z = end_pnt_local.z();

            path.bezier_curve_points.push_back(pnt);
            line_strip.points.push_back(p);
        }
        path_update_pub_.publish(line_strip);
    }

    void MPPublisher::onInitKF() {
        // Dimensions:
        uint i_ = 3;  // Input
        uint m_ = 3;  // Output
        uint s_ = 3;  // State

        //both initial and setup
        A_ = MatrixXf(s_, s_).setIdentity();
        B_ = MatrixXf(s_, i_).setZero();
        H_ = MatrixXf(m_, s_).setIdentity();

        Q_ = MatrixXf(s_, s_).setIdentity();
        R_ = MatrixXf(m_, m_).setIdentity();

        x0_ = VectorXf(s_);//offset
        // No inputs, system subjects only to random perturbation
        u_ = VectorXf(i_).setZero();
        z_ = VectorXf(m_).setZero();

        // process noise w~N(0,Q) 0.1
        float p_noise = 1.0f;
        // observation noise v~N(0,R) 1
        float o_noise = 1.0f;
        Q_ << p_noise, 0.0, 0.0,
                0.0, p_noise, 0.0,
                0.0, 0.0, p_noise;
        R_ << o_noise, 0.0, 0.0,
                0.0, o_noise, 0.0,
                0.0, 0.0, o_noise;

        kf_.InitSystem(A_, B_, H_, Q_, R_);
    }

    void MPPublisher::drawGoal(TVec3 goal, double resolution, const Eigen::Vector4d& color, int id ) {
        Eigen::Vector3d target;
        TVec3 target_ENU;
        changeToTurbineFrame(goal, target_ENU);
        target << target_ENU.x(), target_ENU.y(), target_ENU.z();
        visualization_->drawGoal(target, resolution, color, id);
    }

    void MPPublisher::drawGeometricPath(const vector<Eigen::Vector3d>& path) {
        vector<Eigen::Vector3d> path_in_local;
        for (auto point: path) {
            TVec3 pnt, pnt_local_ENU;
            pnt << point.x(), point.y(), point.z();
            changeToTurbineFrame(pnt, pnt_local_ENU);
            path_in_local.push_back(pnt_local_ENU.cast<double >());
        }
        visualization_->drawGeometricPath(path_in_local, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    }

    void MPPublisher::drawBspline(NonUniformBspline& bspline) {
        visualization_->drawBspline(bspline, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), false, 0.2,
                                    Eigen::Vector4d(1, 0, 0, 1));
    }

    void MPPublisher::changeToTurbineFrame(TVec3 &pnt, TVec3 &pos_in_Local_ENU) {
        TVec3 pos_in_Local_EUS =
                FrameTransform::GetInstance()->TransFrame(pnt, TurbineEUS, LocalEUS);
        pos_in_Local_ENU = EUS2ENU(pos_in_Local_EUS);
    }

}