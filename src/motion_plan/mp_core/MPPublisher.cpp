//
// Created by zhouhua on 2020/12/7.
//

#include "motion_plan/mp_core/MPPublisher.h"

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
    marker_blade_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/blade_fitting", 10);
    marker_path_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/path_fitting", 10);
    path_update_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/path_update", 10);
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
    TVec3 pos_in_Local_ENU = pos;
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "map";
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
    ros_pcl.header.frame_id = "map";
    ros_pcl.header.stamp = ros::Time::now();
    flight_corridor_pub_.publish(ros_pcl);
}

void MPPublisher::displayTrajWithColor(vector<TVec3> &path, double resolution, const Eigen::Vector4d &color,
                                       int id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
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


void MPPublisher::drawGoal(TVec3 goal, double resolution, const Eigen::Vector4d& color, int id ) {
    visualization_->drawGoal(goal.cast<double>(), resolution, color, id);
}

void MPPublisher::drawGeometricPath(const vector<Eigen::Vector3d>& path) {
    vector<Eigen::Vector3d> path_in_local;
    for (auto point: path) {
        TVec3 pnt, pnt_local_ENU;
        pnt << point.x(), point.y(), point.z();
        path_in_local.push_back(pnt.cast<double >());
    }
    visualization_->drawGeometricPath(path_in_local, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
}

void MPPublisher::drawBspline(NonUniformBspline& bspline) {
    visualization_->drawBspline(bspline, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), false, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));
}
