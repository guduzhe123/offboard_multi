//
// Created by zhouhua on 19-9-22.
//

#include "PathCreator.hpp"

PathCreator* PathCreator::l_pInst = NULL;

PathCreator::PathCreator() {

}

void PathCreator::onInit() {

}

void PathCreator::uav_add_way_points(vector<geometry_msgs::PoseStamped> &uav_way_points) {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    std::reverse(uav_way_points.begin(), uav_way_points.end());
}

void PathCreator::usv_add_way_points(vector<geometry_msgs::PoseStamped> &usv_way_points) {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -40;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = -40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
}

PathCreator* PathCreator::geInstance() {
    if (l_pInst == NULL) {
        l_pInst = new PathCreator();
    }
    return l_pInst;
}