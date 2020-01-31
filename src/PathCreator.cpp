//
// Created by zhouhua on 19-9-22.
//

#include "PathCreator.hpp"

PathCreator* PathCreator::l_pInst = NULL;

PathCreator::PathCreator() {

}

void PathCreator::onInit(IMsgRosManager *msg_manager) {
    FlightManager::getInstance()->OnInitConfig(msg_manager);
    CreatVehicle();
    CreatFunction();
}

void PathCreator::uav_add_way_points(vector<geometry_msgs::PoseStamped> &uav_way_points) {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 40;
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

    way_point.pose.position.x = 40;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -40;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -50;
    way_point.pose.position.y = -50;
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

void PathCreator::CreatFunction() {
    MultiFormationFactory FormationFactory;
    AvoidanceFactory avoidanceFactory;

    FlightManager::getInstance()->AddFunctionProgress(&FormationFactory);
    FlightManager::getInstance()->AddFunctionProgress(&avoidanceFactory);
}

void PathCreator::CreatVehicle() {
    vector<geometry_msgs::PoseStamped> uav_way_points;
    vector<geometry_msgs::PoseStamped> usv_way_points;
    uav_add_way_points(uav_way_points);
    usv_add_way_points(usv_way_points);

    MultiBoatControlFactory m_boat_control;
    MultiDroneControlFactory m_drone_control;

    m_drone_control.VehicleControlCreator()->onInit(uav_way_points);
    m_boat_control.VehicleControlCreator()->onInit(usv_way_points);

    FlightManager::getInstance()->AddVehicleControlProgress(&m_boat_control);
    FlightManager::getInstance()->AddVehicleControlProgress(&m_drone_control);
}

void PathCreator::CreateFormationInit(const int config) {
    util_log("3333333");
    MultiFormationFactory FormationFactory;
    IControlFunction* m_func = FormationFactory.FunctionCreator();
    m_func->Oninit(config);
}

PathCreator* PathCreator::geInstance() {
    if (l_pInst == NULL) {
        l_pInst = new PathCreator();
    }
    return l_pInst;
}