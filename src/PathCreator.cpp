//
// Created by zhouhua on 19-9-22.
//

#include "PathCreator.hpp"

PathCreator* PathCreator::l_pInst = NULL;

PathCreator::PathCreator() {

}

void PathCreator::onInit(IMsgRosManager *msg_manager, const bool is_uav_follow) {
    FlightManager::getInstance()->OnInitConfig(msg_manager);
    is_uav_follow_ = is_uav_follow;
    CreatVehicle();
    CreatFunction();
}

void PathCreator::initMotionPlan() {
    ActionMotionPlanFactory action_mp;
    FlightManager::getInstance()->AddFunctionProgress(&action_mp);

}

void PathCreator::uav_add_way_points(vector<geometry_msgs::PoseStamped> &uav_way_points) {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = K_uav_height;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 10;
    way_point.pose.position.z = K_uav_height;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 10;
    way_point.pose.position.y = 10;
    way_point.pose.position.z = K_uav_height;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 10;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = K_uav_height;
    uav_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = K_uav_height;
    uav_way_points.push_back(way_point);

    std::reverse(uav_way_points.begin(), uav_way_points.end());
}

void PathCreator::usv_add_way_points(vector<geometry_msgs::PoseStamped> &usv_way_points) {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 5;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -20;
    way_point.pose.position.y = 10;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);


    way_point.pose.position.x = -20;
    way_point.pose.position.y = 30;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = -40;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);


    way_point.pose.position.x = -20;
    way_point.pose.position.y = 50;
    way_point.pose.position.z = 0;

    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 20;
    way_point.pose.position.y = 50;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 0;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
}

void PathCreator::CreatFunction() {
    MultiUAVFormationFactory UAVFormationFactory;
    MultiUSVFormationFactory USVFormationFactory;
    AvoidanceFactory avoidanceFactory;
    USVAvoidanceFactory usv_avo_factory;

    FlightManager::getInstance()->AddFunctionProgress(&avoidanceFactory);
    FlightManager::getInstance()->AddFunctionProgress(&usv_avo_factory);
    FlightManager::getInstance()->AddFunctionProgress(&UAVFormationFactory);
    FlightManager::getInstance()->AddFunctionProgress(&USVFormationFactory);

    FlightManager::getInstance()->FunctionStateUpdate(&UAVFormationFactory);// update avoidance function state.
}

void PathCreator::CreatVehicle() {
    vector<geometry_msgs::PoseStamped> uav_way_points;
    vector<geometry_msgs::PoseStamped> usv_way_points;
    uav_add_way_points(uav_way_points);
    usv_add_way_points(usv_way_points);

    MultiBoatControlFactory m_boat_control;
    MultiDroneControlFactory m_drone_control;
    MultiUUVControlFactory m_uuv_control;

    m_drone_control.VehicleControlCreator()->onInit(uav_way_points, is_uav_follow_);
    m_boat_control.VehicleControlCreator()->onInit(usv_way_points, is_uav_follow_);
    m_uuv_control.VehicleControlCreator()->onInit(usv_way_points, false);

    FlightManager::getInstance()->AddVehicleControlProgress(&m_boat_control);
    FlightManager::getInstance()->AddVehicleControlProgress(&m_drone_control);
    FlightManager::getInstance()->AddVehicleControlProgress(&m_uuv_control);
}

void PathCreator::CreateUAVFormationInit(const int config) {
    MultiUAVFormationFactory FormationFactory;
    IControlFunction* m_func = FormationFactory.FunctionCreator();
    m_func->Oninit(config);

}

void PathCreator::CreateUSVFormationInit(const int config) {
    MultiUSVFormationFactory USVFormationFactory;
    IControlFunction* usv_func = USVFormationFactory.FunctionCreator();
    usv_func->Oninit(config);
    chlog::info("data","usv formation begin");
}


PathCreator* PathCreator::geInstance() {
    if (l_pInst == NULL) {
        l_pInst = new PathCreator();
    }
    return l_pInst;
}