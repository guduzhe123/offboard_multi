//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_PATHCREATOR_HPP
#define OFFBOARD_PATHCREATOR_HPP

#include "Cinc.hpp"
#include "FlightManager.hpp"
#include "IControlFunction.hpp"
#include "IMsgRosManager.hpp"
#include "Multi_formation.hpp"
#include "Avoidance.hpp"
#include "IVehicleControl.hpp"
#include "MultDroneControl.hpp"
#include "MultBoatControl.hpp"

using namespace Eigen;
using namespace std;

class PathCreator {
public:

    PathCreator();

    ~PathCreator() = default;

    void onInit(IMsgRosManager *msg_manager);

    void uav_add_way_points(vector<geometry_msgs::PoseStamped> &uav_way_points);

    void usv_add_way_points(vector<geometry_msgs::PoseStamped> &usv_way_points);

    void CreatFunction();

    void CreatVehicle();

    static PathCreator* geInstance();

private:
    static PathCreator* l_pInst;

};
#endif //OFFBOARD_PATHCREATOR_HPP
