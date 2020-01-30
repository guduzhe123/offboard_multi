//
// Created by zhouhua on 2020/1/26.
//

#ifndef OFFBOARD_IVEHICLECONTROL_HPP
#define OFFBOARD_IVEHICLECONTROL_HPP

#include "Cinc.hpp"

class IVehicleControl {
public:
        virtual ~IVehicleControl() {};

        virtual void onInit(vector<geometry_msgs::PoseStamped> way_points) = 0;

        virtual void DoProgress() = 0;

        virtual void chooseLeader() = 0;

        virtual void getData() = 0;

        virtual void setVehicleCtrlData() = 0;
};

class IVehicleControlFactory {
public:
    virtual ~IVehicleControlFactory() {};

    virtual IVehicleControl* VehicleControlCreator() = 0;
};
#endif //OFFBOARD_IVEHICLECONTROL_HPP
