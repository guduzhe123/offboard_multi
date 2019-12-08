//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_PATHCREATOR_HPP
#define OFFBOARD_PATHCREATOR_HPP

#include "Cinc.hpp"
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;

class PathCreator {
public:
    struct TPathInfo {
        int m_path_id;
        float target_heading_local;
        float line_angle;
        TVec3 m_insp_vector;
    };

    PathCreator();

    ~PathCreator() = default;

    void onInit();

    void uav_add_way_points(vector<geometry_msgs::PoseStamped> &uav_way_points);

    void usv_add_way_points(vector<geometry_msgs::PoseStamped> &usv_way_points);

    static PathCreator* geInstance();

private:
    static PathCreator* l_pInst;

};
#endif //OFFBOARD_PATHCREATOR_HPP
