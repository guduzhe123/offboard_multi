//
// Created by zhouhua on 19-9-22.
//

#ifndef OFFBOARD_PATHCREATOR_HPP
#define OFFBOARD_PATHCREATOR_HPP

#include <math.h>
#include <Eigen/Core>

using namespace Eigen;

class PathCreator {
public:
    struct TPathInfo {
        int m_path_id;    // 0~3 for tracking blade, -1 for blade root, -2 for blade tip //
        float target_heading_local;
        float line_angle;
        Vector3f m_insp_vector;
    };

    void Init();

    void CreatPath();

private:

};
#endif //OFFBOARD_PATHCREATOR_HPP
