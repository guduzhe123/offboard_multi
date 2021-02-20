//
// Created by zhouhua on 2020/12/7.
//

#ifndef WINDAPPCORE_MPPUBLISHER_H
#define WINDAPPCORE_MPPUBLISHER_H

#endif //WINDAPPCORE_MPPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"
#include "offboard/Bspline.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int8.h>
#include "motion_plan/traj_utils/planning_visualization.h"
#include "Cinc.hpp"

#ifndef rad2deg
#define rad2deg                      57.295779513
#endif


const static float K_fitting_max = 10;

using namespace std;
using namespace fast_planner;

class MPPublisher{
public:
    MPPublisher();
    ~MPPublisher() = default;

    void OnInit(const float state);

    void DrawTrajCommand(const TVec3 &pos, const TVec3 &vec, const Eigen::Vector4d &color, int id);

    void DrawFlightCorridor(TVec3 &start_point, TVec3 &end_point);

    bool updateMPTarget(const float dist, const vector<TVec3> &waypoints);

    void updateDroneData(const TVec3 &drone_pos);

    void displayTrajWithColor(vector<TVec3> &path, double resolution, const Eigen::Vector4d &color,
                              int id);
    void drawGoal(TVec3 goal, double resolution, const Eigen::Vector4d& color, int id = 0);

    void drawGeometricPath(const vector<Eigen::Vector3d>& path);

    void drawBspline(NonUniformBspline& bspline);


private:
    void calcDistToCenter(float &dist, const TVec3 &cur_pos, TVec3 start_pos, TVec3 end_pos);

    ros::Publisher  motionPlanPub_, mp_drone_pos_update_pub_, mp_track_distance_pub_,
            cmd_vis_pub_, pos_cmd_pub, traj_pub_, drone_vel_sp_pub_, mp_state_pub_, marker_blade_pub_,
            marker_path_pub_, fitting_path_pub_, flight_corridor_pub_, path_update_pub_;

    Sp<PlanningVisualization> visualization_;

    float dist_config_;

    bool drone_update_;
    bool KF_init_;
    int mp_state_;
    int traj_pub_time_;

    TVec3 drone_pos_;
    vector<TVec3> wayPoints_;

};