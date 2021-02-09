//
// Created by zhouhua on 2021/2/9.
//

#ifndef OFFBOARD_PCLROSMESSAGEMANAGER_HPP
#define OFFBOARD_PCLROSMESSAGEMANAGER_HPP

#include "Cinc.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeKey.h>

class PCLROSMessageManager {
public:
    PCLROSMessageManager();
    ~PCLROSMessageManager() = default;

    void OnInit(ros::NodeHandle &nh);
    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m);
    void getOctomap(octomap_msgs::Octomap &octomap);

private:
    ros::Subscriber lidar_point_sub_;
    ros::Publisher octomap_pub_;

    octomap_msgs::Octomap octomap_;
    octomap::OcTree *tree_ = new octomap::OcTree(0.5);

    void PubOctomap(octomap::OcTree *tree, const ros::Publisher &pub);
};
#endif //OFFBOARD_PCLROSMESSAGEMANAGER_HPP
