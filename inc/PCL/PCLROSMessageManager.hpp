//
// Created by zhouhua on 2021/2/9.
//

#ifndef OFFBOARD_PCLROSMESSAGEMANAGER_HPP
#define OFFBOARD_PCLROSMESSAGEMANAGER_HPP

#include "Cinc.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeKey.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

class PCLROSMessageManager {
public:
    PCLROSMessageManager();
    ~PCLROSMessageManager() = default;

    void OnInit(ros::NodeHandle &nh);
    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m);
    void getOctomap(octomap_msgs::Octomap &octomap);
    void setVehicleMessage(const M_Drone& usv);

private:
    ros::Subscriber lidar_point_sub_;
    ros::Publisher octomap_pub_, transformed_cloud_pub_, ground_removal_pub_;

    M_Drone usv_;

    octomap_msgs::Octomap octomap_;
    octomap::OcTree *tree_ = new octomap::OcTree(0.5);
    //发布八叉树
    void PubOctomap(octomap::OcTree *tree, const ros::Publisher &pub);
    //半径消除
    void radiusRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                       float radius, int min_neighbors,
                       pcl::IndicesConstPtr &cloud_filtered_indices);
    //地面消除
    void groundRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                      const pcl::IndicesConstPtr &cloud_filtered_indices);
    Eigen::Isometry3f get_transformation_matrix();
    void PubPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &o_cloud, const ros::Publisher &pub,
                       string frame = "map");
};
#endif //OFFBOARD_PCLROSMESSAGEMANAGER_HPP
