//
// Created by zhouhua on 2021/2/9.
//

#include "PCL/PCLROSMessageManager.hpp"

PCLROSMessageManager::PCLROSMessageManager(){

}

void PCLROSMessageManager::OnInit(ros::NodeHandle &nh) {
    lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("os_cloud_node/points", 500,
                                                               &PCLROSMessageManager::cloudHandler, this);
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("pcl/Global_octomap", 3);

}

void PCLROSMessageManager::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::size_t i = 0; i < simple_cloud_ptr->size(); i++) {
        pcl::PointXYZ pnt = simple_cloud_ptr->points[i];
        tree_->updateNode(octomap::point3d(pnt.x, pnt.y, pnt.z), true);
    }
    tree_->updateInnerOccupancy();
    PubOctomap(tree_, octomap_pub_);
}

void PCLROSMessageManager::PubOctomap(octomap::OcTree *tree, const ros::Publisher &pub) {
    if (pub.getNumSubscribers() <= 0) return;
    octomap_.header.frame_id = "world";
    octomap_.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*tree, octomap_);

    pub.publish(octomap_);
}

void PCLROSMessageManager::getOctomap(octomap_msgs::Octomap &octomap) {
    octomap = octomap_;
}

