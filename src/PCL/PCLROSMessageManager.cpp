//
// Created by zhouhua on 2021/2/9.
//

#include "PCL/PCLROSMessageManager.hpp"

PCLROSMessageManager::PCLROSMessageManager(){

}

void PCLROSMessageManager::OnInit(ros::NodeHandle &nh) {
    lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("os_cloud_node/points", 500,
                                                               &PCLROSMessageManager::cloudHandler, this);
    transformed_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar/Transformed_points", 3);
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("pcl/Global_octomap", 3);

}

void PCLROSMessageManager::setVehicleMessage(const M_Drone& usv) {
    usv_ = usv;
}


void PCLROSMessageManager::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m) {
    /*ros point cloud to pcl point cloud*/
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*m, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *raw_cloud_ptr);

    /*pcl points filter*/
    radiusRemoval(raw_cloud_ptr, simple_cloud_ptr, 0.5, 4);

    /*transform point cloud*/
    pcl::transformPointCloud(*simple_cloud_ptr, *transformed_cloud, get_transformation_matrix().matrix());

    PubPointCloud(transformed_cloud, transformed_cloud_pub_);

    for (std::size_t i = 0; i < transformed_cloud->size(); i++) {
        pcl::PointXYZ pnt = transformed_cloud->points[i];
        if (pnt.z < 0.8) continue;
        util_log("usv1 pcl points (%.2f, %.2f, %.2f)", pnt.x, pnt.y, pnt.z);
        tree_->updateNode(octomap::point3d(pnt.x, pnt.y, pnt.z), true);
    }
    tree_->updateInnerOccupancy();
    PubOctomap(tree_, octomap_pub_);
}

void PCLROSMessageManager::radiusRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                        float radius, int min_neighbors) {
    if (input_cloud->empty() || radius <= 0 || min_neighbors <= 0) return;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(input_cloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_neighbors);
    //outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*output_cloud);
}

void
PCLROSMessageManager::PubPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &o_cloud, const ros::Publisher &pub,
                                      string frame) {
    if (pub.getNumSubscribers() <= 0) return;
    if (o_cloud->empty()) return;
    sensor_msgs::PointCloud2 ros_pcl;
    pcl::toROSMsg(*o_cloud, ros_pcl);
    ros_pcl.header.frame_id = std::move(frame);
    ros_pcl.header.stamp = ros::Time::now();
    pub.publish(ros_pcl);
}

void PCLROSMessageManager::PubOctomap(octomap::OcTree *tree, const ros::Publisher &pub) {
    if (pub.getNumSubscribers() <= 0) return;
    octomap_.header.frame_id = "map";
    octomap_.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*tree, octomap_);

    pub.publish(octomap_);
}

void PCLROSMessageManager::getOctomap(octomap_msgs::Octomap &octomap) {
    octomap = octomap_;
}

Eigen::Isometry3f PCLROSMessageManager::get_transformation_matrix() {
    Eigen::Isometry3f transformation_matrix;
    transformation_matrix = Eigen::Isometry3f::Identity();
    Eigen::AngleAxisf gimbal_yaw(usv_.yaw * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf gimbal_pitch(usv_.pitch * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf gimbal_roll(usv_.roll * M_PI / 180.0f, Eigen::Vector3f::UnitX());
    Eigen::Matrix3f vehicle_world;
    vehicle_world = gimbal_yaw * gimbal_pitch * gimbal_roll;

    transformation_matrix.rotate(vehicle_world);
    Eigen::Vector3f vehicle_pos = TVec3(usv_.current_local_pos.pose.position.x,
                                        usv_.current_local_pos.pose.position.y,
                                        usv_.current_local_pos.pose.position.z + 2); // 2 is the velodyne lidar at the usv z position
    transformation_matrix.pretranslate(vehicle_pos);
    return transformation_matrix;
}

