//
// Created by zhouhua on 2021/2/9.
//

#include "PCL/PCLROSMessageManager.hpp"

PCLROSMessageManager::PCLROSMessageManager(){

}

void PCLROSMessageManager::OnInit(ros::NodeHandle &nh) {
/*    lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 1,
                                                               &PCLROSMessageManager::cloudHandler, this);*/
    lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1,
                                                               &PCLROSMessageManager::cloudHandler, this);

    local_position_sub_ = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &PCLROSMessageManager::local_pos_cb, this);


    transformed_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar/Transformed_points", 1);
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("pcl/Global_octomap", 1);

    ground_removal_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ground_removal_lidar", 1);
    nh.param("is_sim", is_sim_, false);

}

void PCLROSMessageManager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv_.current_local_pos = *msg;
    double yaw, roll, pitch;
//    EulerAngles angles;
//    yaw = Calculate::getInstance()->quaternion_get_yaw(usv_.current_local_pos.pose.orientation, angles);
    Calculate::getInstance()->quaternion_to_rpy(usv_.current_local_pos.pose.orientation, roll, pitch, yaw);

    usv_.roll = roll * 180 / M_PI;
    usv_.pitch = pitch * 180 / M_PI;
    usv_.yaw = yaw * 180 / M_PI;
}

void PCLROSMessageManager::setVehicleMessage(const M_Drone& usv) {
    usv_ = usv;
}

void PCLROSMessageManager::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m) {
    /*ros point cloud to pcl point cloud*/
    ROS_INFO_STREAM("pcl: [thread=" << boost::this_thread::get_id() << "]");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*m, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voselGride_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_remove (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(pcl_pc2, *raw_cloud_ptr);

    pcl::fromROSMsg(*m, *raw_cloud_ptr);

    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*raw_cloud_ptr, *raw_cloud_ptr, indices);

    /*pcl points filter*/
    pcl::IndicesConstPtr cloud_filtered_indices;
//    voselGrid(raw_cloud_ptr, raw_cloud_ptr);
    radiusRemoval(raw_cloud_ptr, simple_cloud_ptr, 0.5, 4, cloud_filtered_indices);
    groundRemove(raw_cloud_ptr, cloud_ground_remove, cloud_filtered_indices);

    sensor_msgs::PointCloud2 lidar_ground_removal;
    lidar_ground_removal.header = m->header;
    pcl::toROSMsg(*cloud_ground_remove, lidar_ground_removal);
    ground_removal_pub_.publish(lidar_ground_removal);

    /*transform point cloud*/
    pcl::transformPointCloud(*cloud_ground_remove, *transformed_cloud, get_transformation_matrix().matrix());

    PubPointCloud(transformed_cloud, transformed_cloud_pub_);

    tree_->clear();


    for (std::size_t i = 0; i < transformed_cloud->size(); i++) {
        pcl::PointXYZ pnt = transformed_cloud->points[i];
        if (pnt.z < -2) continue;
        TVec3 point = TVec3{pnt.x, pnt.y, pnt.z};
        if (point.norm() < 2.0f) continue;
        tree_->updateNode(octomap::point3d(pnt.x, pnt.y, pnt.z), true);
    }
    tree_->updateInnerOccupancy();
    PubOctomap(tree_, octomap_pub_);
}


void PCLROSMessageManager::voselGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    if (input_cloud->empty()) return;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter (*output_cloud);
    chlog::info("data", "voxel input cloud size = ", input_cloud->size(),
            ", output size = ", output_cloud->size());
}

void PCLROSMessageManager::radiusRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                        float radius, int min_neighbors,
                                        pcl::IndicesConstPtr &cloud_filtered_indices) {
    if (input_cloud->empty() || radius <= 0 || min_neighbors <= 0) return;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(input_cloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_neighbors);
    //outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*output_cloud);
    cloud_filtered_indices = outrem.getIndices();
}

void PCLROSMessageManager::groundRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                        const pcl::IndicesConstPtr &cloud_filtered_indices) {
    //plane extracting
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.5);
    seg.setInputCloud (input_cloud);
    seg.setIndices(cloud_filtered_indices);
//    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers, *coefficients_plane);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*output_cloud);
    chlog::info("data","input_cloud size = ", input_cloud->points.size(),
            ", output cloud size = ", output_cloud->points.size(),
            ", cloud_filtered_indices size = ", cloud_filtered_indices->size());
}

void
PCLROSMessageManager::PubPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &o_cloud, const ros::Publisher &pub,
                                      string frame) {
//    if (pub.getNumSubscribers() <= 0) return;
    if (o_cloud->empty()) return;
    sensor_msgs::PointCloud2 ros_pcl;
    pcl::toROSMsg(*o_cloud, ros_pcl);
    ros_pcl.header.frame_id = std::move(frame);
    ros_pcl.header.stamp = ros::Time::now();
    pub.publish(ros_pcl);
}

void PCLROSMessageManager::PubOctomap(octomap::OcTree *tree, const ros::Publisher &pub) {
//    if (pub.getNumSubscribers() <= 0) return;
    octomap_.header.frame_id = "map";
    octomap_.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*tree, octomap_);

    pub.publish(octomap_);
}

void PCLROSMessageManager::getOctomap(octomap_msgs::Octomap &octomap) {
    chlog::info("data", "update octomap!");
    octomap = octomap_;
}

Eigen::Isometry3f PCLROSMessageManager::get_transformation_matrix() {
    Eigen::Isometry3f transformation_matrix;
    transformation_matrix = Eigen::Isometry3f::Identity();
//    Eigen::AngleAxisf gimbal_yaw(((usv_.yaw + 180.0f) * M_PI / 180.0f ), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf gimbal_yaw(((usv_.yaw ) * M_PI / 180.0f ), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf gimbal_pitch(0 * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf gimbal_roll(0 * M_PI / 180.0f, Eigen::Vector3f::UnitX());
    Eigen::Matrix3f vehicle_world;
    vehicle_world = gimbal_yaw * gimbal_pitch * gimbal_roll;

    transformation_matrix.rotate(vehicle_world);
    Eigen::Vector3f vehicle_pos = TVec3(usv_.current_local_pos.pose.position.x,
                                        usv_.current_local_pos.pose.position.y,
                                        usv_.current_local_pos.pose.position.z + 2); // 2 is the velodyne lidar at the usv z position

    if (!is_sim_) {
        vehicle_pos.z() = usv_.current_local_pos.pose.position.z;
    }
    transformation_matrix.pretranslate(vehicle_pos);
    return transformation_matrix;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_points");
    PCLROSMessageManager pclM;
    ros::NodeHandle nh("~");
    pclM.OnInit(nh);

    ros::spin();
    return 0;
}


