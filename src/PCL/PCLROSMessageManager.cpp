//
// Created by zhouhua on 2021/2/9.
//

#include "PCL/PCLROSMessageManager.hpp"

PCLROSMessageManager::PCLROSMessageManager(){

}

void PCLROSMessageManager::OnInit(ros::NodeHandle &nh) {
    nh.param("is_sim", is_sim_, false);
    nh.param("/danger_distance", danger_distance_, 3.0);
    if (is_sim_) {
        lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("lslidar_point_cloud", 1,
                                                                  &PCLROSMessageManager::cloudHandler, this);
        // lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1,
        //                                                           &PCLROSMessageManager::cloudHandler, this);
    } else {
        lidar_point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 1,
                                                               &PCLROSMessageManager::cloudHandler, this);
    }

    local_position_sub_ = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20,  &PCLROSMessageManager::local_pos_cb, this);


    transformed_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar/Transformed_points", 1);
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("pcl/Global_octomap", 1);

    ground_removal_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ground_removal_lidar", 1);
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    allocateMemory();
    resetParameters();
}

void PCLROSMessageManager::allocateMemory() {
    fullCloud_.reset(new pcl::PointCloud<PointType>());
    groundCloud_.reset(new pcl::PointCloud<PointType>());
    fullCloud_->points.resize(N_SCAN*Horizon_SCAN);
    groundCloud_->points.resize(N_SCAN*Horizon_SCAN);
    cout << "init111 fullCloud_ size = " << fullCloud_->points.size() << endl;
}

void PCLROSMessageManager::resetParameters() {
    fullCloud_->clear();
    groundCloud_->clear();
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    fullCloud_->points.resize(N_SCAN*Horizon_SCAN);
    std::fill(fullCloud_->points.begin(), fullCloud_->points.end(), nanPoint);
}

void PCLROSMessageManager::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv_.current_local_pos = *msg;
    double yaw, roll, pitch;
//    EulerAngles angles;
//    yaw = Calculate::getInstance()->quaternion_get_yaw(usv_.current_local_pos.pose.orientation, angles);
    Calculate::getInstance()->quaternion_to_rpy(usv_.current_local_pos.pose.orientation, roll, pitch, yaw);
}

void PCLROSMessageManager::setVehicleMessage(const M_Drone& usv) {
    usv_ = usv;
}

void PCLROSMessageManager::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m) {
    /*ros point cloud to pcl point cloud*/
//   ROS_INFO_STREAM("pcl: [thread=" << boost::this_thread::get_id() << "]");

    resetParameters();

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voselGride_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_remove (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*m, *raw_cloud_ptr);

    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*raw_cloud_ptr, *raw_cloud_ptr, indices);

    /*pcl points filter*/
    pcl::IndicesConstPtr cloud_filtered_indices;
    if (!is_sim_)  voselGrid(raw_cloud_ptr, raw_cloud_ptr);

    voselGride_ptr->points.clear();
    if (0) {
        for (std::size_t i = 0; i < raw_cloud_ptr->size(); i++) {
            pcl::PointXYZ pnt = raw_cloud_ptr->points[i];
            TVec3 point = TVec3{pnt.x, pnt.y, pnt.z};
    //        if (pnt.z < -0.2) continue;
            if (point.norm() < 1) continue;
            voselGride_ptr->points.push_back(pnt);
        }
    } else {
        projectPointCloud(raw_cloud_ptr);
        checkGround(voselGride_ptr);
    }

    cout << "voselGride_ptr size = " << voselGride_ptr->size() << endl;
    if (is_sim_) voselGride_ptr = raw_cloud_ptr;
    radiusRemoval(voselGride_ptr, simple_cloud_ptr, 0.5, 3, cloud_filtered_indices);
    groundRemove(simple_cloud_ptr, cloud_ground_remove, cloud_filtered_indices);
    /*transform point cloud*/
    pcl::transformPointCloud(*cloud_ground_remove, *transformed_cloud, get_transformation_matrix().matrix());

    PubPointCloud(transformed_cloud, transformed_cloud_pub_);

    tree_->clear();


    for (std::size_t i = 0; i < transformed_cloud->size(); i++) {
        pcl::PointXYZ pnt = transformed_cloud->points[i];
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
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*output_cloud);
    chlog::info("pcl", "voxel input cloud size = ", input_cloud->size(),
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
    if (output_cloud->is_dense) {
        output_cloud->is_dense = false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*output_cloud,*output_cloud, indices);
    }
    outrem.filter(*output_cloud);
    cloud_filtered_indices = outrem.getIndices();
}

void PCLROSMessageManager::projectPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    PointType thisPoint;

    cloudSize = input_cloud->points.size();
    cout << "input size = " << cloudSize << endl;

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = input_cloud->points[i].x;
        thisPoint.y = input_cloud->points[i].y;
        thisPoint.z = input_cloud->points[i].z;
        // find the row and column index in the iamge for this point
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;

        // 计算是否在雷达垂直线数范围外
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        // 计算是否在水平分辨率外
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;

        index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud_->points[index] = thisPoint;
    }
}

bool PCLROSMessageManager::checkGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (size_t j = 0; j < Horizon_SCAN; ++j){
        for (size_t i = 0; i < groundScanInd; ++i){

            lowerInd = j + ( i )*Horizon_SCAN;
            upperInd = j + (i+1)*Horizon_SCAN;

            diffX = fullCloud_->points[upperInd].x - fullCloud_->points[lowerInd].x;
            diffY = fullCloud_->points[upperInd].y - fullCloud_->points[lowerInd].y;
            diffZ = fullCloud_->points[upperInd].z - fullCloud_->points[lowerInd].z;
//            cout << "diffx = " << diffX << endl;
            if (isnan(diffX) || isnan(diffY) || isnan(diffZ)) {
                groundMat.at<int8_t>(i,j) = -1;
                continue;
            }

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 15){
                groundMat.at<int8_t>(i,j) = 1;
                groundMat.at<int8_t>(i+1,j) = 1;
            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat.at<int8_t>(i,j) == -1 || isnan(fullCloud_->points[j + i * Horizon_SCAN].x) ||
                isnan(fullCloud_->points[j + i * Horizon_SCAN].y) || isnan(fullCloud_->points[j + i * Horizon_SCAN].z)) {
                continue;
            }
            if (groundMat.at<int8_t>(i,j) == 1 /*|| rangeMat.at<float>(i,j) == FLT_MAX*/){
                groundCloud_->push_back(fullCloud_->points[j + i*Horizon_SCAN]);
                continue;
            }
            chlog::info("pcl", "points = " , fullCloud_->points[j + i * Horizon_SCAN].x ,
                        ", " , fullCloud_->points[j + i * Horizon_SCAN].y, ", ",
                        fullCloud_->points[j + i * Horizon_SCAN].z);
            output_cloud->points.push_back(fullCloud_->points[j + i * Horizon_SCAN]);
        }
    }
    PubPointCloud(groundCloud_, ground_removal_pub_);

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
//    seg.setIndices(cloud_filtered_indices);
//    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers, *coefficients_plane);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*output_cloud);
    chlog::info("pcl","input_cloud size = ", input_cloud->points.size(),
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
    chlog::info("pcl", "update octomap!");
    octomap = octomap_;
}

Eigen::Isometry3f PCLROSMessageManager::get_transformation_matrix() {
    Eigen::Isometry3f transformation_matrix;
    transformation_matrix = Eigen::Isometry3f::Identity();
    Eigen::AngleAxisf gimbal_yaw;
    if (!is_sim_) {
        gimbal_yaw = Eigen::AngleAxisf(((usv_.yaw + 180.0f) * M_PI / 180.0f ), Eigen::Vector3f::UnitZ());
    } else {
        gimbal_yaw = Eigen::AngleAxisf(((usv_.yaw ) * M_PI / 180.0f ), Eigen::Vector3f::UnitZ());
    }
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
    chlog::initChannel("pcl");
    chlog::setEnCout("pcl", false);

    PCLROSMessageManager pclM;
    ros::NodeHandle nh("~");
    pclM.OnInit(nh);

    ros::MultiThreadedSpinner spinner(5);
    spinner.spin();
    return 0;
}


