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
#include <pcl/point_types.h>
#include <opencv/cv.h>

#include "Calculate.hpp"

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;

typedef pcl::PointXYZ  PointType;
// HDL-32E
extern const int N_SCAN = 32;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 41.33/float(N_SCAN-1);
extern const float ang_bottom = 30.67;
extern const int groundScanInd = 20;

extern const float sensorMinimumRange = 1.0;
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

class PCLROSMessageManager {
public:
    PCLROSMessageManager();
    ~PCLROSMessageManager() = default;

    void OnInit(ros::NodeHandle &nh);
    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &m);
    void getOctomap(octomap_msgs::Octomap &octomap);
    void setVehicleMessage(const M_Drone& usv);

private:
    ros::Subscriber lidar_point_sub_, local_position_sub_;
    ros::Publisher octomap_pub_, transformed_cloud_pub_, ground_removal_pub_;

    M_Drone usv_;

    octomap_msgs::Octomap octomap_;
    octomap::OcTree *tree_ = new octomap::OcTree(0.5);

    bool is_sim_;
    double danger_distance_;


    pcl::PointCloud<PointType>::Ptr fullCloud_; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    cv::Mat groundMat; // ground matrix for ground cloud marking
    pcl::PointCloud<PointType>::Ptr groundCloud_;
    PointType nanPoint;


    void PubOctomap(octomap::OcTree *tree, const ros::Publisher &pub);
    void radiusRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                       float radius, int min_neighbors,
                       pcl::IndicesConstPtr &cloud_filtered_indices);
    void voselGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    void groundRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                      const pcl::IndicesConstPtr &cloud_filtered_indices);
    Eigen::Isometry3f get_transformation_matrix();
    void PubPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &o_cloud, const ros::Publisher &pub,
                       string frame = "map");
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void allocateMemory();
    void resetParameters();
    void projectPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);
    bool checkGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);
};
#endif //OFFBOARD_PCLROSMESSAGEMANAGER_HPP
