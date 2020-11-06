//
// Created by eric on 19-7-18.
//http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

image_transport::Publisher image_raw_pub;
ros::Publisher cam_img_raw_pub_;
ros::Publisher cam_img_pub_;
ros::Publisher gray_img_raw_pub_;
ros::Subscriber cam_img_compressed_sub;
ros::Subscriber gray_img_compressed_sub;
ros::Subscriber gazebo_gray_img_compressed_sub;
bool is_rosbag_replay_mode_ = true;
bool is_gazebo_sim = false;
bool use_rqt_ui = false;

void CamImgCb(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    Mat image;
    try {
        image = imdecode(Mat(msg->data), CV_LOAD_IMAGE_COLOR);
        //if (!is_gazebo_sim) cvtColor(image, image, COLOR_BGR2RGB);//raw is rgb8

        if (is_rosbag_replay_mode_) {
            int live_view_width_ = 1024;
            resize(image, image, Size(live_view_width_, live_view_width_ * image.rows / image.cols), 0, 0,
                   INTER_LINEAR);
        }
        if (!use_rqt_ui)
            imshow("cam_img_sub", image);
        cv::waitKey(5);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert to image!");
    }
    sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //image_raw_pub.publish(img_msg);
    //使用这个避免重复发布 compressed image
    cam_img_raw_pub_.publish(img_msg);
    cam_img_pub_.publish(img_msg);
}

void GrayImgCb(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    //cout << "GrayImgCb\n";
    Mat image;
    try {
        image = imdecode(Mat(msg->data), CV_LOAD_IMAGE_GRAYSCALE);
        //if (is_rosbag_replay_mode_) {
        //int width = 1024;
        //resize(image, image, Size(width, width * image.rows / image.cols), 0, 0, INTER_LINEAR);
        //}
        if (!use_rqt_ui) {
            imshow("gray_img_sub", image);
            cv::waitKey(5);
        }

    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert to image!");
    }
    sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    //image_raw_pub.publish(img_msg);
    //使用这个避免重复发布 compressed image
    gray_img_raw_pub_.publish(img_msg);
}

void GazeboGrayImgCb(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    is_gazebo_sim = true;
    GrayImgCb(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compressed_image_to_raw");
    ROS_INFO("compressed_image_to_raw run!");
    ros::NodeHandle nh;
    string cam_img_compressed, cam_img_raw;// /camera/image_raw
    string gray_img_compressed, gray_img_raw, gazebo_gray_img_compressed;
//    nh.param<std::string>("cam_img_compressed", cam_img_compressed, "/camera/live_view/compressed");
    nh.param<std::string>("cam_img_compressed", cam_img_compressed, "/usb_cam/image_raw/compressed");
    nh.param<std::string>("gray_img_compressed", gazebo_gray_img_compressed, "/depth_cam/image_raw/compressed");
    //nh.param<std::string>("cam_img_compressed", cam_img_compressed, "/camera/live_view/compressed");
    //nh.param<std::string>("cam_img_raw", cam_img_raw, "/camera/live_view");
    nh.param<std::string>("cam_img_raw", cam_img_raw, "/camera/live_view_raw");
    nh.param<std::string>("gray_img_raw", gray_img_raw, "/ce30/gray_image");
    //nh.param<bool>("is_rosbag_replay_mode_", is_rosbag_replay_mode_, true);
    nh.param<bool>("use_rqt_ui", use_rqt_ui, false);

    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber image_sub_ = it.subscribe("/camera/live_view", 1, &ImageCb);
    cam_img_compressed_sub = nh.subscribe<sensor_msgs::CompressedImage>
            (cam_img_compressed, 10, CamImgCb);
    cam_img_raw_pub_ = nh.advertise<sensor_msgs::Image>(cam_img_raw, 10);
    cam_img_pub_ = nh.advertise<sensor_msgs::Image>("/camera/live_view", 10);


    /*ros::Rate loop_rate(5);
    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }*/
    ros::spin();
}