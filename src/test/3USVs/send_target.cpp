//
// Created by zhouhua on 2021/5/13.
//

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped usv1_target_local_pos_rc_;
geometry_msgs::PoseStamped usv2_target_local_pos_rc_;
geometry_msgs::PoseStamped usv3_target_local_pos_rc_;

geometry_msgs::PoseStamped usv1_local_pos_rc_;
geometry_msgs::PoseStamped usv2_local_pos_rc_;
geometry_msgs::PoseStamped usv3_local_pos_rc_;

bool is_usv1_local_pos_rc_ = false;
bool is_usv2_local_pos_rc_ = false;
bool is_usv3_local_pos_rc_ = false;

bool is_usv1_local_pos_sp_ = false;
bool is_usv2_local_pos_sp_ = false;
bool is_usv3_local_pos_sp_ = false;

void usv1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv1_target_local_pos_rc_ = *msg;
    is_usv1_local_pos_sp_ = true;
}
void usv2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv2_target_local_pos_rc_ = *msg;
    is_usv2_local_pos_sp_ = true;
}
void usv3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv3_target_local_pos_rc_ = *msg;
    is_usv3_local_pos_sp_ = true;
}

void usv1_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv1_local_pos_rc_ = *msg;
    is_usv1_local_pos_rc_ = true;
}
void usv2_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv2_local_pos_rc_ = *msg;
    is_usv2_local_pos_rc_ = true;
}
void usv3_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    usv3_local_pos_rc_ = *msg;
    is_usv3_local_pos_rc_ = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "send_target");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    usv1_target_local_pos_rc_.pose.position.x = 0;
    usv1_target_local_pos_rc_.pose.position.y = 0;
    usv1_target_local_pos_rc_.pose.position.z = 0;
    usv2_target_local_pos_rc_ = usv1_target_local_pos_rc_;
    usv3_target_local_pos_rc_ = usv1_target_local_pos_rc_;

    ros::Subscriber usv1_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv1/setpoint_position/local", 20,  &usv1_local_pos_cb);
    ros::Subscriber usv2_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv2/setpoint_position/local", 20,  &usv2_local_pos_cb);
    ros::Subscriber usv3_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv3/setpoint_position/local", 20,  &usv3_local_pos_cb);

    ros::Subscriber usv1_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv1/mavros/local_position/pose", 20,  &usv1_pos_cb);
    ros::Subscriber usv2_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv2/mavros/local_position/pose", 20,  &usv2_pos_cb);
    ros::Subscriber usv3_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/usv3/mavros/local_position/pose", 20,  &usv3_pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/usv1/mavros/setpoint_position/local", 100);
    ros::Publisher usv2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/usv2/mavros/setpoint_position/local", 100);
    ros::Publisher usv3_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/usv3/mavros/setpoint_position/local", 100);
    while(ros::ok()){

        // init target as current pos if doesn't receive target
        if (is_usv1_local_pos_rc_ && !is_usv1_local_pos_sp_) {
            usv1_target_local_pos_rc_ = usv1_local_pos_rc_;
        }
        if (is_usv2_local_pos_rc_ && !is_usv2_local_pos_sp_) {
            usv2_target_local_pos_rc_ = usv2_local_pos_rc_;
        }
        if (is_usv3_local_pos_rc_ && !is_usv3_local_pos_sp_) {
            usv3_target_local_pos_rc_ = usv3_local_pos_rc_;
        }

        local_pos_pub.publish(usv1_target_local_pos_rc_);
        usv2_local_pos_pub.publish(usv2_target_local_pos_rc_);
        usv3_local_pos_pub.publish(usv3_target_local_pos_rc_);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}