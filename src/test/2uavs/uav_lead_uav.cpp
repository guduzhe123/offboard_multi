//
// Created by zhouhua on 2020/5/3.
//
#include <test/2uav2/dataMan.hpp>
#include "test/2uav2/uav_lead_uav.hpp"

usv_lead_uav* usv_lead_uav::l_pInst = NULL;

usv_lead_uav::usv_lead_uav():
        uav_state_(TAKEOFF),
        uav_reached_(false),
        is_get_takeoff_pos_(false),
        command_(-1){

}

usv_lead_uav* usv_lead_uav::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new usv_lead_uav();
    }
    return l_pInst;
}

void usv_lead_uav::onInit() {
    ros::NodeHandle uav_nh("uav1");
    uav_control_.reset(new uav_ros_Manager);
    uav_control_->uavOnInit(uav_nh);

    ros::NodeHandle usv_nh("uav2");
    usv_control_.reset(new usv_ros_Manager);
    usv_control_->usvOnInit(usv_nh);

    ros::NodeHandle nh("~");
    nh.param<double>("formation_distance", formation_distance_, 5);
    util_log("formation distance = %.2f", formation_distance_);
    usvLocalPositionSp();
}

void usv_lead_uav::getData() {
    multiVehicle = dataMan::getInstance()->GetData();
}

void usv_lead_uav::doProgress() {
    dataMan::getInstance()->getCommand(command_);
    util_log("uav1 avoidance = %.2f, uav2 avoidance = %.2f", multiVehicle.uav1.avoidance_pos.z(), multiVehicle.uav2.avoidance_pos.z());
    usvlocalControl();
    uavlocalControl();
    DataMan::getInstance()->SetDroneControlData(multiVehicle);
    dataMan::getInstance()->SetDroneControlData(multiVehicle);
}

void usv_lead_uav::usvLocalPositionSp() {
    way_point.pose.position.x = 30;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 15;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 5;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
}

// uav2
void usv_lead_uav::usvlocalControl() {
    current_usv_local_pos_ = multiVehicle.uav2.current_local_pos;
    if (!usv_way_points.empty()) {
        if (uav_reached_) {
            way_point = usv_way_points.back();
            way_point.pose.position.z = multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();
            if (pos_reached(current_usv_local_pos_, usv_way_points.back())) {
                ROS_INFO("Finished one way point = (%.2f, %.2f, %.2f)",
                         usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                         usv_way_points.back().pose.position.z);
                usv_way_points.pop_back();

                if (!usv_way_points.empty()) {
                    ROS_INFO("Goto next way point = (%.2f, %.2f, %.2f)",
                             usv_way_points.back().pose.position.x, usv_way_points.back().pose.position.y,
                             usv_way_points.back().pose.position.z);
                } else {
                    ROS_INFO("Finish all target points!");
                }
            }
        }
        multiVehicle.uav2.target_local_pos_sp = way_point;
    }

    if (command_ == ALLRETURN) {
        way_point.pose.position.x = 0;
        way_point.pose.position.y = 0;
        way_point.pose.position.z = 15;
        uav_state_ = RETURN;

        way_point.pose.position.z = multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();
        if (pos_reached(current_usv_local_pos_, way_point) && uav_reached_) {
            int command = ALLLAND;
            dataMan::getInstance()->setCommand(command);
        }
    }

    usv_control_->usvPosSp(way_point);
}

void usv_lead_uav::uavlocalControl() {
    GetTakeoffPos(multiVehicle.uav2, multiVehicle.uav1, follow_slave_first_local_);
    formation(multiVehicle.uav2, multiVehicle.uav1, follow_slave_formation_);
    current_uav_local_pos_ = multiVehicle.uav1.current_local_pos;
    multiVehicle.uav1.target_local_pos_sp = uav_way_point;

    switch (uav_state_) {
        case TAKEOFF: {
            uav_way_point.pose.position.x = 0;
            uav_way_point.pose.position.y = 0;
            uav_way_point.pose.position.z = 15;
            uav_control_->uavPosSp(uav_way_point);
            usv_control_->usvPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FORMATION;
            }
        }
            break;

        case FORMATION: {
            // uav at head of the usv
            uav_way_point.pose.position.x = follow_slave_formation_.x();
            uav_way_point.pose.position.y = follow_slave_formation_.y();
            uav_way_point.pose.position.z += multiVehicle.uav1.avoidance_pos.z();
            uav_control_->uavPosSp(uav_way_point);

            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FOLLOW;
                uav_reached_ = true;
            }
        }
            break;

        case FOLLOW : {
            uav_reached_ = false;
            uav_way_point.pose.position.x = multiVehicle.uav2.current_local_pos.pose.position.x + follow_slave_formation_.x();
            uav_way_point.pose.position.y = multiVehicle.uav2.current_local_pos.pose.position.y + follow_slave_formation_.y();
            uav_way_point.pose.position.z = multiVehicle.uav1.current_local_pos.pose.position.z + multiVehicle.uav1.avoidance_pos.z();

            uav_control_->uavPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_state_ = FOLLOW;
                uav_reached_ = true;
            }
        }
            break;

        case RETURN: {
            uav_reached_ = false;
            uav_way_point.pose.position.x = 0;
            uav_way_point.pose.position.y = 0;
            uav_way_point.pose.position.z = 15;
            uav_way_point.pose.position.z += multiVehicle.uav1.avoidance_pos.z();
            uav_control_->uavPosSp(uav_way_point);
            if (pos_reached(current_uav_local_pos_, uav_way_point)) {
                uav_reached_ = true;
            }
        }
            break;

        default:
            break;
    }
}

bool usv_lead_uav::pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;
    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < 2.0f;
}

void usv_lead_uav::GetTakeoffPos(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_first_local) {
    if (slave.current_state.armed && !is_get_takeoff_pos_) {
        master_start_gps_ = GlobalPosition{master.latitude, master.longtitude, 0};
        slave_takeoff_gps_ = GlobalPosition{slave.latitude, slave.longtitude, 0};
        util_log("master_start_gps_ = ( %.9f, %.9f)", master_start_gps_.latitude, master_start_gps_.longitude);
        util_log("slave_takeoff_gps_ = ( %.9f, %.9f)", slave_takeoff_gps_.latitude, slave_takeoff_gps_.longitude);

        Calculate::getInstance()->GetLocalPos(master_start_gps_, slave_takeoff_gps_, follow_slave_first_local);
        util_log("follow_slave_first_local = ( %.2f, %.2f, %.2f)", follow_slave_first_local.x(), follow_slave_first_local.y(),
                 follow_slave_first_local.z());

        is_get_takeoff_pos_ = true;
    }
}

void usv_lead_uav::formation(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_formation) {
    TVec3 slave_local_formation = {0,0,0};
    switch (command_) {
        case SLAVEBACKX: {
            slave_local_formation = TVec3(-formation_distance_, 0, slave.current_local_pos.pose.position.z);
            break;
        }
        case SLAVEFORWARDX: {
            slave_local_formation = TVec3(formation_distance_, 0, slave.current_local_pos.pose.position.z);
            break;
        }
        case SLAVEBACKY: {
            slave_local_formation = TVec3(0, formation_distance_, slave.current_local_pos.pose.position.z);
            break;
        }
        case SLAVEFORWARDY: {
            slave_local_formation = TVec3(0, -formation_distance_, slave.current_local_pos.pose.position.z);
            break;
        }
        default:
            slave_local_formation = TVec3(-formation_distance_, 0, slave.current_local_pos.pose.position.z);
            break;
    }

    TVec3 slave_first_local;
    Calculate::getInstance()->GetLocalPos(master_start_gps_, slave_takeoff_gps_, slave_first_local);

    follow_slave_formation.x() = slave_local_formation.x() + slave_first_local.x();
    follow_slave_formation.y() = slave_local_formation.y() + slave_first_local.y();

    util_log("slave formation x = %.2f, y = %.2f", follow_slave_formation.x(), follow_slave_formation.y());
    util_log("slave slave_local_formation x = %.2f, y = %.2f", slave_local_formation.x(), slave_local_formation.y());
    util_log("slave slave_first_local x = %.2f, y = %.2f", slave_first_local.x(), slave_first_local.y());
}

