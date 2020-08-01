//
// Created by zhouhua on 2020/5/3.
//
#include <test/2uav-usv/dataMan.hpp>
#include <test/2uav-usv/avoidance.hpp>
#include "test/2uav-usv/uav_lead_uav.hpp"

usv_lead_uav* usv_lead_uav::l_pInst = NULL;

usv_lead_uav::usv_lead_uav():
        uav_state_(TAKEOFF),
        uav_reached_(false),
        is_get_takeoff_pos_(false),
        is_avoidance_(false),
        command_(-1),
        danger_distance_(-1){

}

usv_lead_uav* usv_lead_uav::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new usv_lead_uav();
    }
    return l_pInst;
}

void usv_lead_uav::onInit() {
    util_log("1usv lead 2uavs!");
    ros::NodeHandle uav1_nh("uav1");
    uav1_control_.reset(new uav_ros_Manager);
    uav1_control_->uavOnInit(uav1_nh);

    ros::NodeHandle uav2_nh("uav2");
    uav2_control_.reset(new uav2_ros_Manager);
    uav2_control_->uavOnInit(uav2_nh);

    ros::NodeHandle usv_nh("usv1");
    usv_control_.reset(new usv1_ros_Manager);
    usv_control_->usvOnInit(usv_nh);

    ros::NodeHandle nh("~");
    int waypoint_num_;
    nh.param<double>("formation_distance", formation_distance_, 5);
    nh.param("waypoint_num", waypoint_num_, -1);
    nh.param("danger_distance", danger_distance_, 0.0);
    util_log("formation distance = %.2f, waypoint_num_ = %d", formation_distance_, waypoint_num_);
    usvLocalPositionSp();
}

void usv_lead_uav::getData() {
    multiVehicle = DataMan::getInstance()->GetData();
}

void usv_lead_uav::doProgress() {
    dataMan::getInstance()->getCommand(command_);
    util_log("uav1 avoidance = %.2f, uav2 avoidance = %.2f", multiVehicle.uav1.avoidance_pos.z(), multiVehicle.uav2.avoidance_pos.z());
    checkCollision(danger_distance_);
    follow_uavlocalControl();
    lead_uavlocalControl();
    DataMan::getInstance()->SetDroneControlData(multiVehicle);
    dataMan::getInstance()->SetDroneControlData(multiVehicle);
}

void usv_lead_uav::usvLocalPositionSp() {
    way_point.pose.position.x = 10;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 10;
    way_point.pose.position.y = -10;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = -10;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    usv_way_points.push_back(way_point);

    std::reverse(usv_way_points.begin(), usv_way_points.end());
}

// uav2
void usv_lead_uav::lead_uavlocalControl() {
    util_log("uav_reached = %d, uav_state = %d", uav_reached_, uav_state_);
    if (uav_state_ == TAKEOFF || uav_state_ == FORMATION || uav_state_ == TURN_YAW) {
        way_point.pose.position.x = 0;
        way_point.pose.position.y = 0;
        way_point.pose.position.z = 15;

        if (is_avoidance_) way_point.pose.position.z =
                multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();

    } else {
        current_usv_local_pos_ = multiVehicle.uav2.current_local_pos;
        if (command_ != FOLLOW_USV) {
            if (!usv_way_points.empty()) {
                way_point = usv_way_points.back();
                if (is_avoidance_)
                    way_point.pose.position.z =
                            multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();
                if (pos_reached(current_usv_local_pos_, way_point) && uav_reached_) {
                    util_log("Finished one way point = (%.2f, %.2f, %.2f)",
                             way_point.pose.position.x, way_point.pose.position.y,
                             way_point.pose.position.z);
                    usv_way_points.pop_back();

                    if (!usv_way_points.empty()) {
                        util_log("Goto next way point = (%.2f, %.2f, %.2f)",
                                 way_point.pose.position.x, way_point.pose.position.y,
                                 way_point.pose.position.z);
                    } else {
                        util_log("Finish all target points!");
                    }
                }
            } else if (uav_state_ == FOLLOW_UAV) {
                if (is_avoidance_) {
                    way_point.pose.position.z =
                            multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();
                } else {
                    way_point.pose.position.z = multiVehicle.uav2.current_local_pos.pose.position.z;
                }
            }
        } else {
            util_log("command follow usv");
            way_point.pose.position.x = multiVehicle.usv1.current_local_pos.pose.position.x;
            way_point.pose.position.y = multiVehicle.usv1.current_local_pos.pose.position.y;
            if (is_avoidance_) {
                way_point.pose.position.z =
                        multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();
            } else {
                way_point.pose.position.z = multiVehicle.uav2.current_local_pos.pose.position.z;
            }
            util_log("follow uav way_point = %.2f, = %.2f, = %.2f", way_point.pose.position.x, way_point.pose.position.y, way_point.pose.position.z );
        }
    }


    if (command_ == ALLRETURN) {
        way_point.pose.position.x = 0;
        way_point.pose.position.y = 0;
        way_point.pose.position.z = 15;

        if (is_avoidance_) way_point.pose.position.z = multiVehicle.uav2.current_local_pos.pose.position.z + multiVehicle.uav2.avoidance_pos.z();
        if (pos_reached(current_usv_local_pos_, way_point) && uav_reached_) {
            int command = ALLLAND;
            dataMan::getInstance()->setCommand(command);
        }
    }

    multiVehicle.uav2.target_local_pos_sp = way_point;
    util_log("received avoidance data = %.2f", multiVehicle.uav2.avoidance_pos.z());
    util_log("uav way_point = %.2f, = %.2f, = %.2f", way_point.pose.position.x, way_point.pose.position.y, way_point.pose.position.z );
    uav2_control_->uavPosSp(way_point);
}

void usv_lead_uav::follow_uavlocalControl() {
    DroneControl uav1_ctrl {};
    GetTakeoffPos(multiVehicle.uav2, multiVehicle.uav1, follow_slave_first_local_);
    formation(multiVehicle.uav2, multiVehicle.uav1, follow_slave_formation_);
    current_uav_local_pos_ = multiVehicle.uav1.current_local_pos;
    multiVehicle.uav1.target_local_pos_sp = uav1_ctrl.target_pose;
    if (command_ == ALLRETURN) uav_state_ = RETURN;

    switch (uav_state_) {
        case TAKEOFF: {
            uav1_ctrl.target_pose.pose.position.x = 0;
            uav1_ctrl.target_pose.pose.position.y = 0;
            uav1_ctrl.target_pose.pose.position.z = 10;
            uav1_ctrl.speed_ctrl = false;
            uav1_ctrl.target_pose.pose.orientation = multiVehicle.uav1.current_local_pos.pose.orientation;
            uav1_control_->uavPosSp(uav1_ctrl);
            if (pos_reached(current_uav_local_pos_, uav1_ctrl.target_pose)) {
                uav_state_ = TURN_YAW;
//                uav_reached_ = true;
            }
        }
            break;

        case TURN_YAW: {
            float yaw_err;
            yaw_err = multiVehicle.uav1.yaw - multiVehicle.uav2.yaw;
            if (fabs(yaw_err) > 5) {
                uav1_ctrl.speed_ctrl = true;
                uav1_ctrl.target_heading = multiVehicle.uav2.yaw;
                util_log("target heading = %.2f", uav1_ctrl.target_heading);
            } else {
                uav1_ctrl.speed_ctrl = false;
                uav1_ctrl.target_heading = multiVehicle.uav1.yaw;
                uav_state_ = FORMATION;
            }
            uav1_control_->uavPosSp(uav1_ctrl);
        }
            break;

        case FORMATION: {
            uav1_ctrl.target_pose.pose.position.x = follow_slave_formation_.x();
            uav1_ctrl.target_pose.pose.position.y = follow_slave_formation_.y();
            if (is_avoidance_) {
                uav1_ctrl.target_pose.pose.position.z += multiVehicle.uav1.avoidance_pos.z();
            } else {
                uav1_ctrl.target_pose.pose.position.z = way_point.pose.position.z - 2;
            }
            uav1_ctrl.speed_ctrl = false;
            uav1_ctrl.target_pose.pose.orientation = multiVehicle.uav2.current_local_pos.pose.orientation;
            uav1_control_->uavPosSp(uav1_ctrl);

            if (pos_reached(current_uav_local_pos_, uav1_ctrl.target_pose)) {
                uav_state_ = FOLLOW_UAV;
                uav_reached_ = true;
            }
        }
            break;

        case FOLLOW_UAV : {
            uav_reached_ = false;
            uav1_ctrl.target_pose.pose.position.x = way_point.pose.position.x + follow_slave_formation_.x();
            uav1_ctrl.target_pose.pose.position.y = way_point.pose.position.y + follow_slave_formation_.y();
            if (is_avoidance_) {
                uav1_ctrl.target_pose.pose.position.z =
                        multiVehicle.uav1.current_local_pos.pose.position.z + multiVehicle.uav1.avoidance_pos.z();
            } else {
                uav1_ctrl.target_pose.pose.position.z = way_point.pose.position.z - 2;
            }

            uav1_ctrl.speed_ctrl = false;
            uav1_ctrl.target_pose.pose.orientation = multiVehicle.uav1.current_local_pos.pose.orientation;
            uav1_control_->uavPosSp(uav1_ctrl);
            if (pos_reached(current_uav_local_pos_, uav1_ctrl.target_pose)) {
                uav_state_ = FOLLOW_UAV;
                uav_reached_ = true;
            }
        }
            break;

        case RETURN: {
            uav_reached_ = false;
            uav1_ctrl.target_pose.pose.position.x = 0;
            uav1_ctrl.target_pose.pose.position.y = 0;
            uav1_ctrl.target_pose.pose.position.z = 10;
            if (is_avoidance_) {
                uav1_ctrl.target_pose.pose.position.z += multiVehicle.uav1.avoidance_pos.z();
            } else {
                uav1_ctrl.target_pose.pose.position.z = way_point.pose.position.z - 2;
            }
            uav1_ctrl.speed_ctrl = false;
            uav1_ctrl.target_pose.pose.orientation = multiVehicle.uav1.current_local_pos.pose.orientation;
            uav1_control_->uavPosSp(uav1_ctrl);
            if (pos_reached(current_uav_local_pos_, uav1_ctrl.target_pose)) {
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
    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < 0.8f;
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
            util_log("Received formation: slave back x");
            slave_local_formation = TVec3(-formation_distance_, 0, slave.current_local_pos.pose.position.z);
            break;
        }
        case SLAVEFORWARDX: {
            util_log("Received formation: slave forward x");
            slave_local_formation = TVec3(formation_distance_, 0, slave.current_local_pos.pose.position.z);
            break;
        }
        case SLAVEBACKY: {
            util_log("Received formation: slave back y");
            slave_local_formation = TVec3(0, formation_distance_, slave.current_local_pos.pose.position.z);
            break;
        }
        case SLAVEFORWARDY: {
            util_log("Received formation: slave forward y");
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

void usv_lead_uav::checkCollision(double danger_distance) {
    bool is_collision;
    avoidance::getInstance()->checkCollision(is_collision, is_avoidance_, danger_distance);
    if (is_collision) {
        int command = ALLSTOP;
        dataMan::getInstance()->setCommand(command);
    }

}

