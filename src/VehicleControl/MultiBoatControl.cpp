//
// Created by zhouhua on 2020/1/26.
//

#include "VehicleControl/MultBoatControl.hpp"
MultiBoatControl* MultiBoatControl:: l_lint = NULL;

MultiBoatControl::MultiBoatControl() :
        usv_state_(USV_INIT),
        is_formation_(false),
        update_takeoff_(false),
        state_changed_(false),
        usv1_reached_(false),
        usv2_reached_(false),
        usv3_reached_(false),
        formation_config_(0),
        usv_waypoints_size_init_(0){

}

MultiBoatControl* MultiBoatControl::getInstance() {
    if (l_lint == NULL) {
        l_lint = new MultiBoatControl();
    }
    return l_lint;
}

void MultiBoatControl::onInit(vector<geometry_msgs::PoseStamped> way_points, bool is_uav_follow) {
    util_log("boat control start! sizeof usv waypoints = %d", way_points.size());
    uav_way_points_init_ = way_points;

}

void MultiBoatControl::getData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
    is_formation_ = m_multi_vehicle_.leader_usv.is_formation;
    config_ = m_multi_vehicle_.user_command;
    if (config_ == VF_USV_TRIANGLE || config_ == VF_USV_INVERSION_TRIANGLE || config_ == VF_USV_LINE_HORIZONTAL
        || config_ == VF_USV_LINE_VERTICAL) {
        formation_config_ = config_;
    }
    if (m_multi_vehicle_.user_command == VF_USV_CIRCLE && !state_changed_) {
        usv_state_ = USV_CIRCLE_INIT;
    }
}

void MultiBoatControl::DoProgress() {
    util_log("is usv in formation = %d", is_formation_);
    if (is_formation_) {
        return;
    }

    util_log("leader usv movement_state = %d", usv_state_);
    util_log("leader usv armed = %d", m_multi_vehicle_.leader_usv.current_state.armed);
    if (m_multi_vehicle_.leader_usv.current_state.mode == "OFFBOARD" /*&& m_multi_vehicle_.leader_uav.movement_state == FALLOW_USV*/) {
        mavros_msgs::CommandBool arm_cmd;
        switch (usv_state_) {
            case USV_INIT:
                util_log("control: usv1 waypoint size = %d", m_multi_vehicle_.usv1.waypointList.waypoints.size());
                if (!m_multi_vehicle_.usv1.waypointList.waypoints.empty()) {
                    for (auto &i : m_multi_vehicle_.leader_usv.waypointList.waypoints) {
                        GlobalPosition takeoff, waypnt;
                        geometry_msgs::PoseStamped target_init;
                        TVec3 target_local;
                        takeoff.longitude = m_multi_vehicle_.leader_usv.homePosition.geo.longitude;
                        takeoff.latitude = m_multi_vehicle_.leader_usv.homePosition.geo.latitude;
                        waypnt.longitude = i.y_long;
                        waypnt.latitude = i.x_lat;
                        Calculate::getInstance()->GetLocalPos(takeoff, waypnt, target_local);
                        target_init.pose.position.x = -target_local.x();
                        target_init.pose.position.y = -target_local.y();
                        target_init.pose.position.z = 0;
                        usv_way_points_.push_back(target_init);
                        util_log("target_local = (%.2f, %.2f)", target_local.x(), target_local.y());
                    }
                    std::reverse(usv_way_points_.begin(), usv_way_points_.end());
                    util_log("boat mission waypoint size = %d", usv_way_points_.size());
                } else {
                    for (auto & i : uav_way_points_init_) {
                        geometry_msgs::PoseStamped target_body;
                        Calculate::getInstance()->bodyFrame2LocalFrame(i, target_body,
                                                                       (float)(m_multi_vehicle_.usv1.yaw * M_PI / 180.0f));
                        usv_way_points_.push_back(target_body);
                    }
                }
                usv_waypoints_size_init_ = usv_way_points_.size();
                usv_state_ = USV_WAYPOINT;
                break;

            case USV_WAYPOINT: {
                util_log("boat local size = %d", usv_way_points_.size());
                if (!usv_way_points_.empty()) {
                    m_multi_vehicle_.leader_usv.target_local_pos_sp = usv_way_points_.back();
                    target_pos_ = usv_way_points_.back();
//                    Calculate::getInstance()->bodyFrame2LocalFrame(body_pos_, target_pos_,init_yaw_);

                    if (pos_reached(m_multi_vehicle_.usv1.current_local_pos, target_pos_,
                                    usv_position_allow_reached_) && !usv1_reached_) {
                        usv1_reached_ = true;
                        util_log("usv1 disarm at one point");
                    }

                    util_log("usv2 local (x = %.2f, y = %.2f, z = %.2f)", m_multi_vehicle_.usv2.current_local_pos.pose.position.x,
                             m_multi_vehicle_.usv2.current_local_pos.pose.position.y, m_multi_vehicle_.usv2.current_local_pos.pose.position.y);
                    util_log("target_usv2 = (%.2f, %.2f, %.2f)", target_usv2_init_.pose.position.x,
                            target_usv2_init_.pose.position.y, target_usv2_init_.pose.position.z);
                    if (pos_reached(m_multi_vehicle_.usv2.current_local_pos, target_usv2_init_,
                                    usv_position_allow_reached_) && !usv2_reached_) {
                        usv2_reached_ = true;
                        util_log("usv2 disarm at one point");
                    }

                    if (pos_reached(m_multi_vehicle_.usv3.current_local_pos, target_usv3_init_,
                                    usv_position_allow_reached_) && !usv3_reached_) {
                        usv3_reached_ = true;
                        util_log("usv3 disarm at one point");
                    }

                    util_log("usv1_reached_ = %d, usv2_reached_ = %d, usv3_reached_ = %d", usv1_reached_, usv2_reached_ ,usv3_reached_);
                    if (usv1_reached_) {
                        if (usv_way_points_.size() > usv_waypoints_size_init_ - 2
                            && usv2_reached_ && usv3_reached_) {
                            usv_way_points_.pop_back();
                        } else if (usv_way_points_.size() <= usv_waypoints_size_init_ - 2) {
                            usv_way_points_.pop_back();
                        }
                        util_log("Finished one way point = (%.2f, %.2f, %.2f)",
                                 usv_way_points_.back().pose.position.x, usv_way_points_.back().pose.position.y,
                                 usv_way_points_.back().pose.position.z);

                        if (!usv_way_points_.empty()) {
                            util_log("Goto next way point = (%.2f, %.2f, %.2f)",
                                     usv_way_points_.back().pose.position.x, usv_way_points_.back().pose.position.y,
                                     usv_way_points_.back().pose.position.z);
                        } else {
                            util_log("Finish all target points!");
                            usv_state_ = USV_DISARM;
                        }
                        usv1_reached_ = false;
                        usv2_reached_ = false;
                        usv3_reached_ = false;

                    }

                    TVec3 usv1_target = TVec3(target_pos_.pose.position.x, target_pos_.pose.position.y, 0);
                    TVec3 usv1_cur = TVec3(m_multi_vehicle_.usv1.current_local_pos.pose.position.x,
                                           m_multi_vehicle_.usv1.current_local_pos.pose.position.y,
                                           0);
                    TVec3 vec = (usv1_target - usv1_cur).normalized();
                    float target_yaw = atan2(vec.y(), vec.x()) * 180 / M_PI;
                    if (fabs(target_yaw - m_multi_vehicle_.usv1.yaw) < 10 ) {
                        PathCreator::geInstance()->CreateUSVFormationInit(formation_config_);
                    }
                    util_log("targte yaw = %.2f, m_multi_vehicle_.usv1.yaw = %.2f", target_yaw, m_multi_vehicle_.usv1.yaw);

                } else {
                    usv_state_ = USV_DISARM;
                }

                setVehicleCtrlData();
                break;
            }

            case USV_DISARM:
                // disarm all.
                util_log("Disarm all usv");
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 0;
                break;

            case USV_CIRCLE_INIT: {
                TCircleConfig circle_config;
                circle_config.target_heading = m_multi_vehicle_.usv1.yaw;
                circle_config.m_radius = 20;
                geometry_msgs::PoseStamped target_local;
                geometry_msgs::PoseStamped body;
                body.pose.position.x = circle_config.m_radius;
                body.pose.position.y = 0;
                body.pose.position.z = 0;
                Calculate::getInstance()->bodyFrame2LocalFrame(body, target_local,
                                                               (float)(m_multi_vehicle_.usv1.yaw * M_PI / 180.0f));

                circle_config.m_circle_pos = TVec3(m_multi_vehicle_.usv1.current_local_pos.pose.position.x + target_local.pose.position.x,
                                                   m_multi_vehicle_.usv1.current_local_pos.pose.position.y + target_local.pose.position.y,
                                                   m_multi_vehicle_.usv1.current_local_pos.pose.position.z + target_local.pose.position.z);


                circle_config.m_start_pos = TVec3(m_multi_vehicle_.usv1.current_local_pos.pose.position.x,
                                                  m_multi_vehicle_.usv1.current_local_pos.pose.position.y,
                                                  m_multi_vehicle_.usv1.current_local_pos.pose.position.z);
                circle_config.m_speed = 2.0;

                ActionCircle::getInstance()->onInit(circle_config);
                usv_state_ = USV_CIRCLE;
                util_log("USV Circle init!!!!");
                ActionCircle::getInstance()->doProgress(circle_config.m_start_pos, m_multi_vehicle_.usv1.yaw);
                TCircleOutput circle_output;
                ActionCircle::getInstance()->GetOutput(circle_output);
                usv_way_points_.clear();
                usv_way_points_ = circle_output.usv_way_points_;
                usv_waypoints_size_init_ = usv_way_points_.size();
                usv_state_ = USV_WAYPOINT;
                state_changed_ = true;
                break;
            }


            case USV_FOLLOW_UUV_FORMATION_INIT: {
                way_bear_ = Calculate::getInstance()->get_bearing_to_next_waypoint(m_multi_vehicle_.usv1.latitude, m_multi_vehicle_.usv1.longtitude
                        , m_multi_vehicle_.uuv1.latitude, m_multi_vehicle_.uuv1.longtitude);
                usv_state_ = USV_FOLLOW_UUV_FORMATION;
                break;
            }

            case USV_FOLLOW_UUV_FORMATION: {
                geometry_msgs::PoseStamped local_usv1, local_usv2, local_usv3;
                follow_usv1_ = TVec3(0, -1.5* K_multi_usv_formation_distance, m_multi_vehicle_.usv2.current_local_pos.pose.position.z);
                follow_usv2_ = TVec3(0, 1.5*K_multi_usv_formation_distance,  m_multi_vehicle_.usv3.current_local_pos.pose.position.z);
                follow_usv3_ = TVec3(-1.5*K_multi_usv_formation_distance, 0 , m_multi_vehicle_.usv3.current_local_pos.pose.position.z);

                GetTakeoffPos();
                changeToLocalTarget();
                calcFollowUUVPos();
                SetFunctionOutPut();

                if (pos_reached(m_multi_vehicle_.usv1.current_local_pos, m_multi_vehicle_.usv1.target_local_pos_sp, usv_position_allow_reached_)) {
                    util_log("arrived at target position and uuv is at center!");
                    usv_state_ = USV_FOLLOW_UUV;
                }
                break;
            }


            case USV_FOLLOW_UUV: {
                m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position =
                        m_multi_vehicle_.uuv1.current_local_pos.pose.position;
                m_multi_vehicle_.leader_usv.current_local_pos.pose.position =
                        m_multi_vehicle_.uuv1.current_local_pos.pose.position;

                m_multi_vehicle_.usv1.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.target_local_pos_sp, follow_usv1_keep_local_);
                m_multi_vehicle_.usv2.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.current_local_pos, follow_usv2_keep_local_);
                m_multi_vehicle_.usv3.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.current_local_pos, follow_usv3_keep_local_);
                m_multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
                DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);

                break;
            }
        }

    } else {

        if (m_multi_vehicle_.usv1.waypointReached.wp_seq == m_multi_vehicle_.usv1.waypointList.current_seq &&
            m_multi_vehicle_.usv1.waypointReached.wp_seq > 0) {
            if (m_multi_vehicle_.uuv1.longtitude > 0) {
                util_log("usv1 finish all waypoints! Follow usv");
                usv_state_ = USV_FOLLOW_UUV_FORMATION_INIT;
            } else {
                usv_state_ = USV_FOLLOW_UUV;
            }

            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            DataMan::getInstance()->SetUSVState(offb_set_mode, 1);

        } else {
            m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position =
                    m_multi_vehicle_.leader_usv.current_local_pos.pose.position;
            setVehicleCtrlData();
        }
    }
}

 void MultiBoatControl::changeToLocalTarget() {
    geometry_msgs::PoseStamped body_usv1, body_usv2, body_usv3, local_usv1, local_usv2, local_usv3;
    body_usv1.pose.position.x = follow_usv1_.x();
    body_usv1.pose.position.y = follow_usv1_.y();
    body_usv1.pose.position.z = follow_usv1_.z();

    body_usv2.pose.position.x = follow_usv2_.x();
    body_usv2.pose.position.y = follow_usv2_.y();
    body_usv2.pose.position.z = follow_usv2_.z();

    body_usv3.pose.position.x = follow_usv3_.x();
    body_usv3.pose.position.y = follow_usv3_.y();
    body_usv3.pose.position.z = follow_usv3_.z();

     Calculate::getInstance()->bodyFrame2LocalFrame(body_usv1, local_usv1, way_bear_);
    Calculate::getInstance()->bodyFrame2LocalFrame(body_usv2, local_usv2, way_bear_);
    Calculate::getInstance()->bodyFrame2LocalFrame(body_usv3, local_usv3, way_bear_);

     follow_usv1_.x() = local_usv1.pose.position.x;
     follow_usv1_.y() = local_usv1.pose.position.y;
     follow_usv1_.z() = local_usv1.pose.position.z;

     follow_usv2_.x() = local_usv2.pose.position.x;
     follow_usv2_.y() = local_usv2.pose.position.y;
     follow_usv2_.z() = local_usv2.pose.position.z;

     follow_usv3_.x() = local_usv3.pose.position.x;
     follow_usv3_.y() = local_usv3.pose.position.y;
     follow_usv3_.z() = local_usv3.pose.position.z;
}

void MultiBoatControl::GetTakeoffPos() {
    util_log("xxxxxxxxxxxxxxxxxxx, m_multi_vehicle_.uuv1.drone_id = %d, m_multi_vehicle_.uuv1.homePosition.geo.latitude = %.8f",
             m_multi_vehicle_.uuv1.drone_id, m_multi_vehicle_.uuv1.homePosition.geo.latitude);

    if (m_multi_vehicle_.uuv1.drone_id != 0 && m_multi_vehicle_.uuv1.homePosition.geo.latitude != 0) {

        util_log("calculate uuv and usvs home position");
        usv1_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.usv1.homePosition.geo.latitude,
                                               m_multi_vehicle_.usv1.homePosition.geo.longitude,0};
        usv2_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.usv2.homePosition.geo.latitude,
                                               m_multi_vehicle_.usv2.homePosition.geo.longitude,0};
        usv3_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.usv3.homePosition.geo.latitude,
                                               m_multi_vehicle_.usv3.homePosition.geo.longitude,0};
        uuv1_takeoff_gps_pos_ = GlobalPosition{m_multi_vehicle_.uuv1.homePosition.geo.latitude,
                                               m_multi_vehicle_.uuv1.homePosition.geo.longitude,0};

        Calculate::getInstance()->GetLocalPos(uuv1_takeoff_gps_pos_, usv1_takeoff_gps_pos_, follow_usv1_to_uuv1_);
        Calculate::getInstance()->GetLocalPos(uuv1_takeoff_gps_pos_, usv2_takeoff_gps_pos_, follow_usv2_to_uuv1_);
        Calculate::getInstance()->GetLocalPos(uuv1_takeoff_gps_pos_, usv3_takeoff_gps_pos_, follow_usv3_to_uuv1_);

    }
}

void MultiBoatControl::calcFollowUUVPos() {
    // 目标相对位置-当前相对位置+当前在该飞机坐标系下的绝对位置
    target_usv1_.x() = m_multi_vehicle_.uuv1.current_local_pos.pose.position.x + follow_usv1_.x() + follow_usv1_to_uuv1_.x();
    target_usv1_.y() = m_multi_vehicle_.uuv1.current_local_pos.pose.position.y + follow_usv1_.y() + follow_usv1_to_uuv1_.y();
    target_usv1_.z() = 0;
    follow_usv1_keep_local_ = TVec3 (follow_usv1_.x() + follow_usv1_to_uuv1_.x(), follow_usv1_.y() + follow_usv1_to_uuv1_.y(), 0);

    target_usv2_.x() = m_multi_vehicle_.uuv1.current_local_pos.pose.position.x + follow_usv2_.x() + follow_usv2_to_uuv1_.x();
    target_usv2_.y() = m_multi_vehicle_.uuv1.current_local_pos.pose.position.y + follow_usv2_.y() + follow_usv2_to_uuv1_.y();
    target_usv2_.z() = 0;
    follow_usv2_keep_local_ = TVec3 (follow_usv2_.x() + follow_usv2_to_uuv1_.x(), follow_usv2_.y() + follow_usv2_to_uuv1_.y(), 0);

    target_usv3_.x() = m_multi_vehicle_.uuv1.current_local_pos.pose.position.x + follow_usv3_.x() + follow_usv3_to_uuv1_.x();
    target_usv3_.y() = m_multi_vehicle_.uuv1.current_local_pos.pose.position.y + follow_usv3_.y() + follow_usv3_to_uuv1_.y();
    target_usv3_.z() = 0;
    follow_usv3_keep_local_ = TVec3 (follow_usv3_.x() + follow_usv3_to_uuv1_.x(), follow_usv3_.y() + follow_usv3_to_uuv1_.y(), 0);
}

void MultiBoatControl::SetFunctionOutPut() {
    m_multi_vehicle_.usv1.follower_to_leader_pos = target_usv1_;
    m_multi_vehicle_.usv2.follower_to_leader_pos = target_usv2_;
    m_multi_vehicle_.usv3.follower_to_leader_pos = target_usv3_;

    m_multi_vehicle_.usv1.follower_keep_pos = follow_usv1_keep_local_;
    m_multi_vehicle_.usv2.follower_keep_pos = follow_usv2_keep_local_;
    m_multi_vehicle_.usv3.follower_keep_pos = follow_usv3_keep_local_;
    util_log("target_usv1_ = (%.2f, %.2f, %.2f)", target_usv1_.x(), target_usv1_.y(), target_usv1_.z());

    DataMan::getInstance()->SetUSVFormationData(m_multi_vehicle_, 0);

    m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.x = target_usv1_.x();
    m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.y = target_usv1_.y();
    m_multi_vehicle_.usv1.target_local_pos_sp.pose.position.z = target_usv1_.z();

    m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.x = target_usv2_.x();
    m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.y = target_usv2_.y();
    m_multi_vehicle_.usv2.target_local_pos_sp.pose.position.z = target_usv2_.z();

    m_multi_vehicle_.usv3.target_local_pos_sp.pose.position.x = target_usv3_.x();
    m_multi_vehicle_.usv3.target_local_pos_sp.pose.position.y = target_usv3_.y();
    m_multi_vehicle_.usv3.target_local_pos_sp.pose.position.z = target_usv3_.z();

    DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);
}

void MultiBoatControl::chooseLeader() {
    if (m_multi_vehicle_.usv1.current_state.connected
//        && m_multi_vehicle_.usv1.current_state.armed
        /* && m_multi_vehicle_.usv1.current_state.mode == "OFFBOARD"*/) {
        m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv1;
    } else {
        if (m_multi_vehicle_.usv2.current_state.connected
//            && m_multi_vehicle_.usv2.current_state.armed
            /* && m_multi_vehicle_.usv2.current_state.mode == "OFFBOARD"*/) {
            m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv2;
        } else {
            if (m_multi_vehicle_.usv3.current_state.connected
//                &&m_multi_vehicle_.usv3.current_state.armed
                /*&& m_multi_vehicle_.usv3.current_state.mode == "OFFBOARD"*/) {
                m_multi_vehicle_.leader_usv = m_multi_vehicle_.usv3;
            }
        }
    }
    m_multi_vehicle_.leader_usv.movement_state = usv_state_;
    m_multi_vehicle_.leader_uav.is_formation = is_formation_;
    DataMan::getInstance()->SetUSVLeader(m_multi_vehicle_.leader_usv);
    util_log("usv leader = %d" , m_multi_vehicle_.leader_usv.drone_id);
}

geometry_msgs::PoseStamped MultiBoatControl::CalculateTargetPos(geometry_msgs::PoseStamped& target_local_pos, TVec3 &formation_target) {
    geometry_msgs::PoseStamped target_local_pos_sp;
//    util_log("not use formation_target (%.2f, %.2f, %.2f)", formation_target(0), formation_target(1), formation_target(2));
    target_local_pos_sp.pose.position.x = target_local_pos.pose.position.x + formation_target(0);
    target_local_pos_sp.pose.position.y = target_local_pos.pose.position.y + formation_target(1);
    target_local_pos_sp.pose.position.z = target_local_pos.pose.position.z;
    return target_local_pos_sp;
}

void MultiBoatControl::setVehicleCtrlData() {
//    util_log("usv set vehicle control !!!!!");
    m_multi_vehicle_.usv1.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.target_local_pos_sp, m_multi_vehicle_.usv1.follower_keep_pos);
    m_multi_vehicle_.usv2.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.current_local_pos, m_multi_vehicle_.usv2.follower_keep_pos);
    m_multi_vehicle_.usv3.target_local_pos_sp = CalculateTargetPos(m_multi_vehicle_.leader_usv.current_local_pos, m_multi_vehicle_.usv3.follower_keep_pos);
    m_multi_vehicle_.leader_usv.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
    target_usv2_init_ = m_multi_vehicle_.usv2.target_local_pos_sp;
    target_usv3_init_ = m_multi_vehicle_.usv3.target_local_pos_sp;

    if (m_multi_vehicle_.leader_usv.droneControl.speed_ctrl) {
        m_multi_vehicle_.usv1.droneControl = m_multi_vehicle_.leader_usv.droneControl;
        util_log("usv speed_ctrl!!!!!");
    }
    DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);
}

bool MultiBoatControl::pos_reached(geometry_msgs::PoseStamped &current_pos, geometry_msgs::PoseStamped &target_pos,
                                float err_allow){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;

    return sqrt(err_px * err_px + err_py * err_py ) < err_allow;
}

void MultiBoatControl::USVManualControl() {
    m_multi_vehicle_.usv1.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
    m_multi_vehicle_.usv2.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;
    m_multi_vehicle_.usv3.target_local_pos_sp = m_multi_vehicle_.leader_usv.target_local_pos_sp;

    util_log("!!!!!!usv manual control output = (%.2f, %.2f, %.2f)", m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.x,
             m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.y, m_multi_vehicle_.leader_usv.target_local_pos_sp.pose.position.z);

    DataMan::getInstance()->SetBoatControlData(m_multi_vehicle_);
}
