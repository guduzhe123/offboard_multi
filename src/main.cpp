//
// Created by zhouhua on 19-12-5.
//

#include "multi_offboard.hpp"
#include "DataMan.hpp"

//test


int main (int argc, char **argv){
    ros::init(argc, argv, "offboard");
    util_daemonize();
    MultiOffboard::getInstance()->OnInit();
    auto msg_ros = MultiOffboard::getInstance();
    DataMan::getInstance()->OnInit(msg_ros);
    PathCreator::geInstance()->onInit(msg_ros);

    ros::Rate rate(20.0);

    while(ros::ok() && !msg_ros->drone_uav5_.current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    util_log("is uav5 connected = %d", msg_ros->drone_uav5_.current_state.connected);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        msg_ros->drone_pos_update();
        FlightManager::getInstance()->GetData();
        FlightManager::getInstance()->DoProgress();

        if( (msg_ros->drone_uav2_.current_state.mode != "OFFBOARD" ||
             msg_ros->drone_uav3_.current_state.mode != "OFFBOARD" ||
             msg_ros->drone_uav4_.current_state.mode != "OFFBOARD") &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !msg_ros->is_offboard){
            if( msg_ros->drone_uav1_.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                msg_ros->drone_uav1_.set_mode_client.call(offb_set_mode);
                msg_ros->drone_uav2_.set_mode_client.call(offb_set_mode);
                msg_ros->drone_uav3_.set_mode_client.call(offb_set_mode);
                msg_ros->drone_uav4_.set_mode_client.call(offb_set_mode);

                msg_ros->is_offboard = true;
                util_log("UAV Offboard enabled");
            }

            last_request = ros::Time::now();
        } else if (msg_ros->drone_uav1_.current_state.mode == "OFFBOARD"
                    && msg_ros->arm_command_ == 100){ // arm command from ground station
            if( ! msg_ros->drone_uav1_.current_state.armed && !msg_ros->is_armed){
                if( msg_ros->drone_uav1_.arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    msg_ros->drone_uav1_.arming_client.call(arm_cmd);
                    msg_ros->drone_uav2_.arming_client.call(arm_cmd);
                    msg_ros->drone_uav3_.arming_client.call(arm_cmd);
                    msg_ros->drone_uav4_.arming_client.call(arm_cmd);
                    msg_ros->is_armed = true;
                    util_log("UAV Vehicle armed");
                }
            }
        }

        if ((msg_ros->drone_uav5_.current_state.mode != "OFFBOARD" ||
             msg_ros->drone_uav6_.current_state.mode != "OFFBOARD" ||
             msg_ros->drone_uav7_.current_state.mode != "OFFBOARD"))  {
            if (msg_ros->drone_uav5_.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                msg_ros->drone_uav6_.set_mode_client.call(offb_set_mode);
                msg_ros->drone_uav7_.set_mode_client.call(offb_set_mode);
//                util_log("UAS vehicle calling offboard");
//                msg_ros->is_armed = false;
                if (msg_ros->drone_uav5_.current_state.mode == "OFFBOARD") {
                    util_log("USA Vehicle offboard");
                }
            }
        }

        DataMan::getInstance()->PrintData();
        ros::spinOnce();
        rate.sleep();
    }

    retur