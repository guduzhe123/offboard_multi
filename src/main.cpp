//
// Created by zhouhua on 19-12-5.
//

#include "multi_offboard.hpp"

int main (int argc, char **argv){
    ros::init(argc, argv, "offboard");

    MultiOffboard::getInstance()->Oninit();

    util_daemonize();
    ros::Rate rate(20.0);

    while(ros::ok() && !MultiOffboard::getInstance()->drone_uav5_.current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    util_log("is uav5 connected = %d", MultiOffboard::getInstance()->drone_uav5_.current_state.connected);
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        if( (MultiOffboard::getInstance()->drone_uav2_.current_state.mode != "OFFBOARD" ||
             MultiOffboard::getInstance()->drone_uav3_.current_state.mode != "OFFBOARD" ||
             MultiOffboard::getInstance()->drone_uav4_.current_state.mode != "OFFBOARD") &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !MultiOffboard::getInstance()->is_offboard){
            if( MultiOffboard::getInstance()->drone_uav1_.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                MultiOffboard::getInstance()->drone_uav1_.set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->drone_uav2_.set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->drone_uav3_.set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->drone_uav4_.set_mode_client.call(offb_set_mode);

                MultiOffboard::getInstance()->is_offboard = true;
                util_log("UAV Offboard enabled");
            }

            last_request = ros::Time::now();
        } else if (MultiOffboard::getInstance()->drone_uav1_.current_state.mode == "OFFBOARD"){
            if( ! MultiOffboard::getInstance()->drone_uav1_.current_state.armed && !MultiOffboard::getInstance()->is_armed){
                if( MultiOffboard::getInstance()->drone_uav1_.arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    MultiOffboard::getInstance()->drone_uav1_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->drone_uav2_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->drone_uav3_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->drone_uav4_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->is_armed = true;
                    util_log("UAV Vehicle armed");
                }
            }
        }

        if ((MultiOffboard::getInstance()->drone_uav5_.current_state.mode != "OFFBOARD" ||
             MultiOffboard::getInstance()->drone_uav6_.current_state.mode != "OFFBOARD" ||
             MultiOffboard::getInstance()->drone_uav7_.current_state.mode != "OFFBOARD"))  {
            if (MultiOffboard::getInstance()->drone_uav5_.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                MultiOffboard::getInstance()->drone_uav6_.set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->drone_uav7_.set_mode_client.call(offb_set_mode);
//                util_log("UAS vehicle calling offboard");
                MultiOffboard::getInstance()->is_armed = false;
                if (MultiOffboard::getInstance()->drone_uav5_.current_state.mode == "OFFBOARD") {
                    util_log("USA Vehicle offboard");
                }
            }
        } else /*if (MultiOffboard::getInstance()->debug_value_ == 100)*/{
            if( ! MultiOffboard::getInstance()->drone_uav5_.current_state.armed && !MultiOffboard::getInstance()->is_armed){
                if( MultiOffboard::getInstance()->drone_uav5_.arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    MultiOffboard::getInstance()->drone_uav5_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->drone_uav6_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->drone_uav7_.arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->is_armed = true;
                    util_log("USA Vehicle armed");
                }
            }
        }


        MultiOffboard::getInstance()->usv_targte_local_pos();
        MultiOffboard::getInstance()->uav_target_local_pos();
        MultiOffboard::getInstance()->drone_pos_update();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

