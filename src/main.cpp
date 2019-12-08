//
// Created by zhouhua on 19-12-5.
//

#include "multi_offboard.hpp"

int main (int argc, char **argv){
    ros::init(argc, argv, "offboard");

    MultiOffboard::getInstance()->Oninit();

//    util_daemonize();
    ros::Rate rate(20.0);

    while(ros::ok() && !MultiOffboard::getInstance()->uav5_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    util_log("is uav5 connected = %d", MultiOffboard::getInstance()->uav5_current_state.connected);
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( (MultiOffboard::getInstance()->uav1_current_state.mode != "OFFBOARD" || MultiOffboard::getInstance()->uav3_current_state.mode != "OFFBOARD" ||
             MultiOffboard::getInstance()->uav4_current_state.mode != "OFFBOARD" ) &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !MultiOffboard::getInstance()->is_offboard){
            if( MultiOffboard::getInstance()->uav1_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                MultiOffboard::getInstance()->uav2_set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->uav3_set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->uav4_set_mode_client.call(offb_set_mode);

                MultiOffboard::getInstance()->uav5_set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->uav6_set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->uav7_set_mode_client.call(offb_set_mode);

                if (MultiOffboard::getInstance()->uav5_current_state.mode == "OFFBOARD") {
                    util_log("USA Vehicle offboard");
                }
                MultiOffboard::getInstance()->is_offboard = true;
                util_log("UAV Offboard enabled");

            }

            last_request = ros::Time::now();
        } else {
            if( ! MultiOffboard::getInstance()->uav1_current_state.armed && !MultiOffboard::getInstance()->is_armed){
                if( MultiOffboard::getInstance()->uav1_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    MultiOffboard::getInstance()->uav2_arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->uav3_arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->uav4_arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->uav5_arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->uav6_arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->uav7_arming_client.call(arm_cmd);
                    MultiOffboard::getInstance()->is_armed = true;
                    util_log("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if ((MultiOffboard::getInstance()->uav5_current_state.mode != "OFFBOARD" || MultiOffboard::getInstance()->uav6_current_state.mode != "OFFBOARD" ||
             MultiOffboard::getInstance()->uav7_current_state.mode != "OFFBOARD"))  {
            if (MultiOffboard::getInstance()->uav5_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                MultiOffboard::getInstance()->uav6_set_mode_client.call(offb_set_mode);
                MultiOffboard::getInstance()->uav7_set_mode_client.call(offb_set_mode);
//                util_log("UAS vehicle calling offboard");

                if (MultiOffboard::getInstance()->uav5_current_state.mode == "OFFBOARD") {
                    util_log("USA Vehicle offboard");
                }
            }
        }


        MultiOffboard::getInstance()->uav_targte_local_pos();
        MultiOffboard::getInstance()->usv_targte_local_pos();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

