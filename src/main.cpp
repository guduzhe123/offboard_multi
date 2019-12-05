//
// Created by zhouhua on 19-12-5.
//

#include "multi_offboard.hpp"

int main (int argc, char **argv){
    ros::init(argc, argv, "offboard");
    MultiOffboard m_multi;

    m_multi.Oninit();

    util_daemonize();
    ros::Rate rate(20.0);

    while(ros::ok() && !m_multi.uav5_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    util_log("is uav5 connected = %d", m_multi.uav5_current_state.connected);
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( (m_multi.uav1_current_state.mode != "OFFBOARD" || m_multi.uav3_current_state.mode != "OFFBOARD" ||
             m_multi.uav4_current_state.mode != "OFFBOARD" ) &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !m_multi.is_offboard){
            if( m_multi.uav1_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                m_multi.uav2_set_mode_client.call(offb_set_mode);
                m_multi.uav3_set_mode_client.call(offb_set_mode);
                m_multi.uav4_set_mode_client.call(offb_set_mode);

                m_multi.uav5_set_mode_client.call(offb_set_mode);
                m_multi.uav6_set_mode_client.call(offb_set_mode);
                m_multi.uav7_set_mode_client.call(offb_set_mode);

                if (m_multi.uav5_current_state.mode == "OFFBOARD") {
                    util_log("USA Vehicle offboard");
                }
                m_multi.is_offboard = true;
                util_log("UAV Offboard enabled");

            }

            last_request = ros::Time::now();
        } else {
            if( ! m_multi.uav1_current_state.armed && !m_multi.is_armed){
                if( m_multi.uav1_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    m_multi.uav2_arming_client.call(arm_cmd);
                    m_multi.uav3_arming_client.call(arm_cmd);
                    m_multi.uav4_arming_client.call(arm_cmd);
                    m_multi.uav5_arming_client.call(arm_cmd);
                    m_multi.uav6_arming_client.call(arm_cmd);
                    m_multi.uav7_arming_client.call(arm_cmd);
                    m_multi.is_armed = true;
                    util_log("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if ((m_multi.uav5_current_state.mode != "OFFBOARD" || m_multi.uav6_current_state.mode != "OFFBOARD" ||
             m_multi.uav7_current_state.mode != "OFFBOARD"))  {
            if (m_multi.uav5_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                m_multi.uav6_set_mode_client.call(offb_set_mode);
                m_multi.uav7_set_mode_client.call(offb_set_mode);
                util_log("UAS vehicle calling offboard");

                if (m_multi.uav5_current_state.mode == "OFFBOARD") {
                    util_log("USA Vehicle offboard");
                }
            }
        }


        m_multi.uav_targte_local_pos();
        m_multi.usa_targte_local_pos();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

