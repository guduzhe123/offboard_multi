/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


#include "multi_offboard.hpp"

MultiOffboard* MultiOffboard::l_pInst = NULL;

MultiOffboard::MultiOffboard() :
        m_drone_uav1_{},
        m_drone_uav2_{},
        m_drone_uav3_{},
        m_drone_uav4_{},
        m_drone_uav5_{},
        m_drone_uav6_{},
        m_drone_uav7_{},
        drone_uav1_{},
        drone_uav2_{},
        drone_uav3_{},
        drone_uav4_{},
        drone_uav5_{},
        drone_uav6_{},
        drone_uav7_{},
        drone_uav_leader_{},
        drone_usv_leader_{},
        is_offboard(false),
        is_armed(false),
        usv_armed(false),
        curr_altitude(0),
        uav_state_(TAKEOFF),
        usv_state_(USV_INIT),
        leader_uav_id_(UAV1),
        leader_usv_id_(USV1)
        {

}

void MultiOffboard::uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav1_.current_state = *msg;
    m_drone_uav1_.current_state = *msg;
    m_drone_uav1_.drone_id = UAV1;
}

void MultiOffboard::uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav2_.current_state = *msg;
    m_drone_uav2_.current_state = *msg;
    m_drone_uav2_.drone_id = UAV2;
}

void MultiOffboard::uav3_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav3_.current_state = *msg;
    m_drone_uav3_.current_state = *msg;
    m_drone_uav3_.drone_id = UAV3;
}

void MultiOffboard::uav4_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav4_.current_state = *msg;
    m_drone_uav4_.current_state = *msg;
    m_drone_uav4_.drone_id = UAV4;
}

void MultiOffboard::uav5_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav5_.current_state = *msg;
    m_drone_uav5_.current_state = *msg;
    m_drone_uav5_.drone_id = USV1;
}

void MultiOffboard::uav6_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav6_.current_state = *msg;
    m_drone_uav6_.current_state = *msg;
    m_drone_uav6_.drone_id = USV2;
}

void MultiOffboard::uav7_state_cb(const mavros_msgs::State::ConstPtr& msg){
    drone_uav7_.current_state = *msg;
    m_drone_uav7_.current_state = *msg;
    m_drone_uav7_.drone_id = USV3;
}


void MultiOffboard::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_vfr_hud = *msg;
}

void MultiOffboard::uav1_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav1_.altitude = msg->altitude;
    m_drone_uav1_.longtitude = msg->longitude;
    m_drone_uav1_.latitude = msg->latitude;
    m_drone_uav1_.drone_id = UAV1;
}
void MultiOffboard::uav2_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav2_.altitude = msg->altitude;
    m_drone_uav2_.longtitude = msg->longitude;
    m_drone_uav2_.latitude = msg->latitude;
    m_drone_uav2_.drone_id = UAV2;
}
void MultiOffboard::uav3_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav3_.altitude = msg->altitude;
    m_drone_uav3_.longtitude = msg->longitude;
    m_drone_uav3_.latitude = msg->latitude;
    m_drone_uav3_.drone_id = UAV3;
}
void MultiOffboard::uav4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav4_.altitude = msg->altitude;
    m_drone_uav4_.longtitude = msg->longitude;
    m_drone_uav4_.latitude = msg->latitude;
    m_drone_uav4_.drone_id = UAV4;
}
void MultiOffboard::uav5_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav5_.altitude = msg->altitude;
    m_drone_uav5_.longtitude = msg->longitude;
    m_drone_uav5_.latitude = msg->latitude;
    m_drone_uav5_.drone_id = USV1;
}
void MultiOffboard::uav6_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav6_.altitude = msg->altitude;
    m_drone_uav6_.longtitude = msg->longitude;
    m_drone_uav6_.latitude = msg->latitude;
    m_drone_uav6_.drone_id = USV2;
}

void MultiOffboard::uav7_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    m_drone_uav7_.altitude = msg->altitude;
    m_drone_uav7_.longtitude = msg->longitude;
    m_drone_uav7_.latitude = msg->latitude;
    m_drone_uav7_.drone_id = USV3;
}

void MultiOffboard::uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav1_.current_local_pos = *msg;

    m_drone_uav1_.current_local_pos = *msg;
    m_drone_uav1_.drone_id = UAV1;
}

void MultiOffboard::uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav2_.current_local_pos = *msg;

    m_drone_uav2_.current_local_pos = *msg;
    m_drone_uav2_.drone_id = UAV2;
}

void MultiOffboard::uav3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav3_.current_local_pos = *msg;

    m_drone_uav3_.current_local_pos = *msg;
    m_drone_uav3_.drone_id = UAV3;
}

void MultiOffboard::uav4_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav4_.current_local_pos = *msg;

    m_drone_uav4_.current_local_pos = *msg;
    m_drone_uav4_.drone_id = UAV4;
}

void MultiOffboard::uav5_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav5_.current_local_pos = *msg;

    m_drone_uav5_.current_local_pos = *msg;
    m_drone_uav5_.drone_id = USV1;
}

void MultiOffboard::uav6_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav6_.current_local_pos = *msg;

    m_drone_uav6_.current_local_pos = *msg;
    m_drone_uav6_.drone_id = USV2;
}

void MultiOffboard::uav7_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_uav7_.current_local_pos = *msg;

    m_drone_uav7_.current_local_pos = *msg;
    m_drone_uav7_.drone_id = USV3;
}

void MultiOffboard::uav1_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav1_.current_local_pos_sp = *msg;
}
void MultiOffboard::uav2_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav2_.current_local_pos_sp = *msg;
}
void MultiOffboard::uav3_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav3_.current_local_pos_sp = *msg;
}
void MultiOffboard::uav4_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav4_.current_local_pos_sp = *msg;
}
void MultiOffboard::uav5_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav5_.current_local_pos_sp = *msg;
}
void MultiOffboard::uav6_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav6_.current_local_pos_sp = *msg;
}
void MultiOffboard::uav7_local_pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    m_drone_uav7_.current_local_pos_sp = *msg;
}

void MultiOffboard::uav1_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav1 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::uav2_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav2 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::uav3_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav3 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::uav4_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav4 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::uav5_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav5 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::uav6_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav6 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::uav7_debug_value_cb(const mavros_msgs::DebugValue::ConstPtr& msg){
    mavros_msgs::DebugValue debugValue;
    debugValue = *msg;
    util_log("uav7 debug_value x = %.2f, y = %.2f, z = %.2f", debugValue.data[0], debugValue.data[1], debugValue.data[2]);
    int config = (int)debugValue.data[0];
    arm_command_ = config;
//    DataMan::getInstance()->SetUserCommand(config);
    PathCreator::geInstance()->CreateUAVFormationInit(config);
}

void MultiOffboard::OnInit() {
    drone_uav1_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, &MultiOffboard::uav1_state_cb, this);
    drone_uav2_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, &MultiOffboard::uav2_state_cb, this);
    drone_uav3_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, &MultiOffboard::uav3_state_cb, this);
    drone_uav4_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav4/mavros/state", 10, &MultiOffboard::uav4_state_cb, this);
    drone_uav5_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav5/mavros/state", 10, &MultiOffboard::uav5_state_cb, this);
    drone_uav6_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav6/mavros/state", 10, &MultiOffboard::uav6_state_cb, this);
    drone_uav7_.state_sub = nh.subscribe<mavros_msgs::State>
            ("uav7/mavros/state", 10, &MultiOffboard::uav7_state_cb, this);
    uav1_vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("uav1/mavros/vfr_hud", 10, &MultiOffboard::vrf_hud_cb, this);
    uav1_g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav1/mavros/setpoint_velocity/cmd_vel", 100);


    drone_uav1_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    drone_uav1_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    drone_uav1_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 5);
    drone_uav1_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, &MultiOffboard::uav1_local_pos_cb, this);
    drone_uav1_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/global", 10, &MultiOffboard::uav1_global_pos_cb, this);
    drone_uav1_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav1/mavros/setpoint_raw/target_global", 10);
    drone_uav1_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav1/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav1_debug_value_cb, this);
    drone_uav1_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav1/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav1_local_pos_sp_cb, this);

    drone_uav2_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");
    drone_uav2_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");
    drone_uav2_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 5);
    drone_uav2_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, &MultiOffboard::uav2_local_pos_cb, this);
    drone_uav2_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav2/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav2_local_pos_sp_cb, this);
    drone_uav2_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav2/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav2_debug_value_cb, this);
    drone_uav2_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav2/mavros/global_position/global", 10, &MultiOffboard::uav2_global_pos_cb, this);
    drone_uav2_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav2/mavros/setpoint_raw/target_global", 10);

    drone_uav3_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav3/mavros/set_mode");
    drone_uav3_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav3/mavros/cmd/arming");
    drone_uav3_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav3/mavros/setpoint_position/local", 5);
    drone_uav3_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav3/mavros/local_position/pose", 10, &MultiOffboard::uav3_local_pos_cb, this);
    drone_uav3_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav3/mavros/global_position/global", 10, &MultiOffboard::uav3_global_pos_cb, this);
    drone_uav3_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav3/mavros/setpoint_raw/target_global", 10);
    drone_uav3_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav3/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav3_debug_value_cb, this);
    drone_uav3_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav3/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav3_local_pos_sp_cb, this);

    drone_uav4_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav4/mavros/set_mode");
    drone_uav4_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav4/mavros/cmd/arming");
    drone_uav4_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav4/mavros/setpoint_position/local", 5);
    drone_uav4_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav4/mavros/local_position/pose", 10, &MultiOffboard::uav4_local_pos_cb, this);
    drone_uav4_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav4/mavros/global_position/global", 10, &MultiOffboard::uav4_global_pos_cb, this);
    drone_uav4_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav4/mavros/setpoint_raw/target_global", 10);
    drone_uav4_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav4/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav4_debug_value_cb, this);
    drone_uav4_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav4/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav4_debug_value_cb, this);
    drone_uav4_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav4/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav4_local_pos_sp_cb, this);

    drone_uav5_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav5/mavros/set_mode");
    drone_uav5_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav5/mavros/cmd/arming");
    drone_uav5_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav5/mavros/setpoint_position/local", 5);
    drone_uav5_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav5/mavros/local_position/pose", 10, &MultiOffboard::uav5_local_pos_cb, this);
    drone_uav5_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav5/mavros/global_position/global", 10, &MultiOffboard::uav5_global_pos_cb, this);
    drone_uav5_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav5/mavros/setpoint_raw/target_global", 10);
    drone_uav5_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav5/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav5_debug_value_cb, this);
    drone_uav5_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav5/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav5_local_pos_sp_cb, this);

    drone_uav6_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav6/mavros/set_mode");
    drone_uav6_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav6/mavros/cmd/arming");
    drone_uav6_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav6/mavros/setpoint_position/local", 5);
    drone_uav6_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav6/mavros/local_position/pose", 10, &MultiOffboard::uav6_local_pos_cb, this);
    drone_uav6_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav6/mavros/global_position/global", 10, &MultiOffboard::uav6_global_pos_cb, this);
    drone_uav6_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav6/mavros/setpoint_raw/target_global", 10);
    drone_uav6_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav6/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav6_debug_value_cb, this);
    drone_uav6_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav6/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav6_local_pos_sp_cb, this);

    drone_uav7_.set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav7/mavros/set_mode");
    drone_uav7_.arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav7/mavros/cmd/arming");
    drone_uav7_.local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav7/mavros/setpoint_position/local", 5);
    drone_uav7_.local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav7/mavros/local_position/pose", 10, &MultiOffboard::uav7_local_pos_cb, this);
    drone_uav7_.global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav7/mavros/global_position/global", 10, &MultiOffboard::uav7_global_pos_cb, this);
    drone_uav7_.global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("uav7/mavros/setpoint_raw/target_global", 10);
    drone_uav7_.multi_formation_sub = nh.subscribe<mavros_msgs::DebugValue>
            ("uav7/mavros/debug_value/debug_vector", 10, &MultiOffboard::uav7_debug_value_cb, this);
    drone_uav7_.local_pos_sp_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("uav7/mavros/setpoint_raw/target_local", 10, &MultiOffboard::uav7_local_pos_sp_cb, this);
}

MultiOffboard* MultiOffboard::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new MultiOffboard();
    }
    return l_pInst;
}

void MultiOffboard::drone_pos_update() {
    DataMan::getInstance()->SetDroneData(m_drone_uav1_);
    DataMan::getInstance()->SetDroneData(m_drone_uav2_);
    DataMan::getInstance()->SetDroneData(m_drone_uav3_);
    DataMan::getInstance()->SetDroneData(m_drone_uav4_);
    DataMan::getInstance()->SetDroneData(m_drone_uav5_);
    DataMan::getInstance()->SetDroneData(m_drone_uav6_);
    DataMan::getInstance()->SetDroneData(m_drone_uav7_);
}

void MultiOffboard::PublishDronePosControl(const multi_vehicle &multi_vehicles) {
    drone_uav1_.local_pos_pub.publish(multi_vehicles.uav1.target_local_pos_sp);
    drone_uav2_.local_pos_pub.publish(multi_vehicles.uav2.target_local_pos_sp);
    drone_uav3_.local_pos_pub.publish(multi_vehicles.uav3.target_local_pos_sp);
    drone_uav4_.local_pos_pub.publish(multi_vehicles.uav4.target_local_pos_sp);
}

void MultiOffboard::PublishBoatPosControl(const multi_vehicle &multi_vehicles) {
    drone_uav5_.local_pos_pub.publish(multi_vehicles.usv1.target_local_pos_sp);
    drone_uav6_.local_pos_pub.publish(multi_vehicles.usv2.target_local_pos_sp);
    drone_uav7_.local_pos_pub.publish(multi_vehicles.usv3.target_local_pos_sp);
}

void MultiOffboard::SetUAVState(mavros_msgs::SetMode &m_mode) {
    drone_uav1_.set_mode_client.call(m_mode);
    drone_uav2_.set_mode_client.call(m_mode);
    drone_uav3_.set_mode_client.call(m_mode);
    drone_uav4_.set_mode_client.call(m_mode);
}

void MultiOffboard::SetUSVState(mavros_msgs::SetMode &arm_command, int usv_id) {
    if (usv_id == 0) {
        drone_uav5_.arming_client.call(arm_command);
        drone_uav6_.arming_client.call(arm_command);
        drone_uav7_.arming_client.call(arm_command);
    }
    if (usv_id == USV1) {
        drone_uav5_.arming_client.call(arm_command);
    }
    if (usv_id == USV2) {
        drone_uav6_.arming_client.call(arm_command);
    }
    if (usv_id == USV3) {
        drone_uav7_.arming_client.call(arm_command);
    }
}

void MultiOffboard::PublishUUVPosControl(const multi_vehicle &multi_vehicles) {

}
