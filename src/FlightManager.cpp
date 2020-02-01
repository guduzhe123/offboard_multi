//
// Created by zhouhua on 19-9-22.
//

#include "FlightManager.hpp"

FlightManager* FlightManager::l_pInst = NULL;

FlightManager::FlightManager()
                {

}

FlightManager* FlightManager::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new FlightManager();
    }
    return l_pInst;
}

void FlightManager::OnInitConfig(IMsgRosManager *msg_manager) {
    msg_manager_ = msg_manager;
    DataMan::getInstance()->SetCallBack(this);
}

void FlightManager::GetData() {
    m_multi_vehicle_ = DataMan::getInstance()->GetData();
}

void FlightManager::DoProgress() {
    // factory method
    if (m_multi_vehicle_.leader_uav.current_state.armed || m_multi_vehicle_.leader_usv.current_state.armed) {
        auto ifactory_tor = m_control_function_vector_.begin();
        auto factory_end = m_control_function_vector_.end();
        for ( ; ifactory_tor != factory_end; ifactory_tor++) {
            if ((*ifactory_tor) != nullptr) {
                (*ifactory_tor)->GetData();
                (*ifactory_tor)->DoProgress();
            }
        }
    }

    // 运用工厂模式
    auto itor = m_vehicle_control_list_.begin();
    auto end = m_vehicle_control_list_.end();
    for ( ; itor != end; itor++) {
        if ((*itor) != nullptr) {
            (*itor)->getData();
            (*itor)->chooseLeader();
            (*itor)->DoProgress();
        }
    }
}

void FlightManager::AddVehicleControlProgress(IVehicleControlFactory *IVehicleControl) {
    cur_f = IVehicleControl->VehicleControlCreator();
    m_vehicle_control_list_.push_back(cur_f);
    util_log("vehicle control size = %d" , m_vehicle_control_list_.size());
}


void FlightManager::AddFunctionProgress(IFunctionFactory *factory) {
    IControlFunction* m_func;
    m_func = factory->FunctionCreator();
    m_control_function_vector_.push_back(m_func);
}

void FlightManager::FunctionStateUpdate(IFunctionFactory *factory) {
    m_func_ = factory->FunctionCreator();
}

void FlightManager::OnFlightDataUpdate(FDATA_TYPE data_type) {
    if (data_type == FDATA_DRONE_TARGTE) {
        if (drone_avodiance_update_) {
            GetData();
            util_log("uav targte .z = %.2f", m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.z);
            util_log(
                    "height_avoidance_uav1_ = %.2f, height_avoidance_uav2_ = %.2f, height_avoidance_uav3_ = %.2f, height_avoidance_uav4_ = %.2f",
                    m_multi_vehicle_.uav1.avoidance_pos.z(), m_multi_vehicle_.uav2.avoidance_pos.z(),
                    m_multi_vehicle_.uav3.avoidance_pos.z(), m_multi_vehicle_.uav4.avoidance_pos.z());

            if (!isnan(m_multi_vehicle_.uav1.avoidance_pos.z()) && !isnan(m_multi_vehicle_.uav2.avoidance_pos.z()) &&
                !isnan(m_multi_vehicle_.uav3.avoidance_pos.z()) && !isnan(m_multi_vehicle_.uav4.avoidance_pos.z())) {
                m_multi_vehicle_.uav1.target_local_pos_sp.pose.position.z += m_multi_vehicle_.uav1.avoidance_pos.z();
                m_multi_vehicle_.uav2.target_local_pos_sp.pose.position.z += m_multi_vehicle_.uav2.avoidance_pos.z();
                m_multi_vehicle_.uav3.target_local_pos_sp.pose.position.z += m_multi_vehicle_.uav3.avoidance_pos.z();
                m_multi_vehicle_.uav4.target_local_pos_sp.pose.position.z += m_multi_vehicle_.uav4.avoidance_pos.z();
            }
        }
        DataMan::getInstance()->PublishDroneControlData(m_multi_vehicle_); // pulish at last
    }

    if (data_type == FDATA_AVOIDANCE) {
        drone_avodiance_update_ = true;
    }

    if (data_type == FDATA_MANUAL_COMMAND) {
        util_log("get user command = %d", m_multi_vehicle_.user_command);
        m_func_->Oninit(m_multi_vehicle_.user_command);
    }
}



