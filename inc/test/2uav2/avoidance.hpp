//
// Created by zhouhua on 19-12-13.
//

#ifndef OFFBOARD_AVOIDANCE_HPP
#define OFFBOARD_AVOIDANCE_HPP

#include "Cinc.hpp"
#include "dataMan.hpp"
#include "Calculate.hpp"
#include "DataMan.hpp"

static float K_max_avodiance_pos_ = 20;
static float K_avodiance_safe_pos_ = 2;
class avoidance {
public:
    avoidance();

    ~avoidance() {};

    void Oninit(const int config) ;

    void GetData()  ;

    void DoProgress()  ;

    void SetFunctionOutPut() ;

//    void get_uav_avo_output(vector<M_Drone_Avoidace> &m_drone_avoidance);

    void checkVerticalDistance(const multi_vehicle_vec &vehicles);

    void checkHorizontalDistance(const multi_vehicle_vec &vehicles);

    void Getvehicledistance(const M_Drone &vehicle1, const M_Drone &vehicle2, float &distance_h, float &distance_v);

    static  avoidance* getInstance();

private:
    void checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2,
                       float &m_drone_avoidance1, float &m_drone_avoidance2);
    bool checkHorizontalArrive(const M_Drone &vehicle);

    static avoidance* l_lptr;

    bool is_run_avoidance_;
    float distance_h_12_ = 0;

    multi_vehicle_vec multi_vehicle_{};

    M_Drone_Avoidace height_avoidance_uav1_;
    M_Drone_Avoidace height_avoidance_uav2_;
    int config_;

};

#endif //OFFBOARD_AVOIDANCE_HPP
