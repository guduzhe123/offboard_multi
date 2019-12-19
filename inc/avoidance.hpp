//
// Created by zhouhua on 19-12-13.
//

#ifndef OFFBOARD_AVOIDANCE_HPP
#define OFFBOARD_AVOIDANCE_HPP

#include "Cinc.hpp"
#include "Multi_formation.hpp"

class Avoidance {
public:

    Avoidance();

    ~Avoidance()= default;

    void Oninit();

    void uav_avoidance(const M_Drone &mDrone);

    void usv_avoidance();

    void get_uav_avo_output(vector<M_Drone_Avoidace> &m_drone_avoidance);

    void checkHorizontalDistance(const M_Drone &vehicle1, const M_Drone &vehicle2);

    static  Avoidance* getInstance();

private:
    void checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2,
                       float &m_drone_avoidance1, float &m_drone_avoidance2);


    static Avoidance* l_lptr;

    bool is_run_avoidance_;

    multi_vehicle_vec multi_vehicle_{};

    M_Drone_Avoidace height_avoidance_uav1_;
    M_Drone_Avoidace height_avoidance_uav2_;
    M_Drone_Avoidace height_avoidance_uav3_;
    M_Drone_Avoidace height_avoidance_uav4_;


};

#endif //OFFBOARD_AVOIDANCE_HPP
