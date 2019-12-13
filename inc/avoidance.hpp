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

    void uav_avoidance(const M_Drone &m_drone, M_Drone_Avoidace &m_drone_avoidance);

    void usv_avoidance();

    static  Avoidance* getInstance();

private:
    static Avoidance* l_lptr;

};

#endif //OFFBOARD_AVOIDANCE_HPP
